"""Phase E: re-design with measured parameters, and its validation (course objectives CLO 6-12, O1-O5).

  python -m rover.phase_e [--config Config/phase_e.json] [--seeds 8]

Writes Runs/<date>/<stamp>_phase_e/ with REPORT.md, six figures and analysis.json (gains for the
MATLAB cross-check: Matlab/rover_reference_design.m <run folder>). MATLAB/Simulink figures come from
Matlab/acoustic_pursuit_design.m (see the report).

Figures (each answers one question):
  fig1_structure   How controllable/observable is the plant at our operating point? (Gramians vs v0)
  fig2_bandwidth   How fast may the loop be with the measured information delay? (delay and disk margins)
  fig3_poles       Where are the controller and observer poles? (separation, information rate)
  fig4_governor    Why can O1 fail on hardware although the integrator is there? (safety governor vs speed)
  fig5_traces      What does multipath do to a following run? (Tier 3, LQI)
  fig6_tiers       Which demo is feasible at which sensor quality? (success rate per tier)
"""
import argparse
import json
import numpy as np
from scipy.linalg import expm, solve_discrete_are
from .config import create_run, load_config, write_json
from .model import design, linear_model
from .simulation import run_simulation, SCENARIOS, S1_RANGE_BIAS_TABLE

# Validated categorical slots 1-3 (all-pairs safe), text and recessive ink.
BLUE, ORANGE, AQUA = '#2a78d6', '#eb6834', '#1baf7a'
INK, INK2, GRID, REF = '#0b0b0b', '#52514e', '#e4e3df', '#8a8984'
# 's1 measured' is the living room as measured: bearing fringe + room bias (20 Nov s1) and, since 28 Nov,
# the range law too: 8 % pose-to-pose range bias and the systematic deviation (0.75 m low, 2.0 m high).
LEVELS = {'none': {}, 'horn12 target': dict(fringe_ln_sd=.07, room_bearing_sd_deg=10.),
          's1 measured': dict(fringe_ln_sd=.144, room_bearing_sd_deg=23., room_range_ln_sd=.08,
                              range_bias_table=S1_RANGE_BIAS_TABLE)}
# Pre-registered Tier 1 criterion (Local/Docs/NEXT_STEPS_SHORT.md, QW4): stop at 1.0 +- 0.2 m, no contact, within 60 s.
TIER1_TOLERANCE_M = 0.2
LEVEL_COLORS = [BLUE, ORANGE, AQUA]


def style(plt):
    plt.rcParams.update({'font.size': 9, 'axes.edgecolor': REF, 'axes.labelcolor': INK, 'xtick.color': INK2,
                         'ytick.color': INK2, 'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': .6,
                         'axes.spines.top': False, 'axes.spines.right': False, 'figure.facecolor': '#fcfcfb',
                         'axes.facecolor': '#fcfcfb', 'legend.frameon': False, 'lines.linewidth': 2,
                         'axes.titlesize': 10, 'axes.titleweight': 'bold', 'axes.titlecolor': INK})


def effective_delay(cfg):
    """Information delay seen by the loop: transport + half the pair gap (midpoint stamp)
    + half the measurement period (average age between pairs) + half a control period (ZOH)."""
    return cfg['transport_delay']+cfg['pair_gap']/2+cfg['measurement_period']/2+cfg['dt']/2


def augmented(cfg):
    A, B, E, C = linear_model(cfg['nominal_speed'], cfg['desired_range'])
    Aa = np.zeros((4, 4)); Aa[:3, :3] = A; Aa[3, 0] = 1
    return A, B, E, C, Aa, np.vstack([B, np.zeros((1, 2))])


def loop_margins(K, cfg, T=None):
    """Loop-at-a-time margins at the plant input of the state-feedback loop with actuator lags and
    the information delay (the loops are exactly decoupled: K is block-diagonal and G(s) diagonal).
    Delay margin = extra delay beyond T before instability; disk margin alpha (balanced, sigma = 0)."""
    T = effective_delay(cfg) if T is None else T
    _, _, _, _, Aa, Ba = augmented(cfg)
    w = np.logspace(-3, 1.5, 8000); out = []
    for ch, tau in [(0, cfg['tau_speed']), (1, cfg['tau_steer'])]:
        L = np.array([K[ch] @ np.linalg.solve(1j*wi*np.eye(4)-Aa, Ba[:, ch]) / (1j*wi*tau+1)*np.exp(-1j*wi*T) for wi in w])
        ic = np.flatnonzero(np.diff(np.sign(np.abs(L)-1)))
        if not len(ic):
            out.append(dict(wc=np.nan, pm_deg=np.nan, dm_s=np.nan, disk=np.nan)); continue
        i = ic[-1]; wc = w[i]; pm = (180+np.rad2deg(np.angle(L[i]))+180) % 360-180
        alpha = 1/np.max(np.abs((1-L)/(2*(1+L))))
        out.append(dict(wc=float(wc), pm_deg=float(pm), dm_s=float(np.deg2rad(pm)/wc), disk=float(alpha),
                        disk_gm_db=float(20*np.log10((1+alpha/2)/(1-alpha/2))) if alpha < 2 else np.inf,
                        disk_pm_deg=float(np.rad2deg(2*np.arctan(alpha/2)))))
    return out


def pp(cfg, wn):
    return dict(cfg, controller='place', range_poles=[-wn, -1.5*wn], bearing_poles=[-wn, -1.4*wn])


def gramians(cfg, v0, T=6., n=400):
    """Finite-horizon controllability/observability Gramians in tolerance-scaled coordinates."""
    A, B, _, C = linear_model(v0, cfg['desired_range'])
    Sx = np.diag([.15, np.deg2rad(10), np.deg2rad(20)]); Su = np.diag([.1, .3])
    Sy = np.diag([cfg['range_noise_floor_rel']*cfg['desired_range'], np.deg2rad(cfg['bearing_noise_floor_deg'])])
    As = np.linalg.solve(Sx, A@Sx); Bs = np.linalg.solve(Sx, B@Su); Cs = np.linalg.solve(Sy, C@Sx)
    ts = np.linspace(0, T, n); Wc = np.zeros((3, 3)); Wo = np.zeros((3, 3))
    for t in ts:
        F = expm(As*t); Wc += F@Bs@Bs.T@F.T; Wo += F.T@Cs.T@Cs@F
    Wc *= T/n; Wo *= T/n
    return np.linalg.eigvalsh(Wc).min(), np.linalg.eigvalsh(Wo).min(), np.linalg.matrix_rank(np.hstack([B, A@B, A@A@B])), \
        np.linalg.matrix_rank(np.vstack([C, C@A, C@A@A]))


def design_observer(cfg, Tm):
    """Linear 5-state Kalman observer [d_rho, beta, eps, dv_t, w_t] at the measurement period (same as MATLAB)."""
    A, B, E, C = linear_model(cfg['nominal_speed'], cfg['desired_range'])
    Ao = np.block([[A, E], [np.zeros((2, 5))]]); Co = np.hstack([C, np.zeros((2, 2))])
    Phim = expm(Ao*Tm); Qm = np.diag(cfg['observer_qw'])*Tm
    Rm = np.diag([(cfg['range_noise_floor_rel']*cfg['desired_range'])**2, np.deg2rad(cfg['bearing_noise_floor_deg'])**2])
    Pp = solve_discrete_are(Phim.T, Co.T, Qm, Rm); L = Pp@Co.T@np.linalg.inv(Co@Pp@Co.T+Rm)
    z = np.linalg.eigvals(Phim@(np.eye(5)-L@Co))
    return L, z, np.log(z.astype(complex))/Tm


def tier_spec(tier, level):
    mp = LEVELS[level]
    if tier == 1:
        return dict(SCENARIOS['stationary_target'], initial_range_offset=1.0, **mp), dict(desired_range=1.0)
    if tier == 2:
        return dict(target_speed=.10, **mp), dict(desired_range=1.2)
    return dict(SCENARIOS['gentle_turn'], **mp), {}


def tier_outcome(tier, cfg, m, rows):
    a = np.array([r[:13] for r in rows], float); t = a[:, 0]; tail = t > t[-1]-15
    rng_err = float(np.mean(a[tail, 1])-cfg['desired_range']); brms = float(np.rad2deg(np.sqrt(np.mean(a[tail, 2]**2))))
    safe = m['minimum_range_m'] >= cfg['clearance']
    if tier == 1:
        # rows end at cfg['duration'] (60 s), so a latched stand-off was reached within the time limit.
        ok = rows[-1][14] == 'stationary_target_standoff' and safe and abs(a[-1, 1]-cfg['desired_range']) <= TIER1_TOLERANCE_M
        return dict(success=bool(ok), final_range=float(a[-1, 1]), safe=bool(safe))
    ok = m['tracking_fraction'] >= .9 and abs(rng_err) <= .15 and brms <= 20 and safe
    return dict(success=bool(ok), range_error=rng_err, bearing_rms=brms, tracking=m['tracking_fraction'], safe=bool(safe))


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--config', default='Config/phase_e.json'); p.add_argument('--seeds', type=int, default=8)
    a = p.parse_args(argv)
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    style(plt)
    cfg = load_config(a.config); out = create_run('phase_e', cfg); seeds = range(3000, 3000+a.seeds)
    T_eff = effective_delay(cfg); res = {'T_eff_s': T_eff}
    A, B, E, C, Aa, Ba = augmented(cfg)
    res['a'] = cfg['nominal_speed']/cfg['desired_range']

    # ---- 1. structure vs operating point -------------------------------------------------------
    v0s = np.linspace(0, .35, 36); G = np.array([gramians(cfg, v) for v in v0s])
    res['structure_at_v0'] = dict(zip(['wc_min', 'wo_min', 'rank_ctrb', 'rank_obsv'], map(float, gramians(cfg, cfg['nominal_speed']))))
    res['structure_v0_0'] = dict(zip(['wc_min', 'wo_min', 'rank_ctrb', 'rank_obsv'], map(float, gramians(cfg, 0.))))
    fig, ax = plt.subplots(figsize=(6.4, 3.2))
    ax.semilogy(v0s[1:], G[1:, 1], color=BLUE, label='observability  λmin(Wo)')
    ax.semilogy(v0s[1:], G[1:, 0], color=ORANGE, label='controllability  λmin(Wc)')
    ax.axvline(cfg['nominal_speed'], color=REF, ls='--', lw=1)
    ax.text(cfg['nominal_speed']+.005, G[1:, 1].max()*.6, f"operating point\nv0 = {cfg['nominal_speed']} m/s, a = v0/d* = {res['a']:.2f} 1/s",
            color=INK2, fontsize=8, va='top')
    ax.text(.03, G[1, 0]*.9, 'v0 → 0: both ranks drop to 2\n(ε unobservable, β−ε uncontrollable)', color=INK2, fontsize=8, va='top')
    ax.set_xlabel('follower = target speed v0 [m/s]  (d* = %.2f m)' % cfg['desired_range'])
    ax.set_ylabel('smallest Gramian eigenvalue\n(tolerance-scaled, 6 s horizon)')
    ax.set_title('1  Structure: the relative heading ε is weakly observable at walking speed')
    ax.legend(loc='lower right'); fig.tight_layout(); fig.savefig(out/'fig1_structure.png', dpi=160); plt.close(fig)

    # ---- 2. bandwidth vs margins -----------------------------------------------------------------
    wns = np.round(np.arange(.1, .71, .05), 3); M = [loop_margins(design(pp(cfg, w), 'place')['K'], pp(cfg, w)) for w in wns]
    K_lqi = design(cfg, 'lqi')['K']; K_pp = design(cfg, 'place')['K']
    res['margins_lqi'] = loop_margins(K_lqi, cfg); res['margins_place'] = loop_margins(K_pp, cfg)
    res['margins_lqi_no_delay'] = loop_margins(K_lqi, cfg, T=cfg['dt']/2)
    fig, axs = plt.subplots(1, 2, figsize=(9, 3.4))
    for ax, key, lab in [(axs[0], 'dm_s', 'delay margin beyond T_eff [s]'), (axs[1], 'disk', 'disk margin α (balanced)')]:
        for ch, col, name in [(0, BLUE, 'range loop (pole-placement sweep)'), (1, ORANGE, 'bearing loop (pole-placement sweep)')]:
            ax.plot([m[ch]['wc'] for m in M], [m[ch][key] for m in M], color=col, label=name, marker='o', ms=3)
        for ch, col, name in [(0, BLUE, 'LQI design, range'), (1, ORANGE, 'LQI design, bearing')]:
            ax.plot(res['margins_lqi'][ch]['wc'], res['margins_lqi'][ch][key], marker='D', ms=8, color=col, mec=INK, mew=1, ls='none', label=name)
        ax.set_xlabel('gain-crossover frequency ωc [rad/s]'); ax.set_ylabel(lab); ax.set_xlim(0, 1.8)
    axs[0].axhline(T_eff, color=REF, ls='--', lw=1); axs[0].text(1.0, T_eff+.12, f'requirement: +T_eff = {T_eff:.2f} s', color=INK2, fontsize=8)
    axs[1].axhline(.5, color=REF, ls='--', lw=1); axs[1].text(1.0, .55, 'requirement: α ≥ 0.5\n(±4.4 dB, ±28°)', color=INK2, fontsize=8)
    axs[0].legend(loc='upper right', fontsize=7.5); axs[0].set_ylim(0, None); axs[1].set_ylim(0, None)
    fig.suptitle('2  Bandwidth: with T_eff = %.2f s of information delay, crossover must stay below ≈ 0.8 rad/s' % T_eff,
                 fontweight='bold', fontsize=10, color=INK)
    fig.tight_layout(); fig.savefig(out/'fig2_bandwidth.png', dpi=160); plt.close(fig)
    res['bandwidth_sweep'] = [{'wn': float(w), 'range': m[0], 'bearing': m[1]} for w, m in zip(wns, M)]

    # ---- 3. poles: controller vs observer --------------------------------------------------------
    d_lqi = design(cfg, 'lqi'); d_pp = design(cfg, 'place')
    s_lqi = np.log(np.linalg.eigvals(d_lqi['Ad']-d_lqi['Bd']@K_lqi).astype(complex))/cfg['dt']
    s_pp = np.log(np.linalg.eigvals(d_pp['Ad']-d_pp['Bd']@K_pp).astype(complex))/cfg['dt']
    Lo, zo, s_obs = design_observer(cfg, cfg['measurement_period'])
    res['poles'] = {'lqi_s': [str(np.round(x, 4)) for x in s_lqi], 'place_s': [str(np.round(x, 4)) for x in s_pp],
                    'observer_s': [str(np.round(x, 4)) for x in s_obs], 'observer_abs_z': np.abs(zo).tolist()}
    fig, ax = plt.subplots(figsize=(6.4, 3.4))
    for s, col, mk, lab in [(s_lqi, BLUE, 'x', 'LQI controller'), (s_pp, ORANGE, '+', 'pole-placement controller'),
                            (s_obs, AQUA, 'o', 'observer error (Kalman at T_m)')]:
        ax.scatter(s.real, s.imag, color=col, marker=mk, s=60 if mk != 'o' else 36, label=lab, lw=2,
                   facecolors='none' if mk == 'o' else None, zorder=3)
    ax.axvline(0, color=REF, lw=.8); ax.set_xlabel('Re(s) [1/s]'); ax.set_ylabel('Im(s) [rad/s]')
    lo = min(np.min(s_obs.real), np.min(s_lqi.real), np.min(s_pp.real))*1.25; ax.set_xlim(lo, .05)
    k = int(np.argmax(s_obs.real+1e-6*s_obs.imag))
    ax.annotate('heading / target-turn-rate estimate:\nlimited by a = v0/d*, not by tuning', xy=(s_obs[k].real, s_obs[k].imag),
                xytext=(s_obs[k].real-.12, s_obs[k].imag+.12), fontsize=8, color=INK2, ha='right',
                arrowprops=dict(arrowstyle='-', color=REF, lw=.8))
    ax.set_title('3  Poles: range/bearing estimates are fast, the heading estimate is not', fontsize=10)
    ax.legend(loc='lower left', fontsize=8); fig.tight_layout(); fig.savefig(out/'fig3_poles.png', dpi=160); plt.close(fig)

    # ---- 4. governor: O1 vs O4 ---------------------------------------------------------------------
    vts = [.08, .11, .14, .18, .22, .26]; floors = [(.2, ORANGE, 'single tone 0.20ρ'), (.1, BLUE, 'multi-tone 0.10ρ (design)'),
                                                    (.05, AQUA, 'target 0.05ρ')]
    gov = {}
    fig, ax = plt.subplots(figsize=(6.4, 3.3))
    for fl, col, lab in floors:
        rs = []
        for v in vts:
            m, rows = run_simulation(dict(cfg, range_noise_floor_rel=fl, amplitude_noise=1e-6, duration=60.),
                                     dict(target_speed=v), 3000, True)
            r = np.array([q[1] for q in rows]); t = np.array([q[0] for q in rows])
            rs.append(float(np.mean(r[t > 45])) if m['tracking_fraction'] > .5 else np.nan)
        gov[str(fl)] = rs; ax.plot(vts, rs, color=col, marker='o', ms=4, label=lab)
    ax.axhline(cfg['desired_range'], color=REF, ls='--', lw=1); ax.text(.262, cfg['desired_range']-.018, f"d* = {cfg['desired_range']} m", color=INK2, fontsize=8, ha='right', va='top')
    ax.axvline(cfg['nominal_speed'], color=REF, ls=':', lw=1)
    ax.set_xlabel('target speed v_t [m/s]'); ax.set_ylabel('steady true range [m]')
    ax.set_title('4  O1 vs O4: above the knee the safety governor sets the range')
    ax.legend(loc='upper left', fontsize=8); fig.tight_layout(); fig.savefig(out/'fig4_governor.png', dpi=160); plt.close(fig)
    # NaN = no steady tracking (e.g. target faster than the measured max_speed): null in JSON, a gap in the plot.
    res['governor'] = {'target_speeds': vts, 'max_speed': cfg['max_speed'],
                       'steady_range_by_floor': {k: [None if not np.isfinite(x) else x for x in v] for k, v in gov.items()}}

    # ---- 5/6. tiers -------------------------------------------------------------------------------------
    tiers = {}
    for tier in (1, 2, 3):
        ctrls = ['lqi'] if tier == 1 else ['lqi', 'place', 'bearing_only']
        for c in ctrls:
            for lvl in LEVELS:
                spec, over = tier_spec(tier, lvl); cc = dict(cfg, controller=c, **over); outs = []
                for s in seeds:
                    m, rows = run_simulation(cc, dict(spec), s, True)
                    outs.append(tier_outcome(tier, cc, m, rows))
                    if tier == 3 and c == 'lqi' and s == seeds[0]:
                        tiers.setdefault('_trace', {})[lvl] = [(r[0], r[1], r[2]) for r in rows]
                tiers[f'{tier}|{c}|{lvl}'] = outs
                print(f'tier {tier} {c:12s} {lvl:14s} success {sum(o["success"] for o in outs)}/{len(outs)}', flush=True)
    trace = tiers.pop('_trace')
    fig, axs = plt.subplots(2, 1, figsize=(7.2, 4.6), sharex=True)
    for (lvl, tr), col in zip(trace.items(), LEVEL_COLORS):
        tr = np.array(tr); axs[0].plot(tr[:, 0], tr[:, 1], color=col, label=lvl, lw=1.6)
        axs[1].plot(tr[:, 0], np.rad2deg(tr[:, 2]), color=col, lw=1.6)
    axs[0].axhline(cfg['desired_range'], color=REF, ls='--', lw=1); axs[0].set_ylabel('true range [m]')
    axs[0].set_ylim(0, 4)
    for y in (-cfg['bearing_domain_deg'], cfg['bearing_domain_deg']):
        axs[1].axhline(y, color=REF, ls='--', lw=1)
    axs[1].text(1, cfg['bearing_domain_deg']+2, 'calibrated bearing window', color=INK2, fontsize=8)
    axs[1].axvspan(15, 40, color=GRID, alpha=.5, lw=0); axs[1].text(16, -45, 'target turning', color=INK2, fontsize=8)
    axs[1].set_ylim(-55, 55); axs[1].set_ylabel('true bearing [deg]'); axs[1].set_xlabel('time [s]')
    axs[0].legend(loc='upper right', ncol=3, fontsize=8, title='multipath level', title_fontsize=8)
    axs[0].set_title('5  Tier 3 (LQI, target 0.18 m/s with a turn): room bias, not noise, breaks the loop')
    fig.tight_layout(); fig.savefig(out/'fig5_traces.png', dpi=160); plt.close(fig)

    fig, axs = plt.subplots(1, 3, figsize=(10, 3.3), sharey=True)
    names = {1: 'Tier 1  approach to a stationary beacon\n(stand-off 1.0 m, start ≈ 2 m)', 2: 'Tier 2  straight following\n(0.10 m/s, d* = 1.2 m)',
             3: 'Tier 3  following with a turn\n(0.18 m/s, d* = 1.4 m)'}
    summary = {}
    for ax, tier in zip(axs, (1, 2, 3)):
        ctrls = ['lqi'] if tier == 1 else ['lqi', 'place', 'bearing_only']; width = .8/len(ctrls); x = np.arange(len(LEVELS))
        for j, (c, col) in enumerate(zip(ctrls, [BLUE, ORANGE, AQUA])):
            rate = [100*np.mean([o['success'] for o in tiers[f'{tier}|{c}|{l}']]) for l in LEVELS]
            summary[f'{tier}|{c}'] = rate
            bars = ax.bar(x+(j-(len(ctrls)-1)/2)*width, rate, width*.9, color=col, label={'lqi': 'LQI', 'place': 'pole placement', 'bearing_only': 'bearing-only baseline'}[c])
            for b, r in zip(bars, rate):
                ax.text(b.get_x()+b.get_width()/2, r+2, f'{r:.0f}', ha='center', fontsize=7, color=INK2)
        ax.set_xticks(x); ax.set_xticklabels(list(LEVELS), fontsize=8); ax.set_title(names[tier], fontsize=9); ax.set_ylim(0, 112)
        ax.grid(axis='x', visible=False)
    axs[0].set_ylabel('success rate [%%] (%d seeds)' % a.seeds)
    h, l = axs[2].get_legend_handles_labels(); fig.legend(h, l, loc='upper center', ncol=3, fontsize=8, bbox_to_anchor=(.5, .9))
    fig.suptitle('6  Feasible demo vs bearing-sensor quality (x axis: multipath level)', fontsize=10, fontweight='bold', color=INK)
    fig.tight_layout(rect=(0, 0, 1, .86)); fig.savefig(out/'fig6_tiers.png', dpi=160); plt.close(fig)
    res['tiers'] = {k: {'success_rate': float(np.mean([o['success'] for o in v])),
                        'mean_range_error_m': float(np.nanmean([o.get('range_error', np.nan) for o in v])) if 'range_error' in v[0] else None,
                        'mean_bearing_rms_deg': float(np.nanmean([o.get('bearing_rms', np.nan) for o in v])) if 'bearing_rms' in v[0] else None,
                        'mean_tracking': float(np.nanmean([o.get('tracking', np.nan) for o in v])) if 'tracking' in v[0] else None}
                    for k, v in tiers.items()}

    # ---- export ----------------------------------------------------------------------------------------
    from .phase_e_course import course_plots
    res['course'] = course_plots(cfg, out, plt)   # Chapter 6 figures c1..c5 (lecture format)
    res['K'] = K_lqi.tolist(); res['K_place'] = K_pp.tolist(); res['observer_L'] = Lo.tolist()
    write_json(out/'analysis.json', json.loads(json.dumps(res, default=float)))
    write_report(out, cfg, res, summary, a.seeds, a.config)
    print(out)
    return out


def write_report(out, cfg, res, summary, nseeds, config_path='Config/phase_e.json'):
    ml, mp, m0 = res['margins_lqi'], res['margins_place'], res['margins_lqi_no_delay']
    t = lambda k: summary.get(k, [np.nan]*3)
    config_path = str(config_path).replace('\\', '/')
    vehicle = 'measured speed table, provisional lags/braking' if cfg.get('hardware', {}).get('speed_table') else 'provisional vehicle'
    lines = [f"# Phase E results ({out.name})", "",
             f"Config `{config_path}` (amplitude_noise {cfg['amplitude_noise']}, {vehicle}). Operating point d* = {cfg['desired_range']} m, "
             f"v0 = {cfg['nominal_speed']} m/s, a = v0/d* = {res['a']:.3f} 1/s, T_m = {cfg['measurement_period']} s, pair gap {cfg['pair_gap']} s, "
             f"information delay T_eff = {res['T_eff_s']:.2f} s.", "",
             "| Figure | Question | Answer |", "|---|---|---|",
             f"| [1](fig1_structure.png) | Controllable/observable at our operating point? | Yes (ranks 3/3), but λmin(Wo) = {res['structure_at_v0']['wo_min']:.2g}: ε is only seen through β' = aε with a = {res['a']:.2f}; at v0 = 0 both ranks drop to 2. |",
             f"| [2](fig2_bandwidth.png) | How fast may the loop be? | ωn ≲ 0.3 rad/s. LQI: crossover {ml[0]['wc']:.2f}/{ml[1]['wc']:.2f} rad/s, PM {ml[0]['pm_deg']:.0f}/{ml[1]['pm_deg']:.0f}°, delay margin {ml[0]['dm_s']:.2f}/{ml[1]['dm_s']:.2f} s beyond T_eff, disk α {ml[0]['disk']:.2f}/{ml[1]['disk']:.2f}. |",
             f"| [3](fig3_poles.png) | Separation of controller and observer? | Observer |z| per T_m: {', '.join(f'{z:.2f}' for z in sorted(res['poles']['observer_abs_z']))}. |",
             f"| [4](fig4_governor.png) | Why can O1 fail with an integrator? | The safety governor caps speed; O1 holds only left of the knee, which moves right as σρ falls. |",
             "| [5](fig5_traces.png) | What does multipath do? | Noise is filtered; the slowly varying room bias steers the follower off and out of the calibrated window. |",
             f"| [6](fig6_tiers.png) | Which demo is feasible? | Tier 1 {t('1|lqi')[0]:.0f}/{t('1|lqi')[1]:.0f}/{t('1|lqi')[2]:.0f} %, Tier 2 LQI {t('2|lqi')[0]:.0f}/{t('2|lqi')[1]:.0f}/{t('2|lqi')[2]:.0f} %, Tier 3 LQI {t('3|lqi')[0]:.0f}/{t('3|lqi')[1]:.0f}/{t('3|lqi')[2]:.0f} % (none / horn12 target / s1). |",
             "", "## O5 (revised): margins of the sampled MIMO loop", "",
             "Loop-at-a-time at the plant input, state feedback with actuator lags and T_eff. G(s) = diag(−1/s, −(s+a)/s²) and K is block-diagonal, so the two loops are exactly decoupled and loop-at-a-time margins equal the multiloop ones.", "",
             "| Design | Loop | ωc [rad/s] | PM [°] | Delay margin beyond T_eff [s] | Disk α | Disk GM [dB] | Disk PM [°] |", "|---|---|---|---|---|---|---|---|"]
    for name, m in [('LQI', ml), ('pole placement', mp), ('LQI, no information delay', m0)]:
        for ch, lab in [(0, 'range'), (1, 'bearing')]:
            q = m[ch]; lines.append(f"| {name} | {lab} | {q['wc']:.2f} | {q['pm_deg']:.0f} | {q['dm_s']:.2f} | {q['disk']:.2f} | ±{q['disk_gm_db']:.1f} | ±{q['disk_pm_deg']:.0f} |")
    lines += ["", "## Gains", "", f"LQI K (rows δv, ω; cols e, β, ε, ∫e): `{np.round(np.array(res['K']), 4).tolist()}`",
              f"Pole placement K: `{np.round(np.array(res['K_place']), 4).tolist()}`", "",
              "Cross-check in MATLAB: `addpath('Matlab'); rover_reference_design('" + out.as_posix().split('Audio-Tracking-Rover/')[-1] + "')`.", "",
              "## Tier outcomes (mean over seeds)", "", "| Tier, controller, multipath | Success | Range error [m] | Bearing RMS [°] | TRACK |", "|---|---|---|---|---|"]
    for k, v in res['tiers'].items():
        f = lambda x, d=2: '—' if x is None or x != x else f'{x:.{d}f}'
        lines.append(f"| {k.replace('|', ', ')} | {100*v['success_rate']:.0f} % | {f(v['mean_range_error_m'])} | {f(v['mean_bearing_rms_deg'], 1)} | {f(v['mean_tracking'])} |")
    lines += ["", f"{nseeds} seeds per cell; multipath levels: none; horn12 target (fringe 0.07, room 10°); "
              "s1 measured (fringe 0.144, room 23°, range: 8 % pose-to-pose and the 28 Nov range-law table). "
              f"Tier 1 success = stand-off latched within {cfg['duration']:.0f} s at |ρ − 1.0| ≤ {TIER1_TOLERANCE_M} m, no clearance violation.", ""]
    c = res.get('course')
    if c:
        lines += ["## Chapter 6 figures (lecture format)", "",
                  "| Figure | Shows |", "|---|---|",
                  f"| [C1](c1_lqr_effect_of_R.png) | LQR trade-off: peak speed command {c['C1']['0.1']['peak_dv']:.2f} / {c['C1']['1']['peak_dv']:.2f} / {c['C1']['10']['peak_dv']:.2f} m/s for R ×0.1 / ×1 / ×10 (limit {cfg['max_speed']-cfg['nominal_speed']:.2f}). |",
                  "| [C2](c2_lqr_root_locus.png) | Closed-loop poles vs ρ (R = ρ R_design). |",
                  f"| [C3](c3_lqr_vs_lqi.png) | 0.05 m/s target speed step: LQR offset {c['C3']['lqr_offset_m']:.3f} m, LQI {c['C3']['lqi_offset_m']:.1e} m. |",
                  f"| [C4](c4_lqe_tuning.png) | LQE: Kalman gain and estimates for V ×0.01 / ×1 / ×100; ε error minimised at V ≈ {c['C4']['V_scale_opt_eps']:.0f}× for this gentle target. |",
                  f"| [C5](c5_lqg.png) | LQG (LQI + Kalman at T_m): after 10 s, range within {c['C5']['range_max_m']:.2f} m, bearing within {c['C5']['bearing_max_deg']:.0f}°. |", "",
                  f"Continuous LQI gain (scipy CARE, MATLAB `lqi` convention z' = −e): `{np.round(np.array(c['K_lqi_continuous']), 4).tolist()}`.", ""]
    (out/'REPORT.md').write_text('\n'.join(lines), encoding='utf-8')


if __name__ == '__main__':
    main()
