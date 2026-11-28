"""Chapter 6 (LQR / LQI / LQE / LQG) figures in the lecture's format, Python mirror of Matlab course_plots().

Continuous-time designs on the linear model at the operating point (CARE = lqr/lqi, dual CARE = lqe);
the Kalman simulation is the discrete 8-step algorithm at the measurement period T_m. Called by rover.phase_e;
standalone:  python -m rover.phase_e_course [--config Config/phase_e.json] --out <folder>
"""
import argparse
from pathlib import Path
import numpy as np
from scipy.linalg import expm, solve_continuous_are, solve_discrete_are

BLUE, ORANGE, AQUA, INK, REF = '#2a78d6', '#eb6834', '#1baf7a', '#0b0b0b', '#8a8984'
COL = [BLUE, ORANGE, AQUA]


def model(cfg):
    a = cfg['nominal_speed']/cfg['desired_range']
    A = np.array([[0, 0, 0], [0, 0, a], [0, 0, 0.]]); B = np.array([[-1, 0], [0, -1], [0, -1.]])
    E = np.array([[1, 0], [0, 0], [0, 1.]]); C = np.array([[1, 0, 0], [0, 1, 0.]])
    return a, A, B, E, C


def lqr(A, B, Q, R):
    P = solve_continuous_are(A, B, Q, R)
    return np.linalg.solve(R, B.T@P), P


def lqi(cfg, rscale=1.):
    """MATLAB lqi convention: z' = r - y = -e (y = range error), u = -K [x; z]."""
    _, A, B, _, _ = model(cfg)
    Aa = np.zeros((4, 4)); Aa[:3, :3] = A; Aa[3, 0] = -1; Ba = np.vstack([B, np.zeros((1, 2))])
    K, P = lqr(Aa, Ba, np.diag(cfg['lqr_q']), rscale*np.diag(cfg['lqr_r']))
    return K, P, Aa, Ba


def lqe(A, G, C, W, V):
    P = solve_continuous_are(A.T, C.T, G@W@G.T, V)
    return P@C.T@np.linalg.inv(V), P


def simulate(Acl, Bw, x0, w, dt):
    F = expm(np.block([[Acl, Bw], [np.zeros((Bw.shape[1], Acl.shape[0]+Bw.shape[1]))]])*dt)
    n = Acl.shape[0]; Ph, Gd = F[:n, :n], F[:n, n:]; X = np.zeros((len(w), n)); X[0] = x0
    for k in range(len(w)-1):
        X[k+1] = Ph@X[k]+Gd@w[k]
    return X


def kf_run(Ao, Co, W, Rd, dt, km, Y, n, s0):
    """Lecture's 8-step discrete Kalman filter: predict every dt, correct at each measurement."""
    Phi = expm(Ao*dt); Qd = W*dt; xh = np.zeros(5); P = np.diag([s0[0]**2, s0[1]**2, np.deg2rad(20)**2, .05**2, .03**2])
    XH = np.zeros((n, 5)); j = 0
    for k in range(n):
        if k:
            xh = Phi@xh; P = Phi@P@Phi.T+Qd
        if j < len(km) and k == km[j]:
            S = Co@P@Co.T+Rd; K = P@Co.T@np.linalg.inv(S); xh = xh+K@(Y[j]-Co@xh); P = (np.eye(5)-K@Co)@P; j += 1
        XH[k] = xh
    return XH


def course_plots(cfg, out, plt):
    out = Path(out); res = {}
    a, A, B, E, C = model(cfg)
    v0, dvmax = cfg['nominal_speed'], cfg['max_speed']-cfg['nominal_speed']
    wmax = v0*np.tan(np.deg2rad(cfg['max_steer_deg']))/cfg['wheelbase']
    K, P, Aa, Ba = lqi(cfg); res['K_lqi_continuous'] = K.tolist(); res['poles_lqi'] = sorted(np.linalg.eigvals(Aa-Ba@K).real.tolist())
    t = np.arange(0, 40, .02); x0 = np.array([.3, np.deg2rad(15), 0, 0])

    # C1 effect of R
    fig, axs = plt.subplots(2, 2, figsize=(9.6, 5.4), sharex=True); res['C1'] = {}
    for sc, col, lab in zip([.1, 1, 10], COL, ['R × 0.1 (aggressive)', 'R (design)', 'R × 10 (conservative)']):
        Ki, *_ = lqi(cfg, sc); X = simulate(Aa-Ba@Ki, np.zeros((4, 1)), x0, np.zeros((len(t), 1)), .02); U = -X@Ki.T
        for ax, y in zip(axs.flat, [X[:, 0], np.rad2deg(X[:, 1]), U[:, 0], U[:, 1]]):
            ax.plot(t, y, color=col, label=lab)
        res['C1'][str(sc)] = dict(K=Ki.round(4).tolist(), peak_dv=float(np.abs(U[:, 0]).max()), peak_w=float(np.abs(U[:, 1]).max()))
    for y in (dvmax, -v0):
        axs[1, 0].axhline(y, color=REF, ls='--', lw=1)
    for y in (wmax, -wmax):
        axs[1, 1].axhline(y, color=REF, ls='--', lw=1)
    for ax, lab in zip(axs.flat, ['range error δρ [m]', 'bearing β [deg]', 'speed command δv_f [m/s]', 'yaw-rate command ω_f [rad/s]']):
        ax.set_ylabel(lab)
    axs[1, 0].set_xlabel('t [s]'); axs[1, 1].set_xlabel('t [s]')
    h, l = axs[0, 0].get_legend_handles_labels(); fig.legend(h, l, loc='upper center', ncol=3, bbox_to_anchor=(.5, .94), fontsize=8)
    fig.suptitle('C1  LQR trade-off: small R = fast but saturates the actuators (dashed limits); large R = slow', fontsize=10, fontweight='bold')
    fig.tight_layout(rect=(0, 0, 1, .9)); fig.savefig(out/'c1_lqr_effect_of_R.png', dpi=160); plt.close(fig)

    # C2 root locus vs rho
    fig, ax = plt.subplots(figsize=(8, 4))
    for rho in np.logspace(-2, 2, 81):
        Ki, *_ = lqi(cfg, rho); Acl = Aa-Ba@Ki
        pr = np.linalg.eigvals(Acl[np.ix_([0, 3], [0, 3])]); pb = np.linalg.eigvals(Acl[np.ix_([1, 2], [1, 2])])
        ax.plot(pr.real, pr.imag, '.', color=BLUE, ms=4); ax.plot(pb.real, pb.imag, '.', color=ORANGE, ms=4)
    pd = np.linalg.eigvals(Aa-Ba@K)
    ax.plot(pd.real, pd.imag, 'D', mfc='white', mec=INK, ms=8, mew=1.5, label='design ρ = 1')
    ax.plot(0, 0, 'x', color=INK, ms=10, mew=2, label='open-loop poles (all at 0)'); ax.plot(-a, 0, 'o', mfc='none', mec=INK, ms=8, mew=1.5, label='bearing zero at −a')
    ax.plot([], [], '.', color=BLUE, label='range loop (δρ, ∫δρ)'); ax.plot([], [], '.', color=ORANGE, label='bearing loop (β, ε)')
    ax.set_xlim(-2, .05); ax.set_ylim(-.15, .15); ax.axvline(0, color=REF, lw=.8)
    ax.text(-1.95, .115, 'ρ ↓ (cheap control): poles move left, gains grow', fontsize=8, color=INK)
    ax.set_xlabel('Re(s) [1/s]'); ax.set_ylabel('Im(s) [rad/s]'); ax.legend(loc='lower left', fontsize=7.5)
    ax.set_title('C2  LQR root locus vs ρ (R = ρ R_design, ρ = 10⁻²…10²); ρ → ∞ returns the poles to 0', fontsize=10, fontweight='bold')
    fig.tight_layout(); fig.savefig(out/'c2_lqr_root_locus.png', dpi=160); plt.close(fig)

    # C3 LQR vs LQI under a target speed offset
    K3, _ = lqr(A, B, np.diag(cfg['lqr_q'][:3]), np.diag(cfg['lqr_r'])); dvt = .05; w = (t >= 5)*dvt
    X3 = simulate(A-B@K3, E[:, :1], np.zeros(3), w[:, None], .02); XI = simulate(Aa-Ba@K, np.vstack([E[:, :1], [[0]]]), np.zeros(4), w[:, None], .02)
    res['C3'] = dict(K_lqr=K3.round(4).tolist(), lqr_offset_m=float(X3[-1, 0]), lqi_offset_m=float(XI[-1, 0]))
    fig, axs = plt.subplots(1, 2, figsize=(9.6, 3.3))
    axs[0].plot(t, X3[:, 0], color=ORANGE, label='LQR (no integral)'); axs[0].plot(t, XI[:, 0], color=BLUE, label='LQI')
    axs[0].axhline(X3[-1, 0], color=REF, ls=':', lw=1); axs[0].text(.5, X3[-1, 0]+.004, f'LQR offset {X3[-1, 0]:.2f} m', fontsize=8, color=INK)
    axs[1].plot(t, -X3@K3.T[:, 0], color=ORANGE); axs[1].plot(t, -XI@K.T[:, 0], color=BLUE); axs[1].axhline(dvt, color=REF, ls=':', lw=1)
    axs[0].set_ylabel('range error δρ [m]'); axs[1].set_ylabel('speed command δv_f [m/s]')
    for ax in axs:
        ax.set_xlabel('t [s]')
    fig.legend(*axs[0].get_legend_handles_labels(), loc='upper center', ncol=2, bbox_to_anchor=(.5, .9), fontsize=8)
    fig.suptitle('C3  Target speeds up by 0.05 m/s at t = 5 s: LQR keeps an offset, LQI removes it (O1)', fontsize=10, fontweight='bold')
    fig.tight_layout(rect=(0, 0, 1, .85)); fig.savefig(out/'c3_lqr_vs_lqi.png', dpi=160); plt.close(fig)

    # C4 LQE tuning
    Ao = np.block([[A, E], [np.zeros((2, 5))]]); Co = np.hstack([C, np.zeros((2, 2))]); W = np.diag(cfg['observer_qw'])
    sr = cfg['range_noise_floor_rel']*cfg['desired_range']; sb = np.deg2rad(cfg['bearing_noise_floor_deg'])
    Tm = cfg['dt']*round(cfg['measurement_period']/cfg['dt']); V = np.diag([sr**2, sb**2])*Tm
    res['C4'] = {'Ke': {str(v): lqe(Ao, np.eye(5), Co, W, v*V)[0].round(3).tolist() for v in (.01, 1, 100)}}
    rng = np.random.default_rng(7); dt = cfg['dt']; tt = np.arange(0, 60+dt/2, dt); nm = round(Tm/dt)
    wt = np.column_stack([.05*((tt >= 10) & (tt < 25)), .03*((tt >= 30) & (tt < 45))])
    X = simulate(A, E, np.zeros(3), wt, dt); km = np.arange(0, len(tt), nm)
    Y = X[km, :2]+np.column_stack([sr*rng.standard_normal(len(km)), sb*rng.standard_normal(len(km))])
    XH = [kf_run(Ao, Co, W, v*np.diag([sr**2, sb**2]), dt, km, Y, len(tt), (sr, sb)) for v in (.01, 1, 100)]
    vsw = np.logspace(-2, 3, 21); ER = []
    for v in vsw:
        e = kf_run(Ao, Co, W, v*np.diag([sr**2, sb**2]), dt, km, Y, len(tt), (sr, sb))[:, :3]-X
        ER.append(np.rad2deg(np.sqrt(np.mean(e[:, 1:]**2, axis=0))))
    ER = np.array(ER); res['C4']['V_scale_opt_eps'] = float(vsw[np.argmin(ER[:, 1])])
    fig, axs = plt.subplots(4, 1, figsize=(9.6, 8.2)); labs = ['V × 0.01 (trusts the sensor)', 'V (measured noise)', 'V × 100 (trusts the model)']
    for r, (ax, ylab, s) in enumerate(zip(axs[:3], ['δρ [m]', 'β [deg]', 'ε [deg] (hidden)'], [1, 180/np.pi, 180/np.pi])):
        if r < 2:
            ax.plot(tt[km], s*Y[:, r], '.', color=REF, ms=4, label='measurements (every T_m)')
        ax.plot(tt, s*X[:, r], color=INK, lw=2.4, label='true state')
        for xh, col, lab in zip(XH, COL, labs):
            ax.plot(tt, s*xh[:, r], color=col, lw=1.2, label=lab)
        ax.set_ylabel(ylab)
    axs[2].set_xlabel('t [s]  (follower holds v0; target speeds up at 10–25 s, turns at 30–45 s)')
    axs[3].semilogx(vsw, ER[:, 0], '-o', color=BLUE, ms=3, label='β'); axs[3].semilogx(vsw, ER[:, 1], '-o', color=ORANGE, ms=3, label='ε')
    axs[3].axvline(1, color=REF, ls=':', lw=1); axs[3].set_xlabel('V scale (× measured noise)'); axs[3].set_ylabel('RMS error [deg]'); axs[3].legend(fontsize=8)
    fig.legend(*axs[0].get_legend_handles_labels(), loc='upper center', ncol=3, bbox_to_anchor=(.5, .955), fontsize=8)
    fig.suptitle("C4  LQE tuning: small V follows the noise, large V lags; ε is only inferred through β' = aε (bottom: error vs V)", fontsize=10, fontweight='bold')
    fig.tight_layout(rect=(0, 0, 1, .91)); fig.savefig(out/'c4_lqe_tuning.png', dpi=160); plt.close(fig)

    # C5 LQG
    Phd = expm(Ao*Tm); Rm = np.diag([sr**2, sb**2]); Pp = solve_discrete_are(Phd.T, Co.T, W*Tm, Rm)
    Lk = Pp@Co.T@np.linalg.inv(Co@Pp@Co.T+Rm); res['C5'] = {'observer_abs_z': sorted(np.abs(np.linalg.eigvals(Phd@(np.eye(5)-Lk@Co))).tolist())}
    rng = np.random.default_rng(8)
    F = expm(np.block([[A, np.hstack([B, E])], [np.zeros((4, 7))]])*dt); PhB, GdB = F[:3, :3], F[:3, 3:]
    Fo = expm(np.block([[Ao, np.vstack([B, np.zeros((2, 2))])], [np.zeros((2, 7))]])*dt); PhO, GoB = Fo[:5, :5], Fo[:5, 5:]
    X = np.zeros((len(tt), 3)); X[0] = [.3, np.deg2rad(15), 0]; z = 0.; xh = np.zeros(5); P = np.diag([sr**2, sb**2, np.deg2rad(20)**2, .05**2, .03**2])
    XH = np.zeros((len(tt), 5)); U = np.zeros((len(tt), 2)); init = False
    for k in range(len(tt)):
        if k % nm == 0:
            y = X[k, :2]+[sr*rng.standard_normal(), sb*rng.standard_normal()]
            if not init:
                xh = np.r_[y, 0, 0, 0]; init = True
            else:
                S = Co@P@Co.T+Rm; Kk = P@Co.T@np.linalg.inv(S); xh = xh+Kk@(y-Co@xh); P = (np.eye(5)-Kk@Co)@P
        XH[k] = xh; u = -K@np.r_[xh[:3], z]; u = np.clip(u, [-v0, -wmax], [dvmax, wmax]); U[k] = u; z -= xh[0]*dt
        if k < len(tt)-1:
            X[k+1] = PhB@X[k]+GdB@np.r_[u, wt[k]]; xh = PhO@xh+GoB@u; P = PhO@P@PhO.T+W*dt
    m10 = tt > 10; res['C5'].update(range_max_m=float(np.abs(X[m10, 0]).max()), bearing_max_deg=float(np.rad2deg(np.abs(X[m10, 1]).max())))
    fig, axs = plt.subplots(3, 1, figsize=(9.6, 6), sharex=True)
    for r, (ax, ylab, s) in enumerate(zip(axs[:2], ['δρ [m]', 'β [deg]'], [1, 180/np.pi])):
        ax.plot(tt, s*X[:, r], color=INK, lw=2.2, label='true'); ax.plot(tt, s*XH[:, r], color=BLUE, lw=1.2, label='Kalman estimate'); ax.set_ylabel(ylab)
    axs[2].plot(tt, U[:, 0], color=ORANGE, lw=1.2, label='δv_f [m/s]'); axs[2].plot(tt, U[:, 1], color=AQUA, lw=1.2, label='ω_f [rad/s]')
    axs[2].set_ylabel('commands'); axs[2].set_xlabel('t [s]'); axs[2].legend(loc='center left', bbox_to_anchor=(1, .5), fontsize=8)
    fig.legend(*axs[0].get_legend_handles_labels(), loc='upper center', ncol=2, bbox_to_anchor=(.5, .94), fontsize=8)
    fig.suptitle(f"C5  LQG (LQI + Kalman at T_m, linear model): after 10 s range within {res['C5']['range_max_m']:.2f} m, "
                 f"bearing within {res['C5']['bearing_max_deg']:.0f}°", fontsize=10, fontweight='bold')
    fig.tight_layout(rect=(0, 0, 1, .9)); fig.savefig(out/'c5_lqg.png', dpi=160); plt.close(fig)
    return res


def main(argv=None):
    from .config import load_config
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('--config', default='Config/phase_e.json'); p.add_argument('--out', required=True)
    a = p.parse_args(argv)
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    from .phase_e import style
    style(plt); Path(a.out).mkdir(parents=True, exist_ok=True)
    import json; print(json.dumps(course_plots(load_config(a.config), a.out, plt), indent=1))


if __name__ == '__main__':
    main()
