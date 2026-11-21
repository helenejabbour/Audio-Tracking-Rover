"""In-room bearing calibration in the power domain (Phase B, 20 Nov 2026).

Per pose (one capture file, or several files pooled with --pool), the valid pairs are
combined as P = mean(A^2) for each horn side and

    e_meas = sigma^2/(2 delta) * 1/2 ln(P+/P-)      (deg; = phi_c-relative bearing)

is regressed on the true offset e = beta - phi_c:  e_meas = kappa*e + c.

- kappa < 1 is slope compression by reverberant power that does not come from the source;
  c is the in-room offset (it moves the steering null). They become
  `bearing_gain_correction = kappa` and `boresight_deg = -c/kappa` in the proposed config.
- The residual sd is the per-pose fringe/room error left after the correction.
- --pool power-averages every file with the same (beta, phi_c, delta): use it for a set of
  small position shifts (B2 fringe test), i.e. spatial diversity.
- With multi-tone captures (bins.csv next to samples.csv) the amplitudes can be recombined:
  --equalize weights each tone by 1/mean power (speaker/mic response), --tones selects a
  subset of tone frequencies (e.g. --tones 4000 reproduces the old single-tone sensor).

- --per-tone (multi-tone captures) fits each tone's own line L_k = s_k*e + c_k and combines
  the tones with inverse-variance weights w_k = s_k^2/(r_k^2 + v_k) (BLUE), where r_k is the
  per-pose (fringe/room) residual and v_k the per-pair noise. It writes `bin_model` into the
  proposed config. --min-kappa drops tones whose slope is below that fraction of the
  sigma/delta Gaussian (tones where the horn is flat or inverted).
- --validate CAPTURES evaluates the fitted correction on held-out captures (per pose, and
  per pair for the per-tone model): bias, RMS and max error. Nothing held out is used to fit.

Inputs are capture run folders (rover.collect) or legacy CSVs with rho_true/beta_true columns.
"""
import argparse
import csv
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from .config import create_run, load_config, write_json
from .sensing import quality_ok, combine_bins, tone_log_ratios, per_tone_bearing
from .calibrate import read_rows, read_corrections


def _frequency(name):
    try:
        return float(name[3:])
    except ValueError:
        return np.nan


def load_capture(path, fixes=None):
    """Rows of one capture (dicts), with '_bins' (minus, plus, noise_m, noise_p, freqs) if present."""
    path = Path(path)
    samples = path/'samples.csv' if path.is_dir() else path
    rows = read_rows([samples], fixes or {})
    bins_path = samples.parent/'bins.csv' if path.is_dir() else None
    if bins_path is not None and bins_path.exists():
        with bins_path.open(newline='') as f:
            reader = csv.reader(f); header = next(reader)
            m = (len(header)-3)//2
            freqs = [_frequency(h) for h in header[1:1+m]]
            table = {int(r[0]): np.array(r[1:], float) for r in reader}
        for row in rows:
            v = table.get(int(row['seq']))
            if v is not None:
                row['_bins'] = (v[:m], v[m:2*m], v[-2], v[-1], np.array(freqs))
    for row in rows:
        row['_source'] = str(path)
    return rows


def pair_amplitudes(row, tones=None, equalize=None):
    """(A-, A+) of one pair: firmware values, or recombined from its bins."""
    if tones is None and equalize is None:
        return float(row['A_m']), float(row['A_p'])
    if '_bins' not in row:
        raise ValueError(f"{row['_source']}: --tones/--equalize need bins.csv (multi-tone capture)")
    bm, bp, nm, np_, freqs = row['_bins']
    w = np.ones(len(bm)) if equalize is None else np.asarray(equalize, float)
    if tones is not None:
        keep = np.isin(np.round(freqs), np.round(tones))
        if not keep.any():
            raise ValueError(f'none of the tones {tones} is in this capture ({freqs.tolist()})')
        w = w*keep
    return combine_bins(bm, nm, w), combine_bins(bp, np_, w)


def equalization_weights(rows):
    """1/mean power per tone over all valid pairs and both sides, normalised to mean 1."""
    power = [np.concatenate([r['_bins'][0], r['_bins'][1]]).reshape(2, -1)**2
             for r in rows if '_bins' in r and quality_ok(r['flags'])]
    if not power:
        raise ValueError('--equalize needs multi-tone captures with valid pairs')
    mean = np.mean(np.concatenate(power), axis=0)
    w = 1/np.maximum(mean, 1e-9)
    return w/w.mean()


def pose_points(captures, cfg, pool=False, tones=None, equalize=None):
    """One power-domain point per pose: dict(e_true, e_meas, pairs, files)."""
    groups = {}
    for rows in captures:
        valid = [r for r in rows if quality_ok(r['flags'])]
        if tones is not None or equalize is not None:
            valid = [r for r in valid if '_bins' in r]   # a dropped B2 line leaves a pair without bins
        if not valid:
            continue
        r0 = valid[0]
        key = (float(r0['beta_true']), float(r0['phi_c']), float(r0['dlt']))
        if not pool:
            key = key+(r0['_source'],)
        g = groups.setdefault(key, {'pm': [], 'pp': [], 'files': set(), 'rho': set()})
        for r in valid:
            am, ap = pair_amplitudes(r, tones, equalize)
            g['pm'].append(am*am); g['pp'].append(ap*ap)
        g['files'].add(r0['_source']); g['rho'].add(float(r0['rho_true']))
    points = []
    for key, g in groups.items():
        beta, phi, delta = key[:3]
        pm, pp = np.mean(g['pm']), np.mean(g['pp'])
        if pm <= 0 or pp <= 0:
            continue
        h = cfg['sigma_deg']**2/(2*delta)  # deg per unit ln(A+/A-), in degrees
        points.append({'beta_true': beta, 'phi_c': phi, 'delta': delta, 'e_true': beta-phi,
                       'e_meas': float(h*0.5*np.log(pp/pm)), 'pairs': len(g['pm']),
                       'files': sorted(g['files']), 'rho': sorted(g['rho'])})
    return points


def fit_correction(points, fix_slope=None):
    """Robust fit e_meas = kappa*e_true + c. Returns kappa, c and their standard errors."""
    e = np.array([p['e_true'] for p in points]); y = np.array([p['e_meas'] for p in points])
    if not points:
        raise ValueError('no pose has valid pairs')
    if fix_slope is None and (len(points) < 3 or np.ptp(e) < 10):
        fix_slope = 1.0  # offset only: too few poses or too little angle to identify kappa
    if fix_slope is not None:
        res = least_squares(lambda c: y-fix_slope*e-c[0], [0.], loss='soft_l1', f_scale=10.)
        kappa, c = fix_slope, float(res.x[0]); dof = len(y)-1
        r = y-kappa*e-c; s = float(np.sqrt((r @ r)/dof)) if dof > 0 else None
        return {'kappa': float(kappa), 'offset_deg': c, 'kappa_se': None,
                'offset_se_deg': None if s is None else s/np.sqrt(len(y)),
                'residual_sd_deg': s, 'slope_fixed': True, 'poses': len(points)}
    res = least_squares(lambda p: y-p[0]*e-p[1], [1., 0.], loss='soft_l1', f_scale=10.)
    kappa, c = res.x; r = y-kappa*e-c; dof = len(y)-2
    s = np.sqrt((r @ r)/max(dof, 1))
    X = np.column_stack([e, np.ones_like(e)]); cov = s*s*np.linalg.pinv(X.T @ X)
    return {'kappa': float(kappa), 'offset_deg': float(c), 'kappa_se': float(np.sqrt(cov[0, 0])),
            'offset_se_deg': float(np.sqrt(cov[1, 1])), 'residual_sd_deg': float(s),
            'slope_fixed': False, 'poses': len(points)}


def tone_poses(captures, pool=False):
    """Per pose: e_true, delta, per-tone L_k from power means, per-pair L_k (multi-tone only)."""
    groups = {}
    for rows in captures:
        valid = [r for r in rows if quality_ok(r['flags']) and '_bins' in r]
        if not valid:
            continue
        r0 = valid[0]
        key = (float(r0['beta_true']), float(r0['phi_c']), float(r0['dlt']))
        if not pool:
            key = key+(r0['_source'],)
        g = groups.setdefault(key, {'rows': [], 'files': set(), 'rho': set()})
        g['rows'] += valid; g['files'].add(r0['_source']); g['rho'].add(float(r0['rho_true']))
    points = []
    for key, g in groups.items():
        beta, phi, delta = key[:3]
        bins = [r['_bins'][:4] for r in g['rows']]
        bm = np.array([b[0] for b in bins]); bp = np.array([b[1] for b in bins])
        nm = np.array([b[2] for b in bins]); np_ = np.array([b[3] for b in bins])
        pose_L = tone_log_ratios(np.sqrt((bm**2).mean(0)), np.sqrt((bp**2).mean(0)),
                                 np.sqrt((nm**2).mean()), np.sqrt((np_**2).mean()))
        pair_L = np.array([tone_log_ratios(*b) for b in bins])
        points.append({'beta_true': beta, 'phi_c': phi, 'delta': delta, 'e_true': beta-phi,
                       'tone_hz': g['rows'][0]['_bins'][4].tolist(), 'L': pose_L, 'pair_L': pair_L,
                       'pair_bins': bins, 'pairs': len(bins), 'files': sorted(g['files']),
                       'rho': sorted(g['rho'])})
    return points


def fit_tone_model(points, cfg, min_kappa=None, tones=None):
    """Per-tone lines and BLUE weights. Returns (bin_model, per-tone table)."""
    if len(points) < 3 or np.ptp([p['e_true'] for p in points]) < 10:
        raise ValueError('--per-tone needs at least 3 poses spanning at least 10 deg')
    deltas = {p['delta'] for p in points}
    if len(deltas) != 1:
        raise ValueError(f'--per-tone needs a single dither, got {sorted(deltas)}')
    delta = deltas.pop(); freqs = np.array(points[0]['tone_hz'])
    if any(len(p['tone_hz']) != len(freqs) or not np.allclose(p['tone_hz'], freqs) for p in points):
        raise ValueError('captures use different tone sets')
    e = np.array([p['e_true'] for p in points]); Lk = np.array([p['L'] for p in points])
    expected = 2*delta/cfg['sigma_deg']**2  # |d ln(A+/A-)/de| of the Gaussian model, deg^-1
    table = []
    for k, f in enumerate(freqs):
        res = least_squares(lambda q: Lk[:, k]-q[0]*e-q[1], [expected, 0.], loss='soft_l1', f_scale=.1)
        sk, ck = res.x; r = Lk[:, k]-sk*e-ck; rk2 = float(r @ r/max(len(e)-2, 1))
        vk = float(np.mean([np.var(p['pair_L'][:, k], ddof=1) for p in points if p['pairs'] > 1]))
        kap = float(sk/expected)   # e = beta - phi_c: a normal beam has L rising with e
        use = (tones is None or bool(np.isin(round(f), np.round(tones)))) and (min_kappa is None or kap >= min_kappa)
        w = float(sk*sk/(rk2+vk)) if use and sk != 0 else 0.
        table.append({'tone_hz': float(f), 'kappa': kap, 'slope_per_deg': float(sk), 'intercept': float(ck),
                      'pose_residual_deg': float(np.sqrt(rk2)/abs(sk)) if sk else None,
                      'pair_noise_deg': float(np.sqrt(vk)/abs(sk)) if sk else None, 'weight': w})
    wsum = sum(t['weight'] for t in table)
    if wsum <= 0:
        raise ValueError('no tone passes the selection')
    model = {'tone_hz': [t['tone_hz'] for t in table], 'slope_per_deg': [t['slope_per_deg'] for t in table],
             'intercept': [t['intercept'] for t in table], 'weight': [t['weight']/wsum for t in table],
             'delta_deg': float(delta), 'min_kappa': min_kappa, 'poses': len(points)}
    return model, table


def model_bearing_from_L(L, model):
    """BLUE bearing offset (deg) from per-tone log ratios L_k."""
    s, c, w = (np.asarray(model[k], float) for k in ('slope_per_deg', 'intercept', 'weight'))
    use = w > 0
    return float((w[use]*(np.asarray(L)[use]-c[use])/s[use]).sum()/w[use].sum())


def _stats(err):
    err = np.asarray(err, float)
    return {'n': int(len(err)), 'bias_deg': float(err.mean()), 'rms_deg': float(np.sqrt((err**2).mean())),
            'max_abs_deg': float(np.abs(err).max()), 'sd_deg': float(err.std())}


def validate_tone_model(points, model):
    """Errors per pose: from the pose's power-mean L_k, and the mean of per-pair estimates."""
    pose, pair_mean, pair_sd, rows = [], [], [], []
    for p in points:
        e_pairs = np.array([per_tone_bearing(*b, model) for b in p['pair_bins']])
        pose.append(model_bearing_from_L(p['L'], model)-p['e_true'])
        pair_mean.append(e_pairs.mean()-p['e_true']); pair_sd.append(e_pairs.std())
        rows.append({'files': p.get('files'), 'rho': p.get('rho'), 'e_true': p['e_true'], 'pose_error_deg': pose[-1],
                     'pair_mean_error_deg': pair_mean[-1], 'pair_sd_deg': pair_sd[-1], 'pairs': p['pairs']})
    return {'pose_power_mean': _stats(pose), 'pair_mean': _stats(pair_mean),
            'per_pair_sd_deg_median': float(np.median(pair_sd)), 'poses': rows}


def validate_correction(points, fit):
    err = [(p['e_meas']-fit['offset_deg'])/fit['kappa']-p['e_true'] for p in points]
    return {'pose_power_mean': _stats(err),
            'poses': [{'files': p['files'], 'rho': p['rho'], 'e_true': p['e_true'], 'pose_error_deg': x}
                      for p, x in zip(points, err)]}


def _round(d):
    return {k: round(v, 1) if isinstance(v, float) else v for k, v in d.items()}


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('captures', nargs='+', help='capture run folders or legacy CSVs')
    p.add_argument('--config'); p.add_argument('--corrections')
    p.add_argument('--pool', action='store_true', help='power-average files with the same (beta, phi_c, delta)')
    p.add_argument('--tones', type=float, nargs='+', help='use only these tone frequencies (Hz)')
    p.add_argument('--equalize', action='store_true', help='weight tones by 1/mean power')
    p.add_argument('--fix-slope', type=float, help='fit the offset only, with kappa fixed')
    p.add_argument('--per-tone', action='store_true', help='fit a per-tone BLUE model (bin_model)')
    p.add_argument('--min-kappa', type=float, help='--per-tone: drop tones with kappa_k below this')
    p.add_argument('--validate', nargs='+', default=[], help='held-out captures to evaluate on')
    a = p.parse_args(); cfg = load_config(a.config)
    if {str(Path(c).resolve()) for c in a.captures} & {str(Path(c).resolve()) for c in a.validate}:
        p.error('a capture is in both the training and the --validate set')
    fixes = read_corrections(a.corrections)
    captures = [load_capture(c, fixes) for c in a.captures]
    held = [load_capture(c, fixes) for c in a.validate]
    if a.per_tone:
        if a.equalize or a.fix_slope is not None:
            p.error('--per-tone has its own weights and slopes; drop --equalize/--fix-slope')
        points = tone_poses(captures, a.pool)
        model, table = fit_tone_model(points, cfg, a.min_kappa, a.tones)
        proposed = dict(cfg)
        proposed.update(bin_model=model, bearing_gain_correction=1., boresight_deg=0., hardware_ready=False)
        report = {'captures': a.captures, 'validate': a.validate, 'pool': a.pool, 'tones': a.tones,
                  'min_kappa': a.min_kappa, 'per_tone': table, 'bin_model': model,
                  'train': validate_tone_model(points, model)}
        if held:
            report['holdout'] = validate_tone_model(tone_poses(held), model)
        out = create_run('bearing_cal', cfg)
        write_json(out/'proposed_config.json', proposed); write_json(out/'bearing_cal_report.json', report)
        for t, w in zip(table, model['weight']):
            print(f"{t['tone_hz']:7.1f} Hz  kappa {t['kappa']:+5.2f}  pose res {t['pose_residual_deg'] or 0:6.1f}  "
                  f"pair noise {t['pair_noise_deg'] or 0:6.1f}  weight {w:.2f}")
        for name in ('train', 'holdout'):
            if name in report:
                r = report[name]
                print(f"{name:8s} pose {_round(r['pose_power_mean'])}  pair-mean {_round(r['pair_mean'])}  "
                      f"per-pair sd {r['per_pair_sd_deg_median']:.1f}")
        print(out)
        return
    weights = equalization_weights([r for rows in captures for r in rows]) if a.equalize else None
    points = pose_points(captures, cfg, a.pool, a.tones, weights)
    fit = fit_correction(points, a.fix_slope)
    kappa = float(np.clip(fit['kappa'], .2, 1.5))
    proposed = dict(cfg)
    proposed.update(bearing_gain_correction=kappa, boresight_deg=float(-fit['offset_deg']/kappa),
                    hardware_ready=False)
    if weights is not None:
        proposed['bin_weights'] = [float(w) for w in weights]
    out = create_run('bearing_cal', cfg)
    write_json(out/'proposed_config.json', proposed)
    write_json(out/'bearing_cal_report.json', {
        'captures': a.captures, 'pool': a.pool, 'tones': a.tones, 'equalize': a.equalize,
        'bin_weights': None if weights is None else [float(w) for w in weights],
        'fit': fit, 'kappa_clipped': kappa != fit['kappa'], 'points': points,
        'holdout': validate_correction(pose_points(held, cfg, False, a.tones, weights), fit) if held else None,
        'note': 'residual_sd_deg is a per-pose (static) error after the correction, not per-pair noise'})
    for q in points:
        print(f"beta={q['beta_true']:+6.1f} phi_c={q['phi_c']:+6.1f} e_meas={q['e_meas']:+7.1f} "
              f"pairs={q['pairs']:3d} files={len(q['files'])}")
    print({k: (round(v, 4) if isinstance(v, float) else v) for k, v in fit.items()})
    if held:
        print('holdout pose', _round(validate_correction(pose_points(held, cfg, False, a.tones, weights), fit)['pose_power_mean']))
    print(out)


if __name__ == '__main__':
    main()
