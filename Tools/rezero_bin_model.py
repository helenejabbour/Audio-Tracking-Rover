#!/usr/bin/env python3
"""Re-zero the per-tone bearing model on captures at a known bearing (boresight check).

    python Tools/rezero_bin_model.py --config Config/measured_config.json RUN [RUN ...]
    python Tools/rezero_bin_model.py --config Config/measured_config.json RUN ... --out Config/measured_config_rezeroed.json

The s1 bin_model reads -0.4 to -1.3 deg at beta = 0 on the A2 spot (28 Nov 09:58), but -7 deg at
15:08 and -14 to -25 deg at 15:42 after the horn refit, rewiring and volume change. A constant offset
b is removed exactly by shifting every intercept: c_k -> c_k + s_k*b gives e_hat -> e_hat - b, so the
per-tone weights and slopes (the fitted physics) stay untouched. Use captures taken where the Tier 1
runs start and stop, beacon at a taped beta, motors off, 20 s each; room bias is position-specific,
so a re-zero from another spot does not transfer.
"""
import argparse
import csv
import json
import sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from rover.sensing import per_tone_bearing  # noqa: E402


def bearing_errors(run, model):
    """Per valid pair: e_hat - (beta_true - phi_c) in degrees."""
    run = Path(run); m = len(model['tone_hz'])
    rows = list(csv.DictReader((run/'samples.csv').open(newline='')))
    bins = {int(r[0]): np.array(r[1:], float) for r in list(csv.reader((run/'bins.csv').open(newline='')))[1:]}
    out = []
    for r in rows:
        b = bins.get(int(r['seq']))
        if r['valid'] != '1' or b is None or len(b) != 2*m+2:
            continue
        if abs(float(r['dlt'])-model['delta_deg']) > .6:
            raise SystemExit(f'{run.name}: dither {r["dlt"]} differs from the model delta {model["delta_deg"]}')
        e = per_tone_bearing(b[:m], b[m:2*m], b[2*m], b[2*m+1], model)
        out.append(e-(float(r['beta_true'])-float(r['phi_c'])))
    return np.array(out)


def shifted(model, bias_deg):
    s = np.asarray(model['slope_per_deg'], float)
    return dict(model, intercept=(np.asarray(model['intercept'], float)+s*bias_deg).tolist())


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('runs', nargs='+'); p.add_argument('--config', required=True); p.add_argument('--out')
    a = p.parse_args(argv)
    cfg = json.loads(Path(a.config).read_text(encoding='utf-8')); model = cfg.get('bin_model')
    if not model:
        raise SystemExit(f'{a.config} has no bin_model')
    errs = []
    for run in a.runs:
        e = bearing_errors(run, model)
        if not len(e):
            raise SystemExit(f'{run}: no valid pairs with bins')
        errs.append(e)
        print(f'{Path(run).name}: {len(e)} pairs, bearing error median {np.median(e):+.1f} deg, sd {e.std(ddof=1):.1f}')
    # Pairs within a capture share the slow room bias, so the capture is the independent unit.
    per_run = np.array([np.median(e) for e in errs])
    bias = float(np.median(np.concatenate(errs)))
    se = float(per_run.std(ddof=1)/np.sqrt(len(per_run))) if len(per_run) > 1 else float('nan')
    print(f'offset b = {bias:+.1f} deg (capture-to-capture se {se:.1f} deg, {len(per_run)} captures)')
    if a.out:
        cfg['bin_model'] = shifted(model, bias)
        cfg['bin_model_rezero'] = {'offset_removed_deg': bias, 'se_deg': se,
                                   'captures': [Path(r).as_posix() for r in a.runs], 'source_config': Path(a.config).as_posix()}
        Path(a.out).write_text(json.dumps(cfg, indent=2)+'\n', encoding='utf-8')
        print(a.out)
    return bias


if __name__ == '__main__':
    main()
