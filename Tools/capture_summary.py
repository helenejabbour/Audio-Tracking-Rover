#!/usr/bin/env python3
"""Quick look at rover.collect captures: valid/clip fractions, raw pp, T_m and noise.

    python3 Tools/capture_summary.py                 # latest capture under Runs/
    python3 Tools/capture_summary.py RUN [RUN ...]   # given capture folders

Use it to set the beacon volume (E0 tone: median pp 1000-1500 counts, clip 0 %) and to read
the E0 silence noise (median N = Aref) and the measurement period T_m (median / p95 of the
pair-to-pair interval).
"""
import csv
import sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def summary(run):
    rows = list(csv.DictReader((Path(run)/'samples.csv').open(newline='')))
    if not rows:
        return f'{Path(run).name}: no pairs'
    col = lambda k: np.array([float(r[k]) for r in rows])
    pp = np.maximum(col('pp_m'), col('pp_p')); t = col('t_pi'); dt = np.diff(t)
    N = np.concatenate([col('Aref_m'), col('Aref_p')]); A = np.concatenate([col('A_m'), col('A_p')])
    tm = f'T_m median {np.median(dt):.3f} s  p95 {np.percentile(dt, 95):.3f} s' if len(dt) else 'T_m n/a'
    return '\n'.join([
        f"{Path(run).name}  {rows[0]['label']}",
        f"  pairs {len(rows)}  valid {100*col('valid').mean():.1f} %  clip {100*col('clip').mean():.1f} %  "
        f"late {100*col('late').mean():.1f} %",
        f"  pp median {np.median(pp):.0f}  p95 {np.percentile(pp, 95):.0f}  max {pp.max():.0f}  (ADC 0-4095; clip at <=50 / >=4045)",
        f"  {tm}",
        f"  N median {np.median(N):.2f}  A median {np.median(A):.2f} counts"])


def main():
    runs = sys.argv[1:] or [str(max(ROOT.glob('Runs/*/*_capture'), key=lambda p: p.name))]
    for r in runs:
        print(summary(r))


if __name__ == '__main__':
    main()
