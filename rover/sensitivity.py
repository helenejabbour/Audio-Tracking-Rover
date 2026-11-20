"""Assumption sensitivity on new random seeds; does not fit real hardware data."""
import argparse
import csv
import numpy as np
from .config import create_run,load_config,write_json
from .simulation import run_simulation


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config');p.add_argument('--seeds',type=int,default=6)
    p.add_argument('--seed-start',type=int,default=2000);a=p.parse_args()
    if a.seeds<1:p.error('seeds must be positive')
    base=load_config(a.config);out=create_run('sensitivity',base);rows=[];individual=[]
    cases=[('horn_sigma_deg',v,dict(sigma_deg=v)) for v in [25.,40.,80.,120.,160.]]
    cases += [('pair_gap_s',v,dict(pair_gap=v,measurement_period=max(.4,2*v))) for v in [.1,.18,.3,.5]]
    cases += [('amplitude_noise_counts',v,dict(amplitude_noise=v)) for v in [.2,.8,2.,5.]]
    for name,value,changes in cases:
        cfg=dict(base,**changes);results=[]
        for seed in range(a.seed_start,a.seed_start+a.seeds):
            m,_=run_simulation(cfg,'nominal',seed,False);results.append(m)
            individual.append({'parameter':name,'value':value,'config_overrides':changes,**m})
        row={'parameter':name,'value':value,'passes':sum(m['practical_tracking_pass'] for m in results),'n':a.seeds,
             'mean_tail_range_rmse_m':float(np.mean([m['tail_range_rmse_m'] for m in results])),
             'worst_tail_bearing_rms_deg':float(max(m['tail_bearing_rms_deg'] for m in results)),
             'clearance_violations':sum(not m['safe_clearance'] for m in results)}
        rows.append(row);print(row,flush=True)
    write_json(out/'individual_metrics.json',individual);write_json(out/'summary.json',rows)
    with (out/'summary.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    text=['# Assumption sensitivity', '',f'Synthetic trials, seeds {a.seed_start}–{a.seed_start+a.seeds-1}. These values are not a fit to the existing POC.', '',
          'The horn-width sweep changes the correctly calibrated model in both sensor and estimator. It tests noise and within-pair motion sensitivity, not unmodeled beam-shape mismatch. Amplitude constant and other parameters remain the same.', '',
          '| Parameter | Value | Tracking passes | Mean range RMSE (m) | Worst bearing RMS (deg) |', '|---|---:|---:|---:|---:|']
    for r in rows:text.append(f'| {r["parameter"]} | {r["value"]} | {r["passes"]}/{r["n"]} | {r["mean_tail_range_rmse_m"]:.3f} | {r["worst_tail_bearing_rms_deg"]:.2f} |')
    text += ['', 'Broadband level-vs-angle POC values cannot determine the 4 kHz Gaussian sigma. Test the actual local slope, symmetry, repeatability and sequential-pair motion error in held-out recordings.', '']
    (out/'REPORT.md').write_text('\n'.join(text),encoding='utf-8')
    print(out)


if __name__=='__main__':main()
