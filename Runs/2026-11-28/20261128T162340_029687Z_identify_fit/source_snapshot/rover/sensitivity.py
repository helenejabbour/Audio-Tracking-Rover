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
    cases=[('horn_sigma_deg',v,dict(sigma_deg=v),{}) for v in [25.,40.,80.,120.,160.]]
    cases += [('pair_gap_s',v,dict(pair_gap=v,measurement_period=max(.4,2*v)),{}) for v in [.1,.18,.3,.5]]
    cases += [('amplitude_noise_counts',v,dict(amplitude_noise=v),{}) for v in [.2,.8,2.,5.]]
    # Unmodelled constant bearing bias: true horn boresight differs from the calibrated one.
    cases += [('bearing_bias_deg',v,{},dict(boresight_error_deg=v)) for v in [0.,5.,10.,20.,-10.]]
    # Dither trade-off with the measured E4 timing law (gap = 72 ms + 2*delta/300 deg/s,
    # period = gap + 72 ms) and the +-(80-delta) bearing window.
    left=base.get('horn_limit_left_deg',base['horn_limit_deg'])
    cases += [('dither_deg_E4_timing',v,dict(dither_deg=v,pair_gap=.072+v/150,measurement_period=.144+v/150,
                                             bearing_domain_deg=base['horn_limit_deg']-v),{})
              for v in [20.,30.,40.,50.,60.] if v<left]
    cases += [('bearing_noise_floor_deg',v,dict(bearing_noise_floor_deg=v),{}) for v in [5.,10.,15.,20.]]
    # Supervisor uncertainty gates (design choices) scaled together against the measured noise floors.
    cases += [('uncertainty_gate_scale',k,dict(max_range_std=base['max_range_std']*k,
                                               max_bearing_std_deg=base['max_bearing_std_deg']*k),{}) for k in [1.,2.,3.]]
    # Position-dependent multipath (Phase B s1): fast fringe (per-side ln-amplitude sd; 0.144 ~ 12 deg
    # of bearing scatter at sigma 77/delta 50), its spatial period, and the slow room bearing bias.
    cases += [('fringe_ln_sd',v,{},dict(fringe_ln_sd=v)) for v in [0.,.07,.144,.25]]
    cases += [('fringe_length_m_at_sd_0.144',v,{},dict(fringe_ln_sd=.144,fringe_length_m=v)) for v in [.02,.043,.1,.3]]
    cases += [('room_bearing_sd_deg',v,{},dict(room_bearing_sd_deg=v)) for v in [0.,5.,10.,23.]]
    for name,value,changes,spec in cases:
        cfg=dict(base,**changes);results=[]
        for seed in range(a.seed_start,a.seed_start+a.seeds):
            m,_=run_simulation(cfg,spec or 'nominal',seed,False);results.append(m)
            individual.append({'parameter':name,'value':value,'config_overrides':changes,'scenario_overrides':spec,**m})
        row={'parameter':name,'value':value,'passes':sum(m['practical_tracking_pass'] for m in results),'n':a.seeds,
             'mean_tail_range_rmse_m':float(np.mean([m['tail_range_rmse_m'] for m in results])),
             'worst_tail_bearing_rms_deg':float(max(m['tail_bearing_rms_deg'] for m in results)),
             'clearance_violations':sum(not m['safe_clearance'] for m in results),
             'mean_tracking_fraction':float(np.mean([m['tracking_fraction'] for m in results]))}
        rows.append(row);print(row,flush=True)
    write_json(out/'individual_metrics.json',individual);write_json(out/'summary.json',rows)
    with (out/'summary.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    text=['# Assumption sensitivity', '',f'Synthetic trials, seeds {a.seed_start}–{a.seed_start+a.seeds-1}. These values are not a fit to the existing POC.', '',
          'The horn-width sweep changes the correctly calibrated model in both sensor and estimator. It tests noise and within-pair motion sensitivity, not unmodeled beam-shape mismatch. Amplitude constant and other parameters remain the same.', '',
          'bearing_bias_deg offsets the true horn boresight from the calibrated one (constant bias). dither_deg_E4_timing couples dither to the measured pair timing.', '',
          'fringe_* and room_bearing_sd_deg use the position-dependent multipath model (rover.simulation.MultipathModel): a fast fringe field per horn side and a slow antisymmetric room-bias field over the follower and target positions. Phase B s1 estimates: fringe_ln_sd 0.144, room 23 deg.', '',
          '| Parameter | Value | Tracking passes | Mean range RMSE (m) | Worst bearing RMS (deg) | Mean TRACK fraction | Clearance violations |', '|---|---:|---:|---:|---:|---:|---:|']
    for r in rows:text.append(f'| {r["parameter"]} | {r["value"]} | {r["passes"]}/{r["n"]} | {r["mean_tail_range_rmse_m"]:.3f} | {r["worst_tail_bearing_rms_deg"]:.2f} | {r["mean_tracking_fraction"]:.2f} | {r["clearance_violations"]} |')
    text += ['', 'Broadband level-vs-angle POC values cannot determine the 4 kHz Gaussian sigma. Test the actual local slope, symmetry, repeatability and sequential-pair motion error in held-out recordings.', '']
    (out/'REPORT.md').write_text('\n'.join(text),encoding='utf-8')
    print(out)


if __name__=='__main__':main()
