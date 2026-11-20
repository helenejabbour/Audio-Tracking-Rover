"""Reproducible numerical validation, data audit, controller comparison and plots."""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import sys
import time
import numpy as np
from .config import ROOT,load_config,create_run,write_json
from .analysis import run_analysis
from .simulation import SCENARIOS,COLUMNS,run_simulation
from .sensing import legacy_row,estimate


def audit_existing(cfg):
    results=[]
    for path in sorted((ROOT/'Data').glob('*.csv')):
        with path.open(newline='') as f:rows=list(csv.DictReader(f))
        periods=np.diff([float(r['t_ms'])*.001 for r in rows])
        periods=periods[periods>0]
        good=sum(int(r['flags'])==1 for r in rows)
        usable=sum(estimate(legacy_row(r,i),cfg)[0] is not None for i,r in enumerate(rows))
        results.append({'file':str(path.relative_to(ROOT)),'rows':len(rows),'valid_flags':good,'usable_pairs':usable,
                        'period_median_s':float(np.median(periods)) if len(periods) else None,
                        'clipped':sum(bool(int(r['flags'])&2) for r in rows),
                        'pair_gap_median_s':float(np.median([float(r['dt_pair_ms'])*.001 for r in rows])) if rows else None})
    return results


def plots(out,cfg,examples,aggregates,example_seed):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'figure.dpi':140})
    for scenario in ['nominal','outage','noise_drops_reflections']:
        fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True,layout='constrained')
        for method,color in [('lqi','#2364aa'),('place','#dc7c27'),('bearing_only','#707070')]:
            rows=examples[(method,scenario)];a=np.array([r[:13] for r in rows],float)
            axes[0].plot(a[:,0],a[:,1],label=method,color=color)
            axes[1].plot(a[:,0],np.rad2deg(a[:,2]),color=color)
            axes[2].plot(a[:,0],a[:,7],color=color)
        axes[0].axhline(cfg['desired_range'],ls='--',color='black',lw=.8,label='desired separation')
        axes[0].axhline(cfg['clearance'],ls=':',color='#b33232',label='assumed clearance')
        axes[0].set_ylabel('True range (m)');axes[0].legend(ncol=2,fontsize=9)
        axes[1].set_ylabel('True bearing (deg)');axes[2].set_ylabel('Command speed (m/s)');axes[2].set_xlabel('Time (s)')
        for ax in axes:ax.grid(alpha=.2)
        fig.suptitle(f'{scenario.replace("_"," ").title()} | synthetic plant, seed {example_seed}')
        fig.savefig(out/f'{scenario}.png');plt.close(fig)
    rows=examples[('lqi','outage')];a=np.array([r[:13] for r in rows],float)
    fig,ax=plt.subplots(figsize=(8,5),layout='constrained')
    ax.plot(a[:,11],a[:,12],label='target',color='#8c4e9f');ax.plot(a[:,9],a[:,10],label='follower',color='#2364aa')
    ax.set_aspect('equal',adjustable='datalim');ax.set(xlabel='x (m)',ylabel='y (m)',title='Global paths with five-second beacon outage | synthetic')
    ax.legend();ax.grid(alpha=.2);fig.savefig(out/'trajectory_outage.png');plt.close(fig)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config');p.add_argument('--seeds',type=int,default=12)
    p.add_argument('--seed-start',type=int,default=0,help='first random seed; use a new range for held-out validation')
    p.add_argument('--skip-tests',action='store_true');a=p.parse_args()
    if a.seeds<1:p.error('seeds must be positive')
    cfg=load_config(a.config);out=create_run('validation',cfg)
    write_json(out/'STATUS.json',{'state':'in_progress','expected_simulations':3*len(SCENARIOS)*a.seeds})
    write_json(out/'experiment.json',{'seeds':list(range(a.seed_start,a.seed_start+a.seeds)),
                                     'scenarios':SCENARIOS,'example_trace_seed':a.seed_start})
    import scipy,matplotlib
    write_json(out/'environment.json',{'python':sys.version,'numpy':np.__version__,'scipy':scipy.__version__,'matplotlib':matplotlib.__version__})
    if not a.skip_tests:
        test=subprocess.run([sys.executable,'-m','unittest','discover','-s','Tests','-v'],cwd=ROOT,capture_output=True,text=True)
        (out/'tests.txt').write_text(test.stdout+test.stderr,encoding='utf-8')
        if test.returncode:raise SystemExit(f'Tests failed; see {out}/tests.txt')
    analysis=run_analysis(cfg);write_json(out/'analysis.json',analysis)
    audit=audit_existing(cfg);write_json(out/'existing_data_audit.json',audit)
    records=[];examples={};aggregates=[]
    for method in ['lqi','place','bearing_only']:
        c=dict(cfg,controller=method)
        for scenario in SCENARIOS:
            runs=[]
            for seed in range(a.seed_start,a.seed_start+a.seeds):
                metrics,rows=run_simulation(c,scenario,seed,record=seed==a.seed_start)
                record={'controller':method,'scenario':scenario,**metrics};runs.append(record);records.append(record)
                if seed==a.seed_start:
                    examples[(method,scenario)]=rows
                    with (out/f'trace_{method}_{scenario}.csv').open('w',newline='') as f:
                        w=csv.writer(f);w.writerow(COLUMNS);w.writerows(rows)
            agg={'controller':method,'scenario':scenario,'n':a.seeds,
                 'practical_tracking_passes':sum(r['practical_tracking_pass'] for r in runs),
                 'clearance_violations':sum(not r['safe_clearance'] for r in runs),
                 'O3_passes':sum(r['original_O3_pass'] for r in runs),
                 'mean_tail_range_rmse_m':float(np.mean([r['tail_range_rmse_m'] for r in runs])),
                 'worst_tail_bearing_rms_deg':float(max(r['tail_bearing_rms_deg'] for r in runs)),
                 'minimum_range_m':float(min(r['minimum_range_m'] for r in runs)),
                 'zero_motion_runs':sum(r['max_command_speed']==0 for r in runs),
                 'stationary_standoff_passes':sum(abs(r['steady_range_error_m'])<=.15 and r['safe_clearance'] for r in runs) if scenario=='stationary_target' else None}
            aggregates.append(agg)
            print(f'{method:12} {scenario:26} tracking={agg["practical_tracking_passes"]}/{a.seeds} clearance_violations={agg["clearance_violations"]}',flush=True)
    write_json(out/'individual_metrics.json',records);write_json(out/'aggregate_metrics.json',aggregates)
    with (out/'aggregate_metrics.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(aggregates[0]));w.writeheader();w.writerows(aggregates)
    plots(out,cfg,examples,aggregates,a.seed_start)
    lines=['# Validation evidence', '', f'Generated in `{out.name}`. All closed-loop results below are synthetic.', '',
           f'The source hashes, exact configuration, environment, per-seed outcomes and seed-{a.seed_start} time histories are saved beside this file. Seeds: {a.seed_start} through {a.seed_start+a.seeds-1}.', '',
           'Practical tracking means final 10 s mean range error <= 0.15 m, bearing RMS <= 10 deg, no clearance violation and >=70% TRACK time. It is a declared engineering criterion, not the original O3 target.', '',
           '| Controller | Scenario | Tracking passes | Clearance violations | Mean tail range RMSE (m) | Worst bearing RMS (deg) |',
           '|---|---|---:|---:|---:|---:|']
    for r in aggregates:
        lines.append(f'| {r["controller"]} | {r["scenario"]} | {r["practical_tracking_passes"]}/{r["n"]} | {r["clearance_violations"]} | {r["mean_tail_range_rmse_m"]:.3f} | {r["worst_tail_bearing_rms_deg"]:.2f} |')
    lines += ['', 'Stationary-target trials use the same explicitly selected approach mission for all three controller labels; they validate the shared terminal supervisor, not a difference between LQI and pole placement. They use a separate terminal-standoff success test. No-beacon and very slow sensing trials should stop. Unreachable targets and gain bias are intentional limitation tests, retained in the data.', '',
              '## Existing measurements', '', '| File | Rows | Valid | Usable | Median period (s) |', '|---|---:|---:|---:|---:|']
    for r in audit:lines.append(f'| {r["file"]} | {r["rows"]} | {r["valid_flags"]} | {r["usable_pairs"]} | {r["period_median_s"]} |')
    lines += ['', '## Limits of this validation', '',
              '- Synthetic sensing uses the fitted-model family. Real horn, room reflections, speaker directivity, motor noise and timing need held-out experiments.',
              '- A minimum simulated separation is not a hardware collision guarantee. The runtime speed bound depends on calibrated error and braking bounds.',
              '- Original O3 2% settling and 15% overshoot results remain in every run, including failures and non-settling outcomes.',
              '- The sampled-loop analysis includes lag, delay and measurement rate in a local periodic reference. It is not a global nonlinear stability proof or a classical phase-margin certificate.',
              '- MATLAB execution and ESP32 compilation depend on the installed toolchain; see Docs/VALIDATION_STATUS.md for actual execution status.', '']
    (out/'REPORT.md').write_text('\n'.join(lines),encoding='utf-8')
    write_json(out/'STATUS.json',{'state':'complete','simulations':len(records),'seeds_per_scenario':a.seeds})
    print(out,flush=True)


if __name__=='__main__':main()
