"""Compare a hardware command/estimate log with independently measured ground truth."""
import argparse
import csv
from pathlib import Path
import numpy as np
from .config import load_config,create_run,write_json
from .model import wrap
from .simulation import summarize


def evaluate(commands,truth,cfg,offset):
    times=np.array([float(r['elapsed_s']) for r in commands])
    gt_time=np.array([float(r['time_s'])+offset for r in truth])
    if len(gt_time)<3 or np.any(np.diff(gt_time)<=0):raise ValueError('ground truth needs at least three increasing timestamps')
    if np.any(np.diff(times)<=0):raise ValueError('command timestamps must strictly increase')
    keep=(times>=gt_time[0])&(times<=gt_time[-1]);times=times[keep]
    selected=[r for r,k in zip(commands,keep) if k]
    if len(times)<3 or times[-1]-times[0]<10:raise ValueError('at least ten seconds of overlapping ground truth is required')
    def interp(key,angle=False):
        values=np.array([float(r[key]) for r in truth])
        if angle:values=np.unwrap(values)
        return np.interp(times,gt_time,values)
    fx,fy,fh=(interp(k,k.endswith('rad')) for k in ['follower_x_m','follower_y_m','follower_heading_rad'])
    tx,ty=(interp(k) for k in ['target_x_m','target_y_m'])
    rho=np.hypot(tx-fx,ty-fy);beta=wrap(np.arctan2(ty-fy,tx-fx)-fh)
    def number(row,key):
        value=row.get(key)
        return np.nan if value is None or value=='' else float(value)
    rh=np.array([number(r,'estimated_range_m') for r in selected]);bh=np.array([number(r,'estimated_bearing_rad') for r in selected])
    rows=[[t,r,b,rh_,bh_,0.,0.,float(cmd['speed_mps']),float(cmd['steering_rad']),x,y,xt,yt,cmd['mode'],cmd['reason'],0.]
          for t,r,b,rh_,bh_,cmd,x,y,xt,yt in zip(times,rho,beta,rh,bh,selected,fx,fy,tx,ty)]
    summary=summarize(rows,cfg)
    finite=np.isfinite(rh)&np.isfinite(bh)
    summary['range_estimation_rmse_m']=float(np.sqrt(np.mean((rh[finite]-rho[finite])**2))) if finite.any() else None
    summary['bearing_estimation_rmse_deg']=float(np.rad2deg(np.sqrt(np.mean(wrap(bh[finite]-beta[finite])**2)))) if finite.any() else None
    summary['overlap_s']=float(times[-1]-times[0]);summary['ground_truth_offset_s']=offset
    summary['initial_transient_covered']=bool(keep[0])
    if not keep[0]:summary['settling_2pct_s']=None;summary['original_O3_pass']=False
    return summary,np.column_stack([times,rho,rh,beta,bh])


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',required=True);p.add_argument('--commands',required=True);p.add_argument('--ground-truth',required=True)
    p.add_argument('--ground-truth-offset',type=float,required=True,help='command elapsed time = ground-truth time + this offset')
    p.add_argument('--position-uncertainty',type=float,required=True,help='measured ground-truth position uncertainty in m')
    a=p.parse_args();cfg=load_config(a.config)
    with Path(a.commands).open(newline='') as f:commands=list(csv.DictReader(f))
    with Path(a.ground_truth).open(newline='') as f:truth=list(csv.DictReader(f))
    if a.position_uncertainty<=0:p.error('provide a positive measured ground-truth uncertainty')
    summary,plotdata=evaluate(commands,truth,cfg,a.ground_truth_offset)
    summary.update(position_uncertainty_m=a.position_uncertainty,commands_file=a.commands,ground_truth_file=a.ground_truth)
    out=create_run('hardware_evaluation',cfg);write_json(out/'metrics.json',summary)
    np.savetxt(out/'aligned.csv',plotdata,delimiter=',',header='time_s,true_range_m,estimated_range_m,true_bearing_rad,estimated_bearing_rad',comments='')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,1,figsize=(9,6),sharex=True,layout='constrained')
    axes[0].plot(plotdata[:,0],plotdata[:,1],label='independent reference');axes[0].plot(plotdata[:,0],plotdata[:,2],label='acoustic estimate')
    axes[0].axhline(cfg['desired_range'],ls='--',color='black',label='desired');axes[0].set_ylabel('Range (m)');axes[0].legend()
    axes[1].plot(plotdata[:,0],np.rad2deg(plotdata[:,3]));axes[1].plot(plotdata[:,0],np.rad2deg(plotdata[:,4]))
    axes[1].set(xlabel='Time (s)',ylabel='Bearing (deg)')
    fig.savefig(out/'hardware_comparison.png',dpi=150);plt.close(fig)
    print(summary);print(out)


if __name__=='__main__':main()
