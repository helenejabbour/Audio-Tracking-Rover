"""Fit horn/range parameters on training sessions and gate on separate held-out sessions."""
import argparse
import csv
import hashlib
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from .config import create_run,load_config,write_json
from .sensing import legacy_row,estimate


def read_rows(paths):
    rows=[]
    for p in paths:
        with Path(p).open(newline='') as source:
            for row in csv.DictReader(source):
                row=dict(row);row['_file']=str(Path(p).resolve());rows.append(row)
    return rows


def fit_model(rows,cfg):
    data=[]
    for row in rows:
        if int(row['flags'])!=1:continue
        phi=float(row['phi_c']);d=float(row['dlt']);beta=float(row['beta_true']);rho=float(row['rho_true'])
        for angle,key in [(phi-d,'A_m'),(phi+d,'A_p')]:
            amp=float(row[key])
            if rho>0 and amp>=cfg['amplitude_min']:data.append((rho,np.deg2rad(angle-beta),amp))
    a=np.array(data,float)
    if len(a)<40:raise ValueError('at least 20 valid pairs are required')
    r,theta,amp=a.T
    if len(np.unique(r))<3 or len(np.unique(np.round(theta,4)))<7:
        raise ValueError('need at least three distances and seven angular offsets; this is not a one-point fit')
    X=np.column_stack([np.ones(len(r)),np.log(r),theta,theta**2])
    if np.linalg.matrix_rank(X)<4:raise ValueError('calibration design does not identify all parameters')
    def residual(p):
        logk,n,logs,b=p
        return logk-n*np.log(r)-(theta+b)**2/(2*np.exp(2*logs))-np.log(amp)
    result=least_squares(residual,[np.log(cfg['amplitude_constant']),1,np.log(np.deg2rad(cfg['sigma_deg'])),0],
                         bounds=([np.log(.001),.3,np.log(np.deg2rad(5)),-np.deg2rad(30)],
                                 [np.log(1e8),2.5,np.log(np.deg2rad(180)),np.deg2rad(30)]),loss='soft_l1',f_scale=.05)
    if not result.success:raise ValueError(result.message)
    logk,n,logs,b=result.x
    proposed=dict(cfg)
    proposed.update(amplitude_constant=float(np.exp(logk)),range_exponent=float(n),sigma_deg=float(np.rad2deg(np.exp(logs))),
                    boresight_deg=float(np.rad2deg(b)),hardware_ready=False,
                    provenance='Fit from measured training sessions; review held-out results and commission actuators before motion')
    # Do not extrapolate beyond the training geometry.
    proposed['range_domain']=[float(r.min())*.999,float(r.max())*1.001]
    center_offsets=np.array([np.deg2rad(float(x['phi_c'])-float(x['beta_true']))+b for x in rows if int(x['flags'])==1])
    symmetric=min(abs(center_offsets.min()),abs(center_offsets.max()))
    proposed['bearing_domain_deg']=float(np.rad2deg(symmetric)+.1)
    residuals=residual(result.x)
    info={'training_captures':len(rows),'valid_amplitudes':len(r),'log_amplitude_rmse':float(np.sqrt(np.mean(residuals**2))),
          'design_condition_number':float(np.linalg.cond(X)),'fit_cost':float(result.cost),
          'fit_at_parameter_bound':bool(np.any(result.active_mask))}
    return proposed,info


def evaluate(rows,cfg):
    errors=[];rejected={};periods=[];previous={}
    for i,row in enumerate(rows):
        pair=legacy_row(row,i*cfg['measurement_period'])
        m,why=estimate(pair,cfg)
        file=row['_file'];clock=float(row['t_ms'])*.001
        if file in previous and clock>previous[file]:periods.append(clock-previous[file])
        previous[file]=clock
        if m is None:rejected[why]=rejected.get(why,0)+1;continue
        truth_r=float(row['rho_true']);truth_b=np.deg2rad(float(row['beta_true']))
        errors.append([(m.y[0]-truth_r)/truth_r,np.rad2deg(m.y[1]-truth_b)])
    if not errors:return {'passed':False,'valid_fraction':0.,'rejections':rejected,'reason':'no accepted holdout measurements'}
    e=np.array(errors);rrms,brms=np.sqrt(np.mean(e*e,axis=0));rb,bb=np.mean(e,axis=0)
    valid=len(errors)/max(1,len(rows));period=float(np.quantile(periods,.95)) if periods else None
    adequate_grid=len(set(r['rho_true'] for r in rows))>=3 and len(set(r['beta_true'] for r in rows))>=5
    passed=valid>=.95 and rrms<=.1 and brms<=3 and abs(rb)<=.05 and abs(bb)<=2 and period is not None and period<=.5 and adequate_grid
    return {'passed':bool(passed),'valid_fraction':valid,'relative_range_rmse':float(rrms),'bearing_rmse_deg':float(brms),
            'relative_range_bias':float(rb),'bearing_bias_deg':float(bb),'period_p95_s':period,
            'holdout_grid_adequate':adequate_grid,'rejections':rejected,'accepted':len(errors)}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--train',nargs='+',required=True);p.add_argument('--validation',nargs='+',required=True);p.add_argument('--config')
    a=p.parse_args();cfg=load_config(a.config)
    hashes=lambda paths:{hashlib.sha256(Path(x).read_bytes()).hexdigest() for x in paths}
    if hashes(a.train)&hashes(a.validation):p.error('training and validation must use distinct captured data')
    proposed,fit=fit_model(read_rows(a.train),cfg);held=evaluate(read_rows(a.validation),proposed)
    held['passed']=held['passed'] and not fit['fit_at_parameter_bound'] and fit['log_amplitude_rmse']<=.15
    out=create_run('calibration',cfg);write_json(out/'proposed_config.json',proposed)
    write_json(out/'calibration_report.json',{'training_files':a.train,'validation_files':a.validation,
                                            'training_hashes':sorted(hashes(a.train)),'validation_hashes':sorted(hashes(a.validation)),
                                            'fit':fit,'holdout':held})
    print(held);print(out)


if __name__=='__main__':main()
