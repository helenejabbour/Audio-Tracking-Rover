#!/usr/bin/env python3
"""Phase B multi-tone session analysis (20 Nov 2026): train on B0, test on B2 and held-out E3.

  python Tools/phase_b_multitone.py [--runs Runs/2026-11-20]

Reads every rover.collect capture whose label starts with B0_<tag> / B2_<tag> / E3_<tag>
(samples.csv joined with bins.csv on seq). Per pose, the noise-subtracted per-tone power is
averaged over the valid pairs. Reported:

- per-tone B0 slope kappa_k (1 = the sigma=77 deg, delta=50 deg Gaussian model);
- bin-to-bin correlation of the B0 fringe residuals (does frequency diversity decorrelate?);
- bearing estimators, each with an in-room linear correction fitted on B0 only:
  B2 (beta=0, six 1.5 cm positions) swing and sd, E3 bias/RMS/max (held out);
- range: K from the B0 phi_c=0 runs (n = 1), relative error on E3 and the B2 sd.
"""
import argparse
import glob
import json
import os
import numpy as np
import pandas as pd

H = 77.0**2/100  # deg per unit ln(A+/A-) at sigma=77, delta=50
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load(d):
    s = json.load(open(os.path.join(d, 'session.json')))
    df = pd.read_csv(os.path.join(d, 'samples.csv'))
    if len(df) and os.path.exists(os.path.join(d, 'bins.csv')):
        df = df.merge(pd.read_csv(os.path.join(d, 'bins.csv')), on='seq', how='inner')
    df['run'] = os.path.basename(d)[:15]
    return s, df


def poses(root, prefix):
    out = []
    for d in sorted(glob.glob(os.path.join(root, '*_capture'))):
        s = json.load(open(os.path.join(d, 'session.json')))
        if not s['label'].startswith(prefix) or not os.path.exists(os.path.join(d, 'samples.csv')):
            continue
        s, df = load(d)
        v = df[df['valid'] == 1]
        if len(v) < 5 or not any(c.startswith('am_') for c in v.columns):
            continue
        f = np.array([float(c[3:]) for c in v.columns if c.startswith('am_')])
        am = v[[f'am_{x:g}' for x in f]].values; ap = v[[f'ap_{x:g}' for x in f]].values
        nm, np_ = v.N_m.values[:, None], v.N_p.values[:, None]
        Pm = np.maximum((am**2-nm**2).mean(0), 1e-6); Pp = np.maximum((ap**2-np_**2).mean(0), 1e-6)
        out.append(dict(label=s['label'], run=df.run.iloc[0], rho=s['rho'], e=s['beta']-s['phi'], phi=s['phi'],
                        nv=len(v), f=f, Pm=Pm, Pp=Pp, Lbin=.5*np.log(Pp/Pm),
                        Lfw=.5*np.log((v.A_p**2).mean()/(v.A_m**2).mean())))
    return out


def lin(x, y):
    s, c = np.polyfit(x, y, 1)
    return s, c, (y-(s*x+c)).std(ddof=2)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--runs', default=os.path.join(ROOT, 'Runs', '2026-11-20'))
    ap.add_argument('--tag', default='mt', help='label tag: B0_<tag>, B2_<tag>, E3_<tag> (s1: mt)')
    a = ap.parse_args()
    train, b2, e3 = (poses(a.runs, f'{x}_{a.tag}') for x in ('B0', 'B2', 'E3'))
    f = train[0]['f']; E = np.array([p['e'] for p in train])
    print(f'train B0 poses {len(train)}, B2 positions {len(b2)}, E3 poses {len(e3)}')

    fits = np.array([lin(E, np.array([p['Lbin'][k] for p in train])) for k in range(len(f))])
    expected = 2*50/77**2  # d ln(A+/A-)/de, deg^-1
    print('\nper-tone B0 slope kappa_k and fringe residual (deg):')
    for k, x in enumerate(f):
        print(f'  {x:6.0f} Hz  kappa {fits[k, 0]/expected:+5.2f}  residual {fits[k, 2]*H:5.1f}')
    R = np.array([[p['Lbin'][k]-np.polyval(fits[k, :2], p['e']) for k in range(len(f))] for p in train])
    C = np.corrcoef(R.T)
    print('bin-to-bin residual correlation:',
          {int(250*d): round(float(np.mean([C[k, k+d] for k in range(len(f)-d)])), 2) for d in range(1, 5)})

    def subset_power(sel):
        idx = np.isin(f, sel)
        return lambda p: H*.5*np.log(p['Pp'][idx].sum()/p['Pm'][idx].sum())

    def blue(mask):
        s, c, r = fits[mask].T; w = s*s/(r*r)
        return lambda p: float((w*(p['Lbin'][mask]-c)/s).sum()/w.sum())

    est = {'firmware equal power': lambda p: H*p['Lfw']}
    if np.isin(4000, f):  # wide16 (s1) comparisons
        est.update({'4000 Hz only': subset_power([4000]),
                    '3250-4250 equal power': subset_power([3250, 3500, 3750, 4000, 4250])})
    est.update({
           'BLUE per-tone, all tones': blue(np.ones(len(f), bool)),
           'BLUE per-tone, 3250-4500': blue((f >= 3250) & (f <= 4500))})
    rows = []
    for name, fn in est.items():
        s, c, _ = lin(E, np.array([fn(p) for p in train]))
        corr = lambda p: (fn(p)-c)/s
        tr = np.array([corr(p)-p['e'] for p in train])
        b = np.array([corr(p)-p['e'] for p in b2]); e = np.array([corr(p)-p['e'] for p in e3])
        rows.append(dict(estimator=name, kappa=s, train_sd=tr.std(),
                         B2_mean=b.mean(), B2_halfswing=np.ptp(b)/2, B2_sd=b.std(),
                         E3_bias=e.mean(), E3_rms=np.sqrt((e*e).mean()), E3_max=np.abs(e).max()))
        rows[-1]['E3_errors'] = np.round(e, 1).tolist()
    T = pd.DataFrame(rows)
    pd.set_option('display.width', 250)
    print('\nbearing (deg), correction fitted on B0 only:')
    print(T.drop(columns='E3_errors').round(2).to_string(index=False))
    print('E3 poses:', [(p['run'][9:15], p['rho'], p['e'], p['nv']) for p in e3])
    for r in rows:
        print(f"  {r['estimator']:30s} {r['E3_errors']}")

    print('\nrange (K from B0 phi_c=0, n=1), relative error %:')
    for name, sel in [('all tones', None)] + ([('3250-4250', [3250, 3500, 3750, 4000, 4250]), ('4000 only', [4000])] if np.isin(4000, f) else []):
        idx = np.ones(len(f), bool) if sel is None else np.isin(f, sel)
        A = lambda p: np.sqrt(np.sqrt(p['Pm'][idx].mean()*p['Pp'][idx].mean()))
        K = np.mean([A(p)*p['rho'] for p in train if p['phi'] == 0])
        err = [100*(K*np.exp(-p['e']**2/(2*77**2))/A(p)-p['rho'])/p['rho'] for p in e3]
        b = [100*(K/A(p)-p['rho'])/p['rho'] for p in b2]
        print(f'  {name:10s} K={K:5.1f}  E3 {np.round(err).astype(int).tolist()}  B2 sd {np.std(b):.0f}%')


if __name__ == '__main__':
    main()
