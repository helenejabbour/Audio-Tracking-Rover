#!/usr/bin/env python3
"""Pre-emphasis spec for the multi-tone beacon, from the measured speaker+horn response.

  python Tools/beacon_spec.py --runs Runs/2026-11-20 --profile horn12
      ->  Data/stimuli/horn12_spec.json   (then: Tools/make_tone.py --spec ...)

The response is the noise-subtracted per-tone power averaged over every valid pair of every
multi-tone capture (both horn sides, all poses), so single-position fringes average out
(per-pose sd is ~3 dB). It is interpolated linearly in dB onto the target tones and
inverted: gains_db = -(response_db - max), limited to --max-boost dB, so every tone reaches
the ADC at about the same level. Re-run after changing speaker, playback device or horn.
"""
import argparse
import glob
import json
import os
import numpy as np
import pandas as pd

PROFILES = {  # firmware/esp32_acoustic_v2 TONE_K * 31.25 Hz
    'horn12': [k*31.25 for k in range(102, 147, 4)],
    'wide16': [k*31.25 for k in range(80, 201, 8)],
}


def measured_response(run_dirs):
    """(freqs, mean power per tone, number of captures) over multi-tone captures."""
    powers, freqs = [], None
    for d in run_dirs:
        b, s = os.path.join(d, 'bins.csv'), os.path.join(d, 'samples.csv')
        if not (os.path.exists(b) and os.path.exists(s)):
            continue
        df = pd.read_csv(s).merge(pd.read_csv(b), on='seq')
        v = df[df['valid'] == 1]
        if len(v) < 5:
            continue
        f = np.array([float(c[3:]) for c in v.columns if c.startswith('am_')])
        if freqs is None:
            freqs = f
        elif len(f) != len(freqs) or not np.allclose(f, freqs):
            continue  # a different tone profile: do not mix
        am = v[[c for c in v.columns if c.startswith('am_')]].values
        ap = v[[c for c in v.columns if c.startswith('ap_')]].values
        n2 = np.r_[v.N_m.values, v.N_p.values].mean()**2
        powers.append(np.maximum(((am**2).mean(0)+(ap**2).mean(0))/2-n2, 1e-6))
    if not powers:
        raise SystemExit('no multi-tone captures with bins.csv and valid pairs found')
    return freqs, np.mean(powers, axis=0), len(powers)


def spec_for(target, freqs, power, max_boost):
    db = 10*np.log10(power)
    if target[0] < freqs[0] or target[-1] > freqs[-1]:
        raise SystemExit('target tones lie outside the measured band; measure there first')
    resp = np.interp(target, freqs, db)
    gains = np.minimum(resp.max()-resp, max_boost)
    return resp, gains-gains.max()  # loudest tone at 0 dB


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--runs', nargs='+', required=True, help='Runs/<date> folders or capture folders')
    ap.add_argument('--profile', choices=sorted(PROFILES), default='horn12')
    ap.add_argument('--max-boost', type=float, default=10.)
    ap.add_argument('--out')
    a = ap.parse_args()
    dirs = []
    for r in a.runs:
        dirs += [r] if os.path.exists(os.path.join(r, 'session.json')) else sorted(glob.glob(os.path.join(r, '*_capture')))
    freqs, power, n = measured_response(dirs)
    target = PROFILES[a.profile]
    resp, gains = spec_for(target, freqs, power, a.max_boost)
    out = a.out or f'Data/stimuli/{a.profile}_spec.json'
    spec = {'profile': a.profile, 'tone_hz': target, 'gains_db': [round(float(g), 2) for g in gains],
            'measured_response_db': [round(float(r-resp.max()), 2) for r in resp],
            'source_captures': n, 'source_tone_hz': freqs.tolist(), 'max_boost_db': a.max_boost,
            'note': 'gains_db = -(response - max): each tone reaches the ADC at about the same level'}
    with open(out, 'w', encoding='utf-8') as f:
        json.dump(spec, f, indent=2)
    for t, r, g in zip(target, spec['measured_response_db'], spec['gains_db']):
        print(f'{t:7.1f} Hz  response {r:+5.1f} dB  gain {g:+5.1f} dB')
    print(f'{n} captures -> {out}')


if __name__ == '__main__':
    main()
