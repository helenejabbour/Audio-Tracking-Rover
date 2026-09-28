#!/usr/bin/env python3
"""Bearing PoC: Python plays chirps (JBL = system audio output), arms follower, 2-mic TDOA -> bearing.
  python bearing_poc.py snap  COM5                          # ADC noise / clipping check
  python bearing_poc.py log   COM5 --n 30 --out th_+00.npz  # play + capture
  python bearing_poc.py stats th_+00.npz --true-th 0        # prints --tau0 to use
  python bearing_poc.py table th_*.npz --auto-tau0          # error vs angle (angle parsed from filename)
  python bearing_poc.py plot  th_+30.npz --idx 3
  python bearing_poc.py chirp                               # write chirp_loop.wav (play from phone instead)
Convention: A left, B right (facing source); tau = t_B - t_A = s*sin(theta)/c; theta > 0 = source on A's side.
pip install numpy scipy pyserial matplotlib sounddevice"""
import argparse, re, time
import numpy as np
from scipy.signal import butter, sosfiltfilt, correlate, hilbert, get_window
from scipy.signal.windows import tukey

KEYS = ('seq', 'fs', 'n', 'trig', 'over', 'floor')
AUDIO_FS = 48000

def make_chirp(fs, f0, f1, T, amp=1.0):
    t = np.arange(int(round(T * fs))) / fs
    x = np.sin(2 * np.pi * (f0 * t + 0.5 * (f1 - f0) / T * t**2))
    return amp * x * tukey(len(x), 0.2)

def bandpass(x, fs, lo, hi):
    return sosfiltfilt(butter(4, [lo, hi], btype='band', fs=fs, output='sos'), x)

def gcc_phat(a, b, fs, f0, f1, tmax, up=32):
    """tau = t_B - t_A (s), sub-sample via spectral zero-padding; returns (tau, peak quality)."""
    n = len(a); nfft = 1 << (int(np.ceil(np.log2(n))) + 1); w = get_window('hann', n)
    A, B = np.fft.rfft(a * w, nfft), np.fft.rfft(b * w, nfft)
    R = np.conj(A) * B; R /= np.abs(R) + 1e-12
    f = np.fft.rfftfreq(nfft, 1 / fs); R[(f < f0) | (f > f1)] = 0
    r = np.fft.irfft(R, nfft * up); k = int(np.ceil(tmax * fs * up))
    lags = np.r_[np.arange(0, k + 1), np.arange(-k, 0)]
    vals = np.r_[r[:k + 1], r[-k:]]
    i = int(np.argmax(vals))
    return lags[i] / (fs * up), vals[i] / (np.mean(np.abs(vals)) + 1e-12)

def to_theta(tau, p, tau0=None):
    c = 331.3 + 0.606 * p.temp
    t0 = p.tau0 if tau0 is None else tau0
    return np.degrees(np.arcsin(np.clip(c * (tau - t0) / p.s, -1, 1)))

def process(h, A, B, p):
    fs = h['fs']; c = 331.3 + 0.606 * p.temp
    lo, hi = p.f0 - 200, min(p.f1 + 300, 0.45 * fs)
    xa = bandpass(A - np.median(A), fs, lo, hi); xb = bandpass(B - np.median(B), fs, lo, hi)
    ref = make_chirp(fs, p.f0, p.f1, p.T)
    env = np.abs(hilbert(correlate(xa + xb, ref, mode='valid')))
    i0 = int(np.argmax(env)); snr = env[i0] / (np.median(env) + 1e-9)
    w = slice(max(i0 - 20, 0), min(i0 + len(ref) + 20, len(xa)))
    tau, q = gcc_phat(xa[w], xb[w], fs, p.f0, p.f1, 1.2 * p.s / c)
    seg = slice(i0, i0 + len(ref))
    clip = bool(np.any((A < 20) | (A > 4075)) or np.any((B < 20) | (B > 4075)))
    return dict(tau=tau, q=q, snr=snr, clip=clip, over=h['over'],
                ampA=20 * np.log10(np.std(xa[seg]) + 1e-9), ampB=20 * np.log10(np.std(xb[seg]) + 1e-9))

def good(R):
    return [r for r in R if r['snr'] > 6 and r['over'] == 0 and not r['clip']]

class Follower:
    def __init__(self, port):
        import serial
        self.ser = serial.Serial(port, 921600, timeout=1)
        t0 = time.time()
        while time.time() - t0 < 8:                       # opening the port may reset the board
            self.ser.write(b'STAT\n')
            if self.ser.readline().decode(errors='ignore').startswith(('STAT', 'READY')): break
        else:
            raise SystemExit('no reply from follower (port? firmware flashed?)')
        self.ser.reset_input_buffer()

    def _capture(self, timeout):
        t0 = time.time(); h = A = None
        while time.time() - t0 < timeout:
            s = self.ser.readline().decode(errors='ignore').strip()
            if s.startswith('TIMEOUT'): return None
            if s.startswith('CAP,'):
                v = s.split(',')[1:]
                h = dict(zip(KEYS, [int(x) for x in v[:5]] + [float(v[5])]))
            elif s.startswith('A,') and h:
                A = np.array(s[2:].split(','), float)
            elif s.startswith('B,') and h and A is not None:
                B = np.array(s[2:].split(','), float)
                if len(A) == h['n'] == len(B): return h, A, B
                h = A = None
        return None

    def arm(self, ms):
        self.ser.reset_input_buffer(); self.ser.write(f'ARM,{ms}\n'.encode())
        return self._capture(ms / 1000 + 3)

    def snap(self):
        self.ser.reset_input_buffer(); self.ser.write(b'SNAP\n')
        return self._capture(5)

def start_audio(p):
    import sounddevice as sd
    if p.device is not None:
        sd.default.device = int(p.device) if p.device.isdigit() else p.device
    ch = make_chirp(AUDIO_FS, p.f0, p.f1, p.T, p.amp)
    buf = np.zeros(int(AUDIO_FS * p.period), np.float32); buf[:len(ch)] = ch
    sd.play(buf, AUDIO_FS, loop=True)                     # keeps the Bluetooth link awake
    return sd

def load(fn):
    z = np.load(fn); H = [dict(zip(KEYS, r)) for r in z['H']]
    for h in H:
        for k in KEYS[:5]: h[k] = int(h[k])
    return H, z['A'], z['B']

def run_file(fn, p):
    H, A, B = load(fn)
    return [process(h, a, b, p) for h, a, b in zip(H, A, B)]

def cmd_snap(p):
    res = Follower(p.port).snap()
    if res is None: raise SystemExit('no snapshot received')
    h, A, B = res
    for name, x in (('A', A), ('B', B)):
        print(f'{name}: mean {x.mean():7.1f}  rms {x.std():5.2f}  min {x.min():.0f}  max {x.max():.0f}')
    print(f'over = {h["over"]} (must be 0); noise rms should be < 10 counts')

def cmd_log(p):
    F = Follower(p.port)
    sd = None if p.no_play else start_audio(p)
    time.sleep(2.5)                                       # let the Bluetooth stream settle
    H, As, Bs = [], [], []; tries = 0
    try:
        while len(H) < p.n and tries < 4 * p.n:
            tries += 1
            res = F.arm(p.arm_ms)
            if res is None:
                print('  timeout: no chirp heard (raise volume, or STAT/TH to lower threshold)'); continue
            h, A, B = res; r = process(h, A, B, p)
            print(f"{h['seq']:4d} th={to_theta(r['tau'], p):6.1f} deg  tau={r['tau'] * 1e6:7.1f} us  "
                  f"snr={r['snr']:5.1f}  q={r['q']:4.1f}  A-B={r['ampA'] - r['ampB']:+5.1f} dB  "
                  f"over={h['over']} clip={int(r['clip'])}")
            H.append([h[k] for k in KEYS]); As.append(A); Bs.append(B)
            time.sleep(p.gap)
    except KeyboardInterrupt:
        pass
    finally:
        if sd: sd.stop()
    np.savez(p.out, H=np.array(H, float), A=np.array(As), B=np.array(Bs))
    print(f'saved {len(H)} captures -> {p.out}')

def cmd_stats(p):
    c = 331.3 + 0.606 * p.temp
    for fn in p.files:
        R = run_file(fn, p); G = good(R)
        print(f'{fn}: {len(G)}/{len(R)} valid')
        if not G: continue
        tau = np.array([r['tau'] for r in G]); th = to_theta(tau, p)
        print(f'  tau {tau.mean() * 1e6:8.1f} us  std {tau.std() * 1e6:6.1f} us')
        print(f'  th  {th.mean():8.2f} deg  std {th.std():6.2f} deg')
        if p.true_th is not None:
            print(f'  -> use --tau0 {tau.mean() - p.s * np.sin(np.radians(p.true_th)) / c:.3e}')

def cmd_table(p):
    data = {}
    for fn in p.files:
        m = re.search(r'th_([+-]?\d+)', fn)
        if not m: print('skip (no th_<deg> in name):', fn); continue
        data[float(m.group(1))] = good(run_file(fn, p))
    tau0 = p.tau0
    if p.auto_tau0 and data.get(0.0):
        tau0 = float(np.mean([r['tau'] for r in data[0.0]])); print(f'auto tau0 = {tau0:.3e} s (0 deg file)')
    print(' true    n    mean    bias     std   (deg)'); errs = []
    for a in sorted(data):
        G = data[a]
        if not G: print(f'{a:+5.0f}    0'); continue
        th = to_theta(np.array([r['tau'] for r in G]), p, tau0)
        print(f'{a:+5.0f} {len(G):4d} {th.mean():+7.2f} {th.mean() - a:+7.2f} {th.std():7.2f}')
        if not (p.auto_tau0 and a == 0): errs.extend(th - a)
    if errs: print(f'rms error {np.sqrt(np.mean(np.square(errs))):.2f} deg  (pass: < 5)')

def cmd_plot(p):
    import matplotlib.pyplot as plt
    H, A, B = load(p.file); h, a, b = H[p.idx], A[p.idx], B[p.idx]; fs = h['fs']
    lo, hi = p.f0 - 200, min(p.f1 + 300, 0.45 * fs)
    xa, xb = (bandpass(x - np.median(x), fs, lo, hi) for x in (a, b))
    env = np.abs(hilbert(correlate(xa + xb, make_chirp(fs, p.f0, p.f1, p.T), mode='valid')))
    t = np.arange(len(a)) / fs * 1e3
    fig, ax = plt.subplots(2, 1, sharex=True)
    ax[0].plot(t, xa, label='A'); ax[0].plot(t, xb, alpha=.6, label='B')
    ax[0].axvline(h['trig'] / fs * 1e3, color='k', ls=':'); ax[0].legend(); ax[0].set_ylabel('counts')
    ax[1].plot(t[:len(env)], env); ax[1].set_ylabel('matched filter'); ax[1].set_xlabel('ms')
    plt.show()

def cmd_chirp(p):
    from scipy.io import wavfile
    ch = make_chirp(AUDIO_FS, p.f0, p.f1, p.T, p.amp)
    one = np.zeros(int(AUDIO_FS * p.period)); one[:len(ch)] = ch
    wavfile.write(p.out, AUDIO_FS, (np.tile(one, p.reps) * 32767).astype(np.int16))
    print('wrote', p.out)

def main():
    P = argparse.ArgumentParser(); sp = P.add_subparsers(dest='cmd', required=True)
    def common(s):
        s.add_argument('--f0', type=float, default=1000); s.add_argument('--f1', type=float, default=4000)
        s.add_argument('--T', type=float, default=0.03);  s.add_argument('--temp', type=float, default=24)
        s.add_argument('--s', type=float, default=0.15, help='capsule spacing [m]')
        s.add_argument('--tau0', type=float, default=0.0)
        s.add_argument('--amp', type=float, default=0.8); s.add_argument('--period', type=float, default=1.0)
        return s
    s = common(sp.add_parser('snap')); s.add_argument('port')
    s = common(sp.add_parser('log')); s.add_argument('port'); s.add_argument('--n', type=int, default=30)
    s.add_argument('--out', default='run.npz'); s.add_argument('--device'); s.add_argument('--no-play', action='store_true')
    s.add_argument('--arm-ms', type=int, default=2500); s.add_argument('--gap', type=float, default=0.3)
    s = common(sp.add_parser('stats')); s.add_argument('files', nargs='+'); s.add_argument('--true-th', type=float)
    s = common(sp.add_parser('table')); s.add_argument('files', nargs='+'); s.add_argument('--auto-tau0', action='store_true')
    s = common(sp.add_parser('plot')); s.add_argument('file'); s.add_argument('--idx', type=int, default=0)
    s = common(sp.add_parser('chirp')); s.add_argument('--out', default='chirp_loop.wav'); s.add_argument('--reps', type=int, default=30)
    p = P.parse_args()
    dict(snap=cmd_snap, log=cmd_log, stats=cmd_stats, table=cmd_table, plot=cmd_plot, chirp=cmd_chirp)[p.cmd](p)

if __name__ == '__main__':
    main()