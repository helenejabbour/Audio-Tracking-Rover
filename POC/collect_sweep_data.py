"""
Directivity & range POC - sweep collector v4.

Claim under test: the in-band level received by a fixed MAX4466 + ESP32 ADC changes
distinguishably and repeatably with speaker angle / distance.

CHANGES vs v3 (each one fixes a specific failure mode)
  1. Serial buffer is flushed at the START of every trial. v3 slept 0.4 s between
     trials with the ESP32 still streaming, so the next trial's "pre-trigger"
     window began with stale samples - including the previous trial's delayed
     Bluetooth audio. That alone explains "response before t_trigger".
  2. Capture window is 0.3 s before / 0.9 s after trigger (A2DP latency is
     100-300+ ms and jittery; +-100 ms can miss the burst). Latency is measured
     and logged as a diagnostic.
  3. Excitation: Tukey-windowed log chirp (default 1-4 kHz) instead of 600 Hz.
     For a driver of radius a, ka = 2*pi*f*a/c; at 600 Hz ka < 0.4 -> nearly
     omnidirectional, so there is almost no angle signal to measure.
  4. Metrics: band-pass -> sliding-window energy detector -> noise-subtracted
     in-band RMS level (dB re 1 mV), plus per-sub-band levels and the HF-LF
     tilt (range-invariant angle cue). Peak is logged as a diagnostic only.
  5. Every trial gets a status: OK / NOISE / CLIPPED / GLITCH / EDGE / PRE_DIRTY /
     SPARSE. Only OK trials enter statistics. Noise-only ("null") trials run at
     every point to measure the detector's false-alarm behaviour.
  6. Persistent output stream keeps the Bluetooth link open (no wake-up
     truncation of the burst). Linearity check verifies the speaker chain is
     linear (+6.02 dB per 2x drive amplitude) before trusting any amplitude.
  7. Per-point plot: waveform (auto y), envelope in dB re noise sigma (signal vs
     noise at a glance), PSD in window vs noise. End-of-run summary: level vs
     angle/distance, tilt, SNR, d' separability, angle resolution, mirror
     symmetry, 1/r^n fit.

Firmware: plain "adc\\n" lines work; "t_us,adc\\n" lines (micros() timestamp) are
also accepted and give an exact sample rate + dropped-sample detection.

Run:  python directivity_poc_v4.py               (hardware)
      python directivity_poc_v4.py --simulate    (synthetic rig, no hardware)
"""

import argparse
import csv
import os
import sys
import time
from collections import Counter

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy import signal
from scipy.ndimage import median_filter, uniform_filter1d

# ============================================================
# CONFIGURATION
# ============================================================
PORT = "COM4"
BAUD_RATE = 921600
V_REF = 3.3
ADC_MAX = 4095.0
MV_PER_COUNT = V_REF / ADC_MAX * 1000.0   # ADC-referred; ESP32 ADC is nonlinear, treat as relative
ADC_LO, ADC_HI = 150, 3850                # counts outside this = clipping / nonlinear region

# --- protocol
N_TRIALS_PER_POINT = 8
N_WARMUP_TRIALS_TO_DISCARD = 1
N_NULL_TRIALS_PER_POINT = 2               # no playback; measures false-alarm level (0 to disable)
CAPTURE_BEFORE_S = 0.30                   # noise reference window
CAPTURE_AFTER_S = 0.90                    # must exceed BT latency + burst + ringing
INTER_TRIAL_DELAY_S = 0.30
LINEARITY_CHECK = True

SWEEP_MODE = "angle"                      # "angle" | "distance"
SESSION_TAG = "session1"
FIXED_DISTANCE_CM = 30.0
FIXED_ANGLE_DEG = 0.0
ANGLE_SWEEP_POINTS = [0, 45, 90, 135, 180, 225, 270, 315]
DISTANCE_SWEEP_POINTS = [10, 20, 30, 40, 50, 70, 90, 120, 150]
REF_ANGLE_DEG = 0

# --- excitation
AUDIO_FS = 44100
EXC_KIND = "chirp"                        # "chirp" | "tone"
EXC_F0_HZ, EXC_F1_HZ = 1000.0, 4000.0     # F1 is clamped to 0.42 * ADC fs
EXC_TONE_HZ = 600.0
EXC_DURATION_S = 0.10
EXC_AMPLITUDE = 0.6                       # leave headroom: BT speakers have limiters / DRC
KEEPALIVE_DITHER = 1e-4                   # ~-80 dBFS noise keeps A2DP stream from suspending

# --- analysis
N_SUBBANDS = 3                            # log-spaced across the excitation band
FILTER_ORDER = 4
WIN_PAD_S = 0.03                          # energy window = burst + pad
NOISE_GUARD_S = 0.03                      # skip filter start-up in the noise window
DETECT_SNR_DB = 10.0                      # in-window energy / noise energy
BAND_MIN_SNR_DB = 6.0                     # per-sub-band validity
PERSIST_SIGMA, PERSIST_MIN = 2.5, 0.5     # envelope must exceed 2.5 sigma_n for >= 50% of window
SPIKE_MAX_DB = 12.0                       # envelope / local (15 ms) median envelope above this ...
SPIKE_MIN_SIGMA = 8.0                     # ... AND above 8 sigma_n = isolated glitch (not a smooth off-axis tilt)
PRE_DIRTY_RATIO = 2.0                     # RMS/robust-sigma of noise window (Gaussian ~ 1)
FS_TOL = 0.10                             # |fs_trial/fs_global - 1| above this = SPARSE
MIN_OK_TRIALS = 3                         # scored OK trials needed for a point to count
SIGMA_FLOOR_DB = 0.25                     # resolution floor when computing d'

TRIALS_LOG_PATH = "sweep_trials_v4.csv"
PLOTS_DIR = "sweep_plots"

# --- simulation only
SIM_FS, SIM_NOISE, SIM_SCALE = 10000.0, 6.0, 500.0

# runtime state (set in main)
SIMULATE = False
SRC = EMIT = WAVE = None
FS_GLOBAL = None
SUBBAND_EDGES = None


# ============================================================
# Excitation
# ============================================================
def make_excitation(f0, f1):
    n = int(AUDIO_FS * EXC_DURATION_S)
    t = np.arange(n) / AUDIO_FS
    if EXC_KIND == "chirp":
        w = signal.chirp(t, f0, t[-1], f1, method="logarithmic")
    else:
        w = np.sin(2 * np.pi * EXC_TONE_HZ * t)
    w = w * signal.windows.tukey(n, 0.25)
    return (EXC_AMPLITUDE * w / np.max(np.abs(w))).astype(np.float32)


# ============================================================
# I/O: hardware and simulated rig
# ============================================================
class Emitter:
    """Persistent output stream: silence + dither, impulse mixed in on demand."""

    def __init__(self):
        try:
            import sounddevice as sd
        except ImportError:
            raise SystemExit("pip install sounddevice  (needed for the persistent BT stream)")
        self._buf, self._pos = None, 0
        self._rng = np.random.default_rng()
        self.stream = sd.OutputStream(samplerate=AUDIO_FS, channels=1, dtype="float32",
                                      callback=self._cb)
        self.stream.start()

    def _cb(self, outdata, frames, time_info, status):
        outdata[:, 0] = KEEPALIVE_DITHER * self._rng.standard_normal(frames)
        buf = self._buf
        if buf is not None:
            n = min(frames, len(buf) - self._pos)
            outdata[:n, 0] += buf[self._pos:self._pos + n]
            self._pos += n
            if self._pos >= len(buf):
                self._buf = None

    def fire(self, wave, gain=1.0, ctx=None):
        self._pos = 0
        self._buf = (wave * gain).astype(np.float32)

    def close(self):
        self.stream.stop()
        self.stream.close()


class SerialSource:
    def __init__(self, port, baud):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0.001)
        try:
            self.ser.set_buffer_size(rx_size=1 << 20)   # Windows: avoid driver overflow
        except Exception:
            pass
        time.sleep(3.0)                                  # ESP32 resets on port open
        self._txt, self._discard = "", 1
        self.flush()

    def flush(self):
        self.ser.reset_input_buffer()
        self._txt, self._discard = "", 1                 # first line after a flush may be partial

    def read(self):
        """-> list of (adc, t_us or None) for all complete lines received."""
        n = self.ser.in_waiting
        if not n:
            return []
        self._txt += self.ser.read(n).decode("utf-8", errors="ignore")
        head, sep, tail = self._txt.rpartition("\n")
        if not sep:
            return []
        self._txt = tail
        lines = head.split("\n")
        if self._discard:
            lines, self._discard = lines[1:], 0
        out = []
        for line in lines:
            parts = line.strip().split(",")
            try:
                v = int(parts[-1])
                t_us = int(parts[0]) if len(parts) > 1 else None
            except ValueError:
                continue
            if 0 <= v <= 4095:
                out.append((v, t_us))
        return out

    def close(self):
        self.ser.close()


class SimSource:
    """Synthetic ESP32 stream: Gaussian noise + 50 Hz hum + rare glitches + delayed
    copies of the fired excitation shaped by a frequency-dependent directivity and 1/r."""

    def __init__(self, fs=SIM_FS, seed=0):
        self.fs, self.rng = fs, np.random.default_rng(seed)
        self.t0, self.n_gen, self.events = time.perf_counter(), 0, []

    def schedule(self, wave, gain, theta_deg, dist_cm):
        n = int(len(wave) / AUDIO_FS * self.fs)
        tt = np.arange(n) / self.fs
        x = np.interp(tt, np.arange(len(wave)) / AUDIO_FS, wave)
        nfft = 2 * n
        f = np.fft.rfftfreq(nfft, 1 / self.fs)
        p = np.maximum(f / 1500.0, 0.3)                          # narrower beam at HF
        D = np.maximum(((1 + np.cos(np.radians(theta_deg))) / 2) ** p, 0.03)
        H = D * (30.0 / dist_cm) * SIM_SCALE
        y = np.fft.irfft(np.fft.rfft(x, nfft) * H, nfft)[:n + n // 2] * gain
        t_start = time.perf_counter() + 0.22 + 0.01 * self.rng.standard_normal() + dist_cm / 34300.0
        self.events.append((t_start, y))

    def flush(self):
        self.read()

    def read(self):
        n_tot = int((time.perf_counter() - self.t0) * self.fs)
        m = n_tot - self.n_gen
        if m <= 0:
            return []
        idx = self.n_gen + np.arange(m)
        t = idx / self.fs
        x = 2048 + self.rng.normal(0, SIM_NOISE, m) + 3.0 * np.sin(2 * np.pi * 50 * t)
        g = self.rng.random(m) < 1 / 20000.0
        x[g] += 400 * self.rng.choice([-1, 1], g.sum())
        keep = []
        for t_start, y in self.events:
            i0 = int(round((t_start - self.t0) * self.fs))
            a, b = max(self.n_gen, i0), min(n_tot, i0 + len(y))
            if b > a:
                x[a - self.n_gen:b - self.n_gen] += y[a - i0:b - i0]
            if i0 + len(y) > n_tot:
                keep.append((t_start, y))
        self.events, self.n_gen = keep, n_tot
        return [(int(v), None) for v in np.clip(np.round(x), 0, 4095)]

    def close(self):
        pass


class SimEmitter:
    def __init__(self, src):
        self.src = src

    def fire(self, wave, gain=1.0, ctx=None):
        self.src.schedule(wave, gain, *ctx)

    def close(self):
        pass


# ============================================================
# Capture
# ============================================================
def capture(duration_s, rec):
    t_end = time.perf_counter() + duration_s
    while time.perf_counter() < t_end:
        got = SRC.read()
        if got:
            rec.extend(got)
        else:
            time.sleep(0.0005)


def estimate_fs(rec, dur):
    tus = [r[1] for r in rec]
    if all(t is not None for t in tus):
        d = np.diff(np.asarray(tus, float))
        d = d[d > 0]
        med = np.median(d)
        return 1e6 / med, bool(np.any(d > 5 * med))
    return len(rec) / dur, False


def calibrate_fs(seconds=2.0):
    print("Calibrating ADC stream rate ...")
    SRC.flush()
    t0, rec = time.perf_counter(), []
    capture(seconds, rec)
    if len(rec) < 100:
        raise RuntimeError("No usable data on the serial stream.")
    fs, _ = estimate_fs(rec, time.perf_counter() - t0)
    print(f"  ADC fs ~ {fs:.0f} Hz ({len(rec)} samples)\n")
    return fs


# ============================================================
# Analysis
# ============================================================
def analyze_trial(adc, fs, n_pre):
    """adc: raw counts; samples [0, n_pre) are the pre-trigger noise reference."""
    N = len(adc)
    guard = int(NOISE_GUARD_S * fs)
    L = int((EXC_DURATION_S + WIN_PAD_S) * fs)
    if n_pre - guard < 100 or N - n_pre < L + 10:
        return None
    ac_mv = (adc - np.median(adc[:n_pre])) * MV_PER_COUNT
    pre = slice(guard, n_pre)
    e = SUBBAND_EDGES

    def bandpass(lo, hi):
        sos = signal.butter(FILTER_ORDER, [lo, hi], btype="bandpass", fs=fs, output="sos")
        return signal.sosfiltfilt(sos, ac_mv)

    def robust_noise_power(x):        # MAD-based sigma^2: immune to a stray transient
        xp = x[pre]
        return max((1.4826 * np.median(np.abs(xp - np.median(xp)))) ** 2, 1e-12)

    xb = bandpass(e[0], e[-1])
    p_n = robust_noise_power(xb)
    sigma_n = np.sqrt(p_n)
    dirty_ratio = float(np.sqrt(np.mean(xb[pre] ** 2)) / sigma_n)

    # sliding-window energy detector over the post-trigger region
    cs = np.concatenate(([0.0], np.cumsum(xb ** 2)))
    ks = np.arange(n_pre, N - L + 1)
    wsum = cs[ks + L] - cs[ks]
    k = int(ks[np.argmax(wsum)])
    p_w = float(wsum.max() / L)
    snr_db = 10 * np.log10(p_w / p_n)

    env = np.abs(signal.hilbert(xb))
    persist = float(np.mean(env[k:k + L] > PERSIST_SIGMA * sigma_n))
    detected = bool(snr_db >= DETECT_SNR_DB and persist >= PERSIST_MIN)

    rms_mv = float(np.sqrt(max(p_w - p_n, 0.0)))               # noise-power-subtracted
    level_db = 20 * np.log10(rms_mv) if rms_mv > 0 else np.nan
    seg = xb[k:k + L]
    pk = int(np.argmax(np.abs(seg)))
    peak_mv = float(abs(seg[pk]))
    crest_db = float(20 * np.log10(peak_mv / np.sqrt(p_w)))

    env_w = env[k:k + L]                                        # isolated-spike test (local prominence)
    med_w = median_filter(env, size=int(0.015 * fs) | 1, mode="nearest")[k:k + L]
    spike_db = float(20 * np.log10(np.max(env_w / np.maximum(med_w, 1e-9))))
    glitch = bool(spike_db > SPIKE_MAX_DB and env_w.max() > SPIKE_MIN_SIGMA * sigma_n)

    latency_ms = np.nan                                         # energy-onset (5% of window energy)
    cum = np.cumsum(seg ** 2 - p_n)
    if detected and cum[-1] > 0:
        latency_ms = (k + int(np.argmax(cum >= 0.05 * cum[-1])) - n_pre) / fs * 1e3

    bands = [np.nan] * (len(e) - 1)
    if detected:
        for j, (lo, hi) in enumerate(zip(e[:-1], e[1:])):
            xs = bandpass(lo, hi)
            pn, pw = robust_noise_power(xs), float(np.mean(xs[k:k + L] ** 2))
            if pw > pn and 10 * np.log10(pw / pn) >= BAND_MIN_SNR_DB:
                bands[j] = 20 * np.log10(np.sqrt(pw - pn))
    tilt_db = bands[-1] - bands[0] if len(bands) > 1 else np.nan   # HF - LF

    env_s = uniform_filter1d(env, max(1, int(0.002 * fs)))
    return dict(
        N=N, k=k, L=L, n_pre=n_pre, guard=guard, peak_idx=k + pk,
        t_ms=(np.arange(N) - n_pre) / fs * 1e3, ac_mv=ac_mv,
        env_db=20 * np.log10(np.maximum(env_s, 1e-6) / sigma_n),
        snr_db=float(snr_db), level_db=float(level_db), rms_mv=rms_mv, peak_mv=peak_mv,
        crest_db=crest_db, spike_db=spike_db, glitch=glitch, persist=persist, detected=detected, dirty_ratio=dirty_ratio,
        noise_rms_mv=float(sigma_n), raw_noise_mv=float(np.std(ac_mv[pre])),
        bands=bands, tilt_db=float(tilt_db), latency_ms=float(latency_ms),
        clip_count=int(np.sum((adc <= ADC_LO) | (adc >= ADC_HI))),
    )


def classify(a, fire, sparse):
    if sparse:
        return "SPARSE"
    if not fire:
        return "FALSE_ALARM" if a["detected"] else "NULL"
    if a["dirty_ratio"] > PRE_DIRTY_RATIO:
        return "PRE_DIRTY"
    if a["clip_count"] >= 2:
        return "CLIPPED"
    if not a["detected"]:
        return "NOISE"
    if a["glitch"]:
        return "GLITCH"
    if a["k"] + a["L"] >= a["N"] - 2:
        return "EDGE"
    return "OK"


def run_single_trial(theta, dist, gain=1.0, fire=True):
    SRC.flush()                                   # never let stale samples into the window
    t0, rec = time.perf_counter(), []
    capture(CAPTURE_BEFORE_S, rec)
    n_pre = len(rec)
    if fire:
        EMIT.fire(WAVE, gain, (theta, dist))
    capture(CAPTURE_AFTER_S, rec)
    dur = time.perf_counter() - t0
    if len(rec) < 200 or n_pre < 100:
        print(f"  [WARN] only {len(rec)} samples - weak serial throughput.")
        return None
    adc = np.fromiter((r[0] for r in rec), float, len(rec))
    fs, gap = estimate_fs(rec, dur)
    a = analyze_trial(adc, fs, n_pre)
    if a is None:
        return None
    fs_dev = fs / FS_GLOBAL - 1.0
    a.update(fire=fire, theta=theta, dist=dist, gain=gain, fs=fs, fs_dev=fs_dev)
    a["status"] = classify(a, fire, gap or abs(fs_dev) > FS_TOL)
    return a


def linearity_check(theta, dist):
    """Drive amplitude x2 must give +6.02 dB. Slope != 1 -> BT limiter/DRC or ADC compression."""
    gains = [0.25, 0.5, 1.0]
    lv = []
    for g in gains:
        vals = []
        for _ in range(3):
            r = run_single_trial(theta, dist, gain=g)
            if r and r["status"] == "OK":
                vals.append(r["level_db"])
            time.sleep(INTER_TRIAL_DELAY_S)
        lv.append(np.mean(vals) if vals else np.nan)
    x, y = 20 * np.log10(gains), np.array(lv)
    m = np.isfinite(y)
    if m.sum() < 2:
        print("  [linearity] not enough valid trials - check SNR / clipping.")
        return
    slope = np.polyfit(x[m], y[m], 1)[0]
    verdict = "PASS" if 0.9 <= slope <= 1.1 else "FAIL - amplitude comparisons unreliable"
    print(f"  [linearity] levels @ gains {gains}: " + ", ".join(f"{v:.1f}" for v in lv) +
          f" dB  -> slope {slope:.2f} (ideal 1.00)  {verdict}")


# ============================================================
# Aggregation
# ============================================================
def mean_std(vals, n_min=1):
    v = np.asarray([x for x in vals if np.isfinite(x)], float)
    if len(v) < n_min or len(v) == 0:
        return np.nan, np.nan
    return float(v.mean()), (float(v.std(ddof=1)) if len(v) > 1 else np.nan)


def aggregate_point(theta, dist, results):
    sig = [r for r in results if r["fire"]]
    scored = [r for r in sig if not r["is_warmup"]]
    ok = [r for r in scored if r["status"] == "OK"]
    nulls = [r for r in results if not r["fire"]]
    nb = len(SUBBAND_EDGES) - 1
    lv_m, lv_s = mean_std([r["level_db"] for r in ok])
    tl_m, tl_s = mean_std([r["tilt_db"] for r in ok], MIN_OK_TRIALS)
    lat_m, lat_s = mean_std([r["latency_ms"] for r in scored if r["detected"]])
    return dict(
        theta=theta, dist=dist, n_scored=len(scored), n_ok=len(ok),
        status=Counter(r["status"] for r in scored),
        level_mean=lv_m, level_std=lv_s,
        bands=[mean_std([r["bands"][j] for r in ok], MIN_OK_TRIALS) for j in range(nb)],
        tilt_mean=tl_m, tilt_std=tl_s, lat_mean=lat_m, lat_std=lat_s,
        snr_med=float(np.median([r["snr_db"] for r in scored])) if scored else np.nan,
        null_snr_max=max((r["snr_db"] for r in nulls), default=np.nan),
        null_false_alarms=sum(r["status"] == "FALSE_ALARM" for r in nulls),
        verdict="OK" if len(ok) >= MIN_OK_TRIALS else "NOISE/INVALID",
    )


# ============================================================
# Plotting
# ============================================================
def plot_point(point_value, variable, results, agg):
    os.makedirs(PLOTS_DIR, exist_ok=True)
    sig = [r for r in results if r["fire"]]
    nulls = [r for r in results if not r["fire"]]
    if not sig:
        return
    fig, (ax_t, ax_e, ax_p) = plt.subplots(3, 1, figsize=(9.5, 11.5),
                                           gridspec_kw={"height_ratios": [3, 2.2, 2.2]})
    cols = plt.cm.tab10(np.arange(10))
    # auto y-scale from the detected windows (a stray glitch outside them must not squash the burst);
    # if nothing was detected, fall back to the 99.5th percentile so the noise itself fills the axes
    det = [np.max(np.abs(r["ac_mv"][r["k"]:r["k"] + r["L"]])) for r in sig if r["detected"]]
    m = max(det) if det else max(np.percentile(np.abs(r["ac_mv"]), 99.5) for r in sig)
    for i, r in enumerate(sig):
        c, w = cols[i % 10], r["is_warmup"]
        ls, al = ("--", 0.35) if w else ("-", 0.8)
        lab = f"#{i + 1} {r['status']}" + (" (warm-up)" if w else "")
        ax_t.plot(r["t_ms"], r["ac_mv"], lw=0.6, ls=ls, alpha=al, color=c, label=lab)
        ax_t.plot(r["t_ms"][r["peak_idx"]], r["ac_mv"][r["peak_idx"]], "o", ms=4, color=c)
        ax_t.axvspan(r["t_ms"][r["k"]], r["t_ms"][min(r["k"] + r["L"], r["N"]) - 1],
                     color=c, alpha=0.04)
        ax_e.plot(r["t_ms"], r["env_db"], lw=0.9, ls=ls, alpha=al, color=c)
    for j, r in enumerate(nulls):
        ax_e.plot(r["t_ms"], r["env_db"], lw=0.8, ls=":", color="0.4",
                  label="noise-only trial" if j == 0 else None)
    ref = sig[-1]
    ax_t.axhspan(-3 * ref["raw_noise_mv"], 3 * ref["raw_noise_mv"], color="0.5", alpha=0.25,
                 label="±3σ raw noise (pre-trigger)")
    ax_t.set_ylim(-1.1 * m, 1.1 * m)                       # automatic, symmetric
    ax_t.axvline(0, color="k", ls=":", lw=1)
    ax_t.set_ylabel("baseline-subtracted signal (mV, ADC-referred)")
    ax_t.set_xlabel("time relative to trigger call (ms); dots = band-limited peak, shading = detected window; y auto-scaled to detected burst")
    ax_t.legend(fontsize=7, ncol=3, loc="upper right")
    ax_t.grid(alpha=0.3)

    ax_e.axhline(0, color="0.3", lw=1, label="noise σ (band RMS)")
    ax_e.axhline(DETECT_SNR_DB + 3.01, color="r", ls="--", lw=1,
                 label=f"detection ({DETECT_SNR_DB:.0f} dB in-window SNR)")
    ax_e.axvline(0, color="k", ls=":", lw=1)
    ax_e.set_ylim(bottom=-10)
    ax_e.set_ylabel("envelope (dB re noise σ)")
    ax_e.set_xlabel("time relative to trigger call (ms)")
    ax_e.legend(fontsize=7, loc="upper right")
    ax_e.grid(alpha=0.3)

    noise_psd = []
    for i, r in enumerate(sig):
        w = r["is_warmup"]
        seg = r["ac_mv"][r["k"]:r["k"] + r["L"]]
        f, p = signal.welch(seg, r["fs"], nperseg=min(256, len(seg)))
        ax_p.plot(f, 10 * np.log10(p + 1e-12), color=cols[i % 10], lw=1,
                  ls="--" if w else "-", alpha=0.35 if w else 0.8)
        fn, pn = signal.welch(r["ac_mv"][r["guard"]:r["n_pre"]], r["fs"], nperseg=256)
        noise_psd.append((fn, pn))
    ax_p.plot(noise_psd[0][0], 10 * np.log10(np.mean([p for _, p in noise_psd], axis=0) + 1e-12),
              color="0.3", lw=2, label="pre-trigger noise (mean)")
    for edge in SUBBAND_EDGES:
        ax_p.axvline(edge, color="b", ls=":", lw=0.8)
    ax_p.set_xlim(0, min(sig[0]["fs"] / 2, 1.6 * SUBBAND_EDGES[-1]))
    ax_p.set_xlabel("frequency (Hz); dotted = sub-band edges")
    ax_p.set_ylabel("PSD (dB re 1 mV²/Hz)")
    ax_p.legend(fontsize=7)
    ax_p.grid(alpha=0.3)

    if variable == "theta_deg":
        title = f"Angle sweep - θ = {point_value}° (distance {FIXED_DISTANCE_CM} cm)"
        fname = f"angle_{int(point_value):03d}deg.png"
    else:
        title = f"Distance sweep - d = {point_value} cm (angle {FIXED_ANGLE_DEG}°)"
        fname = f"dist_{int(point_value):03d}cm.png"
    sub = (f"{agg['verdict']} | OK {agg['n_ok']}/{agg['n_scored']} scored {dict(agg['status'])} | "
           f"L = {agg['level_mean']:.1f} ± {agg['level_std']:.1f} dB re 1 mV | "
           f"median SNR {agg['snr_med']:.1f} dB | null max {agg['null_snr_max']:.1f} dB | "
           f"latency {agg['lat_mean']:.0f} ± {agg['lat_std']:.0f} ms")
    fig.suptitle(f"{title}\n{sub}", fontsize=9.5, color="k" if agg["verdict"] == "OK" else "firebrick")
    fig.tight_layout()
    out = os.path.join(PLOTS_DIR, f"{SESSION_TAG}_{fname}")
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print(f"  --> saved plot: {out}")


def _min_span(ax, span=2.0):
    """Keep tilt panels from magnifying sub-dB jitter: enforce a minimum visible y-span (dB)."""
    lo, hi = ax.get_ylim()
    if hi - lo < span:
        c = 0.5 * (lo + hi)
        ax.set_ylim(c - span / 2, c + span / 2)


def _snr_panel(ax, xs, aggs, xlabel, log=False):
    ok = np.array([a["verdict"] == "OK" for a in aggs])
    snr = np.array([a["snr_med"] for a in aggs])
    if log:
        ax.plot(xs, snr, "o-", color="tab:blue", label="median SNR (signal trials)")
        ax.plot(xs[~ok], snr[~ok], "rx", ms=9, label="not OK")
        ax.set_xscale("log")
    else:
        pos = np.arange(len(xs))
        ax.bar(pos, snr, color=np.where(ok, "tab:blue", "firebrick"))
        ax.set_xticks(pos)
        ax.set_xticklabels([f"{x:g}" for x in xs])
    nx = np.arange(len(xs)) if not log else xs
    ax.plot(nx, [a["null_snr_max"] for a in aggs], "k^", label="noise-only max")
    ax.axhline(DETECT_SNR_DB, color="r", ls="--", label="detection threshold")
    ax.set_xlabel(xlabel)
    ax.set_ylabel("in-window SNR (dB)")
    ax.legend(fontsize=7)
    ax.grid(alpha=0.3)


def plot_summary_angle(aggs, ref):
    th = np.array([a["theta"] for a in aggs], float)
    L = np.array([a["level_mean"] for a in aggs])
    S = np.array([a["level_std"] for a in aggs])
    dL = L - ref["level_mean"]
    fig = plt.figure(figsize=(12.5, 9.5))
    ax1 = fig.add_subplot(2, 2, 1, projection="polar")
    ax2, ax3, ax4 = (fig.add_subplot(2, 2, i) for i in (2, 3, 4))
    rmin = min(-10.0, np.nanmin(dL - np.nan_to_num(S)) - 3)
    order = np.argsort(th)
    tt, dd, ss = np.r_[th[order], th[order][0]], np.r_[dL[order], dL[order][0]], np.r_[S[order], S[order][0]]
    ax1.set_theta_zero_location("N")
    ax1.set_theta_direction(-1)
    ax1.plot(np.radians(tt), dd, "o-")
    ax1.fill_between(np.radians(tt), dd - np.nan_to_num(ss), dd + np.nan_to_num(ss), alpha=0.25)
    ax1.set_rlim(rmin, 3)
    ax1.set_title("Broadband ΔL re reference angle (dB), ±1σ single-trial", fontsize=9, pad=14)

    for j, (lo, hi) in enumerate(zip(SUBBAND_EDGES[:-1], SUBBAND_EDGES[1:])):
        mb = np.array([a["bands"][j][0] for a in aggs]) - ref["bands"][j][0]
        sb = np.array([a["bands"][j][1] for a in aggs])
        ax2.errorbar(th, mb, yerr=sb, marker="o", capsize=2, label=f"{lo:.0f}-{hi:.0f} Hz")
    ax2.errorbar(th, dL, yerr=S, color="k", lw=2, marker="s", capsize=2, label="broadband")
    ax2.set_xlabel("angle (deg)")
    ax2.set_ylabel("ΔL (dB)")
    ax2.set_title("Sub-band directivity", fontsize=9)
    ax2.legend(fontsize=7)
    ax2.grid(alpha=0.3)

    ax3.errorbar(th, [a["tilt_mean"] for a in aggs], yerr=[a["tilt_std"] for a in aggs],
                 marker="o", capsize=2)
    ax3.set_xlabel("angle (deg)")
    ax3.set_ylabel("tilt  L_HF - L_LF (dB)")
    ax3.set_title("Spectral tilt (range-invariant angle cue)", fontsize=9)
    _min_span(ax3)
    ax3.grid(alpha=0.3)
    _snr_panel(ax4, th, aggs, "angle (deg)")
    ax4.set_title("Detectability per point", fontsize=9)
    fig.tight_layout()
    out = os.path.join(PLOTS_DIR, f"{SESSION_TAG}_summary_angle.png")
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print(f"  --> saved summary plot: {out}")


def plot_summary_distance(aggs, fit):
    r = np.array([a["dist"] for a in aggs], float)
    L = np.array([a["level_mean"] for a in aggs])
    S = np.array([a["level_std"] for a in aggs])
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 4.6))
    ax1.errorbar(r, L, yerr=S, marker="o", capsize=2, label="measured")
    good = np.isfinite(L)
    if good.any():
        r0 = r[good][0]
        ax1.plot(r, L[good][0] - 20 * np.log10(r / r0), "k--", lw=1, label="ideal 1/r (-6 dB/doubling)")
    if fit:
        ax1.plot(r, fit["icpt"] + fit["slope"] * np.log10(r), "r-", lw=1,
                 label=f"fit: n = {fit['n']:.2f}, R² = {fit['r2']:.3f}")
    ax1.set_xscale("log")
    ax1.set_xlabel("distance (cm)")
    ax1.set_ylabel("L (dB re 1 mV rms)")
    ax1.legend(fontsize=7)
    ax1.grid(alpha=0.3, which="both")
    ax2.errorbar(r, [a["tilt_mean"] for a in aggs], yerr=[a["tilt_std"] for a in aggs],
                 marker="o", capsize=2)
    ax2.set_xscale("log")
    ax2.set_xlabel("distance (cm)")
    ax2.set_ylabel("tilt  L_HF - L_LF (dB)")
    ax2.set_title("should stay flat if range-invariant", fontsize=9)
    _min_span(ax2)
    ax2.grid(alpha=0.3, which="both")
    _snr_panel(ax3, r, aggs, "distance (cm)", log=True)
    fig.tight_layout()
    out = os.path.join(PLOTS_DIR, f"{SESSION_TAG}_summary_distance.png")
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print(f"  --> saved summary plot: {out}")


# ============================================================
# Summary statistics
# ============================================================
def _dprime(m1, s1, m2, s2):
    if not np.all(np.isfinite([m1, s1, m2, s2])):
        return np.nan
    s1, s2 = max(s1, SIGMA_FLOOR_DB), max(s2, SIGMA_FLOOR_DB)
    return abs(m1 - m2) / np.sqrt(0.5 * (s1 ** 2 + s2 ** 2))


def summarize(aggs, variable):
    n = len(aggs)
    rows, fit, ref = [], None, None
    if variable == "theta_deg":
        x = np.array([a["theta"] for a in aggs], float)
        step = np.median(np.diff(x)) if n > 1 else 0.0
        wrap = n > 2 and (x.max() - x.min() + step) >= 360 - 1e-6
        ref = next((a for a in aggs if a["theta"] == REF_ANGLE_DEG and a["verdict"] == "OK"),
                   next((a for a in aggs if a["verdict"] == "OK"), None))
    else:
        x = np.array([a["dist"] for a in aggs], float)
        wrap = False
    for i, a in enumerate(aggs):
        j = i + 1 if i + 1 < n else (0 if wrap else None)
        row = dict(point=x[i], verdict=a["verdict"], n_ok=a["n_ok"], snr_db=a["snr_med"],
                   level_db=a["level_mean"], level_std=a["level_std"],
                   tilt_db=a["tilt_mean"], tilt_std=a["tilt_std"],
                   dL_db=(a["level_mean"] - ref["level_mean"]) if ref else np.nan,
                   dprime_level=np.nan, dprime_tilt=np.nan, sigma_theta_deg=np.nan)
        if j is not None and a["verdict"] == "OK" and aggs[j]["verdict"] == "OK":
            b = aggs[j]
            row["dprime_level"] = _dprime(a["level_mean"], a["level_std"], b["level_mean"], b["level_std"])
            row["dprime_tilt"] = _dprime(a["tilt_mean"], a["tilt_std"], b["tilt_mean"], b["tilt_std"])
            if variable == "theta_deg":
                dth = abs(((x[j] - x[i] + 180) % 360) - 180)
                slope = abs(a["level_mean"] - b["level_mean"]) / dth
                s = 0.5 * (max(a["level_std"], SIGMA_FLOOR_DB) + max(b["level_std"], SIGMA_FLOOR_DB))
                row["sigma_theta_deg"] = s / slope if slope > 0 else np.inf
        rows.append(row)

    print("\n" + "=" * 96)
    print(f"SUMMARY ({variable})   d' = separation to NEXT point in units of single-trial σ "
          f"(>= 2 distinguishable)")
    hdr = f"{'point':>7} {'verdict':>13} {'nOK':>4} {'SNR':>6} {'L(dB)':>7} {'σL':>5} " \
          f"{'ΔL':>7} {'tilt':>7} {'σtilt':>6} {'d\'_L':>6} {'d\'_tilt':>7} {'σθ(deg)':>8}"
    print(hdr)
    for r in rows:
        print(f"{r['point']:7g} {r['verdict']:>13} {r['n_ok']:4d} {r['snr_db']:6.1f} {r['level_db']:7.1f} "
              f"{r['level_std']:5.2f} {r['dL_db']:7.1f} {r['tilt_db']:7.1f} {r['tilt_std']:6.2f} "
              f"{r['dprime_level']:6.1f} {r['dprime_tilt']:7.1f} {r['sigma_theta_deg']:8.1f}")
    bad = [f"{r['point']:g}" for r in rows if r["verdict"] != "OK"]
    if bad:
        print(f"\nFLAGGED (noise-limited / invalid): {bad}")

    if variable == "theta_deg" and wrap:
        print("\nMirror symmetry (θ vs 360-θ), expected ~0 for an axisymmetric source:")
        for a in aggs:
            if 0 < a["theta"] < 180:
                m = next((b for b in aggs if b["theta"] == 360 - a["theta"]), None)
                if m and a["verdict"] == "OK" and m["verdict"] == "OK":
                    d = a["level_mean"] - m["level_mean"]
                    s = np.hypot(max(a["level_std"], SIGMA_FLOOR_DB), max(m["level_std"], SIGMA_FLOOR_DB))
                    print(f"  {a['theta']:g} vs {m['theta']:g}: ΔL = {d:+.2f} dB ({abs(d) / s:.1f}σ)"
                          + ("  <- setup asymmetry / repositioning error" if abs(d) > 2 * s else ""))
        print("Note: one mic + amplitude only resolves |θ|; θ and -θ are indistinguishable.")

    if variable == "distance_cm":
        pts = [a for a in aggs if a["verdict"] == "OK"]
        if len(pts) >= 3:
            lx = np.log10([a["dist"] for a in pts])
            y = np.array([a["level_mean"] for a in pts])
            slope, icpt = np.polyfit(lx, y, 1)
            res = y - (slope * lx + icpt)
            r2 = 1 - res.var() / y.var() if y.var() > 0 else np.nan
            fit = dict(slope=slope, icpt=icpt, n=-slope / 20, r2=r2)
            print(f"\nPropagation fit L = a - 20·n·log10(r):  n = {fit['n']:.2f}  R² = {r2:.3f}  "
                  f"(free field n = 1; n -> 0 in the reverberant field)")

    os.makedirs(PLOTS_DIR, exist_ok=True)
    path = os.path.join(PLOTS_DIR, f"{SESSION_TAG}_{variable}_summary.csv")
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows({k: (f"{v:.4f}" if isinstance(v, float) else v) for k, v in r.items()} for r in rows)
    print(f"Summary table -> {path}")
    if variable == "theta_deg" and ref:
        plot_summary_angle(aggs, ref)
    elif variable == "distance_cm":
        plot_summary_distance(aggs, fit)


# ============================================================
# Logging + sweep
# ============================================================
def csv_fields():
    return (["session", "mode", "trial_in_point", "kind", "is_warmup", "theta_deg", "distance_cm",
             "status", "snr_db", "level_db", "rms_mv", "peak_mv", "crest_db", "spike_db", "persist", "tilt_db"]
            + [f"band{j}_db" for j in range(len(SUBBAND_EDGES) - 1)]
            + ["latency_ms", "noise_rms_mv", "fs_hz", "fs_dev_pct", "clip_count", "dirty_ratio", "wall_time_s"])


def ensure_csv_header(path, fields):
    if os.path.exists(path):
        with open(path, newline="") as f:
            if next(csv.reader(f), None) != fields:
                raise SystemExit(f"{path} has a different column layout - use a new TRIALS_LOG_PATH.")
        return
    with open(path, "w", newline="") as f:
        csv.DictWriter(f, fieldnames=fields).writeheader()


def log_trial(r, i, variable, t_start):
    row = {"session": SESSION_TAG, "mode": variable, "trial_in_point": i,
           "kind": "signal" if r["fire"] else "null", "is_warmup": int(r["is_warmup"]),
           "theta_deg": r["theta"], "distance_cm": r["dist"], "status": r["status"],
           "snr_db": r["snr_db"], "level_db": r["level_db"], "rms_mv": r["rms_mv"],
           "peak_mv": r["peak_mv"], "crest_db": r["crest_db"], "spike_db": r["spike_db"],
           "persist": r["persist"],
           "tilt_db": r["tilt_db"], "latency_ms": r["latency_ms"], "noise_rms_mv": r["noise_rms_mv"],
           "fs_hz": r["fs"], "fs_dev_pct": 100 * r["fs_dev"], "clip_count": r["clip_count"],
           "dirty_ratio": r["dirty_ratio"], "wall_time_s": time.perf_counter() - t_start}
    for j, b in enumerate(r["bands"]):
        row[f"band{j}_db"] = b
    row = {k: (f"{v:.4f}" if isinstance(v, float) else v) for k, v in row.items()}
    with open(TRIALS_LOG_PATH, "a", newline="") as f:
        csv.DictWriter(f, fieldnames=csv_fields()).writerow(row)


def run_sweep(points, variable):
    ensure_csv_header(TRIALS_LOG_PATH, csv_fields())
    t_start, aggs, first = time.perf_counter(), [], True
    for pv in points:
        theta, dist = (pv, FIXED_DISTANCE_CM) if variable == "theta_deg" else (FIXED_ANGLE_DEG, pv)
        msg = f"Set mic angle to {theta} deg (distance {dist} cm)" if variable == "theta_deg" \
            else f"Set distance to {dist} cm (angle {theta} deg)"
        if not SIMULATE:
            input(f"\n>>> {msg}. Press ENTER when in position...")
        else:
            print(f"\n>>> [sim] {msg}")

        if first:
            run_single_trial(theta, dist)                   # settle: wake BT link, prime buffers
            time.sleep(INTER_TRIAL_DELAY_S)
            if LINEARITY_CHECK:
                linearity_check(theta, dist)
            first = False

        results = []
        plan = [(True, i) for i in range(N_TRIALS_PER_POINT)] + \
               [(False, N_TRIALS_PER_POINT + j) for j in range(N_NULL_TRIALS_PER_POINT)]
        for fire, i in plan:
            tag = f"trial {i + 1}" if fire else "null "
            print(f"  {tag} @ θ={theta}, d={dist} ...", end=" ", flush=True)
            r = run_single_trial(theta, dist, fire=fire)
            if r is None:
                print("SKIPPED")
            else:
                r["is_warmup"] = fire and i < N_WARMUP_TRIALS_TO_DISCARD
                print(f"{r['status']:<11} SNR={r['snr_db']:5.1f} dB  L={r['level_db']:6.1f} dB  "
                      f"tilt={r['tilt_db']:+5.1f}  lat={r['latency_ms']:5.0f} ms"
                      + ("  (warm-up)" if r["is_warmup"] else ""))
                results.append(r)
                log_trial(r, i, variable, t_start)
            time.sleep(INTER_TRIAL_DELAY_S)

        agg = aggregate_point(theta, dist, results)
        print(f"  --> {agg['verdict']}: OK {agg['n_ok']}/{agg['n_scored']}  "
              f"L = {agg['level_mean']:.1f} ± {agg['level_std']:.1f} dB  "
              f"null max SNR = {agg['null_snr_max']:.1f} dB  statuses {dict(agg['status'])}")
        plot_point(pv, variable, results, agg)
        aggs.append(agg)

    print(f"\nSweep complete. Trials -> {os.path.abspath(TRIALS_LOG_PATH)}")
    summarize(aggs, variable)


def main(argv=None):
    global SIMULATE, SRC, EMIT, WAVE, FS_GLOBAL, SUBBAND_EDGES, SWEEP_MODE, SESSION_TAG, PORT
    ap = argparse.ArgumentParser()
    ap.add_argument("--simulate", action="store_true")
    ap.add_argument("--mode", choices=["angle", "distance"])
    ap.add_argument("--session")
    ap.add_argument("--port")
    args = ap.parse_args(argv)
    SIMULATE = SIMULATE or args.simulate
    SWEEP_MODE = args.mode or SWEEP_MODE
    SESSION_TAG = args.session or SESSION_TAG
    PORT = args.port or PORT

    if SIMULATE:
        SRC = SimSource()
        EMIT = SimEmitter(SRC)
    else:
        SRC = SerialSource(PORT, BAUD_RATE)
        EMIT = Emitter()
    try:
        FS_GLOBAL = calibrate_fs()
        if EXC_KIND == "chirp":
            f1 = min(EXC_F1_HZ, 0.42 * FS_GLOBAL)
            if f1 < 1.5 * EXC_F0_HZ:
                raise SystemExit(f"ADC fs {FS_GLOBAL:.0f} Hz too low for a {EXC_F0_HZ:.0f} Hz+ chirp.")
            SUBBAND_EDGES = np.geomspace(EXC_F0_HZ, f1, N_SUBBANDS + 1)
            WAVE = make_excitation(EXC_F0_HZ, f1)
            print(f"Excitation: log chirp {EXC_F0_HZ:.0f}-{f1:.0f} Hz, {EXC_DURATION_S * 1e3:.0f} ms; "
                  f"sub-bands {np.round(SUBBAND_EDGES).astype(int).tolist()} Hz")
        else:
            SUBBAND_EDGES = np.array([0.8 * EXC_TONE_HZ, 1.25 * EXC_TONE_HZ])
            WAVE = make_excitation(0, 0)
            print(f"Excitation: {EXC_TONE_HZ:.0f} Hz tone (near-omnidirectional for small drivers)")
        print(f"Mode: {SWEEP_MODE}  Session: {SESSION_TAG}\n")
        if SWEEP_MODE == "angle":
            run_sweep(ANGLE_SWEEP_POINTS, "theta_deg")
        else:
            run_sweep(DISTANCE_SWEEP_POINTS, "distance_cm")
    finally:
        EMIT.close()
        SRC.close()


if __name__ == "__main__":
    main()