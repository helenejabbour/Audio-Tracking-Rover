"""
Directivity & range POC — structured data collector.

Implements the sweep protocol from the POC design:
  - Orthogonal sweeps: angle sweep at fixed distance, distance sweep at
    fixed (best) angle. NOT swept simultaneously - confounds angle and
    distance effects on amplitude if done together.
  - N >= 10 repeated trials per (theta, d) point.
  - Pre-trigger noise floor logged with every trial (for SNR).
  - Repeatability re-run support: same sweep can be logged under a
    different `session` tag and compared later in the analysis script.
  - Continuous stability logging mode: fixed point, long duration, no
    discrete triggering - written as a separate run mode since it has a
    different data shape (continuous stream, not per-trial peaks).

This script only ACQUIRES and LOGS. All plots/tables/metrics are in
analyze_directivity_poc.py, run separately after data collection - this
separation means you can re-analyze without re-running hardware trials,
and re-run trials without re-deriving plotting code.

WORKFLOW:
  1. Set SWEEP_MODE below ("angle", "distance", or "stability").
  2. For angle/distance sweeps: fill SWEEP_POINTS with the (label, value)
     pairs you'll physically set the rig to. The script prompts you to
     confirm each physical position before triggering its trial batch -
     it does NOT move anything for you (no actuation hardware assumed
     at this POC stage).
  3. Run. Follow the prompts. Data appends to sweep_trials.csv (or
     stability_log.csv for stability mode) with a session/mode tag so
     multiple sweeps coexist in one file for later comparison.
"""

from collections import deque
import time
import csv
import os
import sys
import numpy as np
import serial

try:
    import sounddevice as sd
    HAS_SOUNDDEVICE = True
except ImportError:
    import winsound
    HAS_SOUNDDEVICE = False

# ============================================================
# CONFIGURATION — edit per run
# ============================================================
PORT = "COM4"
BAUD_RATE = 921600
SAMPLING_FREQ_HZ = 10000
V_REF = 3.3
ADC_MAX = 4095.0
BIAS_ADC = 2048.0

N_TRIALS_PER_POINT = 12          # >= 10 per POC design
OBS_WINDOW_S = 0.300
PRE_WINDOW_S = 0.050
INTER_TRIAL_DELAY_S = 0.5        # settle time between trials at same point

SWEEP_MODE = "angle"             # "angle" | "distance" | "stability"
SESSION_TAG = "session1"         # change for repeatability re-runs (e.g. "session2")
FIXED_DISTANCE_CM = 30.0         # used when SWEEP_MODE == "angle"
FIXED_ANGLE_DEG = 0.0            # used when SWEEP_MODE == "distance"

# Angle sweep points (degrees) — edit to match your servo/protractor range
ANGLE_SWEEP_POINTS = [0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180]

# Distance sweep points (cm) — edit to match your bench range
DISTANCE_SWEEP_POINTS = [10, 20, 30, 40, 50, 70, 90, 120, 150]

STABILITY_DURATION_S = 300       # 5 min continuous log for stability mode
STABILITY_TRIGGER_INTERVAL_S = 5  # re-trigger impulse every N seconds during stability run

TRIALS_LOG_PATH = "sweep_trials.csv"
STABILITY_LOG_PATH = "stability_log.csv"

# ============================================================
# Serial + audio setup
# ============================================================
ser = serial.Serial(PORT, BAUD_RATE, timeout=0.001)
ser.reset_input_buffer()

AUDIO_FS = 44100
IMPULSE_DURATION = 0.015
IMPULSE_FREQ_HZ = 3000
IMPULSE_TAU_S = 0.003
t_audio = np.linspace(0, IMPULSE_DURATION, int(AUDIO_FS * IMPULSE_DURATION))
_raw_wave = np.sin(2 * np.pi * IMPULSE_FREQ_HZ * t_audio) * np.exp(-t_audio / IMPULSE_TAU_S)
impulse_wave = (_raw_wave / np.max(np.abs(_raw_wave)) * 0.98).astype(np.float32)


def play_audio_impulse_timestamped():
    """Returns perf_counter() timestamped inside the audio driver callback
    at the first block handed to the driver - removes click/call-site
    jitter from the trigger reference. See prior sessions for full
    rationale; unchanged from the validated single-mic POC."""
    first_call_ts = {"t": None}

    def callback(outdata, frames, time_info, status):
        if first_call_ts["t"] is None:
            first_call_ts["t"] = time.perf_counter()
        chunk = impulse_wave[callback.pos: callback.pos + frames]
        if len(chunk) < frames:
            outdata[:len(chunk), 0] = chunk
            outdata[len(chunk):, 0] = 0.0
            raise sd.CallbackStop
        outdata[:, 0] = chunk
        callback.pos += frames

    callback.pos = 0

    if HAS_SOUNDDEVICE:
        stream = sd.OutputStream(samplerate=AUDIO_FS, channels=1, callback=callback, blocksize=256)
        stream.start()
        while first_call_ts["t"] is None:
            time.sleep(0.0005)
        return first_call_ts["t"]
    else:
        ts = time.perf_counter()
        winsound.Beep(3000, 20)
        return ts


# ============================================================
# Acquisition buffers (mirrors validated single-mic POC structure)
# ============================================================
timed_samples = deque(maxlen=SAMPLING_FREQ_HZ * 2)
string_buffer = ""


def drain_serial_into_buffer():
    """Non-blocking read of whatever's arrived; appends (timestamp, adc)
    pairs to timed_samples. Call this in a tight loop while waiting."""
    global string_buffer
    if ser.in_waiting > 0:
        raw = ser.read(ser.in_waiting).decode("utf-8", errors="ignore")
        string_buffer += raw
        if "\n" in string_buffer:
            lines = string_buffer.split("\n")
            string_buffer = lines[-1]
            now = time.perf_counter()
            for line in lines[:-1]:
                val_str = line.strip()
                if val_str.isdigit():
                    raw_adc = int(val_str)
                    if 0 <= raw_adc <= 4095:
                        timed_samples.append((now, raw_adc))


def compute_noise_floor_mv(reference_time, window_s=PRE_WINDOW_S):
    baseline = [adc for (ts, adc) in timed_samples if reference_time - ts <= window_s and ts <= reference_time]
    if len(baseline) < 5:
        return float("nan")
    baseline_ac_mv = (np.array(baseline) - BIAS_ADC) / ADC_MAX * V_REF * 1000.0
    return float(np.sqrt(np.mean(baseline_ac_mv ** 2)))


def run_single_trial(theta_deg, distance_cm):
    """Runs one triggered impulse + window capture. Returns a dict of
    metrics, or None if the window was too sparse to trust."""
    # Drain any stale backlog and let a fresh baseline accumulate briefly
    for _ in range(20):
        drain_serial_into_buffer()
        time.sleep(0.005)

    now = time.perf_counter()
    noise_floor_mv = compute_noise_floor_mv(now)

    ser.reset_input_buffer()
    t_trigger = play_audio_impulse_timestamped()

    # Actively drain serial until the observation window has elapsed
    while time.perf_counter() - t_trigger <= OBS_WINDOW_S:
        drain_serial_into_buffer()

    window = [(ts, adc) for (ts, adc) in timed_samples if 0.0 <= (ts - t_trigger) <= OBS_WINDOW_S]
    if len(window) < 5:
        print(f"  [WARN] Sparse window ({len(window)} samples) — check serial throughput.")
        return None

    ts_arr = np.array([w[0] for w in window])
    adc_arr = np.array([w[1] for w in window])
    ac_mv = (adc_arr - BIAS_ADC) / ADC_MAX * V_REF * 1000.0

    peak_idx = np.argmax(np.abs(ac_mv))
    peak_ac_mv = ac_mv[peak_idx]
    onset_delay_ms = (ts_arr[peak_idx] - t_trigger) * 1000.0
    rms_ac_mv = float(np.sqrt(np.mean(ac_mv ** 2)))

    if noise_floor_mv and noise_floor_mv > 0:
        snr_db = 20.0 * np.log10(max(abs(peak_ac_mv), 1e-9) / noise_floor_mv)
    else:
        snr_db = float("nan")

    return {
        "theta_deg": theta_deg,
        "distance_cm": distance_cm,
        "peak_ac_mv": peak_ac_mv,
        "rms_ac_mv": rms_ac_mv,
        "noise_floor_mv": noise_floor_mv,
        "snr_db": snr_db,
        "onset_delay_ms": onset_delay_ms,
        "n_samples": len(window),
    }


def ensure_csv_header(path, fieldnames):
    if not os.path.exists(path):
        with open(path, "w", newline="") as f:
            csv.DictWriter(f, fieldnames=fieldnames).writeheader()


def run_sweep(sweep_points, sweep_variable):
    """sweep_variable: 'theta_deg' or 'distance_cm'. The OTHER variable is
    held at its FIXED_* value throughout, per the orthogonal-sweep design."""
    fieldnames = ["session", "mode", "trial_in_point", "theta_deg", "distance_cm",
                  "peak_ac_mv", "rms_ac_mv", "noise_floor_mv", "snr_db",
                  "onset_delay_ms", "n_samples", "wall_time_s"]
    ensure_csv_header(TRIALS_LOG_PATH, fieldnames)

    t_start = time.perf_counter()

    for point_value in sweep_points:
        if sweep_variable == "theta_deg":
            theta, dist = point_value, FIXED_DISTANCE_CM
            prompt = f"Set mic angle to {theta} deg (distance fixed at {dist} cm)"
        else:
            theta, dist = FIXED_ANGLE_DEG, point_value
            prompt = f"Set distance to {dist} cm (angle fixed at {theta} deg)"

        input(f"\n>>> {prompt}. Press ENTER when in position...")

        results = []
        for i in range(N_TRIALS_PER_POINT):
            print(f"  Trial {i+1}/{N_TRIALS_PER_POINT} @ theta={theta}, d={dist} ...", end=" ")
            r = run_single_trial(theta, dist)
            if r is not None:
                print(f"RMS={r['rms_ac_mv']:.1f}mV  SNR={r['snr_db']:.1f}dB")
                results.append(r)
                with open(TRIALS_LOG_PATH, "a", newline="") as f:
                    row = {
                        "session": SESSION_TAG, "mode": sweep_variable,
                        "trial_in_point": i, "wall_time_s": f"{time.perf_counter() - t_start:.2f}",
                        **{k: (f"{v:.4f}" if isinstance(v, float) else v) for k, v in r.items()}
                    }
                    csv.DictWriter(f, fieldnames=fieldnames).writerow(row)
            else:
                print("SKIPPED (sparse window)")
            time.sleep(INTER_TRIAL_DELAY_S)

        if results:
            rms_vals = [r["rms_ac_mv"] for r in results]
            print(f"  --> point summary: mean RMS={np.mean(rms_vals):.1f}mV  "
                  f"std={np.std(rms_vals):.1f}mV  n={len(results)}")

    print(f"\nSweep complete. Data appended to {os.path.abspath(TRIALS_LOG_PATH)}")


def run_stability_log():
    """Continuous fixed-point logging: periodic triggered impulses over a
    long duration, to check for drift/intermittent faults that discrete
    short sweeps wouldn't reveal."""
    fieldnames = ["session", "theta_deg", "distance_cm", "trigger_index",
                  "peak_ac_mv", "rms_ac_mv", "noise_floor_mv", "snr_db",
                  "onset_delay_ms", "wall_time_s"]
    ensure_csv_header(STABILITY_LOG_PATH, fieldnames)

    input(f"\n>>> Set fixed point: theta={FIXED_ANGLE_DEG} deg, d={FIXED_DISTANCE_CM} cm. "
          f"Press ENTER to begin {STABILITY_DURATION_S}s stability run...")

    t_start = time.perf_counter()
    trigger_index = 0

    while time.perf_counter() - t_start < STABILITY_DURATION_S:
        r = run_single_trial(FIXED_ANGLE_DEG, FIXED_DISTANCE_CM)
        wall_t = time.perf_counter() - t_start
        if r is not None:
            print(f"  t={wall_t:6.1f}s  RMS={r['rms_ac_mv']:.1f}mV  SNR={r['snr_db']:.1f}dB")
            with open(STABILITY_LOG_PATH, "a", newline="") as f:
                row = {
                    "session": SESSION_TAG, "trigger_index": trigger_index,
                    "wall_time_s": f"{wall_t:.2f}",
                    **{k: (f"{v:.4f}" if isinstance(v, float) else v) for k, v in r.items()}
                }
                csv.DictWriter(f, fieldnames=fieldnames).writerow(row)
        trigger_index += 1
        time.sleep(max(0.0, STABILITY_TRIGGER_INTERVAL_S - OBS_WINDOW_S))

    print(f"\nStability run complete. Data written to {os.path.abspath(STABILITY_LOG_PATH)}")


if __name__ == "__main__":
    print(f"Mode: {SWEEP_MODE}  Session: {SESSION_TAG}")
    try:
        if SWEEP_MODE == "angle":
            run_sweep(ANGLE_SWEEP_POINTS, "theta_deg")
        elif SWEEP_MODE == "distance":
            run_sweep(DISTANCE_SWEEP_POINTS, "distance_cm")
        elif SWEEP_MODE == "stability":
            run_stability_log()
        else:
            print(f"Unknown SWEEP_MODE '{SWEEP_MODE}' — must be 'angle', 'distance', or 'stability'.")
            sys.exit(1)
    finally:
        ser.close()
