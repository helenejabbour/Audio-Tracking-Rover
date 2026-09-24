"""
Directivity & range POC — sweep collector (v2).

PURPOSE (keep this in view — every design choice below serves it):
  Prove that RMS/peak amplitude, measured from a fixed mic position,
  changes in a distinguishable, repeatable way as SPEAKER DISTANCE and
  ANGLE change. That's the entire claim this POC needs to support before
  any rover/arm control work is justified. Nothing else matters yet -
  not sub-ms timing, not SNR-in-dB precision, not TDOA.

WHY THIS VERSION IS DIFFERENT FROM THE PREVIOUS ONE:
  The waveform diagnostic proved the impulse IS detected, strongly (a
  swing from ADC~75 to ADC~2008 - close to full scale). But it also
  showed the ACOUSTIC RESPONSE ARRIVING BEFORE t_trigger, which is
  physically impossible - it means t_trigger (from the audio driver
  callback) and the ADC sample timestamps (from batched serial reads)
  are on two clocks that don't agree, likely because serial reads
  timestamp BATCH ARRIVAL, not per-sample acquisition time.

  Rather than debug that clock alignment (irrelevant to THIS POC's
  goal), this version sidesteps it entirely:
    - Capture a generous window (before AND after each trigger).
    - Compute a baseline from the pre-trigger portion of THAT SAME
      window (not a separately-timed noise-floor call).
    - Find the impulse by searching the WHOLE window for the largest
      deviation from that baseline - wherever it falls in time.
    - Report peak deviation and RMS deviation as the two amplitude
      metrics used for angle/distance comparison.
  This is deliberately simpler and more robust than anything tried so
  far, because precise timing was never required for the actual goal.

WORKFLOW: same as before - set SWEEP_MODE, fill in sweep points, run,
follow prompts. Data appends to sweep_trials.csv.
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
# CONFIGURATION
# ============================================================
PORT = "COM4"
BAUD_RATE = 921600
V_REF = 3.3
ADC_MAX = 4095.0

N_TRIALS_PER_POINT = 12
CAPTURE_BEFORE_S = 0.10     # pre-trigger baseline window
CAPTURE_AFTER_S = 0.10      # post-trigger search window — generous on
                            # purpose since we no longer trust precise
                            # trigger-to-arrival alignment; the impulse
                            # will be somewhere in [-100ms, +100ms] and
                            # that's good enough for an amplitude compare
INTER_TRIAL_DELAY_S = 0.4

SWEEP_MODE = "angle"             # "angle" | "distance"
SESSION_TAG = "session1"
FIXED_DISTANCE_CM = 30.0
FIXED_ANGLE_DEG = 0.0

ANGLE_SWEEP_POINTS = [0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180]
DISTANCE_SWEEP_POINTS = [10, 20, 30, 40, 50, 70, 90, 120, 150]

TRIALS_LOG_PATH = "sweep_trials.csv"

# ============================================================
# Audio impulse (unchanged: 600 Hz, confirmed clearly detectable)
# ============================================================
AUDIO_FS = 44100
IMPULSE_DURATION = 0.10
IMPULSE_FREQ_HZ = 600
IMPULSE_TAU_S = 0.06
t_audio = np.linspace(0, IMPULSE_DURATION, int(AUDIO_FS * IMPULSE_DURATION))
_raw_wave = np.sin(2 * np.pi * IMPULSE_FREQ_HZ * t_audio) * np.exp(-t_audio / IMPULSE_TAU_S)
impulse_wave = (_raw_wave / np.max(np.abs(_raw_wave)) * 0.98).astype(np.float32)


def play_impulse():
    """Fire-and-forget playback. We no longer rely on this call's return
    time for alignment - see module docstring. All we need is for the
    sound to occur somewhere inside our capture window."""
    if HAS_SOUNDDEVICE:
        sd.play(impulse_wave, AUDIO_FS)
    else:
        winsound.Beep(3000, 20)


# ============================================================
# Serial setup
# ============================================================
ser = serial.Serial(PORT, BAUD_RATE, timeout=0.001)
time.sleep(3.0)
ser.reset_input_buffer()

print("Verifying serial stream before starting sweep...")
_verify_start = time.time()
_verify_bytes = 0
while time.time() - _verify_start < 1.0:
    if ser.in_waiting > 0:
        _verify_bytes += len(ser.read(ser.in_waiting))
if _verify_bytes == 0:
    raise RuntimeError(f"No data received on {PORT} at {BAUD_RATE} baud in 1s.")
print(f"  OK - received {_verify_bytes} bytes.\n")
ser.reset_input_buffer()

string_buffer = ""


def read_available_samples():
    """Non-blocking: returns any (timestamp, adc) pairs that arrived
    since the last call. Timestamps are batch-arrival times, not
    per-sample - fine for this POC since we only need relative
    ordering within a window, not absolute latency."""
    global string_buffer
    out = []
    if ser.in_waiting > 0:
        raw = ser.read(ser.in_waiting).decode("utf-8", errors="ignore")
        string_buffer += raw
        if "\n" in string_buffer:
            lines = string_buffer.split("\n")
            string_buffer = lines[-1]
            now = time.perf_counter()
            for line in lines[:-1]:
                v = line.strip()
                if v.isdigit():
                    val = int(v)
                    if 0 <= val <= 4095:
                        out.append((now, val))
    return out


def run_single_trial(theta_deg, distance_cm):
    """Captures CAPTURE_BEFORE_S before triggering and CAPTURE_AFTER_S
    after, then searches the ENTIRE combined window for the largest
    deviation from the pre-trigger baseline - see module docstring for
    why this replaces trigger-timestamp-aligned windowing."""
    samples = []

    t_pre_start = time.perf_counter()
    while time.perf_counter() - t_pre_start < CAPTURE_BEFORE_S:
        samples.extend(read_available_samples())

    if len(samples) < 10:
        print(f"  [WARN] Only {len(samples)} pre-trigger samples — weak serial throughput.")
        return None

    baseline_adc = np.array([s[1] for s in samples])
    baseline_mean = baseline_adc.mean()
    baseline_ac_mv = (baseline_adc - baseline_mean) / ADC_MAX * V_REF * 1000.0
    noise_floor_rms_mv = float(np.sqrt(np.mean(baseline_ac_mv ** 2)))

    play_impulse()

    t_post_start = time.perf_counter()
    while time.perf_counter() - t_post_start < CAPTURE_AFTER_S:
        samples.extend(read_available_samples())

    if len(samples) < 20:
        print(f"  [WARN] Only {len(samples)} total samples — capture too sparse.")
        return None

    all_adc = np.array([s[1] for s in samples])
    # Deviation from the PRE-TRIGGER baseline mean, computed once above -
    # this is the reference point regardless of where in time the actual
    # impulse response lands, sidestepping the broken t_trigger alignment.
    ac_mv = (all_adc - baseline_mean) / ADC_MAX * V_REF * 1000.0

    peak_idx = np.argmax(np.abs(ac_mv))
    peak_ac_mv = float(ac_mv[peak_idx])
    rms_ac_mv = float(np.sqrt(np.mean(ac_mv ** 2)))

    if noise_floor_rms_mv > 0:
        snr_db = 20.0 * np.log10(max(abs(peak_ac_mv), 1e-9) / noise_floor_rms_mv)
    else:
        snr_db = float("nan")

    return {
        "theta_deg": theta_deg,
        "distance_cm": distance_cm,
        "peak_ac_mv": peak_ac_mv,
        "rms_ac_mv": rms_ac_mv,
        "noise_floor_mv": noise_floor_rms_mv,
        "snr_db": snr_db,
        "n_samples": len(samples),
    }


def ensure_csv_header(path, fieldnames):
    if not os.path.exists(path):
        with open(path, "w", newline="") as f:
            csv.DictWriter(f, fieldnames=fieldnames).writeheader()


def run_sweep(sweep_points, sweep_variable):
    fieldnames = ["session", "mode", "trial_in_point", "theta_deg", "distance_cm",
                  "peak_ac_mv", "rms_ac_mv", "noise_floor_mv", "snr_db",
                  "n_samples", "wall_time_s"]
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
                print(f"peak={r['peak_ac_mv']:+.1f}mV  RMS={r['rms_ac_mv']:.1f}mV  SNR={r['snr_db']:.1f}dB")
                results.append(r)
                with open(TRIALS_LOG_PATH, "a", newline="") as f:
                    row = {
                        "session": SESSION_TAG, "mode": sweep_variable,
                        "trial_in_point": i, "wall_time_s": f"{time.perf_counter() - t_start:.2f}",
                        **{k: (f"{v:.4f}" if isinstance(v, float) else v) for k, v in r.items()}
                    }
                    csv.DictWriter(f, fieldnames=fieldnames).writerow(row)
            else:
                print("SKIPPED")
            time.sleep(INTER_TRIAL_DELAY_S)

        if results:
            rms_vals = [r["rms_ac_mv"] for r in results]
            print(f"  --> point summary: mean RMS={np.mean(rms_vals):.1f}mV  "
                f"std={np.std(rms_vals):.1f}mV  n={len(results)}")

    print(f"\nSweep complete. Data appended to {os.path.abspath(TRIALS_LOG_PATH)}")


if __name__ == "__main__":
    print(f"Mode: {SWEEP_MODE}  Session: {SESSION_TAG}")
    try:
        if SWEEP_MODE == "angle":
            run_sweep(ANGLE_SWEEP_POINTS, "theta_deg")
        elif SWEEP_MODE == "distance":
            run_sweep(DISTANCE_SWEEP_POINTS, "distance_cm")
        else:
            print(f"Unknown SWEEP_MODE '{SWEEP_MODE}'.")
            sys.exit(1)
    finally:
        ser.close()