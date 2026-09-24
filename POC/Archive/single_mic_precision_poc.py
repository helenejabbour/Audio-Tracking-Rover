"""
Single-mic delay precision POC.

Goal: before building the 2-mic TDOA array, verify that click-independent
onset-delay measurements (driver-callback-timestamped, per the earlier fix)
are:
  1. REPEATABLE at a fixed distance - stddev should be small relative to
     the delay CHANGE you expect between your test distances.
  2. LINEAR in distance - slope should be close to 1/c = 2.915 us/cm,
     intercept is your fixed (distance-independent) latency floor
     (DAC->speaker->diaphragm->ADC->buffer chain).

Workflow:
  1. Set the mic at a fixed distance from the laptop speaker.
  2. Enter that distance in the textbox (cm).
  3. Click "Trigger Impulse" N times (default 10) WITHOUT moving anything.
  4. Move the mic to the next distance, repeat.
  5. Click "Analyze & Plot" to see mean +/- stddev per distance and a
     linear fit overlaid, printed with the residual from theoretical c.

All raw trials are logged to CSV regardless of when you analyze, so you
can re-run analysis on old data too.
"""

from collections import deque
import time
import csv
import os
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, TextBox
import numpy as np
import serial

try:
    import sounddevice as sd
    HAS_SOUNDDEVICE = True
except ImportError:
    import winsound
    HAS_SOUNDDEVICE = False

# --- Configuration ---
PORT = "COM4"
BAUD_RATE = 921600  # must match Serial.begin() in the ESP32-S3 firmware
BUFFER_SIZE = 500
SAMPLING_FREQ_HZ = 10000
V_REF = 3.3
ADC_MAX = 4095.0
BIAS_ADC = 2048.0
SPEED_OF_SOUND_M_S = 343.0

ser = serial.Serial(PORT, BAUD_RATE, timeout=0.001)

dt_ms = (1.0 / SAMPLING_FREQ_HZ) * 1000.0
time_axis = np.linspace(-(BUFFER_SIZE - 1) * dt_ms, 0, BUFFER_SIZE)
adc_buffer = deque([BIAS_ADC] * BUFFER_SIZE, maxlen=BUFFER_SIZE)
timed_samples = deque(maxlen=SAMPLING_FREQ_HZ * 2)

# --- Impulse Sound Generator ---
AUDIO_FS = 44100
IMPULSE_DURATION = 0.015
IMPULSE_FREQ_HZ = 3000
IMPULSE_TAU_S = 0.003
t_audio = np.linspace(0, IMPULSE_DURATION, int(AUDIO_FS * IMPULSE_DURATION))
_raw_wave = np.sin(2 * np.pi * IMPULSE_FREQ_HZ * t_audio) * np.exp(-t_audio / IMPULSE_TAU_S)
impulse_wave = (_raw_wave / np.max(np.abs(_raw_wave)) * 0.98).astype(np.float32)


def play_audio_impulse_timestamped():
    """Returns perf_counter() timestamped inside the audio callback at the
    first block handed to the driver - removes click-to-driver-queue
    jitter (10-40 ms) from the delay measurement. See prior discussion:
    this does NOT remove DAC->speaker->air latency (fixed, not jitter)
    or USB-serial poll granularity (~20 ms, addressed by trial averaging
    here rather than eliminated)."""
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


# --- Global State ---
is_running = True
string_buffer = ""
is_tracking = False
t_trigger = 0.0
pre_trigger_noise_floor_mv = 0.0
obs_window_s = 0.300
pre_window_s = 0.050
trial_counter = 0
DISTANCE_CM = 10.0  # set via textbox before each batch of trials

LOG_PATH = "poc_delay_trials.csv"
if not os.path.exists(LOG_PATH):
    with open(LOG_PATH, "w", newline="") as f:
        csv.writer(f).writerow([
            "trial", "distance_cm", "peak_ac_mv", "rms_ac_mv",
            "noise_floor_rms_mv", "snr_db", "onset_delay_ms", "n_samples_in_window"
        ])

# --- Figure and UI ---
fig, ax_voltage = plt.subplots(figsize=(11, 6))
plt.subplots_adjust(bottom=0.26)

(line_v,) = ax_voltage.plot(time_axis, [(BIAS_ADC / ADC_MAX) * V_REF] * BUFFER_SIZE,
                             color="#007acc", lw=1.0, label="Instantaneous Signal")
ax_voltage.set_title("Single-Mic Delay Precision POC", fontsize=12, fontweight="bold")
ax_voltage.set_xlabel("Time relative to present [ms]")
ax_voltage.set_ylabel("Absolute Voltage [V]", color="#007acc")
ax_voltage.set_ylim(0.0, V_REF)
ax_voltage.axhline(1.65, color="red", linestyle="--", linewidth=0.8, label="1.65V DC Bias")
ax_voltage.tick_params(axis="y", labelcolor="#007acc")
ax_voltage.grid(True, linestyle=":", alpha=0.6)

ax_mv = ax_voltage.twinx()
ax_mv.set_ylabel("AC Amplitude [mV]", color="#d9534f")
ax_mv.set_ylim(-1650.0, 1650.0)
ax_mv.tick_params(axis="y", labelcolor="#d9534f")
ax_mv.grid(False)
ax_voltage.legend(loc="upper left")

ax_btn_run = plt.axes([0.08, 0.03, 0.12, 0.05])
ax_btn_trigger = plt.axes([0.22, 0.03, 0.18, 0.05])
ax_dist_box = plt.axes([0.45, 0.03, 0.15, 0.05])
ax_btn_analyze = plt.axes([0.65, 0.03, 0.20, 0.05])

btn_toggle = Button(ax_btn_run, "Stop", color="#d9534f", hovercolor="#c9302c")
btn_trigger = Button(ax_btn_trigger, "Trigger Impulse", color="#0275d8", hovercolor="#025aa5")
dist_box = TextBox(ax_dist_box, "Dist [cm]: ", initial=str(DISTANCE_CM))
btn_analyze = Button(ax_btn_analyze, "Analyze & Plot", color="#5cb85c", hovercolor="#449d44")


def on_distance_change(text):
    global DISTANCE_CM
    try:
        DISTANCE_CM = float(text.strip())
    except ValueError:
        print(f"[WARN] '{text}' is not a valid number, distance unchanged ({DISTANCE_CM} cm).")


dist_box.on_submit(on_distance_change)
dist_box.on_text_change(on_distance_change)


def toggle_run(event):
    global is_running
    is_running = not is_running
    if is_running:
        btn_toggle.label.set_text("Stop")
        btn_toggle.ax.set_facecolor("#d9534f")
        ser.reset_input_buffer()
    else:
        btn_toggle.label.set_text("Run")
        btn_toggle.ax.set_facecolor("#5cb85c")
    fig.canvas.draw_idle()


def trigger_impulse(event):
    global is_tracking, t_trigger, pre_trigger_noise_floor_mv

    if not is_running:
        print("[WARN] Start the plotter before triggering impulse.")
        return
    if len(timed_samples) < 10:
        print("[WARN] Not enough buffered samples yet for a noise-floor estimate.")
        return

    now = time.perf_counter()
    baseline = [adc for (ts, adc) in timed_samples if now - ts <= pre_window_s]
    if len(baseline) >= 5:
        baseline_ac_mv = (np.array(baseline) - BIAS_ADC) / ADC_MAX * V_REF * 1000.0
        pre_trigger_noise_floor_mv = float(np.sqrt(np.mean(baseline_ac_mv ** 2)))
    else:
        pre_trigger_noise_floor_mv = float("nan")

    ser.reset_input_buffer()
    t_trigger = play_audio_impulse_timestamped()
    is_tracking = True

    print(f"--- [TRIGGER] distance={DISTANCE_CM:.1f} cm  "
          f"noise_floor={pre_trigger_noise_floor_mv:.2f} mV ---")


def analyze_and_plot(event):
    if not os.path.exists(LOG_PATH):
        print("[WARN] No logged trials yet.")
        return

    distances, delays = [], []
    with open(LOG_PATH, "r") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    if len(rows) < 2:
        print("[WARN] Not enough trials logged yet.")
        return

    by_distance = {}
    for row in rows:
        d = float(row["distance_cm"])
        t = float(row["onset_delay_ms"])
        by_distance.setdefault(d, []).append(t)

    print("\n=== Precision Summary (per distance) ===")
    means, stds, dists_sorted = [], [], []
    for d in sorted(by_distance.keys()):
        vals = np.array(by_distance[d])
        mean_ms = vals.mean()
        std_ms = vals.std()
        cv_pct = (std_ms / abs(mean_ms) * 100) if mean_ms != 0 else float("nan")
        print(f"  {d:6.1f} cm : n={len(vals):3d}  mean={mean_ms:8.4f} ms  "
              f"std={std_ms:7.4f} ms  CV={cv_pct:5.1f}%")
        dists_sorted.append(d)
        means.append(mean_ms)
        stds.append(std_ms)

    dists_arr = np.array(dists_sorted)
    means_arr = np.array(means)
    stds_arr = np.array(stds)

    if len(dists_arr) >= 2:
        # Linear fit: delay_ms = slope * distance_cm + intercept
        slope, intercept = np.polyfit(dists_arr, means_arr, 1)
        theoretical_slope_ms_per_cm = (1.0 / SPEED_OF_SOUND_M_S) * 0.01 * 1000.0  # ~0.0029 ms/cm
        print(f"\nLinear fit: delay = {slope:.5f} * distance + {intercept:.4f}  [ms, cm]")
        print(f"Theoretical acoustic slope (1/c): {theoretical_slope_ms_per_cm:.5f} ms/cm")
        print(f"Fitted intercept = fixed latency floor (DAC->speaker->air->ADC chain): {intercept:.4f} ms")

        residual = slope - theoretical_slope_ms_per_cm
        print(f"Slope deviation from theory: {residual:+.5f} ms/cm "
              f"({'plausible' if abs(residual) < theoretical_slope_ms_per_cm else 'LARGE - check setup'})")

        avg_cv = np.nanmean([s / abs(m) * 100 if m != 0 else np.nan for s, m in zip(stds_arr, means_arr)])
        expected_delay_change_per_cm_ms = theoretical_slope_ms_per_cm
        print(f"\nPrecision check: average stddev = {stds_arr.mean():.4f} ms.")
        print(f"  Expected delay change per cm of movement = {expected_delay_change_per_cm_ms:.4f} ms.")
        if stds_arr.mean() > expected_delay_change_per_cm_ms:
            print("  ⚠ Your trial-to-trial noise is LARGER than the signal from a 1cm move.")
            print("    You will not resolve cm-scale distance changes with this setup as-is.")
        else:
            resolvable_cm = stds_arr.mean() / expected_delay_change_per_cm_ms
            print(f"  Estimated minimum resolvable distance change: ~{resolvable_cm:.2f} cm "
                  f"(1-sigma, per-trial; average multiple trials to do better by sqrt(N)).")

        fig2, ax2 = plt.subplots(figsize=(8, 6))
        ax2.errorbar(dists_arr, means_arr, yerr=stds_arr, fmt="o", color="#007acc",
                     capsize=4, label="Measured (mean ± std)")
        fit_x = np.linspace(dists_arr.min(), dists_arr.max(), 100)
        ax2.plot(fit_x, slope * fit_x + intercept, "--", color="#d9534f",
                 label=f"Linear fit: {slope:.4f}·d + {intercept:.3f}")
        ax2.set_xlabel("Distance [cm]")
        ax2.set_ylabel("Onset delay [ms]")
        ax2.set_title("Delay vs. Distance — Precision & Linearity Check")
        ax2.legend()
        ax2.grid(True, linestyle=":", alpha=0.6)
        plt.show()


btn_toggle.on_clicked(toggle_run)
btn_trigger.on_clicked(trigger_impulse)
btn_analyze.on_clicked(analyze_and_plot)


def process_window():
    global trial_counter

    window = [(ts, adc) for (ts, adc) in timed_samples if 0.0 <= (ts - t_trigger) <= obs_window_s]
    if len(window) < 5:
        print("[WARN] Window too sparse — check serial throughput / baud rate.")
        return

    ts_arr = np.array([w[0] for w in window])
    adc_arr = np.array([w[1] for w in window])
    ac_mv = (adc_arr - BIAS_ADC) / ADC_MAX * V_REF * 1000.0

    peak_idx = np.argmax(np.abs(ac_mv))
    peak_ac_mv = ac_mv[peak_idx]
    onset_delay_ms = (ts_arr[peak_idx] - t_trigger) * 1000.0
    rms_ac_mv = float(np.sqrt(np.mean(ac_mv ** 2)))

    if pre_trigger_noise_floor_mv and pre_trigger_noise_floor_mv > 0:
        snr_db = 20.0 * np.log10(abs(peak_ac_mv) / pre_trigger_noise_floor_mv)
    else:
        snr_db = float("nan")

    trial_counter += 1

    print(f"Trial #{trial_counter}  [{DISTANCE_CM:.1f} cm]")
    print(f"  Peak AC Amplitude   : {peak_ac_mv:+.1f} mV")
    print(f"  RMS AC Amplitude    : {rms_ac_mv:.1f} mV")
    print(f"  Noise Floor (pre)   : {pre_trigger_noise_floor_mv:.2f} mV RMS")
    print(f"  SNR                 : {snr_db:.1f} dB")
    print(f"  Onset Delay         : {onset_delay_ms:.4f} ms")
    print(f"  Samples in window   : {len(window)} / expected ~{int(obs_window_s * SAMPLING_FREQ_HZ)}")
    print("-" * 50)

    with open(LOG_PATH, "a", newline="") as f:
        csv.writer(f).writerow([
            trial_counter, f"{DISTANCE_CM:.1f}", f"{peak_ac_mv:.2f}", f"{rms_ac_mv:.2f}",
            f"{pre_trigger_noise_floor_mv:.2f}", f"{snr_db:.2f}", f"{onset_delay_ms:.4f}", len(window)
        ])


def update_frame(frame):
    global string_buffer, is_tracking

    if ser.in_waiting > 0:
        raw_data = ser.read(ser.in_waiting).decode("utf-8", errors="ignore")
        string_buffer += raw_data

        if "\n" in string_buffer:
            lines = string_buffer.split("\n")
            string_buffer = lines[-1]
            now = time.perf_counter()

            if is_running:
                for line in lines[:-1]:
                    val_str = line.strip()
                    if val_str.isdigit():
                        raw_adc = int(val_str)
                        if 0 <= raw_adc <= 4095:
                            adc_buffer.append(raw_adc)
                            timed_samples.append((now, raw_adc))

            if is_tracking and (time.perf_counter() - t_trigger > obs_window_s):
                is_tracking = False
                process_window()

    if is_running:
        voltage_array = (np.array(adc_buffer) / ADC_MAX) * V_REF
        line_v.set_ydata(voltage_array)

    return (line_v,)


ani = FuncAnimation(fig, update_frame, interval=20, blit=True, cache_frame_data=False)

try:
    plt.show()
finally:
    ser.close()
    print(f"\nLogged trials written to {os.path.abspath(LOG_PATH)}")
