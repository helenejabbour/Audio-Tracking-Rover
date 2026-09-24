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
BAUD_RATE = 115200
BUFFER_SIZE = 500
SAMPLING_FREQ_HZ = 10000
V_REF = 3.3
ADC_MAX = 4095.0
BIAS_ADC = 2048.0  # nominal 1.65V bias point

ser = serial.Serial(PORT, BAUD_RATE, timeout=0.001)

dt_ms = (1.0 / SAMPLING_FREQ_HZ) * 1000.0
time_axis = np.linspace(-(BUFFER_SIZE - 1) * dt_ms, 0, BUFFER_SIZE)
adc_buffer = deque([BIAS_ADC] * BUFFER_SIZE, maxlen=BUFFER_SIZE)

# Each entry: (perf_counter_timestamp, raw_adc). Kept separately from the
# plot ring buffer so windowing math isn't coupled to display resolution.
timed_samples = deque(maxlen=SAMPLING_FREQ_HZ * 2)  # ~2s of history

# --- Impulse Sound Generator ---
AUDIO_FS = 44100
IMPULSE_DURATION = 0.030
t_audio = np.linspace(0, IMPULSE_DURATION, int(AUDIO_FS * IMPULSE_DURATION))
impulse_wave = (
    np.sin(2 * np.pi * 3000 * t_audio) * np.exp(-t_audio / 0.003)
).astype(np.float32)


def play_audio_impulse():
    if HAS_SOUNDDEVICE:
        sd.play(impulse_wave, AUDIO_FS)
    else:
        winsound.Beep(3000, 20)


# --- Global State ---
is_running = True
string_buffer = ""

is_tracking = False
t_trigger = 0.0
pre_trigger_noise_floor_mv = 0.0  # RMS of PRE_WINDOW_S before trigger
obs_window_s = 0.300
pre_window_s = 0.050  # 50 ms baseline capture before trigger

trial_counter = 0
LABEL = "unlabeled"  # set via textbox: e.g. "0deg_10cm"

LOG_PATH = "mic_trials.csv"
if not os.path.exists(LOG_PATH):
    with open(LOG_PATH, "w", newline="") as f:
        csv.writer(f).writerow([
            "trial", "label", "peak_ac_mv", "rms_ac_mv",
            "noise_floor_rms_mv", "snr_db", "onset_delay_ms",
            "n_samples_in_window"
        ])

# --- Figure and UI ---
fig, ax_voltage = plt.subplots(figsize=(11, 6))
plt.subplots_adjust(bottom=0.26)

(line_v,) = ax_voltage.plot(
    time_axis, [(BIAS_ADC / ADC_MAX) * V_REF] * BUFFER_SIZE,
    color="#007acc", lw=1.0, label="Instantaneous Signal",
)
ax_voltage.set_title("MAX4466 Impulse Response — Orientation/Distance Sweep", fontsize=12, fontweight="bold")
ax_voltage.set_xlabel("Time relative to present [ms]", fontsize=10)
ax_voltage.set_ylabel("Absolute Voltage [V]", fontsize=10, color="#007acc")
ax_voltage.set_ylim(0.0, V_REF)
ax_voltage.axhline(1.65, color="red", linestyle="--", linewidth=0.8, label="1.65V DC Bias")
ax_voltage.tick_params(axis="y", labelcolor="#007acc")
ax_voltage.grid(True, linestyle=":", alpha=0.6)

ax_mv = ax_voltage.twinx()
ax_mv.set_ylabel("AC Amplitude [mV]", fontsize=10, color="#d9534f")
ax_mv.set_ylim(-1650.0, 1650.0)
ax_mv.tick_params(axis="y", labelcolor="#d9534f")
ax_mv.grid(False)
ax_voltage.legend(loc="upper left")

ax_btn_run = plt.axes([0.15, 0.03, 0.13, 0.05])
ax_btn_trigger = plt.axes([0.30, 0.03, 0.18, 0.05])
ax_label_box = plt.axes([0.55, 0.03, 0.30, 0.05])

btn_toggle = Button(ax_btn_run, "Stop", color="#d9534f", hovercolor="#c9302c")
btn_trigger = Button(ax_btn_trigger, "Trigger Impulse", color="#0275d8", hovercolor="#025aa5")
label_box = TextBox(ax_label_box, "Label: ", initial=LABEL)


def on_label_change(text):
    global LABEL
    LABEL = text.strip() if text.strip() else "unlabeled"


label_box.on_submit(on_label_change)
label_box.on_text_change(on_label_change)


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

    # Baseline noise floor from samples already in hand (no acoustic assumption)
    baseline = [adc for (ts, adc) in timed_samples if now - ts <= pre_window_s]
    if len(baseline) >= 5:
        baseline_ac_mv = (np.array(baseline) - BIAS_ADC) / ADC_MAX * V_REF * 1000.0
        pre_trigger_noise_floor_mv = float(np.sqrt(np.mean(baseline_ac_mv ** 2)))
    else:
        pre_trigger_noise_floor_mv = float("nan")

    ser.reset_input_buffer()
    t_trigger = now
    is_tracking = True

    play_audio_impulse()
    print(f"\n--- [IMPULSE TRIGGERED] label='{LABEL}' "
          f"noise_floor_rms={pre_trigger_noise_floor_mv:.2f} mV ---")


btn_toggle.on_clicked(toggle_run)
btn_trigger.on_clicked(trigger_impulse)


def process_window():
    """Called once the observation window has closed. Pulls every
    (timestamp, adc) pair recorded strictly after t_trigger and computes
    peak, RMS, SNR, and onset delay in one pass — all derived from the
    same acoustic-window slice, not from a running single-sample max."""
    global trial_counter

    window = [(ts, adc) for (ts, adc) in timed_samples
              if 0.0 <= (ts - t_trigger) <= obs_window_s]

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

    print(f"Trial #{trial_counter}  [{LABEL}]")
    print(f"  Peak AC Amplitude   : {peak_ac_mv:+.1f} mV")
    print(f"  RMS AC Amplitude    : {rms_ac_mv:.1f} mV  (window-integrated)")
    print(f"  Noise Floor (pre)   : {pre_trigger_noise_floor_mv:.2f} mV RMS")
    print(f"  SNR                 : {snr_db:.1f} dB")
    print(f"  Onset Delay         : {onset_delay_ms:.2f} ms  (click-to-peak; "
          f"NOT acoustic ToF — includes OS audio + USB-serial latency)")
    print(f"  Samples in window   : {len(window)} / expected ~{int(obs_window_s * SAMPLING_FREQ_HZ)}")
    print("-" * 50)

    with open(LOG_PATH, "a", newline="") as f:
        csv.writer(f).writerow([
            trial_counter, LABEL, f"{peak_ac_mv:.2f}", f"{rms_ac_mv:.2f}",
            f"{pre_trigger_noise_floor_mv:.2f}", f"{snr_db:.2f}",
            f"{onset_delay_ms:.2f}", len(window)
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
