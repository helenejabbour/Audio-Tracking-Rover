from collections import deque
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
import numpy as np
import serial

# Try loading sounddevice; fallback to winsound if on Windows without sounddevice
try:
    import sounddevice as sd

    HAS_SOUNDDEVICE = True
except ImportError:
    import winsound

    HAS_SOUNDDEVICE = False

# --- Configuration ---
PORT = "COM4"  # '/dev/ttyACM0' (Linux) or '/dev/cu.usbmodem*' (macOS)
BAUD_RATE = 115200
BUFFER_SIZE = 500  # Samples displayed on screen
SAMPLING_FREQ_HZ = 10000  # 10 kHz ESP32 sampling rate
V_REF = 3.3
ADC_MAX = 4095.0

# Initialize serial interface
ser = serial.Serial(PORT, BAUD_RATE, timeout=0.001)

# Time axis array in milliseconds
dt_ms = (1.0 / SAMPLING_FREQ_HZ) * 1000.0
time_axis = np.linspace(-(BUFFER_SIZE - 1) * dt_ms, 0, BUFFER_SIZE)

# Ring buffer for raw ADC counts
adc_buffer = deque([2048] * BUFFER_SIZE, maxlen=BUFFER_SIZE)

# --- Impulse Sound Generator ---
AUDIO_FS = 44100  # Audio sampling rate [Hz]
IMPULSE_DURATION = 0.015  # 15 ms duration
t_audio = np.linspace(0, IMPULSE_DURATION, int(AUDIO_FS * IMPULSE_DURATION))
# 3 kHz sine wave with fast exponential decay (tau = 3 ms)
impulse_wave = (
    0.9 * np.sin(2 * np.pi * 3000 * t_audio) * np.exp(-t_audio / 0.003)
).astype(np.float32)


def play_audio_impulse():
    """Play short, high-amplitude acoustic impulse."""
    if HAS_SOUNDDEVICE:
        sd.play(impulse_wave, AUDIO_FS)
    else:
        winsound.Beep(3000, 20)  # 3 kHz for 20 ms


# --- Global Tracking State ---
is_running = True
string_buffer = ""

# Peak detection state
is_tracking = False
t_trigger = 0.0
t_peak = 0.0
max_adc = 0
obs_window_s = 0.300  # 300 ms measurement window

# --- Figure and UI Layout ---
fig, ax_voltage = plt.subplots(figsize=(11, 6))
plt.subplots_adjust(bottom=0.20)

(line_v,) = ax_voltage.plot(
    time_axis,
    [(2048 / ADC_MAX) * V_REF] * BUFFER_SIZE,
    color="#007acc",
    lw=1.2,
    label="Instantaneous Signal",
)

ax_voltage.set_title(
    "Acoustic Impulse Response & Latency Detector",
    fontsize=12,
    fontweight="bold",
)
ax_voltage.set_xlabel("Time relative to present [ms]", fontsize=10)
ax_voltage.set_ylabel("Absolute Voltage [V]", fontsize=10, color="#007acc")
ax_voltage.set_ylim(0.0, V_REF)
ax_voltage.axhline(
    1.65, color="red", linestyle="--", linewidth=0.8, label="1.65V DC Bias"
)
ax_voltage.tick_params(axis="y", labelcolor="#007acc")
ax_voltage.grid(True, linestyle=":", alpha=0.6)

# Secondary Y-Axis: Centered AC Amplitude [mV]
ax_mv = ax_voltage.twinx()
ax_mv.set_ylabel("AC Audio Amplitude [mV]", fontsize=10, color="#d9534f")
ax_mv.set_ylim(-1650.0, 1650.0)
ax_mv.tick_params(axis="y", labelcolor="#d9534f")
ax_mv.grid(False)

ax_voltage.legend(loc="upper left")

# --- UI Controls ---
ax_btn_run = plt.axes([0.30, 0.03, 0.15, 0.05])
ax_btn_trigger = plt.axes([0.52, 0.03, 0.20, 0.05])

btn_toggle = Button(ax_btn_run, "Stop", color="#d9534f", hovercolor="#c9302c")
btn_trigger = Button(
    ax_btn_trigger, "Trigger Impulse", color="#0275d8", hovercolor="#025aa5"
)


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
    global is_tracking, t_trigger, t_peak, max_adc
    if not is_running:
        print("[WARN] Start the plotter before triggering impulse.")
        return

    # Clear serial FIFO to reduce driver buffering delay
    ser.reset_input_buffer()

    max_adc = 0
    t_peak = time.perf_counter()
    t_trigger = time.perf_counter()
    is_tracking = True

    # Play sound non-blockingly
    play_audio_impulse()
    print("\n--- [IMPULSE TRIGGERED] ---")


btn_toggle.on_clicked(toggle_run)
btn_trigger.on_clicked(trigger_impulse)


# --- Real-Time Processing Loop ---
def update_frame(frame):
    global string_buffer, is_tracking, max_adc, t_peak

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

                            # Track maximum value within observation window
                            if is_tracking and (
                                now - t_trigger <= obs_window_s
                            ):
                                # Measure absolute displacement from 1.65V bias (2048 counts)
                                if abs(raw_adc - 2048) > abs(max_adc - 2048):
                                    max_adc = raw_adc
                                    t_peak = now

            # Process end of observation window
            if is_tracking and (
                time.perf_counter() - t_trigger > obs_window_s
            ):
                is_tracking = False
                delta_t_ms = (t_peak - t_trigger)
                v_measured = (max_adc / ADC_MAX) * V_REF
                v_ac_mv = (v_measured - 1.65) * 1000.0

                print(f"Max ADC Count : {max_adc} / 4095")
                print(f"Max Voltage   : {v_measured:.3f} V")
                print(f"Peak AC Amplitude : {v_ac_mv:+.1f} mV")
                print(
                    f"Clock Difference (Δt) : {delta_t_ms:.8f} us (Trigger -> Peak)"
                )
                print("-----------------------------\n")

    if is_running:
        voltage_array = (np.array(adc_buffer) / ADC_MAX) * V_REF
        line_v.set_ydata(voltage_array)

    return (line_v,)


ani = FuncAnimation(
    fig, update_frame, interval=20, blit=True, cache_frame_data=False
)

try:
    plt.show()
finally:
    ser.close()