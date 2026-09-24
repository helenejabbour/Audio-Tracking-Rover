"""
Dual-mic TDOA bearing monitor.

Receives one CSV line per estimate from the ESP32:
    theta_deg,delta_t_us,peak_corr,confidence,rms1_mv,rms2_mv

The ESP32 does all timing-critical work (synchronized dual-ADC capture +
cross-correlation). This script only logs and visualizes scalars, so
USB-serial polling jitter cannot corrupt the bearing estimate - it can only
add latency to when you SEE the estimate, which doesn't matter for a
rotating-arm search/refine loop.

Confidence gating: peak_corr near 0 means the "delay" is uncorrelated noise,
not a real bearing. Use CONFIDENCE_THRESHOLD to reject low-quality estimates
before feeding them to trajectory control - this is the direct software
analog of the front-back ambiguity / silence case in the firmware.
"""

from collections import deque
import csv
import os
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import serial

PORT = "COM4"
BAUD_RATE = 921600
HISTORY_LEN = 200  # rolling window of estimates shown on screen
CONFIDENCE_THRESHOLD = 25.0  # percent; below this, treat theta as unreliable

LOG_PATH = "doa_estimates.csv"
if not os.path.exists(LOG_PATH):
    with open(LOG_PATH, "w", newline="") as f:
        csv.writer(f).writerow([
            "t_s", "theta_deg", "delta_t_us", "peak_corr", "confidence",
            "rms1_mv", "rms2_mv", "accepted"
        ])

ser = serial.Serial(PORT, BAUD_RATE, timeout=0.05)
ser.reset_input_buffer()

t0 = time.perf_counter()
theta_hist = deque([np.nan] * HISTORY_LEN, maxlen=HISTORY_LEN)
conf_hist = deque([0.0] * HISTORY_LEN, maxlen=HISTORY_LEN)
rms1_hist = deque([0.0] * HISTORY_LEN, maxlen=HISTORY_LEN)
rms2_hist = deque([0.0] * HISTORY_LEN, maxlen=HISTORY_LEN)

fig, (ax_theta, ax_amp) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
plt.subplots_adjust(hspace=0.3)

(line_theta,) = ax_theta.plot(range(HISTORY_LEN), list(theta_hist), color="#007acc", lw=1.3, label="Bearing estimate")
ax_theta.axhline(0, color="gray", linestyle=":", linewidth=0.8)
ax_theta.set_ylim(-95, 95)
ax_theta.set_ylabel("theta [deg]\n(+ = toward mic1)")
ax_theta.set_title("TDOA Bearing Estimate (confidence-gated)", fontweight="bold")
ax_theta.grid(True, linestyle=":", alpha=0.5)
ax_theta.legend(loc="upper left")

(line_rms1,) = ax_amp.plot(range(HISTORY_LEN), list(rms1_hist), color="#5cb85c", lw=1.0, label="Mic1 RMS")
(line_rms2,) = ax_amp.plot(range(HISTORY_LEN), list(rms2_hist), color="#d9534f", lw=1.0, label="Mic2 RMS")
ax_amp.set_ylabel("AC RMS [mV]")
ax_amp.set_xlabel("Sample index (most recent = right)")
ax_amp.grid(True, linestyle=":", alpha=0.5)
ax_amp.legend(loc="upper left")

# Twin axis for confidence, overlaid on the bearing plot
ax_conf = ax_theta.twinx()
(line_conf,) = ax_conf.plot(range(HISTORY_LEN), list(conf_hist), color="#999999", lw=0.8, alpha=0.6, label="Confidence")
ax_conf.set_ylim(0, 100)
ax_conf.set_ylabel("confidence [%]", color="#999999")
ax_conf.axhline(CONFIDENCE_THRESHOLD, color="#999999", linestyle="--", linewidth=0.6)

string_buffer = ""
sample_count = 0


def parse_line(line):
    parts = line.strip().split(",")
    if len(parts) != 6:
        return None
    try:
        theta, dt_us, peak_corr, conf, rms1, rms2 = [float(p) for p in parts]
        return theta, dt_us, peak_corr, conf, rms1, rms2
    except ValueError:
        return None


def update_frame(frame):
    global string_buffer, sample_count

    if ser.in_waiting > 0:
        raw = ser.read(ser.in_waiting).decode("utf-8", errors="ignore")
        string_buffer += raw

        if "\n" in string_buffer:
            lines = string_buffer.split("\n")
            string_buffer = lines[-1]

            for line in lines[:-1]:
                parsed = parse_line(line)
                if parsed is None:
                    continue  # header line or malformed row

                theta, dt_us, peak_corr, conf, rms1, rms2 = parsed
                accepted = conf >= CONFIDENCE_THRESHOLD
                t_now = time.perf_counter() - t0
                sample_count += 1

                theta_hist.append(theta if accepted else np.nan)
                conf_hist.append(conf)
                rms1_hist.append(rms1)
                rms2_hist.append(rms2)

                with open(LOG_PATH, "a", newline="") as f:
                    csv.writer(f).writerow([
                        f"{t_now:.4f}", f"{theta:.2f}", f"{dt_us:.2f}",
                        f"{peak_corr:.4f}", f"{conf:.1f}",
                        f"{rms1:.2f}", f"{rms2:.2f}", int(accepted)
                    ])

                if sample_count % 20 == 0:
                    tag = "ACCEPT" if accepted else "REJECT (low confidence)"
                    print(f"[{tag}] theta={theta:+6.1f} deg  "
                          f"dt={dt_us:+7.2f} us  conf={conf:5.1f}%  "
                          f"rms1={rms1:6.1f} mV  rms2={rms2:6.1f} mV")

    line_theta.set_ydata(list(theta_hist))
    line_conf.set_ydata(list(conf_hist))
    line_rms1.set_ydata(list(rms1_hist))
    line_rms2.set_ydata(list(rms2_hist))
    ax_amp.relim()
    ax_amp.autoscale_view(scalex=False)

    return line_theta, line_conf, line_rms1, line_rms2


ani = FuncAnimation(fig, update_frame, interval=20, blit=False, cache_frame_data=False)

try:
    plt.show()
finally:
    ser.close()
    print(f"\nLogged estimates written to {os.path.abspath(LOG_PATH)}")
