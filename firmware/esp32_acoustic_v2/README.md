# Acoustic frontend V2

Compile for `esp32:esp32:esp32` with Arduino-ESP32 3.3.12 (installed on the development
machine). No external libraries. USB serial is the initial integration path; power
the ESP32 from the follower Pi. Bluetooth speaker playback remains independent.

The microphone output must remain within the ESP32 ADC input range. Use a suitable
3.3 V microphone supply, common ground, and a separate regulated servo supply. Do
not power an SG90 from the ESP32 3.3 V rail. Verify your exact board pinout.

Before attaching the horn, verify center, sign, travel limits and pulse widths. The
default constants are commissioning assumptions. Positive horn angle must point
left when viewed from above. Measure settling time with the mounted horn.

`P0`, `D20`, `S40`, and `?` are newline-terminated commands. The host must use
the actual angles in each packet. Commands are polled during settling; sampling
blocks for about 32 ms. Overflowing and nonfinite commands are rejected.

Both timestamp values are capture midpoints in `micros()` ticks. `emit_us` is the
timestamp immediately before formatting the line; serial serialization time is
part of the measured transport delay. CRC covers only the ASCII bytes before `*`.
See `rover/sensing.py` for the matching parser and timestamp unwrapping.

The timed one-shot ADC capture rejects sample overruns or more than 2% span error.
It is not a DMA sampler. If E0 reports timing errors, do not relax the quality flag:
measure the sample timing and migrate to the ESP-IDF raw continuous/DMA API, or
select a lower sample rate and compatible tone. The Arduino averaged continuous
ADC result is not a replacement for the raw audio waveform.

The earlier wired and wireless sketches are retained as historical sources. Use
this version for new captures. Its board compilation does not validate physical
ADC timing, servo positioning or acoustic accuracy.
