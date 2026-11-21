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

## Multi-tone sensing (frequency diversity, 20 Nov 2026)

A single 4 kHz tone is coherent, so room reflections add with a position-dependent phase
(fringes about 4 cm apart). That swung $\hat\beta$ by about ±70° over a 7.5 cm speaker shift
(Phase B). The sketch now averages power over many tones.

- **Tones.** 16 tone bins, 2500–6250 Hz every 250 Hz (`TONE_K` = 80…200, step 8; bin $k$ = $k \cdot 31.25$ Hz). 4000 Hz is one of them, so the old single-tone value is in the per-bin log.
- **Noise reference.** The 16 empty bins midway between the tones (`NOISE_K` = $k + 4$, in band). The fixed 3 kHz reference is gone.
- **Window.** Hann (`USE_HANN=1`), normalised so a bin-centred tone reads the same amplitude as the old rectangular Goertzel. It also stops a bin-centred tone leaking into the noise bins.
- **Combination.** $A = \sqrt{\max(0,\ \overline{a_k^2} - N^2)}$, an equal-weight power mean minus the in-band noise power. This value goes in `R2` as `Am`/`Ap`.
- **Validity.** $A \ge 3$ and $A \ge 2N$ on the combined value. The old per-capture single-bin test ($4N$) rejected captures that fell into a fringe null, which biased the pairs that survived.
- **New `B2` line,** sent just before each `R2` with the same `seq`: `B2,seq,M,a-_1..a-_M,a+_1..a+_M,Nm,Np*CRC16`. It holds the raw per-tone amplitudes.
  - `rover.collect` writes them to `bins.csv`, with columns named by frequency from the `C2` line.
  - `rover.live` recombines them when `bin_weights` is set in the config.
  - Hosts that only parse `R2` are unaffected.
- **`C2`** now reports `tones=16,tone_k=80:200,noise_bins=16,noise_k=84:204,window=hann,combine=power_nsub,snr_min=2.0`. The `tone=`/`reference=` fields are gone.
- **Timing.**
  - The serial TX buffer is now 1024 bytes, and the next servo move starts before the lines are printed, so roughly 370 bytes per pair never block sampling.
  - The 32 Goertzels add about 1–2 ms per capture.
  - Re-check $T_m$ in E0 tone. The E4 gap law should be unchanged, but that is not yet measured.
- **Legacy single tone.** Set `TONE_K={128}`, `NOISE_K={96}` and `USE_HANN=0`. $A$ is then noise-subtracted, which the old sketch did not do.
- **Beacon.** Generate it with `python Tools/make_tone.py --multitone --amp 0.9`, which writes `Data/stimuli/multitone_2500-6250Hz_16.wav`.
  - It uses Schroeder phases, giving a crest factor of 1.84 and 0.173 FS per tone.
  - Each tone is about 9 dB below the old 0.5 FS sine. Raise the playback volume and log it.
- **Host config.** `Config/measured_multitone.json` sets `snr_ratio_min = 2`. Its σ, $K$, κ and noise values are still the single-tone placeholders.
- **Compile.** Verified with esp32 core 3.3.12: 22 % flash USB-only, 82 % with `USE_BT=1` ([log](logs/compile_20261120_multitone.log)).

### Tone profiles: `horn12` is now the default (horn tuning, 20 Nov 2026)

Session s1 with `wide16` showed two problems:
- The horn has a bearing slope only at about 3.25–4.5 kHz.
- In silence, lines at exactly 3, 4, 5 and 6 kHz reach 3–6 counts, probably the 1 kHz USB frame rate.

`BEACON_PROFILE` (compile-time) selects the tone set:

| Profile | Tones (`TONE_K`) | Noise bins | Beacon |
|---|---|---|---|
| `2` = `horn12` (default) | 12 tones, 3187.5–4562.5 Hz every 125 Hz (k = 102…146 step 4) | midway, k ± 2, without k = 128 (4 kHz) | `Data/stimuli/horn12.wav` |
| `1` = `wide16` (session s1) | 16 tones, 2500–6250 Hz every 250 Hz | midway, k + 4 | `Data/stimuli/multitone_2500-6250Hz_16.wav` |

- **Interference.** With `AVOID_KHZ`, a `static_assert` checks that no tone or noise bin sits within one bin of a 1 kHz multiple. With the Hann window, a bin-centred line leaks into ±1 bin only.
- **`C2`** now starts with `profile=horn12` (or `wide16`). `rover.live` refuses to run if a configured `bin_model` was fitted on different tones.
- **The horn12 tones are half-Hz** (e.g. 3187.5 Hz), so `make_tone.py` writes a 2 s whole-cycle block.
- **Pre-emphasis.** `Tools/beacon_spec.py` measures the speaker+horn response per tone from earlier captures (average over all poses and both horn sides) and writes `horn12_spec.json`. The gains run from −7.4 to 0 dB, so every tone reaches the ADC at about the same level.
- **Rebuild the spec** whenever the speaker, playback device or horn changes:
  `python Tools/beacon_spec.py --runs Runs/<date> --profile horn12`, then
  `python Tools/make_tone.py --spec Data/stimuli/horn12_spec.json --amp 0.9`.
- **Build `wide16`:** `arduino-cli compile --fqbn esp32:esp32:esp32 --build-property "compiler.cpp.extra_flags=-DBEACON_PROFILE=1" firmware/esp32_acoustic_v2`.
- **Compile.** horn12: 22 % flash, no warnings ([log](logs/compile_20261120_horn12.log)). It has not been flashed yet.

## Wireless: Bluetooth Classic SPP (20 Nov 2026)

`USE_BT=1` (optional; the default build is USB only, the deployment path) adds a Bluetooth serial port named `AcousticFollower`, running
alongside USB. Every line goes to USB and also to Bluetooth while a client is connected.
Commands are accepted from either link. The `R2`/`C2` protocol and CRC are unchanged, so
`rover.collect`, `rover.live` and the tests only need a different `--port`.

- **Board:** a classic ESP32 ("ESP32 Dev Module"). The S2, S3, C3 and C6 have no Bluetooth
  Classic, and the build stops with an error on them.
- **Core:** `BluetoothSerial` is deprecated and will be off by default in Arduino-ESP32 4.0.
  Stay on core 3.x (3.3.12 is tested).
- **Flash use:** 82 % of the default 1.3 MB app partition with Bluetooth, 22 % without.
- **Bluetooth build** (USB-only is the default):
  `arduino-cli compile --fqbn esp32:esp32:esp32 --build-property "compiler.cpp.extra_flags=-DUSE_BT=1" firmware/esp32_acoustic_v2`
- **Non-blocking output:** a line that does not fit in the Bluetooth transmit buffer is
  dropped and counted, so sampling and servo timing are never held up by the radio. The
  count appears as `bt_drop=` in the `C2` line (send `?`). The `C2` line also reports
  `link=usb` or `link=usb+bt`. Pairs emitted while no client is connected are lost and
  show up as `seq` gaps.
- **Connection events:** the firmware prints `# bt client connected` and
  `# bt client disconnected`. It re-sends `C2` on connect.

### Pairing

Raspberry Pi (BlueZ):

```bash
bluetoothctl          # then: power on, scan on, pair <MAC>, trust <MAC>, quit
sudo rfcomm bind 0 <MAC> 1
python3 -m rover.collect --config Config/measured_sensor.json --port /dev/rfcomm0 ...
```

Windows: pair `AcousticFollower` in Settings → Bluetooth. Then open *More Bluetooth
settings → COM Ports* and use the **Outgoing** port.

### Before relying on the wireless link

The radio changes two things the controller depends on. Re-measure both:

1. **Transport delay and jitter.** SPP adds tens of ms of variable, bursty latency.
   - Repeat E0 over Bluetooth and compare `now - pair.time` (the `age_s` column of
     `measurement_decisions.csv` in a dry `rover.live` run) with the same run over USB.
   - Set `transport_delay` and `max_measurement_age` from the distribution's p95.
   - The commissioning gate rejects `transport_delay > 0.15` s.
2. **ADC noise and timing.** Run E0 silence and E0 tone with the default build and with the
   `USE_BT=0` build. Compare $s_n$, the valid percentage and the `late` flag. The radio runs
   on core 0 and sampling on core 1, and GPIO34 is on ADC1 (ADC2 is the one that conflicts
   with the radio), but this has not been measured.

The SPP link and the target's Bluetooth speaker are separate. Do not drive both from the
same host adapter during a run: A2DP and SPP on one adapter can make the tone drop out.

Safety does not depend on the link:
- If the SPP link drops or stalls, `rover.live` stops through its serial-error path or the
  1.3 s measurement watchdog.
- Keep the physical motor cut within reach anyway.
