# Hardware implementation and experiment guide

Start here. The Python software and simulated evidence are provided; the physical
system is **not yet validated**. Follow the gates in order. Stop at a failed gate,
save the evidence, and resolve the cause before moving to the next stage.

## 1. What exists and what changed

The historical report, professor feedback, syllabus, POC, raw data, MATLAB and
Simulink files remain in place. New implementation:

- `rover/`: shared sensor conversion, observer, controller, simulation and Pi runner.
- `Firmware/esp32_acoustic_v2/`: USB firmware with CRC, separate capture timestamps,
  physical servo limits, pair angle snapshots and timing-quality rejection.
- `Config/simulation.json`: synthetic assumptions; motion is disabled by default.
- `Config/hardware_template.json`: measured-configuration instructions and actuator-table schema.
- `Runs/`: immutable timestamped evidence folders, with configuration and source hashes.
- `Docs/CONTROL_DESIGN.md`: derivations, decisions, course objectives and limitations.
- `Docs/VALIDATION_STATUS.md`: exact tested revision, results and outstanding checks.

The original wired sketch starts with a 90-degree dither even though commands limit
it to 60 degrees. Its 110-degree servo center plus an 80-degree horn limit can request
190 degrees. Both old sketches can mark a timing-overrun capture valid. The V2 sketch
removes these issues; the legacy logger now rejects timing-overrun flags. The old
sketches are historical, not the deployment path.

The latest supplied static log has only five pairs, median period 12.84 s, zero valid
pairs and two clipped pairs. Earlier silence logs contain two pairs each. The source
sketch alone does not establish why the flashed unit was this slow: confirm which
binary, settings and servo motion produced those logs. Do not label that timing fixed
until new measurements show it.

## 2. Software setup and reproducible commands

Run commands from the repository root. On Windows use `python`; on the Pi use
`python3` in the environment containing the dependencies.

```powershell
python -m pip install -r requirements.txt
python -m unittest discover -s Tests -v
python -m rover.validate --seeds 12
python -m rover.live --config Config/simulation.json --replay Data/E0_static_20261022_134646.csv
```

The last command must report zero accepted pairs and zero nonzero commands. Every
new validation, capture, replay or live run creates
`Runs/YYYY-MM-DD/YYYYMMDDTHHMMSS_microsecondsZ_<task>/`. UTC timestamps are explicit.
Keep the corresponding config/manifest with all results. Never overwrite `Data/`.

For MATLAB comparison, open MATLAB locally and run:

```matlab
addpath('Matlab');
R = rover_reference_design('Runs/2026-11-15/<completed_validation_folder>');
```

This independently computes the gain using base-MATLAB Riccati iteration and asserts
agreement with the saved Python gain. This session could not execute MATLAB because
the installed application could not contact its licensing service. The legacy `.slx`
model is not part of the validated deployment path.

## 3. Build and wire the frontend

Use the classic ESP32 board corresponding to the actual GPIO34/GPIO32 wiring and
Arduino-ESP32 3.x. V2 uses the built-in LEDC PWM API and needs no ESP32Servo library.

```powershell
arduino-cli compile --fqbn esp32:esp32:esp32 Firmware/esp32_acoustic_v2
```

The local compile attempt selected installed core 3.3.12 but Windows blocked the
Xtensa compiler with `Illegal System DLL Relocation`; compilation is **unverified**.
Build successfully in your normal Arduino IDE/toolchain before uploading. Do not
infer firmware correctness from Python tests. Save the compile output with the run.

| Connection | Required check |
|---|---|
| MAX4466 OUT to GPIO34 | Verify the actual ESP32 pinout and ADC range. Microphone output must not exceed the input rating. |
| MAX4466 supply | Suitable 3.3 V supply and common ground; check DC bias and gain headroom. |
| SG90 signal to GPIO32 | Separate regulated servo power, shared ground; do not power the servo from the ESP32 3.3 V rail. |
| ESP32 USB to follower Pi | Data-capable cable and stable power; only one process opens the serial port. |
| Horn | Center, mechanical clearance, positive-left direction and safe pulse range checked before mounting it. |
| PiCar | Wheels forward and steering-left signs verified with wheels lifted. Physical motor power cut within reach. |

First detach the horn/load, inspect `SERVO_CENTER`, `SERVO_SIGN`, pulse widths and
angle limits, and try `P0`, `P20`, `P-20`. Then mount the horn and measure settling.
V2 reports actual rounded/clamped angles; never reconstruct them from requested
settings. A command received between the halves takes effect on the next pair.

## 4. E0: prove basic acquisition before fitting anything

Play `tone_4000Hz.wav` continuously through the target speaker at fixed moderate
volume. Disable equalization, normalization and automatic gain if the playback
chain offers them. Keep speaker orientation fixed. Begin playback several seconds
before capture. Unsynchronized short bursts are incompatible with these continuous
paired measurements: one half can capture silence.

Capture silence, stationary tone at about 1.2 m, and tone with the follower motors
running while the wheels are lifted. Example for the stationary tone:

```powershell
python -m rover.collect --port COM6 --label E0_tone --rho 1.2 --beta 0 --phi 0 --delta 20 --settle-ms 40 --duration 60 --notes "room; speaker volume; mic gain mark; battery; motor state; firmware build"
```

On Linux the port is typically `/dev/ttyUSB0` or `/dev/ttyACM0`; verify it locally.
For silence, use the same command with label `E0_silence` and playback off. Each
capture saves raw serial, CRC-checked samples, configuration, session notes and summary.

Gate: at least 100 tone pairs, >=95% valid, no persistent clipping/timing errors,
95th-percentile pair period <=0.5 s. The 95th-percentile pair gap and serial transport
delay must fit the configured age/error budget. The silence run should produce no
accepted target estimates. Turn down mic/speaker gain if clipping occurs; do not
raise clip thresholds. If the 16 kHz timed ADC misses its schedule, diagnose timing
or implement raw DMA sampling before continuing. Do not suppress the timing flag.

Measure transport delay with a logic analyzer/serial timestamp experiment, or a
device-clock request/response experiment with a documented round-trip bound. The
current host mapping subtracts a supplied transport delay; it does not measure that
constant itself. Include serial serialization time and Pi scheduling. Capture the
distribution, not only its mean.

## 5. E1–E3: calibrate and independently test the acoustic model

Keep the receiver/speaker reference heights, source orientation and gains fixed.
At least three distances spanning the intended working range are needed, for example
0.8, 1.2, 2.0 and 3.0 m if the hardware achieves adequate SNR there. A distance is an
example test point, not an assumed successful operating limit.

At each distance, measure at least seven bearings covering both sides of boresight,
with finer 5–10-degree spacing around the intended tracking cone. Record at least
20–30 valid pairs per condition. Vary horn angle relative to a **stationary speaker**
when characterizing receiver directivity, so speaker radiation pattern does not
masquerade as horn response. Also test actual source aspect changes separately.

Reposition the equipment and repeat a separate session for validation. Include
motor-on noise and the actual floor/room. Do not randomly split adjacent rows from
one stationary capture and call that an independent validation session.

```powershell
python -m rover.calibrate --train <training_samples_1.csv> <training_samples_2.csv> --validation <heldout_samples_1.csv> <heldout_samples_2.csv>
```

Pass all relevant capture files explicitly (PowerShell can expand an array of paths).
The fit saves a `proposed_config.json` with hardware motion still disabled and a
`calibration_report.json`. It estimates K, range exponent n, sigma and boresight.
The validation grid needs at least three distances and five target bearings within
the calibrated angular domain. Train over a wider angular span than the intended
validation/tracking span to leave room for the fitted boresight offset.

Initial gates: held-out bearing RMSE <=3 degrees, range relative RMSE <=10%, bearing
bias <=2 degrees, range bias <=5%, valid fraction >=95%, period p95 <=0.5 s, acceptable
fit residual and no optimizer parameter stuck at a bound. Report failures as failures.

If bearing fails, first check sign/center, source orientation, frequency sensitivity,
motor interference and whether the horn is too broad near boresight. If the model
shape fails but the measurements are repeatable, a measured beam lookup model can
be developed and separately validated; it is not implemented or silently enabled here.
If the horn is fundamentally insensitive or too slow, the existing two-microphone
chirp POC is a possible next hardware path, but it also needs its own calibration.

If range alone fails, **do not run this distance-regulating controller with invented
K**. You can demonstrate stationary acoustic bearing estimation, or add an independent
range sensor and validate its target association. The bearing-only comparison here
still uses the calibrated range for its stop supervisor; it is not a range-free safe
deployment fallback. A changed sensing objective must be stated in the final project.

## 6. E4–E5: identify timing, geometry and actuators

Measure and record:

| Quantity | How to obtain it | Configuration/output |
|---|---|---|
| Wheelbase and reference-point offsets | Tape/caliper measurements of the actual loaded rover | `wheelbase`, `clearance` |
| Forward speed vs motor command | Timed straight runs over a measured distance; >=5 speeds, >=5 repeats, battery/load noted | `hardware.speed_table`, `min_speed`, `max_speed` |
| Actual wheel steering vs servo command | Measure wheel angles; verify left/right sign and center, >=5 points | `hardware.steering_table`, `max_steer_deg` |
| Turning radius / yaw rate | Overhead video or marked constant-radius arcs | Validate omega=v*tan(delta)/L; do not assume the nominal servo angle equals road-wheel angle |
| Speed and steering transients | Video step responses; time to 63% of final change for an initial first-order estimate | `tau_speed`, `tau_steer`, acceleration and steering-rate bounds |
| Braking distance/deceleration | Stop from each intended speed on the actual floor; use conservative worst repeated result | `braking_deceleration`, `stop_margin` |
| Pair period, gap, transport delay | V2 capture timestamps and host/logic-analyzer timing | `measurement_period`, `pair_gap`, `transport_delay`, `timeout`, age/history limits |
| Measurement noise/model mismatch | Repeated static and moving held-out errors | `amplitude_noise`, range/bearing floors, within-pair rate bounds |

Use the SunFounder calibration utility/manual to collect the initial actuator table;
the controller intentionally refuses motion without that table. Speed-table rows
are `[measured m/s, PWM 0..100]`, sorted by speed. Steering rows are `[measured road-wheel
degrees, servo command degrees]`, sorted by road-wheel angle; servo values may increase
or decrease depending on linkage sign. Never extrapolate beyond the table.

The desired separation must exceed chassis clearance plus uncertainty, loss-time
travel and measured braking distance. Recompute this after every material speed,
timeout, battery or load change. Target speed must remain below the follower's
repeatable sustainable speed with enough headroom to catch up.

## 7. E6: validate open-loop prediction

With a stationary beacon, execute short, slow known follower command sequences and
record overhead video. Measure follower/target positions and follower heading using
floor markers and a calibrated camera view. Compare the independent trajectory with
both the acoustic estimates and the nonlinear model. Repeat on a different initial
bearing and distance. A tape measure at the start/end alone cannot validate transient
bearing or settling time.

Do not use commanded motor speed as actual ground truth or integrate an unaided IMU
and label its position exact. Quantify the video/tape measurement uncertainty.

## 8. Prepare the measured configuration and run without actuation

Start from the fitted `proposed_config.json`. Replace all relevant platform, timing,
noise and operating-point assumptions using E0–E6. Add the `hardware` object from
`Config/hardware_template.json`, fill the measured tables and link the evidence files.
Keep `hardware_ready: false` until commissioning is complete.

```powershell
python -m rover.validate --config Config/measured_config.json --seeds 12
python -m rover.live --config Config/measured_config.json --port COM6 --duration 60
```

Without `--drive`, the runner reads sound and logs the commands it would have sent;
it does not import the motor driver. Verify signs, estimates, decision reasons,
timing, stale-data handling, horn limits and near-target behavior. Unplug the USB
sensor and mute the speaker separately; each must produce zero forward commands
within the configured loss interval. Feed an invalid/clipped stream; it must not
refresh the loss timer.

There is an additional process heartbeat for motor commands, but it does not replace
a physical motor power cut. Test the actual stop behavior on your Pi and motor board.

## 9. E7–E9: commissioning and closed-loop demonstration

1. **Wheels lifted:** install/verify the SunFounder `picar` package on the Pi. Check
   forward/left signs, command limits, Ctrl+C and loss of the controller process.
   Complete the four commissioning flags and evidence-file list. Only now set
   `hardware_ready: true` in the measured configuration.
2. **Stationary target on clear floor:** explicitly pass `--mission approach`, start
   inside the calibrated forward cone, at low speed with a nearby operator. Confirm terminal standoff and latched STOP.
   A new trial requires restarting the run. An arbitrary target approaching the
   follower is outside the collision bound.
3. **Straight moving target:** use an open-loop target profile slower than the
   follower. Start the beacon before follower acquisition. Record both rovers and a
   synchronization event in the overhead video. Keep each run within the available
   floor length; 0.18 m/s for 60 s needs over 10 m of travel.
4. **Repeat and compare:** run LQI, pole placement and bearing-only with identical
   operating conditions, >=5 repeats each. Save failed runs too. Then test gentle
   turns, brief signal loss and modest initial offsets within the measured envelope.

On the follower Pi, with a fully measured/commissioned config:

```bash
python3 -m rover.live --config Config/measured_config.json --port /dev/ttyUSB0 --duration 15 --drive
```

For the stationary-beacon commissioning trial, add `--mission approach` to the
follower command. This mode assumes the beacon is stationary because you declared
that task; it is not an automatic inference about an unknown moving target.

On the target Pi, using its **own** calibrated actuator table and documented readiness:

```bash
python3 -m rover.target --config Config/target_measured_config.json --speed 0.18 --duration 15 --drive
```

Without `--drive`, the target script also records only a dry run. The current shared
motion gate requires the commissioning flags for either role; the target's sensor
flag should reference the system's successful sensor validation record, not imply
that the target has a microphone.

## 10. Evaluate actual trials and complete the team's interpretation

Export independent ground truth as CSV with columns:

```text
time_s,follower_x_m,follower_y_m,follower_heading_rad,target_x_m,target_y_m
```

Synchronize its clock to `commands.csv` elapsed time. The argument below means
`command elapsed time = ground-truth time + offset`. Supply the measured position
uncertainty; the example value is not a certification.

```powershell
python -m rover.evaluate_trial --config Config/measured_config.json --commands <live_run>/commands.csv --ground-truth <trial_ground_truth.csv> --ground-truth-offset 0.0 --position-uncertainty 0.02
```

This saves aligned data, estimation/tracking errors, minimum separation, settling,
overshoot and a comparison figure. At least ten seconds of overlapping independent
reference data is required. Missing start-of-trial coverage invalidates the settling
claim. Model prediction and simulation results must remain labeled separately from
hardware measurements.

For the course presentation, explain the model, controllability/observability,
controller/observer choice, closed-loop poles, limits and comparison outcomes in
your own words. The syllabus restricts AI to assistive use. This repository supports
understanding and experiments; it is not a substitute for the team's experimental
work or submitted interpretation.

## Measurements to send back first

The smallest useful next package is **three 60-second V2 E0 captures** (silence,
stationary tone, and motor-on tone), the firmware compile log and board identity,
speaker/mic settings, and measured wheelbase plus approximate minimum sustainable
speed. If those pass, send the angular/range training and held-out sessions, actuator
tables, timing distributions and braking measurements. No additional experimental
values have been fabricated in the software results.
