# Validation status — 15 November 2026

**Software implementation and simulation validation are complete for the declared
test suite. Hardware tracking, ESP32 compilation and MATLAB execution are not
validated.** The current package is ready for the team's gated commissioning work,
not a claim that the physical project has already succeeded.

## Authoritative runs

- [Final controller comparison](../Runs/2026-11-15/20261115T163143_844160Z_validation/REPORT.md):
  432 simulations = 3 controller labels × 12 scenarios × 12 seeds, seeds 3000–3011.
- [Final assumption sensitivity](../Runs/2026-11-15/20261115T163216_183097Z_sensitivity/REPORT.md):
  78 simulations, seeds 4000–4005, varying horn width, pair timing and amplitude noise.
- [Real-log replay](../Runs/2026-11-15/20261115T162526_373664Z_replay/summary.json):
  five supplied static-test packets, zero accepted, zero nonzero commands.

The final two simulation runs preserve source snapshots, SHA-256 hashes, exact
configuration and per-seed metrics. The controller comparison also saves its test
output, numerical analysis, environment versions, example traces and figures.
Development and interrupted runs remain clearly identified in `Runs/README.md`.

## Results of the selected LQI following controller

These are **synthetic** results under explicitly assumed geometry, acoustic response,
noise and actuator parameters. The nominal configuration uses 1.2 m separation,
0.18 m/s target speed, 20 Hz control, 0.4 s paired measurements and a 1.3 s loss timer.

| Test | Outcome across 12 seeds |
|---|---|
| Nominal straight following | 12/12 practical tracking passes; mean final-10-s range RMSE 0.0041 m |
| Target speed increase | 12/12 practical tracking passes |
| Gentle turn | 12/12 practical tracking passes |
| Actuator speed/lag mismatch | 12/12 practical tracking passes |
| Five-second beacon outage | 12/12 practical tracking passes after stationary reacquisition |
| Doubled noise, 15% drops, 12% reflection events, timing jitter | 11/12 practical tracking passes; mean tail range RMSE 0.121 m |
| Declared stationary-beacon approach | 12/12 terminal-standoff passes; mean final range error about +0.060 m; worst tail bearing RMS 2.74 degrees |
| No beacon | 12/12 zero-motion runs |
| 1.2 s pair period / 0.55 s gap | 12/12 zero-motion runs; the configured confidence gate prevents tracking |
| Historical 12.84 s timing | 12/12 zero-motion runs |
| True acoustic gain 20% above calibration | True following-distance error about +0.241 m; range-regulation criterion fails |
| Target faster than the follower | Tracking fails, as expected from the speed constraint |

All 432 comparison trials remained above the assumed 0.40 m clearance. This is a
finite simulation result, **not** a hardware collision guarantee. The bearing-only
baseline does not regulate nominal separation; occasional passes in biased/stress
scenarios do not make its distance behavior robust. Pole placement performs similarly
to LQI in these tests; there is no claimed statistical superiority between them.

The stationary trial uses the same explicit `approach` mission for every controller
label. It tests the shared terminal behavior, not LQI versus pole placement at a
zero-speed equilibrium. The moving-target controller retains its own design model.

“Practical tracking pass” requires final-10-s mean range error <=15 cm, bearing RMS
<=10 degrees, >=70% TRACK time and no clearance violation. Terminal approach uses
a separate final range/clearance test. See the raw metrics rather than applying one
success definition to incompatible tasks.

**The original six-second 2% settling objective is not met by the current tuning.**
Its failures and overshoot metrics remain in the results. No classical 45-degree
phase-margin or 6 dB full-runtime gain-margin claim is made. The periodic linear
analysis checks sample period, delay, actuator lag and gain scaling; the nonlinear
runtime is assessed by simulation. Retune only after actual delay/noise/actuator
measurements establish an achievable transient specification.

## Assumption sensitivity

The final sensitivity sweep produced no clearance violations. It shows:

- With a correctly calibrated Gaussian beam, broadening sigma from 25 to 160 degrees
  increases worst tail bearing RMS from about 0.33 to 7.44 degrees under the assumed
  amplitude SNR. This does not validate the actual horn's beam shape.
- A 0.5 s within-pair gap with 1.0 s pair period fails the practical tracking criterion
  in all six trials. A slow scan cannot be hidden by running control at 20 Hz.
- The tested additive-noise sweep passes at 0.2–5 counts with the assumed K=100;
  this cannot be transferred to another microphone gain or received amplitude without
  recalibrating the SNR and covariance.

## Verification actually performed

| Check | Status |
|---|---|
| Python unit/regression suite | **26 tests passed** in the final comparison run |
| Numerical nonlinear-to-linear Jacobian | Max errors approximately 2.5e-14 (A), 1.0e-12 (B) |
| Nominal controllability/observability | Both rank 3; stopped virtual model rank 2; disturbance-augmented model ranks 3/5 |
| Discrete LQI stability / Lyapunov check | Spectral radius about 0.99343; positive P; residual about 1.4e-14 |
| Forced LTI trajectory, LTV transition and similarity realization | Numerically checked |
| Delayed EKF correction | Compared with an in-order reference, including fractional capture time |
| CRC, nonfinite/invalid data, duplicate/reset/wrap handling | Tested |
| No-signal, stale-data, uncertainty stop and motor-process heartbeat | Tested with software/fake drivers |
| Held-out calibration logic | Synthetic known-parameter recovery and invalid-data rejection tested |
| Hardware-log analysis | Reference-time alignment tested with independently constructed synthetic trajectories |
| CLI / Python compilation | Entry-point help and Python syntax compilation checked |
| Physical Pi motor driver, USB timing, ADC, horn or braking | **Not tested on hardware** |
| ESP32 V2 firmware compilation | **Blocked before compilation** by Windows compiler startup error |
| MATLAB reference execution | **Blocked** by unavailable MathWorks licensing service |
| Original Simulink `.slx` | Retained, **not validated**; not used by the new runtime |

The firmware attempt selected `esp32:esp32` core 3.3.12 and failed starting
`xtensa-esp32-elf-g++.exe` with `Illegal System DLL Relocation`. MATLAB R2026b could
not contact required MathWorks services (error 5202), even after using a writable
temporary preferences directory. These are actual toolchain failures, not approval
requests or successful build results.

## First physical action

Follow [IMPLEMENTATION_GUIDE.md](IMPLEMENTATION_GUIDE.md), starting with the firmware
build and three E0 captures listed in [MEASUREMENTS_NEEDED.md](MEASUREMENTS_NEEDED.md).
The saved real data currently fail acquisition: median pair period 12.84 s and zero
valid samples in the static test. No controller gain should be treated as a hardware
gain until that blocker and the held-out sensor/actuator gates are resolved.
