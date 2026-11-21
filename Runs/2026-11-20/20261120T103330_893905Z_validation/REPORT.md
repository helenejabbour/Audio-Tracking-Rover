# Validation evidence

Generated in `20261120T103330_893905Z_validation`. All closed-loop results below are synthetic.

The source hashes, exact configuration, environment, per-seed outcomes and seed-3000 time histories are saved beside this file. Seeds: 3000 through 3011.

Practical tracking means final 10 s mean range error <= 0.15 m, bearing RMS <= 10 deg, no clearance violation and >=70% TRACK time. It is a declared engineering criterion, not the original O3 target.

| Controller | Scenario | Tracking passes | Clearance violations | Mean tail range RMSE (m) | Worst bearing RMS (deg) |
|---|---|---:|---:|---:|---:|
| lqi | nominal | 0/12 | 0 | 10.221 | 5.47 |
| lqi | speed_step | 0/12 | 0 | 11.976 | 5.30 |
| lqi | gentle_turn | 0/12 | 0 | 9.854 | 20.89 |
| lqi | noise_drops_reflections | 0/12 | 0 | 10.228 | 5.52 |
| lqi | outage | 0/12 | 0 | 10.221 | 5.47 |
| lqi | gain_bias_20pct | 0/12 | 0 | 10.190 | 5.18 |
| lqi | slow_sensor | 0/12 | 0 | 10.237 | 5.52 |
| lqi | legacy_12s_sensor | 0/12 | 0 | 10.237 | 5.52 |
| lqi | actuator_mismatch | 0/12 | 0 | 10.210 | 5.47 |
| lqi | stationary_target | 0/12 | 0 | 0.343 | 14.73 |
| lqi | no_beacon | 0/12 | 0 | 10.237 | 5.52 |
| lqi | unreachable_target | 0/12 | 0 | 27.860 | 4.77 |
| place | nominal | 0/12 | 0 | 10.221 | 5.36 |
| place | speed_step | 0/12 | 0 | 11.976 | 5.20 |
| place | gentle_turn | 0/12 | 0 | 9.854 | 20.78 |
| place | noise_drops_reflections | 0/12 | 0 | 10.228 | 5.52 |
| place | outage | 0/12 | 0 | 10.221 | 5.36 |
| place | gain_bias_20pct | 0/12 | 0 | 10.190 | 4.64 |
| place | slow_sensor | 0/12 | 0 | 10.237 | 5.52 |
| place | legacy_12s_sensor | 0/12 | 0 | 10.237 | 5.52 |
| place | actuator_mismatch | 0/12 | 0 | 10.210 | 5.36 |
| place | stationary_target | 0/12 | 0 | 0.343 | 14.73 |
| place | no_beacon | 0/12 | 0 | 10.237 | 5.52 |
| place | unreachable_target | 0/12 | 0 | 27.860 | 4.77 |
| bearing_only | nominal | 0/12 | 0 | 10.221 | 5.41 |
| bearing_only | speed_step | 0/12 | 0 | 11.976 | 5.24 |
| bearing_only | gentle_turn | 0/12 | 0 | 9.854 | 20.83 |
| bearing_only | noise_drops_reflections | 0/12 | 0 | 10.228 | 5.52 |
| bearing_only | outage | 0/12 | 0 | 10.221 | 5.41 |
| bearing_only | gain_bias_20pct | 0/12 | 0 | 10.190 | 5.02 |
| bearing_only | slow_sensor | 0/12 | 0 | 10.237 | 5.52 |
| bearing_only | legacy_12s_sensor | 0/12 | 0 | 10.237 | 5.52 |
| bearing_only | actuator_mismatch | 0/12 | 0 | 10.213 | 5.41 |
| bearing_only | stationary_target | 0/12 | 0 | 0.343 | 14.73 |
| bearing_only | no_beacon | 0/12 | 0 | 10.237 | 5.52 |
| bearing_only | unreachable_target | 0/12 | 0 | 27.860 | 4.77 |

Stationary-target trials use the same explicitly selected approach mission for all three controller labels; they validate the shared terminal supervisor, not a difference between LQI and pole placement. They use a separate terminal-standoff success test. No-beacon and very slow sensing trials should stop. Unreachable targets and gain bias are intentional limitation tests, retained in the data.

## Existing measurements

| File | Rows | Valid | Usable | Median period (s) |
|---|---:|---:|---:|---:|
| Data\E0_silence_20261022_134226.csv | 2 | 0 | 0 | 12.879999999999999 |
| Data\E0_silence_20261022_134309.csv | 2 | 0 | 0 | 12.879999999999999 |
| Data\E0_silence_20261022_134614.csv | 2 | 0 | 0 | 12.879999999999999 |
| Data\E0_static_20261022_134646.csv | 5 | 0 | 0 | 12.840000000000003 |

## Limits of this validation

- Synthetic sensing uses the fitted-model family. Real horn, room reflections, speaker directivity, motor noise and timing need held-out experiments.
- A minimum simulated separation is not a hardware collision guarantee. The runtime speed bound depends on calibrated error and braking bounds.
- Original O3 2% settling and 15% overshoot results remain in every run, including failures and non-settling outcomes.
- The sampled-loop analysis includes lag, delay and measurement rate in a local periodic reference. It is not a global nonlinear stability proof or a classical phase-margin certificate.
- MATLAB execution and ESP32 compilation depend on the installed toolchain; see Docs/VALIDATION_STATUS.md for actual execution status.
