# Validation evidence

Generated in `20261115T163143_844160Z_validation`. All closed-loop results below are synthetic.

The source hashes, exact configuration, environment, per-seed outcomes and seed-3000 time histories are saved beside this file. Seeds: 3000 through 3011.

Practical tracking means final 10 s mean range error <= 0.15 m, bearing RMS <= 10 deg, no clearance violation and >=70% TRACK time. It is a declared engineering criterion, not the original O3 target.

| Controller | Scenario | Tracking passes | Clearance violations | Mean tail range RMSE (m) | Worst bearing RMS (deg) |
|---|---|---:|---:|---:|---:|
| lqi | nominal | 12/12 | 0 | 0.004 | 0.54 |
| lqi | speed_step | 12/12 | 0 | 0.004 | 0.47 |
| lqi | gentle_turn | 12/12 | 0 | 0.004 | 0.57 |
| lqi | noise_drops_reflections | 11/12 | 0 | 0.121 | 3.11 |
| lqi | outage | 12/12 | 0 | 0.004 | 0.54 |
| lqi | gain_bias_20pct | 0/12 | 0 | 0.241 | 0.53 |
| lqi | slow_sensor | 0/12 | 0 | 10.237 | 5.52 |
| lqi | legacy_12s_sensor | 0/12 | 0 | 10.237 | 5.52 |
| lqi | actuator_mismatch | 12/12 | 0 | 0.004 | 0.41 |
| lqi | stationary_target | 0/12 | 0 | 0.060 | 2.74 |
| lqi | no_beacon | 0/12 | 0 | 10.237 | 5.52 |
| lqi | unreachable_target | 0/12 | 0 | 24.439 | 3.89 |
| place | nominal | 12/12 | 0 | 0.004 | 0.46 |
| place | speed_step | 12/12 | 0 | 0.004 | 0.42 |
| place | gentle_turn | 12/12 | 0 | 0.004 | 0.55 |
| place | noise_drops_reflections | 11/12 | 0 | 0.121 | 2.34 |
| place | outage | 12/12 | 0 | 0.004 | 0.46 |
| place | gain_bias_20pct | 0/12 | 0 | 0.241 | 0.44 |
| place | slow_sensor | 0/12 | 0 | 10.237 | 5.52 |
| place | legacy_12s_sensor | 0/12 | 0 | 10.237 | 5.52 |
| place | actuator_mismatch | 12/12 | 0 | 0.004 | 0.39 |
| place | stationary_target | 0/12 | 0 | 0.060 | 2.74 |
| place | no_beacon | 0/12 | 0 | 10.237 | 5.52 |
| place | unreachable_target | 0/12 | 0 | 24.443 | 3.44 |
| bearing_only | nominal | 0/12 | 0 | 0.170 | 0.29 |
| bearing_only | speed_step | 0/12 | 0 | 0.265 | 0.42 |
| bearing_only | gentle_turn | 0/12 | 0 | 0.170 | 0.38 |
| bearing_only | noise_drops_reflections | 10/12 | 0 | 0.745 | 2.47 |
| bearing_only | outage | 1/12 | 0 | 0.158 | 0.35 |
| bearing_only | gain_bias_20pct | 12/12 | 0 | 0.036 | 0.29 |
| bearing_only | slow_sensor | 0/12 | 0 | 10.237 | 5.52 |
| bearing_only | legacy_12s_sensor | 0/12 | 0 | 10.237 | 5.52 |
| bearing_only | actuator_mismatch | 0/12 | 0 | 0.809 | 0.49 |
| bearing_only | stationary_target | 0/12 | 0 | 0.060 | 2.74 |
| bearing_only | no_beacon | 0/12 | 0 | 10.237 | 5.52 |
| bearing_only | unreachable_target | 0/12 | 0 | 26.515 | 4.16 |

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
