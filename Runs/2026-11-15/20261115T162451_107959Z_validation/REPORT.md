# Validation evidence

Generated in `20261115T162451_107959Z_validation`. All closed-loop results below are synthetic.

The source hashes, exact configuration, environment, per-seed outcomes and seed-1000 time histories are saved beside this file. Seeds: 1000 through 1011.

Practical tracking means final 10 s mean range error <= 0.15 m, bearing RMS <= 10 deg, no clearance violation and >=70% TRACK time. It is a declared engineering criterion, not the original O3 target.

| Controller | Scenario | Tracking passes | Clearance violations | Mean tail range RMSE (m) | Worst bearing RMS (deg) |
|---|---|---:|---:|---:|---:|
| lqi | nominal | 12/12 | 0 | 0.005 | 0.73 |
| lqi | speed_step | 12/12 | 0 | 0.005 | 0.63 |
| lqi | gentle_turn | 12/12 | 0 | 0.005 | 0.70 |
| lqi | noise_drops_reflections | 11/12 | 0 | 0.084 | 3.61 |
| lqi | outage | 12/12 | 0 | 0.005 | 0.73 |
| lqi | gain_bias_20pct | 0/12 | 0 | 0.240 | 0.71 |
| lqi | slow_sensor | 0/12 | 0 | 10.197 | 4.68 |
| lqi | legacy_12s_sensor | 0/12 | 0 | 10.197 | 4.68 |
| lqi | actuator_mismatch | 12/12 | 0 | 0.004 | 0.55 |
| lqi | stationary_target | 0/12 | 0 | 0.120 | 11.59 |
| lqi | no_beacon | 0/12 | 0 | 10.197 | 4.68 |
| lqi | unreachable_target | 0/12 | 0 | 24.225 | 1.93 |
| place | nominal | 12/12 | 0 | 0.005 | 0.59 |
| place | speed_step | 12/12 | 0 | 0.005 | 0.50 |
| place | gentle_turn | 12/12 | 0 | 0.005 | 0.60 |
| place | noise_drops_reflections | 11/12 | 0 | 0.084 | 2.33 |
| place | outage | 12/12 | 0 | 0.005 | 0.59 |
| place | gain_bias_20pct | 0/12 | 0 | 0.240 | 0.58 |
| place | slow_sensor | 0/12 | 0 | 10.197 | 4.68 |
| place | legacy_12s_sensor | 0/12 | 0 | 10.197 | 4.68 |
| place | actuator_mismatch | 12/12 | 0 | 0.005 | 0.39 |
| place | stationary_target | 0/12 | 0 | 0.112 | 12.08 |
| place | no_beacon | 0/12 | 0 | 10.197 | 4.68 |
| place | unreachable_target | 0/12 | 0 | 24.228 | 1.84 |
| bearing_only | nominal | 0/12 | 0 | 0.170 | 0.34 |
| bearing_only | speed_step | 0/12 | 0 | 0.257 | 0.50 |
| bearing_only | gentle_turn | 0/12 | 0 | 0.170 | 0.39 |
| bearing_only | noise_drops_reflections | 7/12 | 0 | 0.210 | 2.06 |
| bearing_only | outage | 0/12 | 0 | 0.161 | 0.39 |
| bearing_only | gain_bias_20pct | 12/12 | 0 | 0.036 | 0.33 |
| bearing_only | slow_sensor | 0/12 | 0 | 10.197 | 4.68 |
| bearing_only | legacy_12s_sensor | 0/12 | 0 | 10.197 | 4.68 |
| bearing_only | actuator_mismatch | 0/12 | 0 | 0.765 | 0.53 |
| bearing_only | stationary_target | 0/12 | 0 | 0.013 | 7.98 |
| bearing_only | no_beacon | 0/12 | 0 | 10.197 | 4.68 |
| bearing_only | unreachable_target | 0/12 | 0 | 26.446 | 3.73 |

Stationary-target trials use a separate terminal-standoff success test; no-beacon and very slow sensing trials should stop. Unreachable targets and gain bias are intentional limitation tests. They are not excluded from the data.

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
