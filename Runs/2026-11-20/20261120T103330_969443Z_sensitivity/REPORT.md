# Assumption sensitivity

Synthetic trials, seeds 4000–4005. These values are not a fit to the existing POC.

The horn-width sweep changes the correctly calibrated model in both sensor and estimator. It tests noise and within-pair motion sensitivity, not unmodeled beam-shape mismatch. Amplitude constant and other parameters remain the same.

bearing_bias_deg offsets the true horn boresight from the calibrated one (constant bias). dither_deg_E4_timing couples dither to the measured pair timing.

| Parameter | Value | Tracking passes | Mean range RMSE (m) | Worst bearing RMS (deg) | Mean TRACK fraction | Clearance violations |
|---|---:|---:|---:|---:|---:|---:|
| horn_sigma_deg | 25.0 | 0/6 | 10.245 | 4.27 | 0.00 | 0 |
| horn_sigma_deg | 40.0 | 0/6 | 10.231 | 4.16 | 0.00 | 0 |
| horn_sigma_deg | 80.0 | 0/6 | 10.230 | 4.15 | 0.00 | 0 |
| horn_sigma_deg | 120.0 | 0/6 | 10.231 | 4.13 | 0.00 | 0 |
| horn_sigma_deg | 160.0 | 0/6 | 10.241 | 4.27 | 0.00 | 0 |
| pair_gap_s | 0.1 | 0/6 | 0.338 | 0.61 | 0.62 | 0 |
| pair_gap_s | 0.18 | 0/6 | 0.407 | 0.63 | 0.56 | 0 |
| pair_gap_s | 0.3 | 0/6 | 10.230 | 4.11 | 0.00 | 0 |
| pair_gap_s | 0.5 | 0/6 | 10.245 | 4.27 | 0.00 | 0 |
| amplitude_noise_counts | 0.2 | 0/6 | 10.226 | 4.16 | 0.00 | 0 |
| amplitude_noise_counts | 0.8 | 0/6 | 10.230 | 4.15 | 0.00 | 0 |
| amplitude_noise_counts | 2.0 | 0/6 | 10.235 | 4.19 | 0.00 | 0 |
| amplitude_noise_counts | 5.0 | 0/6 | 10.245 | 4.27 | 0.00 | 0 |
| bearing_bias_deg | 0.0 | 0/6 | 10.230 | 4.15 | 0.00 | 0 |
| bearing_bias_deg | 5.0 | 0/6 | 10.228 | 4.08 | 0.00 | 0 |
| bearing_bias_deg | 10.0 | 0/6 | 10.228 | 3.99 | 0.00 | 0 |
| bearing_bias_deg | 20.0 | 0/6 | 10.233 | 4.27 | 0.00 | 0 |
| bearing_bias_deg | -10.0 | 0/6 | 10.230 | 4.29 | 0.00 | 0 |
| dither_deg_E4_timing | 20.0 | 0/6 | 0.200 | 1.86 | 0.88 | 0 |
| dither_deg_E4_timing | 30.0 | 0/6 | 0.351 | 0.93 | 0.62 | 0 |
| dither_deg_E4_timing | 40.0 | 0/6 | 10.162 | 3.51 | 0.01 | 0 |
| dither_deg_E4_timing | 50.0 | 0/6 | 10.230 | 4.15 | 0.00 | 0 |
| dither_deg_E4_timing | 60.0 | 0/6 | 10.243 | 4.26 | 0.00 | 0 |
| bearing_noise_floor_deg | 5.0 | 0/6 | 10.226 | 4.06 | 0.00 | 0 |
| bearing_noise_floor_deg | 10.0 | 0/6 | 10.227 | 4.15 | 0.00 | 0 |
| bearing_noise_floor_deg | 15.0 | 0/6 | 10.227 | 4.15 | 0.00 | 0 |
| bearing_noise_floor_deg | 20.0 | 0/6 | 10.230 | 4.15 | 0.00 | 0 |
| uncertainty_gate_scale | 1.0 | 0/6 | 10.230 | 4.15 | 0.00 | 0 |
| uncertainty_gate_scale | 2.0 | 0/6 | 0.422 | 1.21 | 0.98 | 0 |
| uncertainty_gate_scale | 3.0 | 0/6 | 0.422 | 1.21 | 0.98 | 0 |

Broadband level-vs-angle POC values cannot determine the 4 kHz Gaussian sigma. Test the actual local slope, symmetry, repeatability and sequential-pair motion error in held-out recordings.
