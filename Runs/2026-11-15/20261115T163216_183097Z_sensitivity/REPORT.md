# Assumption sensitivity

Synthetic trials, seeds 4000–4005. These values are not a fit to the existing POC.

The horn-width sweep changes the correctly calibrated model in both sensor and estimator. It tests noise and within-pair motion sensitivity, not unmodeled beam-shape mismatch. Amplitude constant and other parameters remain the same.

| Parameter | Value | Tracking passes | Mean range RMSE (m) | Worst bearing RMS (deg) |
|---|---:|---:|---:|---:|
| horn_sigma_deg | 25.0 | 6/6 | 0.004 | 0.33 |
| horn_sigma_deg | 40.0 | 6/6 | 0.003 | 0.69 |
| horn_sigma_deg | 80.0 | 6/6 | 0.003 | 2.37 |
| horn_sigma_deg | 120.0 | 6/6 | 0.005 | 4.60 |
| horn_sigma_deg | 160.0 | 6/6 | 0.009 | 7.44 |
| pair_gap_s | 0.1 | 6/6 | 0.003 | 0.69 |
| pair_gap_s | 0.18 | 6/6 | 0.003 | 0.69 |
| pair_gap_s | 0.3 | 6/6 | 0.006 | 0.48 |
| pair_gap_s | 0.5 | 0/6 | 0.177 | 0.77 |
| amplitude_noise_counts | 0.2 | 6/6 | 0.001 | 0.17 |
| amplitude_noise_counts | 0.8 | 6/6 | 0.003 | 0.69 |
| amplitude_noise_counts | 2.0 | 6/6 | 0.008 | 1.75 |
| amplitude_noise_counts | 5.0 | 6/6 | 0.021 | 4.25 |

Broadband level-vs-angle POC values cannot determine the 4 kHz Gaussian sigma. Test the actual local slope, symmetry, repeatability and sequential-pair motion error in held-out recordings.
