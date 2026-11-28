# Phase E results (20261128T105051_993805Z_phase_e)

Config `Config/phase_e.json` (measured sensor and timing, provisional vehicle). Operating point d* = 1.4 m, v0 = 0.18 m/s, a = v0/d* = 0.129 1/s, T_m = 0.477 s, pair gap 0.405 s, information delay T_eff = 0.52 s.

| Figure | Question | Answer |
|---|---|---|
| [1](fig1_structure.png) | Controllable/observable at our operating point? | Yes (ranks 3/3), but λmin(Wo) = 0.42: ε is only seen through β' = aε with a = 0.13; at v0 = 0 both ranks drop to 2. |
| [2](fig2_bandwidth.png) | How fast may the loop be? | ωn ≲ 0.3 rad/s. LQI: crossover 0.68/0.58 rad/s, PM 47/59°, delay margin 1.21/1.78 s beyond T_eff, disk α 0.79/1.01. |
| [3](fig3_poles.png) | Separation of controller and observer? | Observer |z| per T_m: 0.74, 0.74, 0.85, 0.92, 0.92. |
| [4](fig4_governor.png) | Why can O1 fail with an integrator? | The safety governor caps speed; O1 holds only left of the knee, which moves right as σρ falls. |
| [5](fig5_traces.png) | What does multipath do? | Noise is filtered; the slowly varying room bias steers the follower off and out of the calibrated window. |
| [6](fig6_tiers.png) | Which demo is feasible? | Tier 1 100/100/50 %, Tier 2 LQI 100/100/0 %, Tier 3 LQI 100/100/0 % (none / horn12 target / s1). |

## O5 (revised): margins of the sampled MIMO loop

Loop-at-a-time at the plant input, state feedback with actuator lags and T_eff. G(s) = diag(−1/s, −(s+a)/s²) and K is block-diagonal, so the two loops are exactly decoupled and loop-at-a-time margins equal the multiloop ones.

| Design | Loop | ωc [rad/s] | PM [°] | Delay margin beyond T_eff [s] | Disk α | Disk GM [dB] | Disk PM [°] |
|---|---|---|---|---|---|---|---|
| LQI | range | 0.68 | 47 | 1.21 | 0.79 | ±7.2 | ±43 |
| LQI | bearing | 0.58 | 59 | 1.78 | 1.01 | ±9.7 | ±54 |
| pole placement | range | 0.75 | 44 | 1.03 | 0.72 | ±6.5 | ±40 |
| pole placement | bearing | 0.73 | 49 | 1.17 | 0.83 | ±7.7 | ±45 |
| LQI, no information delay | range | 0.68 | 66 | 1.70 | 1.29 | ±13.4 | ±66 |
| LQI, no information delay | bearing | 0.58 | 75 | 2.27 | 1.52 | ±17.2 | ±74 |

## Gains

LQI K (rows δv, ω; cols e, β, ε, ∫e): `[[-0.6689, -0.0, 0.0, -0.1077], [-0.0, -0.4024, -0.1722, -0.0]]`
Pole placement K: `[[-0.7394, 0.0, 0.0, -0.1325], [0.0, -0.9625, 0.2523, 0.0]]`

Cross-check in MATLAB: `addpath('Matlab'); rover_reference_design('Runs/2026-11-28/20261128T105051_993805Z_phase_e')`.

## Tier outcomes (mean over seeds)

| Tier, controller, multipath | Success | Range error [m] | Bearing RMS [°] | TRACK |
|---|---|---|---|---|
| 1, lqi, none | 100 % | — | — | — |
| 1, lqi, horn12 target | 100 % | — | — | — |
| 1, lqi, s1 measured | 50 % | — | — | — |
| 2, lqi, none | 100 % | 0.00 | 0.6 | 0.98 |
| 2, lqi, horn12 target | 100 % | 0.05 | 8.7 | 0.98 |
| 2, lqi, s1 measured | 0 % | 2.70 | 44.4 | 0.35 |
| 2, place, none | 100 % | 0.00 | 0.2 | 0.98 |
| 2, place, horn12 target | 100 % | 0.06 | 8.2 | 0.98 |
| 2, place, s1 measured | 0 % | 1.49 | 34.4 | 0.53 |
| 2, bearing_only, none | 100 % | -0.07 | 0.3 | 0.98 |
| 2, bearing_only, horn12 target | 100 % | -0.06 | 8.9 | 0.98 |
| 2, bearing_only, s1 measured | 0 % | 0.71 | 32.0 | 0.64 |
| 3, lqi, none | 100 % | 0.00 | 0.6 | 0.98 |
| 3, lqi, horn12 target | 100 % | 0.04 | 17.7 | 0.96 |
| 3, lqi, s1 measured | 0 % | 6.23 | 42.7 | 0.24 |
| 3, place, none | 100 % | 0.00 | 0.7 | 0.98 |
| 3, place, horn12 target | 100 % | 0.07 | 13.8 | 0.98 |
| 3, place, s1 measured | 0 % | 4.12 | 14.1 | 0.33 |
| 3, bearing_only, none | 100 % | -0.09 | 0.5 | 0.98 |
| 3, bearing_only, horn12 target | 100 % | -0.07 | 12.5 | 0.98 |
| 3, bearing_only, s1 measured | 0 % | 6.05 | 6.4 | 0.29 |

2 seeds per cell; multipath levels: none; horn12 target (fringe 0.07, room 10°); s1 measured (0.144, 23°).

## Chapter 6 figures (lecture format)

| Figure | Shows |
|---|---|
| [C1](c1_lqr_effect_of_R.png) | LQR trade-off: peak speed command 0.53 / 0.20 / 0.09 m/s for R ×0.1 / ×1 / ×10 (limit 0.17). |
| [C2](c2_lqr_root_locus.png) | Closed-loop poles vs ρ (R = ρ R_design). |
| [C3](c3_lqr_vs_lqi.png) | 0.05 m/s target speed step: LQR offset 0.102 m, LQI 3.1e-05 m. |
| [C4](c4_lqe_tuning.png) | LQE: Kalman gain and estimates for V ×0.01 / ×1 / ×100; ε error minimised at V ≈ 178× for this gentle target. |
| [C5](c5_lqg.png) | LQG (LQI + Kalman at T_m): after 10 s, range within 0.17 m, bearing within 12°. |

Continuous LQI gain (scipy CARE, MATLAB `lqi` convention z' = −e): `[[-0.6776, -0.0, -0.0, 0.1095], [-0.0, -0.4082, -0.1734, 0.0]]`.
