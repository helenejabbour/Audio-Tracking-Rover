# Phase E results (20261128T162605_746284Z_phase_e)

Config `Config/phase_e.json` (measured sensor and timing, provisional vehicle). Operating point d* = 1.4 m, v0 = 0.18 m/s, a = v0/d* = 0.129 1/s, T_m = 0.483 s, pair gap 0.405 s, information delay T_eff = 0.49 s.

| Figure | Question | Answer |
|---|---|---|
| [1](fig1_structure.png) | Controllable/observable at our operating point? | Yes (ranks 3/3), but λmin(Wo) = 0.42: ε is only seen through β' = aε with a = 0.13; at v0 = 0 both ranks drop to 2. |
| [2](fig2_bandwidth.png) | How fast may the loop be? | ωn ≲ 0.3 rad/s. LQI: crossover 0.68/0.58 rad/s, PM 48/60°, delay margin 1.23/1.80 s beyond T_eff, disk α 0.81/1.03. |
| [3](fig3_poles.png) | Separation of controller and observer? | Observer |z| per T_m: 0.78, 0.78, 0.84, 0.92, 0.92. |
| [4](fig4_governor.png) | Why can O1 fail with an integrator? | The safety governor caps speed; O1 holds only left of the knee, which moves right as σρ falls. |
| [5](fig5_traces.png) | What does multipath do? | Noise is filtered; the slowly varying room bias steers the follower off and out of the calibrated window. |
| [6](fig6_tiers.png) | Which demo is feasible? | Tier 1 88/88/50 %, Tier 2 LQI 0/0/0 %, Tier 3 LQI 0/0/0 % (none / horn12 target / s1). |

## O5 (revised): margins of the sampled MIMO loop

Loop-at-a-time at the plant input, state feedback with actuator lags and T_eff. G(s) = diag(−1/s, −(s+a)/s²) and K is block-diagonal, so the two loops are exactly decoupled and loop-at-a-time margins equal the multiloop ones.

| Design | Loop | ωc [rad/s] | PM [°] | Delay margin beyond T_eff [s] | Disk α | Disk GM [dB] | Disk PM [°] |
|---|---|---|---|---|---|---|---|
| LQI | range | 0.68 | 48 | 1.23 | 0.81 | ±7.5 | ±44 |
| LQI | bearing | 0.58 | 60 | 1.80 | 1.03 | ±9.9 | ±55 |
| pole placement | range | 0.75 | 45 | 1.05 | 0.74 | ±6.8 | ±41 |
| pole placement | bearing | 0.73 | 50 | 1.20 | 0.85 | ±7.9 | ±46 |
| LQI, no information delay | range | 0.68 | 66 | 1.70 | 1.29 | ±13.4 | ±66 |
| LQI, no information delay | bearing | 0.58 | 75 | 2.27 | 1.52 | ±17.2 | ±74 |

## Gains

LQI K (rows δv, ω; cols e, β, ε, ∫e): `[[-0.6689, -0.0, 0.0, -0.1077], [-0.0, -0.4024, -0.1722, -0.0]]`
Pole placement K: `[[-0.7394, 0.0, 0.0, -0.1325], [0.0, -0.9625, 0.2523, 0.0]]`

Cross-check in MATLAB: `addpath('Matlab'); rover_reference_design('Runs/2026-11-28/20261128T162605_746284Z_phase_e')`.

## Tier outcomes (mean over seeds)

| Tier, controller, multipath | Success | Range error [m] | Bearing RMS [°] | TRACK |
|---|---|---|---|---|
| 1, lqi, none | 88 % | — | — | — |
| 1, lqi, horn12 target | 88 % | — | — | — |
| 1, lqi, s1 measured | 50 % | — | — | — |
| 2, lqi, none | 0 % | 3.13 | 10.4 | 0.33 |
| 2, lqi, horn12 target | 0 % | 3.32 | 14.0 | 0.31 |
| 2, lqi, s1 measured | 0 % | 4.21 | 19.9 | 0.18 |
| 2, place, none | 0 % | 2.60 | 11.9 | 0.44 |
| 2, place, horn12 target | 0 % | 4.04 | 15.0 | 0.21 |
| 2, place, s1 measured | 0 % | 4.59 | 10.9 | 0.13 |
| 2, bearing_only, none | 0 % | 2.58 | 10.9 | 0.44 |
| 2, bearing_only, horn12 target | 0 % | 3.60 | 15.2 | 0.26 |
| 2, bearing_only, s1 measured | 0 % | 4.32 | 18.3 | 0.15 |
| 3, lqi, none | 0 % | 9.04 | 15.0 | 0.04 |
| 3, lqi, horn12 target | 0 % | 9.00 | 15.7 | 0.05 |
| 3, lqi, s1 measured | 0 % | 9.03 | 13.7 | 0.05 |
| 3, place, none | 0 % | 9.08 | 13.5 | 0.04 |
| 3, place, horn12 target | 0 % | 9.03 | 14.5 | 0.05 |
| 3, place, s1 measured | 0 % | 9.06 | 16.9 | 0.04 |
| 3, bearing_only, none | 0 % | 9.06 | 14.0 | 0.04 |
| 3, bearing_only, horn12 target | 0 % | 9.00 | 15.9 | 0.05 |
| 3, bearing_only, s1 measured | 0 % | 9.04 | 15.5 | 0.04 |

8 seeds per cell; multipath levels: none; horn12 target (fringe 0.07, room 10°); s1 measured (0.144, 23°).

## Chapter 6 figures (lecture format)

| Figure | Shows |
|---|---|
| [C1](c1_lqr_effect_of_R.png) | LQR trade-off: peak speed command 0.53 / 0.20 / 0.09 m/s for R ×0.1 / ×1 / ×10 (limit 0.02). |
| [C2](c2_lqr_root_locus.png) | Closed-loop poles vs ρ (R = ρ R_design). |
| [C3](c3_lqr_vs_lqi.png) | 0.05 m/s target speed step: LQR offset 0.102 m, LQI 3.1e-05 m. |
| [C4](c4_lqe_tuning.png) | LQE: Kalman gain and estimates for V ×0.01 / ×1 / ×100; ε error minimised at V ≈ 178× for this gentle target. |
| [C5](c5_lqg.png) | LQG (LQI + Kalman at T_m): after 10 s, range within 0.56 m, bearing within 12°. |

Continuous LQI gain (scipy CARE, MATLAB `lqi` convention z' = −e): `[[-0.6776, -0.0, -0.0, 0.1095], [-0.0, -0.4082, -0.1734, 0.0]]`.
