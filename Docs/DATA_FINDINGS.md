# Data findings: all kept tests and data (to 28 Nov 2026)

One-page synthesis of every kept dataset: POC, Oct–Nov raw logs, 20 Nov bench and desk runs, 27–28 Nov rover1 runs. Numbers are taken from the run files and the analysis logs cited per section. All times UTC. "Volume NN %" is the speaker volume (older notes say "laptop 20 % / speaker 20 %": read as volume 20 %).

## 0. Where the data are

| Data | Location | Index |
|---|---|---|
| POC sweeps, bare mic vs horn (11 Oct) | `Data/poc/{bare,horn}/` | `run_registry.csv`, `sweep_plots/*_summary.csv` |
| Single-tone hardware logs E0–E4 (22 Oct – 3 Nov) | `Data/raw/<experiment>/` | [`Data/raw/README.md`](../Data/raw/README.md); fixes in `Data/corrections.csv` |
| Beacon WAVs and specs | `Data/stimuli/` (WAVs gitignored, regenerate with `Tools/make_tone.py`) | `firmware/esp32_acoustic_v2/README.md` |
| Desk runs and rover/bench captures (20–28 Nov) | `Runs/<date>/<stamp>_<task>/` | [`Runs/README.md`](../Runs/README.md) and one `README.md` per date |
| Local-only analysis runs and source snapshots | `Local/Runs/2026-11-20/` | `Local/Runs/README.md` |
| Measured parameter sets | `Config/measured_*.json`, `Config/rover1_hardware.json`, `Matlab/horn_model_parameters.txt` | per-key `sources` blocks |
| Controller design figures (20 Nov) | `Results/control_design/` | `README.md` table |
| Working logs | `Local/Docs/` (`NEXT_STEPS*.md`, `A2_RANGE_REFERENCE.md`, `PRESENTATION_FINDINGS.md`, `PHASE_E.md`) | — |
| Unused (empty, aborted, invalid, superseded, duplicate) | `Unused_Data/` | [`Unused_Data/README.md`](../Unused_Data/README.md), `MANIFEST.csv` |

---

## 1. Proof of concept (11 Oct, `Data/poc/`)

- Speaker at 8 azimuths, 30 cm, chirp at a constant frequency, MAX4466 + ESP32.
- **Bare mic:** 5.2–7.8 dB peak to trough, maximum near 0° (≈ 63 dB), ≈ 2 dB per 45°.
- **Horn:** +2 dB on axis (≈ 65 dB), **10.3–10.5 dB** peak to trough. The horn raises the angular contrast, which motivated the amplitude-ratio bearing sensor.

## 2. Single-tone hardware logs E0–E4 (22 Oct – 3 Nov, `Data/raw/`)

Firmware `esp32_acoustic_wired` (now `firmware/archive/`), 4 kHz tone.

- **E0 baseline (22 Oct):** old firmware, 12.84 s pair period, 0 % valid. Kept for replay and audit (`rover/config.py` `BASELINE_DIR`, input hashes).
- **E1 directivity (6 sessions, φ = −80 … +80°, 1.0 m):** outdoors a flat top within ±40° and ≈ −4 dB at ±80°; indoor shapes differ between sessions (peaks at +30° or ±70°), so the beam measured indoors is not repeatable. The "+80°" files were really at +70° (servo end stop at 180° with centre 110°); corrected in `corrections.csv`.
- **Horn model** (`Matlab/horn_model_parameters.txt`): Gaussian $\sigma = 77^\circ$, $c = 0$ (clipped alternative $w = 25^\circ$, $\sigma = 52^\circ$). Bearing estimator gain $\sigma^2/2\delta \approx 59^\circ$ per neper at $\delta = 50^\circ$.
- **E2 distance sweep, outdoors (0.27–2.03 m, ±3 cm triplets):** $A$ changes ≈ 2× within 3 cm ($\lambda_{4\,\text{kHz}} \approx 8.6$ cm): **multipath fringes**. $K = 84$ counts·m holds only at the 30 % volume and that gain-screw position; outdoor exponent $n = 0.84$ (log-RMSE 0.081). 100 % clipping at 0.33–0.50 m.
- **E3 held-out grid:** per-pair sd 3–7°, but **bias −42 … +30°**, not monotonic in β, and **not repeatable across sessions** (same pose: +80° vs −18°). Range error −65 … +20 %.
- **E4 timing:** deterministic, $T_\text{gap} = 72\,\text{ms} + 2\delta/\omega_s$ with $\omega_s = 300^\circ$/s, $T_m = T_\text{gap} + 72$ ms → $T_m = 0.477$ s at $\delta = 50^\circ$.
- **Ground-truth fixes** (raw logs never edited): `S_right45/…070208` is β = +45°; `0p3m_…132951` is ρ = 0.3 m; `3m_…133408` is ρ = 3.0 m; E1 ±70/80° end-stop rows.

## 3. Desk runs with the measured sensor values (20 Nov, `Runs/2026-11-20/`)

- **Controller suite** (`103330_893905Z_validation`, 432 simulations, 33 tests pass): **0/432 tracking passes**, 0 clearance violations. The supervisor stays in HOLD because its gates (0.25 m, 12°) are tighter than the measured floors ($0.2\rho$, 15–20°).
- **Sensitivity** (`103330_969443Z`): only gates ×2–3, $T_\text{gap} \le 0.18$ s or $\delta \le 30^\circ$ leave HOLD. Diagnostic gates ×2 (`Local/Runs/…103433`): 98 % TRACK, tail range RMSE 0.42 m; a **constant bearing bias passes ≈ 1:1 into bearing RMS**.
- **The range offset is the safety-governor equilibrium**, not a pair-gap lag: the estimator is unbiased (1.622 vs 1.620 m), and the governor ($\rho - 3\sigma_\rho$ − clearance) caps the speed at $v_t$. Range floor $0.2\rho$ → +0.42 m offset, $0.1\rho$ → +0.13 m, $0.05\rho$ → +0.04 m.
- **Single-tone calibration fails** (`110821`, `110826`): σ = 115–145°, boresight −10.7°, held-out bearing RMSE **37°**, range RMSE **33 %**.
- **Room-bias tolerance** (diagnostic, 8 seeds): the loop tracks at ≤ 10° room bias and ≤ 6° fringe; at 15° it loses the target in 3–5/8 runs, at 23° in 7–8/8.

## 4. Phase A bench hardware, `esp32_acoustic_v2` (20 Nov)

- **Acquisition solved:** E0 silence 0/395 accepted (`133416`); E0 tone 99.7 % valid, $T_m$ p95 0.151 s at δ = 1° (`133541`).
- **E4 law reproduced within 1 ms** in two sets; capture order alternates on 100 % of pairs (order bias ≤ 0.2°).
- **Servo-supply clipping:** 31–71 % valid at δ = 20–30°, also with the speaker off (59 % clip), falling to 25 % with 100 ms settle → servo-motion transients on the supply/ADC. Noise floor $s_n \approx 4.6$ counts vs 0.7 in the earlier indoor data.
- Bearing at β = 0 moved from −20° to −4° between two sets 20 min apart (scatter < 2° within a set).

## 5. Phase B: multipath diagnosis and the multi-tone fix (20 Nov)

- **Single tone, B2 fringe test** (`150910`–`151314`): **±70° bearing swing over 7.5 cm** of range. The single-tone bearing error is coherent multipath, not noise.
- **Multi-tone wide16, session s1** (16 tones, 2.5–6.25 kHz, Schroeder phase): bin-to-bin fringe correlation ≈ 0, so averaging across tones decorrelates the fringes. Only 3.25–4.5 kHz carry a usable bearing slope.
- **Error split:** held-out per-pose RMS 17–27° ≈ **12° fast fringe + 23° slow room bias** (fringe $\ln$ sd 0.144). The room part is position-specific and is not removed by tone averaging.
- **Multi-tone range** at 1.0–1.5 m: +5, −5, +2, −14 % on 4 held-out poses, against 48 % RMS for a single tone.
- **Phase E design, 20 Nov** (`Local/Runs/…191912`, `Config/phase_e.json`): Tier 1 at s1 multipath **62 %**, Tier 2 12 %, Tier 3 0 %.

## 6. rover1 Phase D (27 Nov, `Runs/2026-11-27/`)

- **Steering:** $\delta = 90^\circ - s$ at all 40 points (4 sweeps), sd 0, backlash < 5° (reading resolution); `max_steer_deg` 45.
- **USB transport delay:** $T_d$ = min RTT/2 (11.2 ms) + p95 arrival jitter (14.3 ms) = **25.4 ms**; 623 packets in 300 s confirm $T_m$ = 0.48 s.
- **Motor-on E0 silence** (lifted, horn12, wall power): gate PASS (0/199 accepted), but in-band $N$ 2.1–2.3 → 9.0 / 11.0 / 11.5 counts at PWM 25/45/80 (×4–5, ≈ +13 dB), raw pp ≈ 2400–3240 of 4095.

## 7. 28 Nov: calibration, power, motors, vehicle, design (`Runs/2026-11-28/`)

### 7.1 Bearing model
- Per-tone `bin_model` re-fitted on the 32 s1 B0 captures (`062919_bearing_cal`): train per-pose RMS 10.6°, **held-out E3 RMS 27.6°** (bias −0.3°, max 56°), per-pair sd 5.8°. With the C5 tolerance (≈ 10°), this supports **Tier 1 only**.

### 7.2 F1: ADC clipping came from the wall adapter
- Wall power, 1 m, nothing changed: clip **4.9–31 %** over six runs while $A$ (27.0–28.1) and $N$ (2.3–2.6) stay stable → intermittent broadband transients.
- Isolation: T1 muted 7.3 %, T2 δ = 1° 8.0 %, T3b servo PWM off 6.2 %, **T4 battery 0 %** (pp median 748, $N$ 1.84). Beacon and servo ruled out; path: adapter → Pi 5 V → ESP32 USB → 3.3 V → mic bias/ADC reference.
- T3: unplugging the servo connector kills the mic signal → **mic and servo share supply/ground wiring**.
- Practice since: all calibration and motor runs on battery; wall-power captures before 08:16 excluded from $K$, $n$.

### 7.3 F2: the range law and its domain
- Model $\ln A = \ln K - n\ln\rho - (\theta+b)^2/2\sigma^2$ on valid pairs, soft-L1, fitted on 1.0–1.5 m.
- **Pooled s1 + 28 Nov (3168 amplitudes): $K = 27.3$, $n = 1.01 \pm 0.02$, σ = 117°, log-RMSE 0.103** → spherical spreading. Per session $n$ differs (0.78 vs 1.14): it depends on where rover and speaker sit. $K$ reproduces across sessions within **1.4 %** (27.15 vs 27.53 at $n = 1$).
- **Domain:** 0.75 m reads **−15 … −24 %** (13/13 captures; also in s1 E3, not outdoors → near reflection); 2.0 m reads **+26 … +45 %** ($\hat\rho \approx 1.5$ m; reverberant field, $r_c = 0.057\sqrt{QV/T_{60}} \approx 0.8$–1.5 m). Range is observable indoors on **1.0–1.5 m** only.
- Range noise ≈ 11 % per pair plus ±10 % pose-to-pose room bias → `range_noise_floor_rel` = 0.15.
- Adopted in `Config/measured_config.json`: σ fixed at 77° (simulator bearing scale), $n = 1$, $K = 30.9$ at volume 20 %, rescaled ×2.18 to **$K = 67.4$** for the live setup (volume 80 %, trimmer mid, blanking firmware); `range_domain` [0.75, 2.5] m.

### 7.4 Listening while driving (A3 and mitigation, rover lifted, 1.0 m)
- Stock gain, PWM 0/25/45/80: valid 47 %, clip 51 %, pp 2922. Reducing the gain trimmer alone: valid 34–70 %, clip 26–65 %: **insufficient**.
- Trimmer mid + **firmware blanking** (`rail_max=10`): clip 22.5 % → **0 %**, valid 66 → 81 %.
- + **volume 80 %**: **100 % valid, 0 % clip at PWM 45 and PWM 80** (`153952`, `161854`), $A$ ≈ 58 vs 56–58 motors off.
- **The motors raise $N$, not the measurement scatter.** Per PWM segment (1 s trimmed at each transition, rover lifted, beacon static):

| Capture, volume | Segment | $N$ | sd of $\sqrt{A_-A_+}$ | Bearing sd (bin_model) |
|---|---|---|---|---|
| `153952`, 80 % | off / **PWM 45** / off | 1.21 / **7.91** / 1.27 | 1.64 / **1.52** / 2.71 | 8.4 / **9.1** / 5.4° |
| `161854`, 80 % | off / **PWM 80** / off | 1.19 / **5.60** / 1.29 | 1.28 / **1.19** / 2.81 | 2.9 / **4.6** / 12.3° |
| `152705`, 40 % | off / **PWM 45** / off | 1.28 / **6.96** / 1.29 | 0.43 / **0.89** / 0.30 | 5.2 / **11.0** / 3.2° |

  At volume 80 % ($A$ ≈ 58) the drive adds nothing measurable; at 40 % ($A$ ≈ 16) it adds ≈ 10° of bearing scatter and 58 % valid. The amplitude scatter follows $\sigma_A \approx N/\sqrt{2M}$ (16-tone power average), not $N$: the live estimator, which used raw $N$ in $R$, overstated the motor-on measurement noise ≈ 3–5×. Fixed with `noise_to_amplitude_sd` = 0.30 (calibrated on the 40 % segment; [review](REVIEW_2026-11-28.md) §2).

### 7.5 Mic/ADC fault (open)
- 13:11: pp = 0 on every pair (mic dead). 14:32–15:03: clip flag on 100 % of pairs with pp ≈ 260–400 and $A$ ≈ 0.5 (mic DC at a rail), at gain minimum. Recovered at 15:08 (`150822`: 100 % valid, $A$ 20.8).
- After the last run (≈ 16:20) the horn servo went silent and the mic again sat at a rail; suspected shared GND/3V3 wiring. **Not resolved in the data.**

### 7.5b Bearing boresight is not stable across the day
- Mean per-tone bearing at a taped β = 0, 1.0 m: **−0.4°** on the A2 spot (09:58, 3 captures), **−7°** at 15:08 (gain minimum, after rewiring), **−14 / −21°** on the floor at 15:42 (volume 80 %, trimmer mid, blanking firmware), **−17 … −44°** with the rover lifted.
- Within one static 90 s capture the 10 s means wander by up to ±13° (`153952`: −26, −44, −33, −22, −14 …) with an operator next to the rover: moving reflectors, not noise.
- Causes not separated: position-specific room bias (expected, 23° sd), a horn shift on the servo spline after the 08:00 refit and the 13–15 h rewiring, volume/trimmer change. The s1 model is therefore calibrated **at the A2 spot only**; re-zero at the Tier 1 start spot (`Tools/rezero_bin_model.py`).

### 7.6 Speed table (floor, battery)
- `155805_identify_speed`, 11 timed runs: PWM 25/45/80 → **0.041 / 0.100 / 0.203 m/s** (se 0.004 / 0.009 / 0.002 m/s). Speed and steer lags (`tau_*`) and braking are still provisional (no step videos).

### 7.7 Phase E with the measured values
- σ choice matters: σ = 117° gives Tier 1 at s1 **38 %** (`105159`), σ = 77° gives **75 %** (`105337`); 77° is kept because the multipath levels are calibrated on that scale.
- 16:26 runs (`162605_746387Z`, `measured_config.json`: Tier 1 100 / 100 / 100 %; `162605_746284Z`, motor variant with true noise 8.0: 88 / 88 / 50 %) are **superseded**: ideal range law, ±0.25 m success band, and a motor noise the data contradict ([review](REVIEW_2026-11-28.md) §2–3).
- **Corrected** (17:32 UTC, measured range law in the s1 level, pre-registered ±0.2 m): motors off `173205_452622Z` **Tier 1 100 / 100 / 50 %**; drive on `173402_499488Z` (true noise 1.9, reported $N$ 7.9) **Tier 1 100 / 100 / 62 %**. With 16 seeds: 56 % motors off, 75 % drive on (±10 % se).
- **Start range matters** (24 seeds, range law): 2.0 m → 54 %, **1.75 m → 83 %**, 1.5 m → 67 %. From 2.0 m the reverberant floor can read as ρ̂ ≈ 1.05 and latch the stand-off at the start.
- Structure: controllable and observable (ranks 3/3) but $\lambda_\text{min}(W_o) = 0.42$: ε is seen only through $\dot\beta = a\varepsilon$ with $a = v_0/d^* = 0.13$ s⁻¹; both ranks drop to 2 at $v_0 = 0$, so in `mission: approach` LQI and bearing-only differ only in the speed law.
- LQI margins (range / bearing): $\omega_c$ 0.68 / 0.58 rad/s, PM 48 / 60°, delay margin 1.23 / 1.80 s beyond $T_\text{eff} = 0.49$ s, disk α 0.81 / 1.03. Bandwidth limit $\omega_n \lesssim 0.3$ rad/s. LQR → LQI removes the 0.102 m speed-step offset.

---

## 8. Measured parameter set (current)

| Quantity | Value | Source |
|---|---|---|
| Horn beam σ, dither δ | 77° (Gaussian), 50° | E1; `horn_model_parameters.txt` |
| $T_m$, $T_\text{gap}$ at δ = 50° | 0.483 s, 0.405 s | E4, 20 Nov, 27 Nov latency |
| Bearing model | per-tone `bin_model`, held-out 27.6° RMS | `062919_bearing_cal` |
| Room bias, fringe | 23°, $\ln$ sd 0.144 | s1 |
| Range law | $K$ 30.9 (vol 20 %) → 67.4 (live), $n = 1$, domain [0.75, 2.5] m, accurate 1.0–1.5 m | pooled s1 + A2 |
| Range noise floor | 0.15 ρ | A2 + s1 E3 |
| Amplitude noise | 1.9 counts, motors off and on; $R$ uses $\max(1.9, 0.30\,N)$ | 28 Nov A3 segments |
| Range-law deviation (living room) | ln(A/A_model): 0.75 m −0.22, 1.0 +0.03, 1.3 −0.02, 1.5 −0.03, 2.0 +0.28; pose-to-pose sd 0.08 | A2 + s1 E3 (`simulation.S1_RANGE_BIAS_TABLE`) |
| Transport delay | 25.4 ms | `165058_identify_latency` |
| Steering | δ = 90 − s, ±45° | `164817_identify_steer` |
| Speed | 0.041 / 0.100 / 0.203 m/s at PWM 25/45/80 | `155805_identify_speed` |
| Vehicle lags, braking | **provisional** | — |

## 9. Data-quality caveats

- **`20261128T094848`, `094916`, `094945` were launched with `--rho 1.0` but were at 1.3 m.** All three are relabelled (`R_mt_r1.30`, ρ = 1.3, originals in `*.orig`). The first two were relabelled a second time after a scp from the Pi overwrote the first fix; the Pi copies still say 1.0, so don't scp those folders back.
- `20261128T094628` (`R_mt_r1.00_bat`) reads 1.27 m; position unconfirmed, excluded.
- $K$ is valid only for the stated volume and gain-trimmer position; any change needs a 1.0 m reference capture and a rescale.
- The s1 bearing model is trained at one spot (ρ 1.00–1.04 m, β = 0) in the living room; room bias is position- and room-specific.
- Wall-power captures (28 Nov before 08:16) and the 27 Nov motor-on silence (wall power) are not comparable with battery captures for $K$, $n$, $N$.
- `Data/raw/Exp3_1/E3_r1.00_b0_20261103_062008.csv` has no `.json` sidecar.

## 10. Open items the data do not yet cover

- No closed-loop hardware run yet (Tier 1 stop test, D1 commissioning, D2 trials).
- No zero check at the Tier 1 start spot (§7.5b); bearing boresight there is unknown to ±25°.
- Mic/servo wiring fault (§7.5) and the shared 5 V/ground path (§7.2).
- 0.75 m mechanism (floor reflection vs height offset) and the battery + sweeping-servo clipping residue (0–17 % per run in A2, all rejected by the validity flag).
- horn12 profile: no B0/E3 calibration captures; demo-room calibration not done.
- Speed/steer step responses and braking distance.
