# Runs, 28 Nov 2026 (rover1, living room)

All times UTC. Beacon wide16 (16 tones, 2.5–6.25 kHz) and `Config/measured_multitone.json` unless noted. Folder names are `20261128T<HHMMSS>_<µs>Z_<task>`; the short IDs below are `<HHMMSS>`. Run paths are unchanged: configs, docs and tools refer to them.

Empty, aborted, incomplete and disregarded runs from this day are in [`Unused_Data/Runs/2026-11-28/`](../../Unused_Data/README.md). Results: [`Docs/DATA_FINDINGS.md`](../../Docs/DATA_FINDINGS.md) §7.

## 1. Bearing model re-fit (desk)

| Run | What | Result |
|---|---|---|
| `062919_bearing_cal` | `rover.bearing_cal --per-tone` on the 20 Nov s1 B0 sweeps (32 captures), E3 held out | **Adopted `bin_model`** (in `Config/measured_config.json`). Train per-pose RMS 10.6°, held-out RMS **27.6°** (6 poses, bias −0.3°, max 56°) |

## 2. A2 range reference on wall power (07:19–07:49), excluded from $K$, $n$

Pi on the USB wall adapter, 20 s, motors off, volume 20 %.

| Run | ρ (m) | Note |
|---|---|---|
| `071930` | 1.00 | horn refitted straight at servo 110; near the wall (superseded by `072421`) |
| `072421`, `072542` | 1.00 | s1 spot (couches) |
| `072830` | 0.75 | |
| `073135` | 1.50 | |
| `074325` | 1.00 | |
| `074601`, `074633`, `074702`, `074733`, `074839`, `074914` | 1.00 | **F1 symptom set**: clip 4.9–31 %, $A$ and $N$ stable |

## 3. F1 power-source isolation (08:00–08:16)

| Run | Test | Clip | Note |
|---|---|---|---|
| `080030` | T1, speaker muted, wall | 7.3 % | beacon ruled out |
| `080238` | T2, δ = 1°, wall | 8.0 % | servo motion ruled out |
| `080632`, `080704` | T3, servo connector unplugged | — | **mic signal dies (pp = 0)**: mic and servo share supply/ground |
| `080931` | T3b, servo signal wire off, wall | 6.2 % | servo current ruled out |
| `081613` | T4, **Pi and laptop on battery** | **0 %** | pp median 748, $N$ 1.84 |

## 4. A2 range reference on battery (09:46–10:18), used for $K$, $n$

29 captures in the pooled fit (`Local/Docs/A2_RANGE_REFERENCE.md`), 20 s each, motors off, horn sweep δ = 50°.

| ρ (m) | Runs | Note |
|---|---|---|
| — | `094628` (`R_mt_r1.00_bat`) | **excluded**: reads 1.27 m, position unconfirmed |
| 1.30 | `094848`, `094916`, `094945` | launched as `--rho 1.0`, relabelled `R_mt_r1.30` / ρ = 1.3 in `session.json` and `samples.csv` (`label`, `rho_true`); originals in `*.orig`. `094848` and `094916` were re-relabelled on 28 Nov evening after a scp from the Pi had overwritten the first relabel; the Pi copies of those two still say 1.0 |
| 1.00 | `095243`, `095313`, `095750`, `095817`, `095844` | `0957–0958` is the tabulated set |
| 0.75 | `100024`, `100058`, `100213`, `100351`, `100435`, `100518`, `100557`, `100629`, `100706`, `100829`, `100910`, `100941`, `101015` | reads −15 … −24 % (near reflection) |
| 1.50 | `101137`, `101209`, `101254` | |
| 2.00 | `101558`, `101621`, `101648`, `101716`, `101746` | reads +26 … +45 % (reverberant floor) |

## 5. Phase E with the measured config (10:50–10:54)

| Run | Config | Tier 1 (none / horn12 / s1 multipath) |
|---|---|---|
| `105051_phase_e` | `Config/phase_e.json` (20 Nov design values), re-run | 100 / 100 / 50 % |
| `105159_phase_e` | measured, σ = 117° (free fit) | 100 / 100 / **38 %** |
| `105337_phase_e` | measured, σ fixed at 77°, $K$ = 30.9 | 100 / 100 / 75 % |

## 6. A3 motor-on tone, rover lifted, 1.0 m (10:58–12:00)

Each capture is paired with the `identify_spin` run a few seconds later (`segments.csv` = PWM schedule in wall-clock time).

| Capture + spin | Setup | Valid | Clip | pp median |
|---|---|---|---|---|
| `105808` + `105857` | stock gain, PWM 0/25/45/80/0 × 15 s | 47 % | 51 % | 2922 |
| `113343` + `113354` | gain reduced, PWM 0/45/0 × 20 s | 58 % | 27 % | 1522 |
| `114413` + `114426` | gain halved | 34 % | 65 % | 4095 |
| `115947` + `120000` | gain halved, repeat | 70 % | 26 % | 1203 |

## 7. Mic/ADC fault and recovery (13:11–15:09)

Kept as fault evidence; none of these is usable for $K$, $n$, $N$ or bearing.

| Runs | State |
|---|---|
| `131105` + `131112` | pp = 0 on every pair: mic signal dead |
| `132745`, `133108`, `133145` (`link_test`) | serial-link checks; 46 % then 100 % valid, $A$ ≈ 11.6 |
| `143249`, `144304`, `144517`, `144547`, `145300`, `145922`, `145956`, `150124`, `150154`, `150222`, `150255`, `150322` (`R_mt_r1.00_g2`) | gain trimmer at minimum, speaker 40 %, mic wire re-routed: clip flag on 100 % of pairs with pp ≈ 260–400 and $A$ ≈ 0.5 (mic DC stuck at a rail) |
| `150822`, `150856` | **recovered**: 100 % valid, $A$ 20.8 / 19.6, $N$ 1.2 |

## 8. Listening while driving, rover lifted, 1.0 m (15:16–16:19)

| Capture + spin | Setup | Valid | Clip | $A$ median |
|---|---|---|---|---|
| `151631` + `151644` | trimmer mid, speaker 40 %, PWM 45 | 66 % | 22.5 % | 15.0 |
| `152705` + `152712` | + firmware blanking `rail_max=10` | 81 % | **0 %** | 16.0 |
| `153952` + `153958` | + **volume 80 %**, PWM 45 | **100 %** | **0 %** | 58.7 |
| `154247`, `154404` | volume 80 %, motors off, on the floor (reference) | 100 % | 0 % | 56–58 |
| `161854` + `161901` | volume 80 %, PWM 80 | **100 %** | **0 %** | 57.3 |

## 9. Vehicle identification and design (15:58–16:26)

| Run | What | Result |
|---|---|---|
| `155805_identify_speed` | A4 speed table on the floor, 11 timed runs (`speed_runs.csv`) | PWM 25/45/80 → 0.041 / 0.100 / 0.203 m/s |
| `162340_identify_fit` | `rover.identify fit` on the speed run + 27 Nov steer + latency | `proposed_vehicle.json` (speed, steering, $T_d$ = 25.4 ms) |
| `162605_746387Z_phase_e` | **`Config/measured_config.json`** (σ_n = 1.9) | **Tier 1 100 / 100 / 100 %**; Tier 2 LQI 100/100/0; Tier 3 100/0/0 |
| `162605_746284Z_phase_e` | **`Config/measured_config_motor.json`** (σ_n = 8.0) | **Tier 1 88 / 88 / 50 %**; Tiers 2–3 0 % |

Both 16:26 runs are superseded by §10 (ideal range law, ±0.25 m band, wrong motor-noise model; see `Docs/REVIEW_2026-11-28.md`).

## 10. Phase E after the evening review (17:32–17:34)

| Run | Config | Tier 1 (none / horn12 / s1 multipath) |
|---|---|---|
| `173205_452622Z_phase_e` | `measured_config.json` (noise scale 0.30), s1 level with the measured range law, ±0.2 m | 100 / 100 / **50 %** |
| `173402_499488Z_phase_e` | `measured_config_motor.json` (true noise 1.9, reported $N$ 7.9) | 100 / 100 / **62 %** |

## Copying from the Pi

The Pi still holds the runs moved to `Unused_Data/`. A plain `scp -r …/Runs/2026-11-28 Runs/` copies them back; copy only new run folders, or delete the returned ones again (they are listed in `Unused_Data/MANIFEST.csv`).
