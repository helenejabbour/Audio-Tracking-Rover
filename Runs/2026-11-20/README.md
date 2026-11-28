# Runs, 20 Nov 2026 (bench, living room, ESP32 on laptop USB)

All times UTC. Results: [`Docs/DATA_FINDINGS.md`](../../Docs/DATA_FINDINGS.md) §3–5. Source snapshots of these runs are in `Local/Runs/2026-11-20/` (gitignored). Moved to [`Unused_Data/Runs/2026-11-20/`](../../Unused_Data/README.md): `160356`, `161423` (empty), `133134` (silence label with the speaker on) and the speaker-off E4 batch `140220`–`140453`.

## 1. Desk runs with the measured sensor values (`Config/measured_sensor.json`)

| Run | Result |
|---|---|
| `20261120T103330_893905Z_validation` | 432 simulations (LQI, pole placement, bearing-only × 12 scenarios × 12 seeds), 33 tests pass: **0/432 tracking passes**, supervisor in HOLD (gates tighter than the measured floors) |
| `20261120T103330_969443Z_sensitivity` | 186 simulations: everything except gates ×2–3, $T_\text{gap}$ ≤ 0.18 s and δ ≤ 30° stays in HOLD |
| `20261120T110821_676516Z_calibration` | outdoor E1 + E2 → E3: **fails**, bearing RMSE 37.3°, range 33 %, boresight −10.7° |
| `20261120T110826_019513Z_calibration` | all E1 + E2 → E3: **fails**, bearing RMSE 37.1°, range 33 % |

## 2. Phase A hardware, single 4 kHz tone (13:32–14:12)

| Runs | What | Result |
|---|---|---|
| `133416` | E0 silence, 60 s | 395 pairs, **0 accepted** |
| `133236`, `133541` | E0 tone, 1.0 m | 99.7 % valid, $T_m$ p95 0.151 s |
| `134337`–`134548`, `135604`–`135815` | E4 timing, δ = 20/30/40/50/60° (two sets) | $T_\text{gap}$ = 72 ms + 2δ/ω_s within 1 ms; capture order alternates on 100 % of pairs |
| `141022`, `141224`, `141250` | clipping diagnostics (tone check, settle 100 ms, silence at δ = 20°) | 59 % clip with the speaker off → servo-supply transients |

## 3. Phase B, single 4 kHz tone (14:44–15:13)

| Runs | What |
|---|---|
| `144407`–`144704` | B0 boresight sweep, φ_c = −30 … +20° (11 captures), ρ = 1.0 m |
| `150910`–`151314` | B2 fringe test, ρ = 1.000–1.075 m in 15 mm steps: **±70° bearing swing over 7.5 cm** |

## 4. Phase B session s1, multi-tone wide16, volume 20 % (16:04–17:37)

Manifest: [`phase_b_s1_manifest.json`](phase_b_s1_manifest.json) (still lists the empty `161423`, now in `Unused_Data/`).

| Runs | What |
|---|---|
| `160409`, `160613` | E0 silence and tone, multi-tone |
| `161546`–`161853` | B0, ρ = 1.00, φ_c = −25 … +20° |
| `162737`–`163334` | B2, ρ = 1.000–1.075 m |
| `163807`–`164330` | B0, ρ = 1.02 m (`164330`: beacon stopped after pair 12; 11 valid pairs used) |
| `164610`–`164928` | B0, ρ = 1.04 m |
| `170436`, `170523`, `170546`, `171305`, `171456`, `172518`, `173305`, `173713` | E3 held out (0.75 m β −30/+15, 1.0 m β 0, 1.5 m β −30/0/+30). `170436` (46 % clipped) and `170546` (17 pairs) were repeated as `170523`, `171305`; `173305`, `173713` carry `annotation_fix.txt` |

The 32 B0 captures train the adopted bearing model (`Runs/2026-11-28/20261128T062919_226852Z_bearing_cal`); B0 + B2 + E3 enter the pooled range fit.
