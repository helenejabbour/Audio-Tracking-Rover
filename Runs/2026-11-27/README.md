# Runs, 27 Nov 2026 (rover1, Phase D)

All times UTC. Results: [`Docs/DATA_FINDINGS.md`](../../Docs/DATA_FINDINGS.md) §6. The three superseded steering runs (`163810` invalid, `164322` wrong values, `164746` aborted) are in [`Unused_Data/Runs/2026-11-27/`](../../Unused_Data/README.md).

| Run | What | Result |
|---|---|---|
| `20261127T164817_249191Z_identify_steer` | steering table, lifted, protractor, 4 sweeps | **valid**: δ = 90 − s at all 40 points, sd 0, backlash < 5° (reading resolution) |
| `20261127T165058_166535Z_identify_latency` | 300 s ping/arrival log over USB | min RTT/2 11.2 ms + p95 jitter 14.3 ms → **$T_d$ = 25.4 ms**; 623 packets → $T_m$ = 0.48 s at δ = 50° |
| `20261127T174120_078885Z_capture` + `20261127T174127_635711Z_identify_spin` | motor-on E0 **silence**, horn12, lifted, PWM 0/25/45/80/0 × 15 s, wall power | gate PASS (0/199 accepted); $N$ 2.1–2.3 → 9.0/11.0/11.5 counts at PWM 25/45/80 (×4–5); pp ≈ 2400–3240 |

Both valid Phase D results feed `Config/rover1_hardware.json` and `Config/measured_config.json`.
