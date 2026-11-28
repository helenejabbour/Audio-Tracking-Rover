# Experiment index

All timestamps are UTC. Each run is `Runs/YYYY-MM-DD/<UTC timestamp>_<task>/`, the layout `rover.config.create_run` writes; run paths are not renamed because configs, docs and tools refer to them. Original Oct–Nov hardware logs are in `Data/raw/` and `Data/poc/`. Findings across all of it: [`Docs/DATA_FINDINGS.md`](../Docs/DATA_FINDINGS.md).

| Date | Index | Contents |
|---|---|---|
| 20 Nov | [2026-11-20/README.md](2026-11-20/README.md) | desk validation, sensitivity and calibration with the measured sensor values; Phase A (E0, E4) and Phase B (single tone, multi-tone session s1) hardware captures |
| 27 Nov | [2026-11-27/README.md](2026-11-27/README.md) | rover1 Phase D: steering table, USB latency, motor-on E0 silence |
| 28 Nov | [2026-11-28/README.md](2026-11-28/README.md) | s1 bearing re-fit, A2 range reference (wall power, then battery), F1 power isolation, A3 motor-on tests, mic fault, listening while driving, speed table, vehicle fit, Phase E with the measured config |

Current reference runs:

| Run | Role |
|---|---|
| [20261120T103330_893905Z_validation](2026-11-20/20261120T103330_893905Z_validation/REPORT.md) | controller suite with the measured sensor values (`Config/measured_sensor.json`), 432 simulations, seeds 3000–3011, 33 tests; 0/432 tracking passes (supervisor HOLD) |
| [20261120T103330_969443Z_sensitivity](2026-11-20/20261120T103330_969443Z_sensitivity/REPORT.md) | sensitivity, measured sensor config, 186 simulations, seeds 4000–4005 |
| [20261120T110821_676516Z_calibration](2026-11-20/20261120T110821_676516Z_calibration/calibration_report.json), [20261120T110826_019513Z_calibration](2026-11-20/20261120T110826_019513Z_calibration/calibration_report.json) | single-tone real-data calibration E1 + E2 → E3; both **fail** (bearing RMSE 37°, range 33 %) |
| [20261128T062919_226852Z_bearing_cal](2026-11-28/20261128T062919_226852Z_bearing_cal/) | adopted per-tone bearing model (s1 B0 → E3 held out 27.6° RMS) |
| [20261127T164817_249191Z_identify_steer](2026-11-27/20261127T164817_249191Z_identify_steer/), [20261127T165058_166535Z_identify_latency](2026-11-27/20261127T165058_166535Z_identify_latency/), [20261128T155805_660561Z_identify_speed](2026-11-28/20261128T155805_660561Z_identify_speed/) | vehicle identification (steering, $T_d$, speed table) |
| [20261128T162605_746387Z_phase_e](2026-11-28/20261128T162605_746387Z_phase_e/REPORT.md), [20261128T162605_746284Z_phase_e](2026-11-28/20261128T162605_746284Z_phase_e/REPORT.md) | Phase E design with `Config/measured_config.json` and `Config/measured_config_motor.json` |

Elsewhere:
- `Local/Runs/2026-11-20/` (gitignored): source snapshots of the 20 Nov runs plus four Local-only analysis runs; see `Local/Runs/README.md`.
- [`Unused_Data/`](../Unused_Data/README.md): empty, aborted, incomplete, invalidated and superseded runs, with the reason for each in `MANIFEST.csv`.

Runs save the configuration and source hashes (`manifest.json`). To reproduce a run, check out the commit whose source hashes match and run the recorded command/config. Input data hashes are in the manifest. Do not mix a newer runtime with an older run's configuration without checking schema and behavioral changes.

The individual metrics retain every seed and failure. Example CSV traces and plots use the first seed in each run; they are not selected as the best-performing example.
