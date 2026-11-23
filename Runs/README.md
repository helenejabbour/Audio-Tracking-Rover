# Experiment index

All timestamps are UTC. Original hardware data remain in `Data/` and `Data/poc/`.

| Run | Role/status |
|---|---|
| [20261120T103330_893905Z_validation](2026-11-20/20261120T103330_893905Z_validation/REPORT.md) | **Current controller suite with experimentally measured sensor values** (`Config/measured_sensor.json`), 432 simulations, seeds 3000–3011, 33 tests; 0/432 tracking passes (supervisor HOLD) |
| [20261120T103330_969443Z_sensitivity](2026-11-20/20261120T103330_969443Z_sensitivity/REPORT.md) | **Current sensitivity, measured sensor config**, 186 simulations, seeds 4000–4005; adds bearing-bias, E4 dither-timing, floor and gate sweeps |
| [20261120T110821_676516Z_calibration](2026-11-20/20261120T110821_676516Z_calibration/calibration_report.json) | **Current** real-data calibration, outdoor E1 + E2 → E3, with E1 servo end-stop corrections; **fails** (bearing RMSE 37.3°, range 33 %, boresight −10.7°) |
| [20261120T110826_019513Z_calibration](2026-11-20/20261120T110826_019513Z_calibration/calibration_report.json) | **Current** real-data calibration, all E1 + E2 → E3, with end-stop corrections; **fails** (bearing RMSE 37.1°, range 33 %) |
| `2026-11-20/*_capture` | Hardware captures from the 20 Nov sessions (E0, E4, B0, B2, E3, single-tone and multi-tone); `phase_b_s1_manifest.json` lists the multi-tone session |

Runs save the configuration and source hashes (`manifest.json`). To reproduce a run,
check out the commit whose source hashes match and run the recorded command/config. Input data hashes are in the manifest.
Do not mix a newer runtime with an older run's configuration without checking schema
and behavioral changes.

The individual metrics retain every seed and failure. Example CSV traces and plots
use the first seed in each run; they are not selected as the best-performing example.
