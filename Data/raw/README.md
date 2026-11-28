# Raw hardware logs

One folder per experiment; each run is a `.csv` (samples) plus a `.json` (arguments, firmware config, summary).
Raw logs are never edited; ground-truth fixes go in `../corrections.csv`.
These logs were previously under `Tools/data/`.

| Folder | Experiment | Runs | Notes |
|---|---|---|---|
| `2026-10-22_E0_baseline/` | E0 legacy static and silence logs | 4 | Used for replay and audit (`rover/config.py`) |
| `Experiment0/` | E0 tone, static ranges (0.3, 1, 3 m) | 9 | Horn and back panel |
| `Experiments 1/Experiment1`, `Experiment1_2`, `Experiment1_3`, `Experiment1_4`, `Experiment1_outdoors`, `Experiment1_out2` | E1 directivity | 18–20 each | Indoor (noisy house) and outdoor |
| `Exp2_1/` | E2 distance sweep, outdoor | 25 | ρ = 0.27–2.03 m |
| `Exp3_1/` | E3 held-out tests, outdoor | 12 | |
| `Exp3_2/` | E3 tile grid | 6 | Tile pitch 0.615 m |
| `small_Exp3/` | E3 tile repeat, right then left | 7 | |
| `Exp4_1/`, `Exp4_2/` | E4 timing | 5 each | Feeds the measured timing in `Config/measured_sensor.json` |
| `S_right45/` | Single ±45° sanity captures, 3 Nov | 2 | `…070208` was really at β = +45° (left); fixed in `../corrections.csv` (keyed by file name) |

`Exp3_1/E3_r1.00_b0_20261103_062008.csv` has no `.json` sidecar (50 pairs; the arguments are in the CSV columns).

Related: firmware in `../../firmware/archive/esp32_acoustic_wired/`, horn parameters in `../../Matlab/horn_model_parameters.txt`.
