# Audio Tracking Rover

Engineering reference implementation for EECE 660 / MECH 653. Design notes:
[`Docs/CONTROL_DESIGN.md`](Docs/CONTROL_DESIGN.md).

Run index: [`Runs/README.md`](Runs/README.md).

The original report, data, MATLAB model and proof-of-concept scripts are retained.
The validated software path is `rover/`, with timestamped experiments in `Runs/`.
Simulation parameters are **assumptions**, not measured hardware values. Physical
tracking is not validated by a simulation pass.

```powershell
python -m rover.validate --seeds 12
python -m rover.sensitivity --seeds 6 --seed-start 4000
python -m rover.live --config Config/simulation.json --replay Data/raw/2026-10-22_E0_baseline/E0_static_20261022_134646.csv
```

Python 3.9+ (the code parses as 3.9; the Pi uses Raspberry Pi OS Bookworm, Python 3.11) with NumPy, SciPy,
Matplotlib and pyserial is required; see `requirements.txt`. The live controller defaults to recording commands without
driving. Hardware motion requires an explicit calibrated configuration and `--drive`.

| Folder | Purpose |
|---|---|
| `rover/` | Shared sensing, observer, controller, simulation, calibration (`calibrate`, in-room `bearing_cal`), vehicle identification (`identify`) and Pi runtime |
| `Config/` | Explicit parameter provenance, measured sensor values and hardware calibration template |
| `firmware/esp32_acoustic_v2/` | Current versioned USB protocol (CRC, capture times, bounded servo commands) |
| `firmware/archive/` | Older sketches: wired and wireless v1, and the POC sketches (`poc/`, `poc_0_test_mic/`) |
| `Matlab/` | Gain cross-check `rover_reference_design.m` and measured horn parameters |
| `Hardware/` | Mechanical design files (horn and tube) |
| `Data/raw/` | Raw hardware logs, one folder per experiment (E0–E4, `small_Exp3`); `2026-10-22_E0_baseline/` is the legacy static log used for replay |
| `Data/poc/`, `Data/stimuli/`, `Data/corrections.csv` | POC sweep results, beacon specs (regenerate the `.wav` files with `Tools/make_tone.py`), ground-truth corrections for raw logs (raw logs are never edited) |
| `Tools/` | Logging, plotting and beacon utilities (`esp_logger.py`, `plot_directivity.py`, `make_tone.py`, `beacon_spec.py`), Phase B session and analysis (`phase_b_session.py`, `phase_b_multitone.py`) |
| `POC/` | Original proof-of-concept analysis scripts |
| `Runs/YYYY-MM-DD/<UTC timestamp>_<task>/` | Hardware captures (20 Nov sessions), validation, sensitivity and calibration runs with config snapshots, metrics and source hashes |
| `Docs/` | Design notes (`CONTROL_DESIGN.md`) and `Progress_Report/` |

Paths are case-sensitive (Pi).

The syllabus permits assistive AI use but reserves submitted work for students'
own understanding and effort. This package is a development and learning reference;
the team must understand the design, conduct the hardware experiments and write its
own submitted interpretation.
