# Audio Tracking Rover

Engineering reference implementation for EECE 660 / MECH 653. Start with
[`Docs/IMPLEMENTATION_GUIDE.md`](Docs/IMPLEMENTATION_GUIDE.md) and
[`Docs/CONTROL_DESIGN.md`](Docs/CONTROL_DESIGN.md).

Current results: [`Docs/VALIDATION_STATUS.md`](Docs/VALIDATION_STATUS.md).
Measurements needed next: [`Docs/MEASUREMENTS_NEEDED.md`](Docs/MEASUREMENTS_NEEDED.md).

The original report, data, MATLAB model and proof-of-concept scripts are retained.
The validated software path is `rover/`, with timestamped experiments in `Runs/`.
Simulation parameters are **assumptions**, not measured hardware values. Physical
tracking is not validated by a simulation pass.

```powershell
python -m unittest discover -s Tests -v
python -m rover.validate --seeds 12
python -m rover.sensitivity --seeds 6 --seed-start 4000
python -m rover.live --config Config/simulation.json --replay Data/E0_static_20261022_134646.csv
```

Python 3.10+ with NumPy, SciPy, Matplotlib and pyserial is required; see
`requirements.txt`. The live controller defaults to recording commands without
driving. Hardware motion requires an explicit calibrated configuration and `--drive`.

| Folder | Purpose |
|---|---|
| `rover/` | Shared sensing, observer, controller, simulation, calibration and Pi runtime |
| `Config/` | Explicit parameter provenance and hardware calibration template |
| `Tests/` | Numerical, protocol, delayed-estimation and runtime regression tests |
| `Firmware/esp32_acoustic_v2/` | Versioned USB protocol with CRC, capture times and bounded servo commands |
| `Matlab/` | Original work plus independent reference design |
| `Runs/YYYY-MM-DD/<UTC timestamp>_<task>/` | Config snapshots, raw outputs, metrics, plots and source hashes |
| `Docs/` | Design rationale, hardware procedure, evidence and course-objective mapping |
| `Data/`, `POC/` | Original experimental evidence; never overwritten by validation |

The syllabus permits assistive AI use but reserves submitted work for students'
own understanding and effort. This package is a development and learning reference;
the team must understand the design, conduct the hardware experiments and write its
own submitted interpretation.
