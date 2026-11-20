# Experiment index

All timestamps are UTC. Original hardware data remain in `Data/` and `POC/Results/`.

| Run | Role/status |
|---|---|
| [20261115T163143_844160Z_validation](2026-11-15/20261115T163143_844160Z_validation/REPORT.md) | **Authoritative current controller suite**, 432 simulations; seeds 3000–3011; 26 tests; source snapshots included |
| [20261115T163216_183097Z_sensitivity](2026-11-15/20261115T163216_183097Z_sensitivity/REPORT.md) | **Authoritative current sensitivity suite**, 78 simulations; seeds 4000–4005; source snapshots included |
| [20261115T162526_373664Z_replay](2026-11-15/20261115T162526_373664Z_replay/summary.json) | Actual legacy static-log replay; zero accepted packets / zero drive commands |
| 20261115T162705_887469Z_sensitivity | Development sensitivity run before explicit stationary-approach mode; superseded |
| 20261115T162451_107959Z_validation | Development comparison before explicit stationary-approach mode; superseded |
| 20261115T162350_760772Z_validation | Interrupted development run; incomplete, do not use for conclusions |
| 20261115T161034_583263Z_validation | Early complete development comparison; superseded after supervisor fixes |

Final runs save configuration and source hashes, and current runs also copy the code
under `source_snapshot/`. To reproduce an old snapshot, use an isolated copy of the
repository with the matching original input data; restore its snapshot source files
there and run the recorded command/config. Input data hashes are in the manifest.
Do not mix a newer runtime with an older run's configuration without checking schema
and behavioral changes.

The individual metrics retain every seed and failure. Example CSV traces and plots
use the first seed in each run; they are not selected as the best-performing example.
