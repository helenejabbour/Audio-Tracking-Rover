# Unused data

Data reviewed on 28 Nov 2026 and judged to carry no usable information for any fit, figure or finding. Nothing was deleted. Each item sits at `Unused_Data/<original path>`, so it can be restored with a single move; `MANIFEST.csv` lists every item with its category, original path, git status, size and reason (125 items, 120 MB).

| Category | Items | What | Examples |
|---|---|---|---|
| `empty` | 20 | captures that recorded 0 pairs (no `samples.csv`), plus the snapshot mirrors of the two tracked ones | 28 Nov failed starts after a port reset; 20 Nov `160356`, `161423` |
| `aborted` | 7 | runs that never wrote their result file, and motor-spin logs whose paired capture was empty | steer `164746`, speed `154825`, `155127`, spins `110212`, `131355`, `131957`, `153227` |
| `incomplete` | 5 | `phase_e` runs that stopped before `REPORT.md`; each was re-run complete within minutes | `104809`, `104919`, `105005`, `162455` ×2 |
| `invalid` | 3 | runs the operator invalidated | steer `163810` (raw protractor values), `164322` (wrong values), link test `132839` (disregard) |
| `excluded` | 14 | 20 Nov runs that `Local/Docs/NEXT_STEPS.md` (Phase A) marks "not to be used for any fit", plus their snapshot mirrors | `133134` (silence label, speaker on), E4 batch `140220`–`140453` (speaker off) |
| `superseded` | 67 | analysis outputs regenerable from kept captures and no longer cited | 66 exploratory 20 Nov `bearing_cal` fits; `Local/Runs/archive/` (15 Nov runs) |
| `duplicate` | 7 | byte-identical copies | `Local/fig1–6.png`; `Local/Data/stimuli/multitone_2500-6250Hz_16.wav` (= `Data/stimuli/` copy) |
| `cache` | 2 | Simulink build cache | `Local/slprj/`, `Local/acoustic_pursuit_loop.slxc` |

Kept on purpose, although they are not used for $K$, $n$ or bearing: the 28 Nov wall-power captures (F1 evidence), the 13:11–15:03 fault captures (open hardware fault), and the s1 runs `170436`, `170546` (repeated, but kept in the s1 tables).

## Git

Tracked items were moved with `git mv` (staged renames, not committed). `Unused_Data/Local/` is gitignored by the existing `Local/` rule; the 28 Nov items are untracked, like the rest of `Runs/2026-11-28/`. Delete the folder, or add it to `.gitignore`, if it should not be committed.

## Pi copy

The Pi still has the 28 Nov runs at their original paths. `scp -r pi@rover1.local:~/Audio-Tracking-Rover/Runs/2026-11-28 Runs/` copies them back; use `MANIFEST.csv` to remove them again, or copy only new folders.
