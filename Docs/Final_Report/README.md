# Final report draft — 15 November 2026

Open `main.tex`. The report uses the progress report's `IEEEtran` journal/A4 class, title, authors, packages, two-column layout, and `IEEEtranN`/BibTeX references. The header now identifies a final-report draft. Red placeholders use `\pending{identifier}{required evidence}`; the abstract also has a red completion note.

Upload the contents of the dated ZIP as a new Overleaf project and select `main.tex` as the main document. Use pdfLaTeX with BibTeX (Overleaf's normal automatic bibliography build). All cited bibliography entries, included sections, and figures are in the package. The original progress-report folder is unchanged.

## Written sections

- Motivation and research question; complete original literature review.
- Architecture, operating assumptions, timing, retained kinematics and local model.
- State-space response, canonical realization, controllability/observability, partial modes, LTI/LTV analysis, and stability.
- Calibrated sensing model, uncertainty propagation, Cartesian observer, and delayed corrections.
- LQI, pole placement, baseline, constrained actuation, approach mission, and supervision.
- Current simulation comparison, sensitivity results, failure cases, and objective status.
- Available experimental evidence, limitations, provisional conclusion, and historical POC appendix.

## Preserved text

The literature review is identical to the progress source after normalizing line endings. The bibliography is byte-identical. The motivation paragraph, research question, kinematic subsection/figure, local model and variable table, architecture figure, linearization appendix, and POC appendix are also preserved verbatim. Hash checks are in `audit/provenance.json` and `audit/source_checks.json`.

The preserved literature review's differential-drive wording is corrected explicitly in the new architecture section. The preserved model's “desired minimal separation” is clarified as the regulation setpoint, distinct from hard clearance. The POC's waveform terminology and relative level interpretation have a qualification and red verification placeholder. Keeping historical text does not imply revalidating every original literature claim or bibliography entry; the original `.bib` contains its own `CHECK` comments.

## Remaining red placeholders

| Identifier | Evidence required |
|---|---|
| Abstract | Final measured accuracy, sensing rate, and model agreement |
| H1 | Geometry, motor/steering calibration, lag, braking, wiring |
| H2 | Held-out tone-specific acoustic calibration |
| H3 | Timing, clock drift, and dynamic estimation accuracy |
| H4 | Physical stopping limits and watchdog behavior |
| V1 | Measured-model simulation reruns and final O1–O5 claims |
| E1 | Bench identification results and experimental protocol |
| E2 | Following and speed-change experiments with comparisons |
| E3 | Turning, signal loss, reacquisition, source aspect |
| E4 | Explicit stationary approach and stop-latch experiments |
| E5 | Model-to-hardware trajectory agreement |
| D1 | Discussion grounded in measured evidence |
| C1 | Final experimental conclusion |
| R1 | Successful builds, tested release, configurations and logs |
| P1 | Original POC waveform and level-reference verification |

The current gains and nominal geometry/noise values are simulation assumptions. Hardware experiments must establish whether those assumptions hold. The report explicitly retains failed transient and calibration-bias tests.

## Checks and limitations

Three audit passes are recorded in `audit/REVIEW_2026-11-15.md`. Source checks found no missing inputs/images, reference labels, bibliography keys, duplicate labels/keys, or unbalanced environments/braces. The new measurement Jacobian was checked by finite differences, canonical matrices were checked numerically, and current Python runtime sources match the evaluated source snapshot.

**PDF compilation and visual page-layout verification remain blocked.** Tectonic is installed but its TeX bundle is not cached and cannot be downloaded in this environment. MiKTeX cannot initialize because registry access/setup is unavailable. No final-report PDF is claimed. Compile in Overleaf and inspect equation widths, float placement, reference rendering, and the preserved literature text before submission. Source-level checks do not replace a successful LaTeX build.

This is a working draft for the team's review and completion, not an experimentally complete report. The course's stated assistive-AI policy still applies to submitted work.

## Repository maintenance

`prepare_sources.py` assembles `main.tex`, extracts the preserved blocks, copies figures and bibliography, and regenerates the results table directly from the saved metrics. It overwrites these generated files; edit that script when changing their generated content. New methodology/result sections in `sections/` are edited directly and are not overwritten. Run `audit_report.py` after edits. These helpers require the surrounding project and are not needed to compile the Overleaf ZIP.
