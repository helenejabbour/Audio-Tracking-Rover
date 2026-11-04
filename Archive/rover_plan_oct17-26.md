# Acoustic rover pursuit: plan for Oct 17–26

**Goal by Oct 26:** measured hardware parameters inside the simulation, a controller tuned on them, and a slow closed-loop run on the rovers. If the range estimate fails the gate below, fall back to bearing-only following. A robust autonomous rendezvous is out of scope.

## Where we stand

- Linearization verified numerically against the report's $\mathbf{A},\mathbf{B},\mathbf{E},\mathbf{C}$ (error $\sim10^{-11}$); controllability/observability hold (ranks 3 and 5 with disturbance states).
- `acoustic_pursuit_sim.py` already runs the nonlinear plant, paired-amplitude sensor, delayed-measurement Kalman filter and LQR on **placeholder** parameters.
- The critical path is therefore **hardware numbers → `Par`**, not more derivation.

## What the simulation says matters

Sweeps with placeholder tuning (10 runs each, order-of-magnitude guidance only):

- **Measurement period $T_m$ dominates.** $T_m=0.4$ s gives 3–5 s settling; $T_m=0.8$ s often exceeds 6 s; $T_m=1.2$ s does not settle. Latency $T_d$ matters less (0.3 → 1.2 s adds about 10 points of overshoot at $T_m=0.4$ s).
- **Noise tolerance.** RMS $\beta$ stays inside O2 (10°) up to per-measurement std of about 3° in $\hat\beta$ and 12% in $\hat\rho$; it is borderline at 6° / 23% and breaks at 12° / 47%.
- **$K$ error is a pure offset.** True range settles at $(K_{true}/K-1)\,d^\ast$ from $d^\ast$, and the loop stays stable. Recalibrate $K$ at $d^\ast$ each session and choose $d^\ast\ge\rho_{\min}/0.7$ to tolerate 30% error.

## Schedule

| Dates | Work | Exit criterion |
|---|---|---|
| Oct 17–18 | Freeze operating point: fixed tone $f_0$ (Goertzel), dither $\delta\approx\sigma$, straight constant-speed target. ESP32 firmware: paired $A_\mp$ capture, timestamps, validity flag, serial packet $(t,\varphi_c,A_-,A_+)$. Pi logger to CSV. | Raw CSV from one static test |
| Oct 18–20 | Calibration campaign E1–E5 (below) | **Go/no-go gate** passed or fallback chosen |
| Oct 20–22 | Load measured values into `Par` (config file). Rerun S1–S3, retune $Q,R$, compute gain/phase margins with Padé delay (O5). Run E6. | Sim reproduces E6 within a stated error |
| Oct 22–24 | Port estimator and controller to the Pi (same code as sim, I/O only). Safe stop. Dry run with wheels lifted, then tethered slow runs. | Rover follows commands and stops safely on loss |
| Oct 24–26 | Closed-loop tests: ≥5 repeats of Table exp. 3 (constant-speed straight target); catch-up (exp. 4) if time. Compute O1–O5 metrics, compare with sim, produce report figures. | Results table and figures |

## Experiments (log raw data, repeat each condition ≥5 times)

- **E1, angular sweep** at 1 m, about −60° to +60°: fit $\sigma$ and boresight offset; check Gaussian residuals.
- **E2, distance sweep** at boresight, 0.5–3 m (6 points): fit $K$; check $A\rho$ is constant (log-log slope ≈ −1).
- **E3, estimator grid**, static: 3 distances × 5 bearings. Error and std of $\hat\beta,\hat\rho$ from Eqs. (beta), (rho).
- **E4, timing:** $T_m$ (pair period), $T_d$ (capture → Pi), servo settling, Goertzel window.
- **E5, actuators:** command → speed (4 points; the minimum stable speed fixes $v_0$), steering → yaw rate (fixes $L$, $\delta_{s,\max}$), speed and yaw lag from a step.
- **E6, open-loop model validation:** drive the follower with a known command profile near a static beacon, compare logged $(\hat\rho,\hat\beta)$ with the nonlinear-model prediction. This directly answers the research question on LTI accuracy.

## Go/no-go gate (Oct 20)

**Go** (state-feedback path) if all hold:

- $\sigma_{\hat\beta}\le3^\circ$ and $\sigma_{\hat\rho}\le10\%$ at $d^\ast$ (E3)
- $A\rho$ constant within about 15% over the working range (E2)
- $T_m\le0.5$ s (E4)

**Otherwise, fallback:** $\omega_f=-k\hat\beta$ at constant $v_0$, stopping when $\sqrt{A_+A_-}\ge A_{stop}$ (amplitude as a proximity switch; needs no $K$). This is also the bearing-only baseline the report already promises, so no work is wasted. If only $T_m$ fails, shorten the dwell or use the two-horn option.

## Non-negotiables

- Safe stop on beacon timeout and on $\hat\rho<\rho_{\min}$ before any moving test.
- Fixed control rate (20 Hz) with timestamped measurements; the observer uses the real measurement rate, not 20 Hz.
- One-point recalibration of $K$ at $d^\ast$ at the start of every session.

## Cut list

- Chirp beacon (the report fixes a tone)
- Pole-placement study (LQR is done)
- Wide-scan reacquisition (stop and hold on loss)
- Simulink port
- IMU ground truth (tape marks suffice)
- Curved target paths

## Small code tasks

- Move the Kalman-replay and control step into a module shared by the sim and the Pi.
- Read `Par` from a JSON/YAML file updated after each calibration.
- Add the O5 margin computation.
