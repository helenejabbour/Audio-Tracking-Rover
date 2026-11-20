# Control design and learning reference

Prepared 15 November 2026. This document explains the implementation and its limits;
it is not a completed experimental paper. The original report remains an input to
the design, not a specification that must be defended after contrary evidence.

## Decision

Use a continuous fixed-frequency beacon, a stationary horn scan for acquisition,
timestamped paired amplitudes, a delayed-measurement observer, and constrained
state feedback. Compare discrete LQI, channel-wise pole placement with integral
action, and a constant-speed bearing-only baseline on the **same** sensing and
supervisory code. Select hardware tuning only after measured calibration passes.

The original approach is viable conditionally. The present data do **not** establish
that the horn has enough local angular sensitivity, that amplitude predicts range
at 1–3 m, or that the frontend can deliver 2–3 valid pairs/s. An optimizer cannot
repair those missing measurements. No software-only method can promise a successful
physical demo before those conditions are tested.

| Choice | Reason and limitation |
|---|---|
| LQI | Transparent state-space design, integral rejection of constant speed mismatch, inexpensive runtime. Local guarantee; limits and uncertainty matter. |
| Channel-wise pole placement | Direct link between desired poles and response; useful independent design and course comparison. Does not optimize effort. |
| Bearing-only baseline | Separates the benefit of distance regulation from steering toward sound. Constant speed does not regulate following distance. Shares the calibrated near-target supervisor for fair comparison. |
| Cartesian CV EKF | Handles stationary targets without an undefined target heading; variable-time updates and explicit covariance. Heading is inferred only when target velocity is sufficiently nonzero. |
| MPC/H-infinity/particle filtering | Additional complexity is not justified by the available plant and noise evidence. Consider only if measured failures identify constraints or non-Gaussian ambiguity that this design cannot handle. |

This is a front-steered PiCar-S. A differential-drive controller that turns at zero
forward speed is not applicable. The supervisor moves the horn, not the chassis,
while acquiring or recovering a lost signal.

## Nonlinear model and operating point

Positive bearing, heading and steering are counterclockwise (left, viewed from above).
Distances refer to the calibrated receiver/speaker reference points. Collision
clearance must include both chassis extents relative to those points.

For range rho, bearing beta and relative heading epsilon, define alpha=epsilon-beta:

\[
\dot\rho=v_t\cos(\epsilon-\beta)-v_f\cos\beta,\qquad
\dot\beta=\frac{v_t\sin(\epsilon-\beta)+v_f\sin\beta}{\rho}-\omega_f,\qquad
\dot\epsilon=\omega_t-\omega_f.
\]

The car constraint is \(\omega_f=(v_f/L)\tan\delta_s\). Steering has no yaw authority
at rest. The simulator integrates two independent global poses and actuator states;
the controller never receives the target pose, true speed or true range.

Linearize about rho=d, beta=epsilon=0, vf=vt=v0>0. Let
\(x=[\rho-d,\beta,\epsilon]^T\), \(u=[v_f-v_0,\omega_f]^T\),
\(w=[v_t-v_0,\omega_t]^T\), and \(a=v_0/d\). Then

\[
A=\begin{bmatrix}0&0&0\\0&0&a\\0&0&0\end{bmatrix},\quad
B=\begin{bmatrix}-1&0\\0&-1\\0&-1\end{bmatrix},\quad
E=\begin{bmatrix}1&0\\0&0\\0&1\end{bmatrix},\quad
C=\begin{bmatrix}1&0&0\\0&1&0\end{bmatrix}.
\]

`rover.analysis` verifies the Jacobian against the nonlinear equations. State-space
models are not unique: for x=Tz, the transformed realization is
\((T^{-1}AT,T^{-1}B,CT)\). The transfer matrix is

\[
G(s)=\operatorname{diag}\left(-\frac1s,-\frac{s+a}{s^2}\right).
\]

For the bearing subsystem, choose
\(T_b=[[-a,-1],[0,-1]]\). This gives
\(A_c=[[0,1],[0,0]], B_c=[0,1]^T, C_c=[-a,-1]\): a controllable
canonical realization of the same transfer function. Its dual gives an observable
canonical realization. A diagonal modal realization is impossible for the nonzero
nilpotent block; use Jordan form instead.

## Trajectories, subspaces and stability

Since A²=0, \(e^{At}=I+At\). For constant input/disturbance f=Bu+Ew,

\[
x(t)=(I+At)x(0)+(tI+\tfrac12t^2A)f.
\]

If a varies with time and A(t)=a(t)N, N²=0, matrices commute and
\(\Phi(t,s)=I+N\int_s^t a(\tau)d\tau\). The forced solution is
\(\Phi(t,s)x(s)+\int_s^t\Phi(t,\tau)f(\tau)d\tau\). This special LTV formula
does not extend to arbitrary noncommuting A(t). Numerical integrations verify both
the forced LTI trajectory and the LTV transition matrix.

| System | Controllability | Observability | Consequence |
|---|---:|---:|---|
| Nominal plant, a>0 | 3/3 | 3/3 | State feedback and an observer can assign all plant/error modes. |
| Virtual-input model, a=0 | 2/3 | 2/3 | A zero mode is uncontrollable and a zero mode unobservable: not stabilizable or detectable. |
| Plant plus constant target-speed/yaw disturbances | 3/5 | 5/5 | Two exogenous zero modes cannot be controlled; they can be estimated for a moving nominal target. |

At a=0, the reachable subspace is span{e1,e2+e3}, and the unobservable subspace is
span{e3}. Coordinates x=[e1,e2+e3,e3]z give two controllable/observable coordinates
and one uncontrollable/unobservable coordinate. Only the two controllable modes
can be shifted. In the actual stopped car, the virtual yaw input is also unavailable;
the physical zero-speed case is more restrictive than the algebraic virtual model.

**Correction to the original MATLAB commentary:** when a>0, the open-loop system
is not Lyapunov stable. A nonzero initial epsilon produces beta(t)=beta(0)+a*t*epsilon(0).
Zero eigenvalues with a nontrivial Jordan block permit unbounded free response.
The transfer matrix is also not BIBO stable. When A=0, the unforced state is constant,
but integrator input-output channels still are not BIBO stable.

For the designed discrete local closed loop F, `analysis.json` records its poles and
the positive-definite solution of \(F^TPF-P=-I\), including the residual. This is a
local linear stability certificate. Saturated/hybrid nonlinear trajectories are
tested separately; they are not covered by the separation principle.

## Controller design

Augment z'=rho-d and use
\(A_a=[[A,0],[C_r,0]], B_a=[B;0]\). Discretize with a block matrix exponential,
so no inverse of the singular A is used. Solve the discrete Riccati equation for
Q and R from the configuration. The command is

\[
u=-K[\hat\rho-d,\hat\beta,\hat\epsilon,z]^T.
\]

The alternative pole design places the range/integral pair and bearing/heading pair
separately using exp(p*T). This makes the MIMO gain choice explicit. Other MIMO gains
can realize the same pole set with different eigenvectors, control effort, coupling
and sensitivity; equality of eigenvalues alone does not make two controllers equivalent.

The implementation clips forward speed and steering, enforces motor deadband,
reduces speed at large bearing and uses integral back-calculation against the
realizable speed. There is no reverse command. The operator explicitly selects
`approach` for a known stationary beacon: this uses proportional range/bearing control,
zero initial target-velocity prior, motor-deadband compensation in the terminal band
and a braking-aware latched stop. This shared terminal mode is not part of the LQI
comparison. The default `follow` mode does not permanently classify a target as
stationary from a noisy velocity estimate. Restart for another trial after a latched stop.

The default desired separation **1.2 m**, nominal speed **0.18 m/s**, and all noise,
geometry and braking numbers in `Config/simulation.json` are engineering assumptions.
They replace the old 0.1 m placeholder; none are certified hardware values.

## Measurement model and calibration

Use the more general calibrated model

\[
A(\phi)=K\rho^{-n}\exp\{-\frac{(\phi+b_0-\beta)^2}{2\sigma^2}\}+\eta.
\]

For actual pair center phi_c and half separation delta,

\[
\hat\beta=\phi_c+b_0+\frac{\sigma^2}{2\delta}\log\frac{A_+}{A_-},\quad
\hat\rho=\left[\frac{K}{\sqrt{A_+A_-}}
\exp\{-\frac{(\phi_c+b_0-\hat\beta)^2+\delta^2}{2\sigma^2}\}\right]^{1/n}.
\]

The fit estimates K, n, sigma and boresight from measured training files. Held-out
files are supplied separately and never used for fitting. Identical file content
cannot appear in both sets. A Gaussian model is accepted only if held-out range,
bearing, bias, validity and timing gates pass. A poor fit is a failed sensor model,
not a reason to loosen the gate until the rover moves.

Amplitude-ratio bearing cancels common gain only if the two captures see sufficiently
similar source level/range. Absolute range cannot distinguish source-level change
from distance: if true K is g times assumed K, estimated range is true range/g^(1/n).
Therefore true equilibrium becomes g^(1/n)*d. Integral action cannot remove this bias.
Speaker aspect, room reflections and battery-dependent gain must be included in the
held-out tests. The 30 cm broadband POC does not calibrate a 4 kHz pair at 1–3 m.

`rover.sensing` propagates log-amplitude uncertainty through the complete Jacobian,
retains range/bearing covariance, adds model-error floors and a within-pair motion
term. It rejects invalid/late/clipped samples, extrapolation and nonfinite values.
A single off-tone bin is a quality indicator, not proof of broadband noise variance;
use repeated static and motor-on tests to set the error floors.

## Runtime observer and timing

The runtime state is \(q=[p_x,p_y,v_{tx},v_{ty}]^T\), resolved in the follower frame.
For known follower speed v and yaw rate omega:

\[
\dot p_x=v_{tx}-v+\omega p_y,\quad
\dot p_y=v_{ty}-\omega p_x,\quad
\dot v_{tx}=\omega v_{ty},\quad
\dot v_{ty}=-\omega v_{tx}.
\]

The constant-input transition uses an exact rotation and arc displacement. White
target-acceleration process noise gives the usual dt³/3, dt²/2 and dt covariance
blocks. Measurements are h(q)=[hypot(px,py),atan2(py,px)]. Updates use wrapped angular
innovations, a chi-square gate and Joseph covariance form.

The acoustic pair is assigned its midpoint time, with additional uncertainty for
sequential capture. The observer restores its state at that time, corrects it and
replays saved actuator inputs to the present. Duplicate/reordered corrections and
measurements outside the history are rejected. Rejected samples do not reset the
loss timer. Unit tests compare delayed replay with an in-order reference.

The follower currently predicts its own motion from the calibrated actuator model;
it has no measured wheel odometry. This model error is explicit in stress tests.
Add encoders/heading measurements if actuator identification is not repeatable.
An IMU alone is not reliable position ground truth for this experiment.

The V2 device protocol includes both capture times and emission time, actual horn
angles, quality flags and CRC. The host unwraps micros and uses a minimum-offset
clock mapping plus measured transport latency. It detects added queue delay; it
cannot discover an unknown constant transport delay. Device reset requires restarting
the run. Clock drift and bounded transport uncertainty must be tested over the planned
run duration; this is not a hardware clock-synchronization protocol.

## Supervisor and stopping bound

ACQUIRE holds the chassis still until three fresh, accepted pairs agree. TRACK applies
feedback. HOLD stops after stale/uncertain observations and permits stationary horn
reacquisition. An uncertain clearance bound triggers recoverable HOLD; a sufficiently
near mean range or explicit approach-mode terminal condition triggers latched STOP.

The speed limiter solves
\(vT+v^2/(2a_b)\le\hat\rho-3\sigma_\rho-r_{clearance}-m\), with
T=timeout+actuator lag, measured braking deceleration a_b and margin m. This is
conditional on the calibrated uncertainty model and a nonapproaching target. It does
not protect against an arbitrary target driving into the follower, an obstacle off
the acoustic line of sight, or a wrong range caused by unmodeled fading. Use a physical
motor power cut and supervised clear-floor trials; add independent proximity sensing
if an automatic collision guarantee is required.

The motor driver lives in a separate heartbeat process. A stalled controller stops
renewing commands, causing that process to stop the motors. This does not cover a
frozen OS, stuck I2C transaction or failed motor driver. Those require the independent
power cut, not a claim about Python exception handling.

## What the validation proves

The saved experiment compares three methods, 12 scenarios and multiple random seeds.
Noise, losses, sequential capture, gain bias, slow sensing, actuator mismatch,
stationary targets and unreachable target speed are included. Every result remains
in the metrics, including non-settling runs; failed runs are not silently discarded.

The original 6 s / 15% O3 target is retained as a separate reported test. A more useful
commissioning measure under noise is final-10-s range error <=15 cm and bearing RMS
<=10 degrees, subject to clearance and enough TRACK time. It is labeled as a revised
engineering criterion, not represented as the professor's original requirement.

O5 is evaluated using a **periodic sampled-loop** local reference with actuator lags,
intermittent measurements, delayed correction and gain scaling. `analysis.json`
reports the monodromy spectral radius for each case. This is more representative
than applying state-feedback LQR margins to an observer-based saturated loop. It does
not establish a classical 45-degree phase margin or 6 dB gain margin for the actual
nonlinear runtime. Those original claims remain unverified until a measured loop
and a stated linearization/loop-break convention are available.

## Course objective and feedback mapping

| Syllabus CLO | Evidence |
|---|---|
| 1 Classification | Continuous nonlinear plant, sampled controller/observer, hybrid supervisor; local LTI and scheduled LTV models above. |
| 2 Nonunique MIMO models/canonical forms | Similarity transformation, canonical bearing realization, transfer-matrix equivalence check. |
| 3 Continuous/discrete, SISO/MIMO, LTI/LTV response | Kinematic simulation, decoupled transfer channels, discrete poles and LTV check. |
| 4 Forced and free trajectories | Closed forms and numerical comparisons with nonzero initial state and forcing. |
| 5 Transition/fundamental matrices | exp(At), commuting LTV Phi and exact ZOH implementation. |
| 6 Controllability/observability matrices | Computed nominal, stopped and disturbance-augmented ranks. |
| 7–9 Subspaces and modes | Explicit stopped-system decomposition, reachable/unobservable subspaces and exogenous modes. |
| 10 Stability characterizations | Jordan-block correction, BIBO analysis, discrete Lyapunov certificate, sampled periodic robustness sweep. |
| 11 State feedback / partial controllability | LQI and pole-placement comparison; stopped uncontrollable zero mode cannot be shifted. |
| 12 Observer / partial observability | Linear observer reference, runtime EKF, delayed-update tests; no claim to recover stationary target heading. |

The professor requested stronger controller validation, attention to measurement
accuracy, validation of assumptions and a more control-centric treatment. The
comparison runs, explicit gates, local stability analysis and documented limitations
directly address those points while retaining the report's strong formulation.

## Primary implementation references

- [SunFounder PiCar-S steering description](https://docs.sunfounder.com/projects/picar-s/en/latest/line_following.html)
- [SunFounder motor driver source](https://github.com/sunfounder/SunFounder_PiCar/blob/master/picar/back_wheels.py)
- [Espressif LEDC API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html)
- [Espressif ADC API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/adc.html)
- [Espressif raw continuous ADC driver](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/adc/adc_continuous.html)

Local sources: `Syllabus_EECE660_MECH653_Fall2026.pdf` (CLOs p.1; assistive-AI policy
p.3), `Progress Report Feedback.pdf` (remarks p.1), and the progress report's model
and assumptions (pp.4–6,9). The engineering derivations above are reproduced explicitly
so they can be checked without trusting either the old script or this text.
