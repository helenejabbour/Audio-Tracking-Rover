# Measurements still needed from the team

The software work can proceed without inventing hardware values. A physical
demonstration cannot be validated until the following evidence exists.

## First package: resolve the present acquisition blocker

Send these before spending time tuning gains:

1. **Board identity and successful V2 firmware compile/upload log.** Confirm exact
   ESP32 model, microphone and servo wiring. The automatic compile on this machine
   was blocked before compilation by a Windows DLL error.
2. **Three 60-second V2 captures:** speaker off; continuous 4 kHz tone at a measured
   distance near 1.2 m; the same tone with motors on and wheels lifted. Keep raw serial,
   `samples.csv`, `session.json` and `summary.json` together. State speaker volume,
   microphone gain, source orientation, battery and room.
3. **Measured wheelbase, reference-point clearance and minimum repeatable forward speed.**

The supplied static capture has zero valid pairs and a 12.84 s median period. Its
five measurements do not support controller tuning. E0 should establish >=100 pairs,
>=95% valid and a pair-period p95 <=0.5 s before proceeding.

## Second package: identify and validate the operating envelope

| Measurement | Why it is needed |
|---|---|
| Angular sweeps at the actual 4 kHz tone, several distances, fine spacing around boresight | Fit horn sensitivity and offset; test symmetry and model shape. |
| Distance sweep with fixed gain/source orientation | Fit K and range exponent; test whether acoustic range is usable. |
| Independently repeated held-out angle/range grid, including motor-on conditions | Test bearing/range errors and biases without fitting the test data. |
| Pair gap, pair period, transport-delay distribution, mounted-servo settling | Set observer timing, uncertainty inflation, watchdog and achievable bandwidth. |
| Speed-to-PWM and wheel-angle-to-servo tables, repeated under load | Convert SI-unit controller commands into physical motion. |
| Speed/steering step response and braking distances | Validate actuator lags, stopping bound and safe separation. |
| Synchronized overhead video with measured scale and position uncertainty | Validate actual tracking/estimation errors and transient response independently. |

See [IMPLEMENTATION_GUIDE.md](IMPLEMENTATION_GUIDE.md) for exact capture commands,
file schemas, experiment order, acceptance gates and the process for loading measured
values. If the sensor gate fails, return the failed data rather than adjusting its
thresholds until motion is enabled.
