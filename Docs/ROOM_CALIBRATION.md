# Room calibration runbook (horn12 acoustic sensor)

Calibrates the rover's horn bearing and range sensor for one room and writes a room config,
`Config/measured_horn12_<room>.json`. Allow about 1.5 h: 15 min of setup, 60 min of captures, 15 min of fitting.

**Why a room needs its own calibration.** Indoors, about half of the received power comes from reflections. That compresses the bearing slope and adds a static, position-dependent bias of up to about 20°. Both are fixed for a given room, so they are fitted in that room (per-tone model, `bin_model`). They don't carry over to another room.

**Recalibrate whenever any of these change:** the room, or its furniture layout near the test area; the MAX4466 gain screw; the beacon WAV, speaker or playback volume; the firmware tone profile; the horn, or its mounting height on the rover.

---

## 1. Equipment and conventions

- **Rover:** Pi with the repo up to date (`git pull`), ESP32 on `/dev/ttyUSB0` running `esp32_acoustic_v2` with the **horn12** profile. Motors stay off during the whole session: `rover.collect` never imports the motor code.
- **Beacon:** speaker playing `Data/stimuli/horn12.wav` **on loop**, from a laptop or phone.
- **Tools:** tape measure (≥ 2.5 m), masking tape, a pen. A protractor helps.
- **Geometry**, all measured from the **horn pivot** (the servo axis) on the floor:
  - $\rho$ = distance from the horn pivot to the speaker's driver centre.
  - $\beta$ = bearing of the speaker, **positive to the rover's LEFT**, 0 = straight ahead along the rover's heading.
  - Speaker driver at **horn height**, speaker **aimed at the horn pivot** in every pose.
- **The rover doesn't move during the session.** Tape its wheel outlines, so that a bump can be undone.
- **Space:** a clear area 2.2 m ahead of the horn and 0.7 m to each side. Keep at least 0.5 m between the speaker and any wall it faces.

### Floor layout

Mark every pose before you start. $x$ is forward along the heading, $y$ is to the left, both from the horn pivot.

| Use | $\rho$ (m) | $\beta$ (°) | $x$ (m) | $y$ (m) |
|---|---|---|---|---|
| E0 / B0 / B2 | 1.00 | 0 | 1.000 | 0.000 |
| B0 | 0.75 | 0 | 0.750 | 0.000 |
| B0 | 1.50 | 0 | 1.500 | 0.000 |
| B0 | 2.00 | 0 | 2.000 | 0.000 |
| B0 | 1.00 | −20 | 0.940 | −0.342 |
| B2 | 1.000 … 1.075 (1.5 cm steps) | 0 | 1.000 … 1.075 | 0.000 |
| E3 | 1.20 | −30 | 1.039 | −0.600 |
| E3 | 1.20 | −15 | 1.159 | −0.311 |
| E3 | 1.20 | 0 | 1.200 | 0.000 |
| E3 | 1.20 | +15 | 1.159 | +0.311 |
| E3 | 1.20 | +30 | 1.039 | +0.600 |
| E3 | 0.75 | −15 | 0.724 | −0.194 |
| E3 | 0.75 | +15 | 0.724 | +0.194 |
| E3 | 1.50 | −15 | 1.449 | −0.388 |
| E3 | 1.50 | +15 | 1.449 | +0.388 |
| E3 | 2.00 | 0 | 2.000 | 0.000 |

- **Training:** B0 sweeps the horn centre $\varphi_c$ from −30° to +20° in 5° steps with the speaker fixed. The script sends the angles. There are 5 lines × 11 angles.
- **Held out:** E3 poses are never on a training line and are used **only** for validation.
- B2 measures the fast fringe: the same pose shifted radially by 1.5 cm each time.

---

## 2. Pre-flight (Pi SSH terminal, `~/Audio-Tracking-Rover`)

```bash
git pull
python3 Tools/pi_check.py --picar-db /home/pi/SunFounder_PiCar/picar/config --port /dev/ttyUSB0
```
- **Must pass**, with `profile=horn12` in the C2 line. If the profile is different, flash `esp32_acoustic_v2` with the horn12 profile first.
- **Check the plan:**
  ```bash
  python3 Tools/phase_b_session.py --profile h12 --tag <tag> --dry-run run | grep -c rover.collect
  ```
  This should print `73`. For `<tag>`, use a short room id plus a session letter, e.g. `living_a` or `b204_a`. Use a new tag for every session.

## 3. Set the beacon volume

Speaker at $\rho$ = 1.0 m, $\beta$ = 0, aimed at the horn, WAV looping. Take a 15 s test capture (the label `VOLCHECK` keeps it out of the analysis):

```bash
python3 -m rover.collect --port /dev/ttyUSB0 --config Config/measured_horn12.json --label VOLCHECK --rho 1.0 --beta 0 --duration 15 --notes "volume check"
python3 Tools/capture_summary.py
```

- **Target:** `pp median` 1000–1500, `clip` 0 %, `valid` ≥ 95 %. Adjust the volume and repeat until all three hold.
- **Write down** the laptop/phone volume, the speaker volume and the gain-screw position. **Don't touch any of them again** for the rest of the session.

## 4. Run the session (about 60 min)

```bash
python3 Tools/phase_b_session.py --profile h12 --port /dev/ttyUSB0 --tag <tag> \
    --volume "laptop __ %, speaker __" --room "<room name>" run
```

- **Each step** prints the setup. `MOVE speaker to …` means place the speaker at that pose from the table. `Speaker unchanged` means the script only turns the horn. Then press **Enter** to capture, `s` to skip or `q` to quit.
- **Order:** E0 silence (speaker **off**, 60 s) → E0 tone (speaker on, 60 s) → B0 (5 lines × 11, 15 s each) → B2 (6, 15 s) → E3 (10, 20 s).
- **Stay still and quiet** during each capture, and keep yourself out of the line between the speaker and the horn.
- **Quitting and resuming:** steps already captured are skipped. Run the same command again to continue.
  - To repeat steps, add `--only B0` (or another step list) and `--redo`.
  - The manifest is `Runs/<date>/phase_b_<tag>_manifest.json`.
- **After E0 tone,** check the level hasn't drifted: `python3 Tools/capture_summary.py`.

**Session log:** write `Runs/<date>/notes_<tag>.md` with the room, the date, the volume settings, the gain-screw mark, the speaker model and aim, the battery LEDs, and anything unusual (people walking, HVAC noise, a door opened).

## 5. Copy to the laptop and commit

In a **laptop PowerShell terminal**, at the repo root:
```powershell
scp -r pi@rover1.local:~/Audio-Tracking-Rover/Runs/<date> Runs/
git add Runs/<date>
git commit -m "horn12 room calibration captures, <room>, tag <tag>"
git push
```

**Before the next `git pull` on the Pi,** move the Pi's own copy of that folder aside. Untracked files that a commit now contains block the pull. After pulling, compare the two copies, ignoring line endings, and only then delete the backup:
```bash
mv Runs/<date> ~/runs_<date>_backup
git pull
diff -r --strip-trailing-cr ~/runs_<date>_backup Runs/<date> && echo IDENTICAL
rm -r ~/runs_<date>_backup        # only after IDENTICAL
```

## 6. Fit (laptop, repo root)

```powershell
python Tools/phase_b_session.py --tag <tag> analyze
python Tools/phase_b_multitone.py --runs Runs/<date> --tag h12
```

`analyze` runs three fits from the manifest:

1. **Bearing:** `rover.bearing_cal --per-tone`, trained on all B0 runs and validated on E3. This writes `Runs/<date>/<stamp>_bearing_cal/proposed_config.json`, which holds the `bin_model`, and `bearing_cal_report.json`.
   - Per tone, it prints $\kappa_k$, the pose residual, the pair noise and the weight.
   - Then the **train** and **holdout** lines: per-pose bias / RMS / max, in degrees.
2. **Fringe:** the same model validated on B2. The spread over 7.5 cm is the fast fringe.
3. **Range:** `rover.calibrate`, trained on the B0 $\varphi_c = 0$ runs at $\beta = 0$ (four distances) and validated on E3. This writes `Runs/<date>/<stamp>_calibration/proposed_config.json`, holding the amplitude constant $K$ and the range exponent $n$, and `calibration_report.json`, holding `holdout.relative_range_rmse`.
   - Its `passed` flag is the old single-tone gate. It will usually read `false`. Ignore it and use the numbers.

`phase_b_multitone.py` is an independent cross-check: per-tone slopes, fringe correlation, and the E3 bias and RMS of several estimators.

- It finds runs by label inside `Runs/<date>`, so keep **one room per date folder**.
- Everything else uses the manifest and doesn't have this limitation.

## 7. Write the room config

```powershell
Copy-Item Config/measured_horn12.json Config/measured_horn12_<room>.json
```
Edit the copy:

| Key | Source |
|---|---|
| `bin_model` | bearing fit, `proposed_config.json` |
| `bearing_gain_correction`, `boresight_deg` | `1.0` and `0.0`: the per-tone model already contains slope and offset |
| `amplitude_constant`, `range_exponent`, `range_domain` | range fit, `proposed_config.json`. Don't take its `sigma_deg` or `boresight_deg`: those belong to the single-tone model |
| `range_noise_floor_rel` | range fit, `holdout.relative_range_rmse` |
| `amplitude_noise` | E0 silence: `N median` from `capture_summary.py` |
| `measurement_period` | E0 tone: `T_m median` (expect 0.477–0.481 s at $\delta$ = 50°) |
| `fringe_ln_sd` | $s_{B2} / (59° \cdot \sqrt{2})$, where $s_{B2}$ is the B2 per-pose bearing sd in degrees |
| `room_bearing_sd_deg` | $\sqrt{\text{RMS}_{E3}^2 - s_{B2}^2}$, where $\text{RMS}_{E3}$ is the held-out per-pose RMS |
| `provenance` and `_comments` | room, date, tag, volume settings, gain mark, the run folders used |

Here $59°$ per neper is $\sigma^2/2\delta$ at $\sigma$ = 77°, $\delta$ = 50°. Keep `room_length_m` at 1.0 unless a bias-correlation line was measured.

Use the room config for every live run in that room: `python3 -m rover.live --config Config/measured_horn12_<room>.json …`. `Config/phase_e.json` is a standalone config. For the Phase E design in that room, copy the same sensor keys into it (`range_noise_floor_rel`, `amplitude_noise`, `measurement_period`, `fringe_ln_sd`, `room_bearing_sd_deg`).

## 8. Gates

| Check | Pass | If it fails |
|---|---|---|
| E0 silence | 0 pairs accepted (`valid 0 %`) | Find the in-band source: HVAC, a charger, USB noise. Don't calibrate yet |
| E0 tone | valid ≥ 95 %, clip 0 %, pp 1000–1500, $T_m$ p95 ≤ 0.50 s | Redo the volume step |
| Bearing, held out (E3) | **pre-registered:** per-pose RMS ≤ 10° → Tier 3; 10–15° → Tier 2; > 15° → Tier 1 only (stationary beacon) | Record the value. It sets the demo tier for this room |
| Range, held out (E3) | `relative_range_rmse` ≤ 0.10 | Keep the larger floor in `range_noise_floor_rel`. The approach governor then stops further out |

**Driving in a calibrated room (updated 28 Nov).** The rover listens while it drives. With the stock firmware every motor-on pair clipped (`Config/rover1_hardware.json` → `motor_on_E0_notes`); with impulse blanking (`rail_max=10`, `firmware/esp32_acoustic_v2/README.md`), gain trimmer mid and **volume 80 %**, motor-on captures are 100 % valid, 0 % clipped at PWM 45 and 80 (`Runs/2026-11-28/20261128T153952_673057Z_capture`, `…161854_838975Z_capture`). The motors raise the in-band $N$ 5–7× but not the amplitude or bearing scatter, so calibration with the motors off stays valid for driving. Conditions:
- Same volume, trimmer and firmware as the calibration; any change needs a 1.0 m reference capture and a $K$ rescale (`Tools/make_measured_config.py --amplitude-constant`).
- Pi and laptop on battery (wall-adapter transients clip 5–31 % of pairs).
- Nobody within about 1 m of the rover or the speaker during a run: a static rover's bearing drifted ±13° within 90 s while an operator stood beside it.
- At the start spot, take two 20 s captures at a taped β = 0 and run `python Tools/rezero_bin_model.py --config <config> RUN RUN`. If the offset exceeds about 5°, write the re-zeroed config with `--out` and use that for the runs (the s1 model read −0.4° on the A2 spot but −14.7° at the 15:42 floor spot).
