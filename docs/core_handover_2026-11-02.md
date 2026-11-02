# Handover: Nontok robot, required function (Control & Parking) — 2 Nov 2026

## 1. The task
Start on the arc ~1 m from the main wall (placed by staff), drive to point **N** in front of the light,
read the colour pad (**green → turn left, red → turn right**), park in that side's box **8 cm from the
side wall (measured at the wheel)**, parallel. Touching a wall = disqualified.

## 2. Hardware
- Arduino Mega runs everything; no computer needed during a run.
- Mecanum wheels: forward, sideways, turning (also combined).
- 5 ultrasonics: front, front-left 45°, front-right 45°, left side, right side.
- Gyro MPU6050, read **only while stopped** (motor noise disturbed its I²C bus on 30 Oct). Its FIFO holds
  yaw only (2.5 s).
- Colour sensor under the front, OLED, start button on A15 (press: 10 s countdown then run; press again: stop).

## 3. Where things are
- Run logic: `firmware/hardware_probe_mega/src/run_logic.cpp`, `Runner::run()` — read this first.
- Hardware side: `firmware/hardware_probe_mega/src/main.cpp`.
- Calibration: `calibration/field-2026-11-03a.json` (stored on the robot): all poses recaptured together on
  2 Nov 23:50 (the turned captures before came from an older N and were up to 40 mm off), right box again 3 Nov 00:17.
- Good version: commit `a8e454b`, tag `good-2026-11-02_2017`; zip `~/Downloads/Nontok_good_2026-11-02_2017.zip`.

## 4. What the robot does (good version)
1. **Creep to N**: one slow continuous drive (45 → 41 PWM) toward the main wall, stopping 30 mm short.
2. **Translate onto N** (front + side walls).
3. **Straighten** from the 45° sensors against the N calibration (≤12° per correction, a sensor far off
   its target ignored).
4. **Translate again**; never an error, it carries on.
5. **Read the colour**.
6. **Turn 90°**: one slow pulse for ~85 %, finished on the gyro.
7. **Creep toward the box**, stop 180 mm out: measure the skew from the side sensor facing the main wall,
   turn it out, strafe onto the box line.
8. **Creep the rest**, small final moves (turns ≤3° near the wall).
Safety: forward drives stop if a 45° sensor sees a wall under 100 mm.

## 5. Calibration on the robot
| Position | Front | Left side | Right side | Left 45° | Right 45° |
|---|---|---|---|---|---|
| N | 264 | 625 | 676 | 641 | 716 |
| N turned left | 572 | – | 215 | – | 651 |
| Left box | 52 | – | 214 | 936 | 277 |
| N turned right | 625 | 206 | – | 626 | – |
| Right box | 55 | 213 | – | 234 | – |

Colour references: green 0.857, red 1.737 (red ÷ green), kept from `field-2026-11-02b.json`.

## 6. How to
- Console: `./tools/start_console.sh`, open http://127.0.0.1:8766.
- Flash (robot on USB, console disconnected from the port):
  `cd firmware/hardware_probe_mega && ~/.platformio/penv/bin/pio run -e mega2560_probe -t upload`
- Recalibrate: place the robot by hand, press the pose's capture in the console (keep still and clear),
  then `python3 tools/export_calibration.py <name>` and `python3 tools/upload_calibration.py calibration/<name>.json`.
  Calibration survives reflashing.
- Back to the good version: `git checkout good-2026-11-02_2017`, then flash.

## 7. How the robot gets from the start to N (localization)
No map and no x/y: N is a **fingerprint** — the 5 readings taken there at calibration. "At N" means the
current readings match it; the gyro holds the heading between moves (read while stopped).

1. **Creep**: the start heading is the reference. All 5 ultrasonics are pinged while driving (front
   ~20 Hz, others ~5 Hz), each with a Kalman filter (distance + closing speed, stray echoes rejected). It
   stops when the predicted position after braking reaches N's front reading + 30 mm.
2. **Close-enough check**: front within 300 mm of N's, or a 45° near its N value.
3. **Translate** (each round measured while still, median of 3 scans):
   forward error = front − 277; sideways error from both side walls (half the difference against the
   calibration, cancels heading), else one wall, else the 45° pair. Accuracy = 100 − weighted rms error
   (mm). The bigger error is corrected by a damped move (forward/back: creep on the front sensor; sideways:
   continuous strafe on the side walls); a gain drops after an overshoot. Ends at 95 %, in a dead band, when
   no longer improving, or after the move limit — never an error.
4. **Straighten** (after position, so a sideways offset is not read as heading): a ~2° probe turn gives
   each 45° sensor's mm per degree; heading error = (calibrated − current) / slope; limits above; up to 3 tries.
5. **Translate again**, then read the colour.

## 8. Lessons learned
- The simulator does not match the field: judge every change by real runs.
- No fast sweeps; the position-correction loop (combined moves) was rejected; creep / point forward /
  translate works better.
- Speeds settled on: creep 45 → 41 PWM, turns 62 / 56 PWM (54 / 50 under-turned), strafes 68 / 80 PWM.
- Side-wall straightening (5 headings) was tried and removed.
- The gyro is accurate (hand 90° read 90.4–90.8°): a crooked result after the turn means N's heading was off.

## 9. Known limits / to do
- **Known failure: when the starting angle set by the professor/staff is too extreme, the run does not
  work.** During the creep the start heading is the only reference; a steep start drives diagonally, the
  front beam can measure a side wall or corner, and straightening (≤12° × 3) cannot always recover.
  Possible fix later: find the main wall's direction before the long creep.
- Right box: the 100 mm corner stop is right at its left 45° reading (~100 mm); it may stop a few mm short.
- Batteries change turn/creep speed: charge fully and test on full batteries before the evaluation.
- Push the good version to GitHub.
