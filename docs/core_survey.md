# Guided field probe and measurement

The Mega can collect a stationary angular scan, fit the three field walls, and estimate the side lights' forward positions and brightness calibration from several known floor poses. Use **Probe & measure the field** in the local dashboard (`./tools/start_dashboard.sh`). All fitting runs on the Mega. The Mac displays replies and retains the data.

This is a **guided survey**: the wheels stay disarmed, and you rotate/reposition the robot by hand. It does not drive blindly through an unknown arena. It cannot find floor-ink box outlines, infer beacon identity from mixed brightness, or measure arbitrary field geometry without a reference. The gyro/sensor wiring must work before starting.

## What it needs and measures

| Required reference / assumption | Why |
|---|---|
| Known side-wall spacing 1,500 mm | Defines the expected side-wall positions and checks measured width |
| Middle light at the back-wall origin, centred between side walls | Defines X/Y; verify this centreline placement |
| Known chassis X/Y and heading at the first station | Anchors the scan; the reported 300 mm marker can serve if the **chassis centre** is placed there |
| Measured ultrasonic face offsets from chassis centre | Prevents reporting sensor-face ranges as chassis coordinates |
| Actual ±45° mounting in the chassis plane | Used to transform each return into a wall point |
| Approximately straight back and side walls, roughly perpendicular | The estimator fits this pictured three-wall arrangement |
| For light fitting: independent per-beacon channels, measured ambient and vertical offsets, chosen optical exponent | Brightness alone does not provide distances or beacon identity |
| Light receiver's horizontal offset from chassis centre is negligible | The current optical model uses chassis XY as receiver XY; an offset installation needs a model correction before applying light estimates |

The wall report contains offsets, slopes, range-fit RMS, hit counts and **observed spans**. It checks that the measured width agrees with 1,500 mm within 60 mm. A missing return is not an observed wall end, and the routine never manufactures a wall length from no-echo readings.

The user confirmed that the side LEDs are attached directly to the side walls. The optional optical fit therefore constrains each side light to the corresponding fitted wall, with zero horizontal stand-off, and the middle light to the origin. It estimates each side light's forward X position and the reference signal at a 1,000 mm slant distance, using the chosen exponent. Attachment to the wall does not determine its forward position or height. It does **not** fit the exponent or discover light modulation/identity. Those physical assumptions must be checked. The middle light gets brightness calibration with its position fixed to the origin.

## Physical setup

1. Identify/trace the installed sensors and select the correct drivers. Enable `SENSOR_WIRING_CONFIRMED` only after verification. The optical stage requires `LIGHT_DRIVER=1` or a correctly adapted independent-beacon interface. The wall stage does not require a valid optical position fix or motor wiring confirmation.
2. Confirm gyro sign: CCW rotation must increase gyro heading. Keep the robot still through startup bias calibration. Close other serial monitors before connecting the dashboard at 115200 baud.
3. Measure each ultrasonic face's **forward** and **left** offsets from chassis centre, in mm. Rightward offsets are negative. Do not copy the numeric test fixtures from the source as hardware measurements.
4. Put the chassis centre at a measured position within the side walls. If the chassis centre is directly over the 300 mm centre marker, use X=300, Y=0. This differs from putting the front underside colour sensor over that marker. Face the back wall for world yaw 180°.
5. Make sure the whole footprint can turn freely at that station. Rotate/reposition by hand; the software never commands motor motion during the survey.

## Wall scan

Use the corresponding dashboard forms, or newline-terminated commands:

```text
STOP
PROBE
SURVEY MOUNTS <left_forward> <left_left> <right_forward> <right_left>
SURVEY BEGIN <chassis_x> <chassis_y> <known_heading>
SURVEY TAKE WALL
```

`PROBE` lists I²C addresses; it does not identify passive analog/frequency modules. The known centre pose would use `SURVEY BEGIN 300 0 180` **only when it matches the measured chassis pose**. Beginning clears previous captures while retaining the staged mount/ambient/optical settings.

After the first capture completes, rotate the robot **in place by hand**, roughly 15° at a time. Stop, settle and capture again. Cover as much of a full turn as the physical clearance allows. Return to the same chassis XY after every turn. Usually about 18–24 headings provide useful wall coverage. Never move while the status says `CAPTURE WALL` / `CAPTURE POINT`.

Each capture ignores the first 350 ms to let the ultrasonic median histories settle, then collects eight fresh updates per range channel. It rejects appreciable gyro motion, excessive range spread, partly intermittent returns or stale updates. Stable no-echo channels are retained as −1, not substituted with a distance. The full capture times out after four seconds. Light-point/ambient captures additionally need 16 fresh light updates.

```text
SURVEY SOLVE
SURVEY STATUS
```

The Mega fits a consensus line near each expected plane and reports success only with enough hits and observed span, small residuals, reasonable wall slope and consistent width. If a wall says `Need data`, add headings/another **known** station and try again. To change station:

```text
SURVEY POSE <measured_chassis_x> <measured_chassis_y>
SURVEY TAKE WALL
```

The gyro heading reference is retained; `POSE` changes only the measured X/Y. Do not re-zero the gyro between stations. `SOLVE` is permitted after `END` as well, so a stopped session's captures can still be analysed. Fitting may take several seconds on the AVR; keep the robot stationary and wait for the final status. It does not run while armed.

## Estimate the light positions

1. Turn off **all three beacons**, preserving the rest of the arena lighting. Send `SURVEY DARK`; wait for `AMBIENT CAPTURED TURN LIGHTS ON`. Then turn the beacons back on. The robot cannot verify that you actually turned them off.
2. Measure the vertical separation from the light receiver to each LED and set the optical exponent:

```text
SURVEY OPTICS <left_height_mm> <middle_height_mm> <right_height_mm> <exponent>
```

Exponent 2 is an inverse-square hypothesis, not a verified sensor property. The sensor response must support the model over the field. Use the actual height differences, including zero only if measured equal-height geometry justifies it.

3. Mark at least six **distinct measured chassis positions** spread in both X and Y. A nine-point grid is recommended. Keep the robot at the same world heading, ideally facing the back wall, for every optical capture. Do not use a row of points, repeat one point, or infer displacement from timed motor commands.
4. At every position, update the known coordinates and capture:

```text
SURVEY POSE <measured_x> <measured_y>
SURVEY TAKE POINT
```

The scan holds at most **40 captures** total, including wall and light points. A 24-heading scan plus a nine-point grid uses 33. Ambient captures do not use a slot. Change mount settings only before collecting captures; `SURVEY CLEAR` clears the dataset if mounts need correction.

5. `SURVEY SOLVE` again. The optical fit uses same-heading, unsaturated, independently positioned observations. It rejects inadequate 2D pose spread, too few unique poses, weak positional sensitivity, competing fits and excessive model error. The displayed spread is a **sensitivity indicator with a noise floor**, not a statistical accuracy guarantee.

Different sensor angular response, reflected light, changed ambient or an incorrect exponent can produce inaccurate estimates even with an apparently reasonable fit. Validate the resulting XY estimates at additional tape-measured positions that were not used to fit. Do not treat the example results from the synthetic software tests as measurements of this field.

## Apply, save and retain data

```text
SURVEY DUMP
SURVEY APPLY
CALIB
SAVE
SURVEY END
```

`APPLY` requires valid fitted walls and copies results to the working RAM profile. For a wall-only scan, it saves the measured wall map without replacing existing beacon models. If a six-or-more-point optical fit was attempted, it clears the old beacon position mask and accepts **only** the newly valid beacon models; failed light fits cannot silently leave an old model marked ready. A first mount calibration or changed mounting clears the taught centre/turn/box local-reference samples. Capture those again after surveying. `SAVE` commits the RAM profile to redundant EEPROM slots; the current format is **v5**, and older records require re-entering calibration.

Applying does not create a colour reference, parking-box target or local ultrasonic Jacobian. Use the existing `COLOR`, `TEACH`, `SAMPLE` workflow for those. After apply/load/reboot, set the mission's world heading again before arming.

The dashboard offers a current **Download survey JSON** link. Every captured frame, status and result is also retained as an event in the active Mac recording, and recordings containing survey replies get a **Survey ↓** export. Exports include raw averaged ranges, beacon channels, measured pose, heading, capture type and Mega uptime. Keep recording enabled during the survey. Download before `CLEAR` or a new `BEGIN`, which replaces the current capture set. Earlier Mac recordings remain available after server restart.

`SURVEY CANCEL` cancels the current averaging window. `SURVEY END` exits the session while retaining captures/results in RAM. Global `STOP`/`!` also cancels and exits. Motion/arming and unrelated calibration commands are blocked while a survey session or capture is active. The dashboard's generic simulation does not pretend to measure a real field.

## Limits and checks

The estimator treats an ultrasonic return as a central-ray endpoint. Real acoustic beam width, angled panels, corners and reflections need physical validation. Prior planes are near back X=0 and side Y=±750; surveying a differently centred, non-rectangular or significantly different field requires changing that model. Width, offsets and angular accuracy also depend on the measured reference poses and sensor mounts.

Host tests cover known synthetic three-wall scans, side-light fitting, stale/moving/duplicate/insufficient data, no-echo handling, fit application gates, serial command parsing and the arm interlock. The dashboard tests cover live and persisted survey JSON exports. AVR builds check memory and driver compatibility. These checks do not establish physical measurement accuracy or fitting time on the actual Mega.
