# Self-driving Lab Assistant — ELEC3848 Group D12

One robot (Jetson Nano + Arduino Mega + mecanum wheels) and one repository serve **two separately graded parts** of the course. The required core runs entirely on the Mega; only the proposed application uses the Jetson. Each part has its own owner and firmware, so the two can be developed in parallel without stepping on each other.

| | **A. Required function: Control & Parking** | **B. Self-proposed application: Lab Assistant** |
|---|---|---|
| Course weight | Required functions evaluation, 8% | Final demo 10%, video 6%, report 10% |
| Developed by | **Core session** (Claude Code session `nontok-bb`) | **Assistant session** (Claude Code session `nontok-c8`) |
| Mega sketch | `firmware/core_robot_mega/` | `firmware/lab_assistant_mega/` |
| Serial protocol | MEGA3: [docs/serial_protocol.md](docs/serial_protocol.md) | TEL/VEL: [docs/lab_assistant_protocol.md](docs/lab_assistant_protocol.md) |
| Jetson entry point | None: standalone Mega firmware | `python -m lab_assistant.main` (rest of `jetson/lab_assistant/`) |
| Python | None onboard; local Mac dashboard/recorder | 3.8+ (Ultralytics container), `requirements-assistant.txt` |
| Details | [docs/core_platform.md](docs/core_platform.md) | [docs/state_machine.md](docs/state_machine.md) |

Session names change when a session is restarted. The roles "core session" and "assistant session" are what count.

---

## A. Required function: Control & Parking

The current user-specified competition sequence is:

1. Start on the outside semicircle and localize using three distinguishable beacons (the side-wall spacing is 1,500 mm).
2. Reach the centre colour-paper target, 300 mm from the middle light. Refine alignment with the two front ultrasonic sensors at ±45°.
3. Read the underside-front colour sensor: green turns left +90°, red turns right −90°, using gyro progress and taught ultrasonic alignment.
4. Navigate to the corresponding recorded box pose, align precisely, then latch the motors stopped and finish.
5. Stream sensor/localization/motion state to this Mac, retaining recordings and time graphs.

### Completion status — 20 October 2026

**The core software is implemented and software-tested against the mission above and the current sensor assumptions. The robot is awaiting hardware integration, calibration and arena trials; it is not yet verified competition-ready.** No firmware has been uploaded to the physical robot in this work.

| Delivered | What is implemented |
|---|---|
| Standalone Mega firmware | PlatformIO project, centre approach, fresh colour voting, green/left or red/right 90° turn, taught-box navigation and a terminal motor-stop latch; telemetry continues after finishing |
| Onboard localization | Adapter for three independent beacon strengths, measured beacon XY, gyro heading reference and local fusion with the two front ±45° ultrasonic sensors |
| Sensor and shield support | MOEBIUS motor pin profiles, frequency-output colour driver selected from the pictured header, alternate I²C colour driver, MPU6050 driver and optional OLED; wiring remains unconfirmed |
| Manual control and fault handling | Bounded USB/Bluetooth moves, timed demo, explicit stop, sensor/clearance checks, loop and mission timeouts, and arming/calibration interlocks |
| Calibration | Colour references, centre/turn/box pose captures and measured ultrasonic derivatives, calibration validation, and redundant CRC-protected EEPROM storage (format v5) |
| Local Mac dashboard | Live sensor/state displays, selectable time graphs, field view, commissioning controls, SQLite recordings, replay and complete CSV exports |
| Guided field survey | Mega-side averaging, three-wall fits, optional beacon-position/brightness fits from known reference poses, quality checks, dashboard controls and current/saved survey JSON exports |
| Guides and checks | Hardware/pin documentation, commissioning and serial command guides, synthetic mission/survey tests, dashboard persistence/HTTP tests and AVR compile checks |

The earlier Jetson-based core was replaced. **All core decisions and field fitting run on the Mega.** The Mac displays/records telemetry and sends operator commands. The separate self-proposed Jetson application in part B was preserved and is outside this core implementation.

### What the field survey does

The side-wall spacing is **1,500 mm**, and the side LEDs are confirmed attached directly to those walls. The survey constrains each side LED to its fitted wall and estimates its forward position; it does not assume the three lights share the same X coordinate. The middle light defines the back-wall origin; the centre paper marker is reported at X=300 mm, Y=0 mm.

The survey is guided: keep motors disarmed, rotate/reposition the robot by hand and capture stationary readings at measured poses. All fitting runs on the Mega. Optical fitting additionally needs independent per-beacon signals, ambient measurements, height differences and a validated distance-response model. The routine cannot detect the inked parking boxes, and synthetic fits are not measurements of this field. See [the survey workflow](docs/core_survey.md).

### What remains before a real competition run

1. Connect the Mega and identify the actual wiring, shield revision, motor directions, gyro, OLED and beacon interface. The colour photo shows `GND/OUT/S2/S3/VCC`, consistent with a TCS230/TCS3200 frequency interface; exact chip identity and cable endpoints remain unverified. `COLOR_DRIVER=2` is selected, but motor/sensor access flags default off.
2. Confirm or adapt the beacon driver. The current optional analog adapter requires three separately resolved strengths; distinguishable beacons alone do not prove that the installed interface supplies these values. A different interface or unsuitable strength-to-distance response needs a driver/model change.
3. Measure sensor mounts and offsets, validate gyro sign and timing, survey/check the field, and calibrate optical response at independent measured positions and headings. Side-light forward positions and height differences remain unmeasured.
4. Capture red/green colour references and the centre, left/right turn and actual box poses, including local ultrasonic offset samples. Box locations are still to be supplied. Save calibration; EEPROM formats older than v5 require re-entering it.
5. Set the known starting heading after reboot/load, then test both colour routes on the real field. Verify clearances, stopping/turning errors and repeatability before relying on the configured acceptance thresholds.

Start the dashboard with `./tools/start_dashboard.sh`, then open **http://127.0.0.1:8765**. Detailed setup is in [the commissioning guide](docs/core_platform.md), [hardware notes](docs/hardware.md) and [serial protocol](docs/serial_protocol.md). Simulation and software tests do not establish physical navigation accuracy.

### Verification completed

Checked on **20 October 2026**:

- Core C++ checks passed in all five host configurations, including both colour routes, localization, calibration/EEPROM recovery, parser/interlocks, frequency colour acquisition and the synthetic field survey.
- All **9** root Python telemetry/dashboard tests passed, including virtual serial communication, persisted recordings and survey JSON exports.
- All four committed PlatformIO profiles compiled: `mega2560`, `mega2560_legacy`, `mega2560_oled` and `mega2560_sh1106`. Two additional sensor/motor-enabled compile configurations also passed; the largest used 5,730 of 8,192 bytes of static RAM. Stack use and physical control-loop timing still require hardware checks.
- Regression checks for the preserved proposed application passed: **22** Jetson tests and its separate Mega sketch compilation. These are software checks, not a proposed-application hardware demo.

Repeatable commands are listed under [Verification](#verification).

## B. Self-proposed application: Lab Assistant

The user says "find the multimeter". The robot undocks, drives to storage, finds the item with YOLO, tells the user, carries it to the workstation on its tray, takes items back to storage, and returns to its wireless charging dock. See the proposal and [docs/state_machine.md](docs/state_machine.md) (S0–S9, T1–T16).

Status: the whole application is drafted. It has 22 passing tests that use fake hardware and has not yet been run on the robot. Every number marked `TODO` in `jetson/config/*.yaml` and `firmware/lab_assistant_mega/config.h` has to be measured on the real robot.

| Proposal module | Team member | Main files |
|---|---|---|
| 1. Ultrasonic + gyroscope | MEKTAVEPONG Rosalin | `firmware/lab_assistant_mega/{ultrasonic,imu,motion}.*`, `jetson/lab_assistant/navigation/` |
| 2. Camera tilt servo | FAUSTINE Sean Wicaksana | `firmware/lab_assistant_mega/tilt_servo.*`, `jetson/lab_assistant/behaviors/search.py` |
| 3. Wireless charging dock | EK-ANANKUL Wirith | `firmware/lab_assistant_mega/charger.*`, `jetson/lab_assistant/behaviors/docking.py`, `perception/markers.py` |
| 4. YOLO equipment recognition | KAWMALI Netitorn | `jetson/lab_assistant/perception/detector.py`, `training/`, `jetson/tools/capture_dataset.py` |

---

## Who owns what

Only edit paths that belong to your part. To change something the other part owns, add it to [Requests](#requests-between-a-and-b) below.

| Path | Owner |
|---|---|
| `firmware/core_robot_mega/` | A (core) |
| `platformio.ini`, `dashboard/`, `tools/start_dashboard.sh`, `tools/telemetry_schema.py` | A |
| `tests/` (repo root), `tools/verify_core.py`, `tools/record_telemetry.py` | A |
| `docs/core_platform.md`, `docs/hardware.md`, `docs/serial_protocol.md`, `docs/core_survey.md` | A |
| `tools/requirements.txt` (local dashboard/recorder) | A |
| `firmware/lab_assistant_mega/` | B (assistant) |
| `jetson/lab_assistant/{main,config,state_machine}.py`, `hardware/`, `navigation/`, `behaviors/`, `perception/`, `voice/` | B |
| `jetson/config/`, `jetson/tools/`, `jetson/tests/`, `training/` | B |
| `docs/state_machine.md`, `docs/lab_assistant_protocol.md`, `jetson/requirements-assistant.txt` | B |
| `README.md`, `.gitignore`, `jetson/lab_assistant/__init__.py`, `.vscode/` | **Shared** |

## Rules for working in parallel

1. **Re-read a shared file just before editing it**, and change only your own part's section. Never rewrite a whole file the other side wrote. Move its content instead and leave a pointer, as was done with `docs/core_platform.md`.
2. **Never overwrite a file you did not create.** If a name you want is already taken, pick another. New docs go under `docs/` with a name that makes the owner obvious.
3. **One source for hardware facts.** Pins and wiring are recorded in [docs/hardware.md](docs/hardware.md) and confirmed in `firmware/core_robot_mega/include/config.h`. The assistant sketch copies the motor pins from there, so update both together. Claim new pins in the [pin table](#pin-allocation) before wiring anything.
4. **Neither part imports the other's Python modules.** The core runs on the Mega with no Python or Jetson dependency.
5. **Run all checks** in [Verification](#verification) before handing work back, not just your own.

---

## One robot, one Mega

The Mega runs **one sketch at a time**. Flash `core_robot_mega` for the parking evaluation and `lab_assistant_mega` for the application demo. Build/upload the core using root `platformio.ini`. Its MEGA3 protocol uses 115200 baud and executes motion/parking locally. The proposed application still uses its separate 115200-baud TEL/VEL protocol. Flash the corresponding firmware; they are not interchangeable.

### Pin allocation

| Pins | Use | Claimed by | Status |
|---|---|---|---|
| PWM 12, 8, 9, 5; DIR1/DIR2 34/35, 37/36, 43/42, A4/A5 (legacy 26/27) | 4 mecanum motors (MOEBIUS TB6612 shield) | A (B copies) | Manufacturer map; revision/wheel signs need physical verification |
| 0 / 1 (Serial0) | Core USB monitor/logging; optional shared Bluetooth | both | Use COM2/COM3 Bluetooth for simultaneous USB logging |
| 22/23, 24/25, 26/27 | Ultrasonic front, left, right (trig/echo) | B | TODO wire |
| 20 / 21 (I2C) | MPU6050 and optional core OLED | A / B | Confirm addresses/models and wiring |
| 10 | Camera tilt servo | B | TODO: match a shield servo header |
| A0 | Wireless charging receiver voltage (through a divider) | B | TODO wire |
| 22/30, 24/25 | Core front ultrasonic +45° left / −45° right | A | Proposed pins; two-sensor mounting specified by user, wiring unconfirmed |
| A6 / A7 / A8 | Core separately resolved left/middle/right beacon channels | A | Analog option only; actual interface unconfirmed |
| 44 / 45 / 47 (S2 / S3 / OUT) | Core frequency colour interface | A | Header identified in IMG_3849; proposed Mega endpoints unconfirmed; Timer5 reserved |

**Sensor placement must work for both parts.** The core now follows the user's two front ±45° ultrasonic layout and front underside colour sensor. Its former three-sensor assumption is removed. The proposed application still has its own front/left/right assumptions; reconcile them before sharing hardware.

### Sharing code between A and B

- **Sensor drivers.** The required core now has independent Mega-side drivers in `firmware/core_robot_mega/src/sensors.cpp`. The proposed application's drivers remain unchanged. They require no cross-imports or shared-library migration.
- **Protocol.** MEGA3 core commands and the proposed TEL/VEL protocol remain separate; one program is flashed at a time. The proposed application is not an input dependency of the core.
- **Dependencies.** The core uses PlatformIO/Arduino libraries only. `jetson/requirements-assistant.txt` is now self-contained. `tools/requirements.txt` is for the local Mac dashboard and optional CSV recorder.

---

## Running

**A. Core / parking.** See [docs/core_platform.md](docs/core_platform.md) and [docs/hardware.md](docs/hardware.md). Open the repository root in PlatformIO:

```bash
pio run -e mega2560
pio run -e mega2560 -t upload
pio device monitor -b 115200
```

Use `./tools/start_dashboard.sh` for live graphs, recordings and calibration. Once commissioned, send `HEADING`, then `ARM` and `START` for the competition mission. Timed `FWD 500` and `DEMO` remain available. Motor/sensor outputs are disabled until the corresponding wiring flags are confirmed. No Jetson connection is needed.

**B. Lab assistant.** Run on the Jetson inside the `ultralytics/ultralytics:latest-jetson-jetpack4` container, which has Python 3.8. Pass `--device` for `/dev/ttyACM0`, `/dev/video0` and `/dev/snd`. Run every command from `jetson/`:

```bash
pip install -r requirements-assistant.txt
python -m tools.teleop                     # drive + live sensor readout (assistant sketch)
python -m tools.calibrate_motion           # measure motion.speed_scale
python -m tools.make_markers --ids 0 1 2   # print the dock/storage/workstation markers
python -m tools.capture_dataset --label multimeter
python -m tools.export_engine models/lab_equipment.pt   # build the TensorRT engine on the Jetson
python -m lab_assistant.main --text        # type "find the multimeter" instead of speaking
python -m lab_assistant.main               # voice control
```

YOLO training runs on a PC or Colab: `python training/split_dataset.py …` then `python training/train.py`.

## Verification

Run from the repo root. Start `./tools/start_dashboard.sh` once to prepare the dashboard virtual environment, then stop the server before running these checks:

```bash
.venv-dashboard/bin/python -m unittest discover -s tests -v # A: telemetry/dashboard HTTP tests
python3 tools/verify_core.py                               # A: mission, localization, EEPROM, colour, survey and parser
(cd jetson && python -m pytest tests -q)                    # B: needs pytest + pyyaml
pio run -e mega2560 -e mega2560_legacy -e mega2560_oled -e mega2560_sh1106
arduino-cli compile --fqbn arduino:avr:mega --build-path /tmp/nontok-assistant-build \
    --libraries ~/Library/Arduino15/libraries firmware/lab_assistant_mega   # needs the Servo library
```

## Requests between A and B

Add a line here, and remove it once the other part has dealt with it.

- **B → A:** Once the motor pins and signs are confirmed on the real board, update `docs/hardware.md`. B will copy them into `firmware/lab_assistant_mega/config.h`.
- **A → B:** Core sensor pins are claimed above. Reconcile front echo pin 30 and the user-specified TWO front ±45° sensors before sharing wiring. Pin 23 can conflict with the legacy shield encoder; 26/27 can conflict with legacy M4 directions.
- **A → B:** Core motor map now follows the MOEBIUS vendor schematic and lives under `include/config.h`; compare signs/revision before updating the proposed sketch.
- **Physical commissioning:** Verify shield revision, remaining sensor models and mounting. IMG_3849 establishes the colour module header, but not its Mega cable endpoints or exact chip marking.

## Repository map

```
README.md                      this file (shared)
docs/
  core_platform.md             A  Mega-only competition, calibration and dashboard
  core_survey.md               A  guided wall/beacon survey, captures and exports
  hardware.md                  A  shield pin profile, commissioning
  serial_protocol.md           A  MEGA3 onboard command/CSV protocol
  state_machine.md             B  S0–S9 / T1–T16 → code
  lab_assistant_protocol.md    B  TEL/VEL protocol
firmware/
  core_robot_mega/             A  include/ configuration and src/ Mega controller
  lab_assistant_mega/          B  motors + heading hold, ultrasonic, gyro, servo, charger
jetson/
  lab_assistant/
    main.py, state_machine.py  B  application main loop and S0–S9
    hardware/ navigation/ behaviors/ perception/ voice/   B
  config/ tools/ tests/        B
  requirements-assistant.txt   B  proposed application only
platformio.ini                 A  Mega build profiles
tests/, tools/verify_core.py   A  host behavior checks
dashboard/                    A  local telemetry UI, SQLite recordings and replay
tools/start_dashboard.sh      A  Mac dashboard launcher
tools/record_telemetry.py      A  optional desktop CSV recorder
training/                      B  YOLO dataset split + training
```
