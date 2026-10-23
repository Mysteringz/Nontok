# Hardware profile and commissioning

## Identified board and remaining checks

The controller photo `IMG_3846.JPG` identifies the **MOEBIUS TECH
4Motor&9Servo shield for Mega 2560**. Its logo, two TB6612FNG chips, four
six-pin M1–M4 connectors, and separate VM/VIN terminal blocks match the
[manufacturer's board reference](https://github.com/MoebiusTech/MecanumRobot-ArduinoMega2560/blob/master/bord.jpg).
The earlier Emakefun identification was incorrect; its I2C MotorDriverBoard is
a different product and its library should not be used for this shield.

The PlatformIO core uses the manufacturer's direct GPIO/PWM pin map from the
[updated 2024-07-02 schematic](https://github.com/MoebiusTech/MecanumRobot-ArduinoMega2560/blob/master/mega2560__2024-7%EF%BC%88update%EF%BC%89.pdf).
The photograph confirms the board family but does not expose the M4 trace routing
or which connector cable runs to each wheel.

| Channel / default wheel assignment | PWM | DIR1 | DIR2 | Default forward levels |
| --- | --- | --- | --- | --- |
| M1 / front left | 12 | 34 | 35 | LOW / HIGH |
| M2 / front right | 8 | 37 | 36 | LOW / HIGH |
| M3 / rear left | 9 | 43 | 42 | LOW / HIGH |
| M4 / rear right, updated board | 5 | A4 | A5 | LOW / HIGH |

The [older 2019 schematic](https://github.com/MoebiusTech/MecanumRobot-ArduinoMega2560/blob/master/mega2560_sch.pdf)
instead routes M4 DIR1/DIR2 to **26/27**. Select this using
`MOEBIUS_LEGACY_M4_PINS = 1` in `firmware/core_robot_mega/include/config.h`;
leave it at 0 for A4/A5. Establish
the revision from course code, board markings, or an unpowered continuity check
before enabling outputs. The manufacturer also changed the M4 encoder-B pin
from 23 to A1 and swapped connector power/ground positions between revisions.
Do not repin the six-wire motor plugs using the other revision's connector diagram.
The core does not read encoder lines or reconfigure their pins.

Default direction levels follow
[the manufacturer's BT2560 sketch](https://github.com/MoebiusTech/MecanumRobot-ArduinoMega2560/blob/master/BT2560.ino).
Its other examples use different wheel polarity conventions. `MOTOR_SIGN` remains
a commissioning setting: positive robot velocity must be checked against actual
wheel rotation. The M1–M4 wheel assignments above follow the manufacturer's layout;
trace the cables to confirm the assembled car uses that arrangement.

## Connections

```text
Computer -- USB data cable --> Mega USB-B (upload / recording only)
Phone -- Bluetooth module --> Mega COM0, COM2 or COM3 (commands)
Mega --> shield --> four wheel motors
Mega --> parking sensors and optional OLED
```

Use the course-approved power arrangements. Motor power and Jetson power must
match their respective hardware specifications; this project does not configure
regulators, charging, shield power jumpers, or battery protection.

The manufacturer's diagram exposes COM0 (`Serial`, TX0=1/RX0=0), COM2
(`Serial2`, TX2=16/RX2=17), and COM3 (`Serial3`, TX3=14/RX3=15).
A Bluetooth module routed to COM0 can contend with USB serial; disconnect or
reroute it using the documented header pin order before USB control. Select
BLUETOOTH_UART=2 or 3 to control via COM2/COM3 while USB records. Default 0
shares Serial0. MEGA3 uses 115200 on USB/Serial0, so a COM0 Bluetooth module
must also be configured for 115200; the factory 9600 setting is incompatible.
Dedicated Serial2/3 Bluetooth remains at 9600. The photo does not establish every
Bluetooth signal connection. OLED support is optional, with SSD1306/SH1106 I2C
profiles; its actual controller must be identified first.
The disabled motor build does not configure or write any motor pins.

## Build and upload

Use PlatformIO with the repository root as the project. The target is
`megaatmega2560` with the Arduino framework. Save a copy of any course firmware
before replacing the board's current program. From the project root:

```bash
pio device list
pio run -e mega2560
pio run -e mega2560 -t upload --upload-port MEGA_PORT
pio device monitor -b 115200 --port MEGA_PORT
```

Replace `MEGA_PORT` with the computer's actual board port. The Jetson is not part
of this build. PlatformIO installs AVR framework/toolchain dependencies; OLED
environments also install U8g2. The
[Mega PlatformIO configuration](https://docs.platformio.org/en/latest/boards/atmelavr/megaatmega2560.html)
documents the target board ID.

## Commissioning order

1. Leave `MOTOR_WIRING_CONFIRMED` at **0**. Upload and send STATUS in the serial
   monitor. ARM must fail with `MOTOR_WIRING_UNCONFIRMED`.
2. Check the board revision and select the M4 direction pins using
   `MOEBIUS_LEGACY_M4_PINS`. Confirm the M1–M4 cables run to the expected wheels.
3. Raise the wheels clear of the floor. Set `MOTOR_WIRING_CONFIRMED` to **1**,
   rebuild and upload. The initial MAX_PWM is capped at 120/255.
4. Send ARM then FWD 200. Verify all four wheel positions and forward
   directions; correct `MOTOR_SIGN` and cable/channel order as necessary.
5. Test backward, left strafe, right strafe, and both rotations. Check the
   mecanum rollers form the layout required by the mixing equations.
6. Test STOP and !. Stop sending manual commands and verify that the final
   timed pulse ends within its requested duration. Nothing resumes on boot.
7. Begin low-speed floor tests, then tune MIN_PWM, MAX_PWM, ROTATION_ARM_MM,
   and MAX_WHEEL_MM_S from measurements.

Without wheel encoders or another feedback source, `DRIVE 80 0 0 1000`
does not guarantee an 80 mm displacement. Likewise rotation rates do not promise
accurate turn angles. The parking routine separately uses actual gyro and range
feedback; its sensor wiring and geometry must be confirmed before enabling it.

## Core competition sensor allocation (proposed, not physically confirmed)

The user specifies **two** ultrasonic sensors at the front, one facing +45° left and the other −45° right, plus a front underside colour sensor. The former three-sensor/front-plus-two-left assumption has been removed.

| Signal | Mega pins | Placement / role |
| --- | --- | --- |
| Front-left +45° ultrasonic trig/echo | 22 / 30 | Left diagonal actual return |
| Front-right −45° ultrasonic trig/echo | 24 / 25 | Right diagonal actual return |
| Independently resolved beacon strengths | A6 / A7 / A8 | Left / middle / right beacon, analog driver option only |
| MPU6050 | SDA 20 / SCL 21 | I²C 0x68; assumed model, verify module and CCW sign |
| TCS34725 option | SDA 20 / SCL 21 | I²C 0x29, selectable only after identification |
| Photo-matched frequency colour interface S2/S3/OUT | 44 / 45 / 47 | Visible five-pin header; D47 is the hardware counter input |
| Optional I²C OLED | SDA 20 / SCL 21 | Default address 0x3c, selectable SSD1306 / SH1106 |

These are **proposed Mega pin assignments**, not cable connections proven by the photograph. `IMG_3849.JPG` shows a colour module with a visible five-pin `GND OUT S2 S3 VCC` header, consistent with the TCS230/TCS3200 frequency-output family. The chip marking is obscured, so its exact model, board supply rating and frequency-scaling wiring are not confirmed. No wires have been moved and no firmware uploaded as part of implementation. The user will connect the board later for probing/commissioning. `PROBE` scans I²C addresses while disarmed; an ACK is not a verified model and cannot detect a passive analog or frequency-output module.

Pin 23 is avoided because it is an M4 encoder input on the older shield. Pins 26/27 and A4/A5 are reserved for the selected M4 revision. TCS3200 pin suggestions do not use encoder pins. The proposed application's separate sensor map must be reconciled before sharing physical wiring; its files were preserved.

`SENSOR_WIRING_CONFIRMED=0` and `LIGHT_DRIVER=0` remain deliberate defaults. `COLOR_DRIVER=2` now selects the interface visible in the new photo, but physical sensor access is still disabled by `SENSOR_WIRING_CONFIRMED=0`. Available choices:

- `LIGHT_DRIVER=1`: three **individually distinguished** analog strength channels. If the real interface is digital, bearing-based, scanned or modulated, adapt the driver/model first. Three beacons do not imply three connected analog pins.
- `COLOR_DRIVER=1`: TCS34725-compatible RGBC over I²C, ID check, 50.4 ms integration and 4× gain, polling completed acquisitions rather than sleeping through integration. Clear readings at or above 75% of the integration-specific saturation level are rejected.
- `COLOR_DRIVER=2`: TCS230/TCS3200-compatible frequency output. S0/S1 and OE are **not visible on the shown header**; the photo does not establish whether they are exposed elsewhere or strapped on the PCB. The default driver does not write unverified scaling/control pins or assume 20% scaling. It selects R/G/B/clear with S2/S3 and counts OUT edges using Mega **Timer5 on D47/T5**. Each filter gets at least 3 ms settling followed by at least a 10 ms gate. Counts are normalized to a 10 ms interval, accounting for ultrasonic work delaying the loop. Gates exceeding 50 ms, zero signal or values above the supported 600 kHz range invalidate the whole four-channel acquisition. Partial frames are never published.

### Colour module pictured in IMG_3849

Read the labels on the PCB, not jumper-wire colours. Looking at the header as photographed, its order is **GND, OUT, S2, S3, VCC**.

| Module label | Proposed Mega connection | Status |
| --- | --- | --- |
| GND | GND | Common logic ground |
| OUT | **D47** | Required Timer5/T5 counter input; replaces the earlier proposed D46 |
| S2 | D44 | Filter select, digital output |
| S3 | D45 | Filter select, digital output |
| VCC | Verified module supply, normally Mega 5V for a confirmed 5V-compatible module | Confirm board rating before power |

The connector photo cannot establish the present Mega endpoints or all connections on the opposite side of the module. Trace the three visible signal wires and verify that S0/S1 select an active frequency scale and OE is enabled before enabling the sensor flag. If these inputs are exposed elsewhere, configure or strap them according to the actual board rather than leaving them floating. An I²C address scan cannot find this module: it must be tested by checking frequency counts as the selected filter/paper changes. Capture `COLOR RED` and `COLOR GREEN` at the actual mounting height and illumination; changing the module, scale setting or illumination requires re-calibration.

Timer5 is reserved exclusively by this core driver. Do not add a Servo library or use Timer5 PWM on D44/D45/D46 in this sketch. D44/D45 remain ordinary GPIO filter controls. The core's existing motor PWM timers are separate. The proposed sketch is a different program and is unchanged. If S0/S1 are present and verified as connected to D40/D41, select `COLOR_SCALE_PINS_EXPOSED=1` (20% scaling, OE wired enabled). The default 0 leaves those unverified pins untouched and requires scaling/enable inputs to be held at valid levels by the actual module/wiring.

The default frequency interface matches the visible header; exact silicon identity remains an inference until markings or hardware checks establish it. The alternate I²C driver is retained for other hardware. Register/filter definitions follow the [TCS34725 datasheet](https://cdn-shop.adafruit.com/datasheets/TCS34725.pdf), [Adafruit driver definitions](https://github.com/adafruit/Adafruit_TCS34725/blob/master/Adafruit_TCS34725.h) and [ams TCS3200 datasheet](https://look.ams-osram.com/m/664723bdb31f55db/original/TCS3200-DS000107.pdf). Verify the actual module's supply/logic voltage and onboard level shifting before wiring; a bare chip and a breakout board are not interchangeable.

Ultrasonics are alternated with at least 40 ms between pings, 24 ms echo timeout, median-of-three valid readings and 350 ms stale invalidation. No echo is not a valid far-distance reading. Their usable return/reflector geometry at every required orientation must be measured. The MPU6050 startup bias calibration runs while all motors are stopped; keep the physical robot stationary throughout startup. Gyro hardware/scale/sign and loop timing require actual board validation.

The core uses local ultrasonic calibration instead of inventing field wall geometry. Read [core_platform.md](core_platform.md) for the coordinate convention, beacon model limitations, centre/turn/box captures, acceptance thresholds and Mac dashboard. Once the board is connected, first verify the remaining sensor models and trace the shield cables, then enable a verified configuration and capture real references.
