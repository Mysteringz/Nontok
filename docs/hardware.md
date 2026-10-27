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
The supplied working `Car_Volt_Feedback_24A.ino` now confirms the updated A4/A5
M4 map. See [the working-car reference](working_car_reference.md) for source facts,
the latest sensor connections and the actual USB/backup probe results.

| Channel / default wheel assignment | PWM | DIR1 | DIR2 | Body-forward levels from working ADVANCE |
| --- | --- | --- | --- | --- |
| M1 / front left | 12 | 34 | 35 | LOW / HIGH |
| M2 / front right | 8 | 37 | 36 | HIGH / LOW |
| M3 / rear left | 9 | 43 | 42 | LOW / HIGH |
| M4 / rear right, updated board | 5 | A4 | A5 | HIGH / LOW |

The [older 2019 schematic](https://github.com/MoebiusTech/MecanumRobot-ArduinoMega2560/blob/master/mega2560_sch.pdf)
instead routes M4 DIR1/DIR2 to **26/27**. Select this using
`MOEBIUS_LEGACY_M4_PINS = 1` in `firmware/core_robot_mega/include/config.h`;
leave it at 0 for A4/A5. Establish
the revision from course code, board markings, or an unpowered continuity check
before enabling outputs. The manufacturer also changed the M4 encoder-B pin
from 23 to A1 and swapped connector power/ground positions between revisions.
Do not repin the six-wire motor plugs using the other revision's connector diagram.
The core does not read encoder lines or reconfigure their pins.

Direction levels now follow the user-provided working sketch: its `ADVANCE()` uses
A/C forward and B/D backoff, giving `MOTOR_SIGN={-1,+1,-1,+1}` in this core.
Host checks compare all eight course button patterns against these macros.
Confirm assembled wheel directions with the wheels raised before floor tests.

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
BLUETOOTH_UART=2 or 3 to control via COM2/COM3 while USB records. The working
sketch confirms **Serial3 at 9600**, now the core default, so its wiring stays unchanged.
MEGA3 uses 115200 on USB/Serial0, so an optionally selected COM0 Bluetooth module
must also be configured for 115200; the factory 9600 setting is incompatible.
Dedicated Serial2/3 Bluetooth remains at 9600. The photo does not establish every
Bluetooth signal connection. The working sketch specifies SSD1306 **128×32** at
0x3c; `mega2560_oled` selects that geometry. The user confirms a four-wire OLED
with no reset wire. The code uses no reset GPIO, leaving D28 for colour S0.
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
3. Resolve the reported A4 light/M4 conflict first (see the wiring update below).
   Raise the wheels clear of the floor. Set `MOTOR_WIRING_CONFIRMED` to **1**,
   rebuild and upload. The initial MAX_PWM is capped at 120/255. The updated
   A4/A5 motor profile intentionally fails to compile while light A still uses A4.
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

## Confirmed sensors — 27 October 2026

Measured on the robot with the hardware-probe console. These rows supersede the 23 October light and gyro entries below.

| Signal | Mega pin / value | Evidence |
| --- | --- | --- |
| LDR 1 (leftmost) | A0 | Probe reads all four live; A1 header pin broken, A3 unused, A4/A5 are M4 |
| LDR 2 | A2 | User swapped positions 2 and 3 on 27 Oct |
| LDR 3 | A7 | As above |
| LDR 4 (rightmost) | A6 | |
| LDR polarity | **higher ADC = brighter** | User flashlight/hand test, 27 Oct; reverses the 23 Oct note |
| Left ultrasonic TRIG / ECHO | D48 / D47 (PL1 / PL2) | Plugs swapped 23 Oct after a left-cable fault |
| Right ultrasonic TRIG / ECHO | D40 / D41 (PG1 / PG0) | As above |
| Front ultrasonic TRIG / ECHO | D22 / D24 (PA0 / PA2) | Added by the user 27 Oct; the probe reads it as the third range (left, right, front). Not yet used by the competition mission |
| Left-side ultrasonic (faces +90°) TRIG / ECHO | D10 / D11 (PB4 / PB5, SERVO row) | Wired by the user 27 Oct; probe range index 3 |
| Right-side ultrasonic (faces −90°) TRIG / ECHO | D6 / D7 (PH3 / PH4, SERVO row) | Wired by the user 27 Oct; probe range index 4 |
| Gyro | MPU6050 at 0x68, shield IMU header SDA D20 / SCL D21, INT D33 | WHO_AM_I 0x68; data-ready INT toggles |
| Gyro yaw sign | CCW (left) turn = raw Z negative, so `GYRO_SIGN = -1` | Hand test: 4 × 90° left read raw Z −357.4° |
| Gyro sampling | 200 Hz into the chip FIFO, ~44 Hz low-pass, ±500 °/s; every sample is integrated | Both firmwares since 27 Oct; the competition firmware previously integrated one polled reading per loop |
| Gyro accuracy | Reads ~0.9 % short (best fit 0.991); stationary drift ~0.003 °/s; back to 0.0° after 720° of turning | Same test, ±500 °/s probe range, after the fixes below |
| Bluetooth module | HC-06-style, firmware `V2.3-20231103`, Serial3 D14/D15 at 9600, name **DIVA** | AT replies over `BTAT`; renamed from "Feetwasher" 27 Oct, PIN unchanged |

**Ultrasonic power:** the SERVO header's red row carries battery voltage (labelled 6–8.4 V), not 5 V. Take the side sensors' VCC from a 5 V pin, such as the red row of the GPIO header; only their signal wires go on the SERVO row. The probe pings all five in the order front-left, right side, front, left side, front-right, so consecutive pings face 90–135° apart. It leaves 15 ms between pings and waits at most 18 ms (about 3 m) for each echo.

**I²C reliability (fixed 27 Oct).** The PlatformIO Wire library (framework-arduino-avr 5.2.0) has no timeout unless asked, and never defines `WIRE_HAS_TIMEOUT`, so the firmware's `#if`-guarded `setWireTimeout` compiled to nothing. Motor noise on SDA/SCL then left a transfer waiting forever; the watchdog rebooted the board within seconds of driving (more than 30 reboots in the 27 Oct logs). Both firmwares now set a 25 ms timeout unconditionally; the limit covers a whole transfer, and a 30-byte read takes ~2.8 ms at 100 kHz. After a timeout they clock the bus free and send STOP. The MPU keeps power through a Mega reset and was seen scrambled (accel −32768 on every axis), so the gyro is now also health-checked (WHO_AM_I and 0.5–1.6 g gravity, every 250 ms) and fully reset when implausible. Its bias is taken only from a still robot, automatically once it is still. Telemetry counts I²C timeouts and gyro resets.

## User-reported wiring update — 23 October 2026

Light and gyro rows in this section are superseded by [Confirmed sensors — 27 October 2026](#confirmed-sensors--27-october-2026).

The user supplied the following installed connections and confirmed that the light pin numbers are **analogue inputs**:

| Installed signal | Mega pin | Status |
| --- | --- | --- |
| Light sensor A, left output | A6 | User confirmed physical move from A4 |
| Light sensor A, right output | A3 | User-reported analogue channel |
| Light sensor B, left output | A2 | User-reported analogue channel |
| Light sensor B, right output | A1 | User-reported analogue channel; check the updated shield's M4 encoder connection |
| Ultrasonic A / left TRIG | PG1 = D40 | User-reported |
| Ultrasonic A / left ECHO | PG0 = D41 | User-reported |
| Ultrasonic B / right TRIG | PL1 = D48 | User-reported |
| Ultrasonic B / right ECHO | PL2 = D47 | User explicitly requested keeping this connection |
| Colour S1 / S0 / S3 / S2 / OUT | D25 / D28 / D29 / D30 / D32 | User-reported PA3 / PA6 / PA7 / PC7 / PC5 |

Port-to-Arduino numbers follow the [official Mega pinout](https://docs.arduino.cc/resources/pinouts/A000067-full-pinout.pdf). Bluetooth and OLED connections remain unchanged. These reports replace the earlier proposed ultrasonic allocation below. The user confirmed the physical A4→A6 move and both light modules powered. The separately approved motor-inhibited diagnostic is installed; the competition firmware has not been uploaded.

**Current colour wiring:** the user specified S1=PA3/D25, S0=PA6/D28, S3=PA7/D29, S2=PC7/D30 and OUT=PC5/D32. The sampler now reads PINC bit 5. Right ultrasonic ECHO stays D47, and no timer is reconfigured. This supersedes the earlier D19/D44/D45 proposal. The competition driver sets S0/S1 to 20% scaling. The approved diagnostic has captured passive pulse activity on D32 while leaving the colour controls untouched; RGB classification is not calibrated or verified.

**M4 source confirmed:** the working sketch uses A4/A5. The user confirmed moving the conflicting light output to A6; the LDRs have since been confirmed on A0/A2/A7/A6 (27 Oct, above). Raised-wheel checks remain required before enabling motors. The compile guard still rejects enabling updated M4 with light A on A4. The user confirmed no OLED reset wire, resolving the working sketch's unused `OLED_RESET=28` assignment; core OLED code never toggles that pin.

The four analogue signals are sensor-side channels, not four identified beacons or a proven mapping to the current three-beacon strength model. Keep `LIGHT_DRIVER=0` until their response and beacon-separation method are established. Do not silently substitute three of these four channels for left/middle/right beacon strengths.

## Current core allocation

The user specifies **two** ultrasonic sensors at the front, one facing +45° left and the other −45° right, plus a front underside colour sensor. The former three-sensor/front-plus-two-left assumption has been removed.

| Signal | Mega pins | Placement / role |
| --- | --- | --- |
| Front-left +45° ultrasonic trig/echo | 48 / 47 | PL1 / PL2 since the 23 Oct plug swap |
| Front-right −45° ultrasonic trig/echo | 40 / 41 | PG1 / PG0 since the 23 Oct plug swap |
| Four LDRs, left → right | A0 / A2 / A7 / A6 | Confirmed 27 Oct; higher ADC = brighter; beacon decoding pending |
| Independent-beacon legacy adapter | A6 / A7 / A8 | Unconnected proposal for a different interface; disabled |
| MPU6050 gyro | SDA 20 / SCL 21, INT 33 | Identified at 0x68; `GYRO_SIGN = -1` confirmed 27 Oct |
| TCS34725 option | SDA 20 / SCL 21 | I²C 0x29, selectable only after identification |
| Colour S1/S0/S3/S2/OUT | 25 / 28 / 29 / 30 / 32 | User-reported ports; D32/PC5 direct sampler |
| Optional I²C OLED | SDA 20 / SCL 21 | Default address 0x3c, selectable SSD1306 / SH1106 |

Ultrasonic/light and colour connections above are user-reported. `IMG_3849.JPG` shows a colour module with a visible five-pin `GND OUT S2 S3 VCC` header, consistent with the TCS230/TCS3200 frequency-output family. The chip marking is obscured, so its exact model and board supply rating are not confirmed. The approved diagnostic has been uploaded and the user confirmed the A6 wire move. Its follow-up capture showed A6=422–553 and A1=478–650, with A3/A2 still zero; right ultrasound returned 48/48 echoes, left none. See [the probe record](working_car_reference.md#follow-up-after-wiring-and-power-confirmation). `PROBE` scans I²C addresses while disarmed; an ACK is not a verified model and cannot detect a passive analogue or frequency-output module.

Pin 23 is avoided because it is an M4 encoder input on the older shield. Pins 26/27 and A4/A5 are reserved for the selected M4 revision. TCS3200 pin suggestions do not use encoder pins. The proposed application's separate sensor map must be reconciled before sharing physical wiring; its files were preserved.

`SENSOR_WIRING_CONFIRMED=0` and `LIGHT_DRIVER=0` remain deliberate defaults. `COLOR_DRIVER=2` now selects the interface visible in the new photo, but physical sensor access is still disabled by `SENSOR_WIRING_CONFIRMED=0`. Available choices:

- `LIGHT_DRIVER=1`: three **individually distinguished** analog strength channels. If the real interface is digital, bearing-based, scanned or modulated, adapt the driver/model first. Three beacons do not imply three connected analog pins.
- `COLOR_DRIVER=1`: TCS34725-compatible RGBC over I²C, ID check, 50.4 ms integration and 4× gain, polling completed acquisitions rather than sleeping through integration. Clear readings at or above 75% of the integration-specific saturation level are rejected.
- `COLOR_DRIVER=2`: TCS230/TCS3200-compatible frequency output. The user now identifies S0/S1 as D28/D25; `COLOR_SCALE_PINS_EXPOSED=1` sets HIGH/LOW for 20% scaling. OE must be enabled by the actual module if present. The driver selects R/G/B/clear with S2/S3. After at least 3 ms settling, it samples **D32/PC5** in 96–256 bursts. Every burst observes 128 samples spaced exactly nine 16 MHz CPU cycles apart (72 µs exposure); interrupts are masked for approximately 73 µs per burst and restored between bursts. Gaps vary by 0–31 µs to reduce repeated phase bias. Active exposure is 6.912–18.432 ms per filter, plus call/interrupt/gap overhead. This bounded work occurs in the sensor loop, not in a per-edge ISR. No timer is reconfigured. Very weak channels extend to the maximum burst count, then fail if fewer than two edges were seen.

Counts remain normalized to **edges per 10 ms of active exposure**, excluding unobserved gaps. Missing signal, invalid exposure, measured frequency above 600 kHz or filter-to-result delays over 100 ms invalidate the entire RGBC frame; partial frames are never published. The supported sampling model is the module's approximately 50% duty square wave up to 600 kHz. Narrow/glitchy pulses or frequencies outside that band can be missed/aliased and need physical signal validation. Weak signals have greater counting uncertainty. Re-capture red/green calibration after changing acquisition, wiring or scaling.

### Colour module pictured in IMG_3849

Read the labels on the PCB, not jumper-wire colours. Looking at the header as photographed, its order is **GND, OUT, S2, S3, VCC**.

| Module label | Proposed Mega connection | Status |
| --- | --- | --- |
| GND | GND | Common logic ground |
| OUT | **D32 / PC5** | User-reported; direct-port input |
| S2 | D30 / PC7 | User-reported filter select |
| S3 | D29 / PA7 | User-reported filter select |
| S0 | D28 / PA6 | User-reported scale control; four-wire OLED has no reset lead here |
| S1 | D25 / PA3 | User-reported scale control |
| VCC | Verified module supply, normally Mega 5V for a confirmed 5V-compatible module | Confirm board rating before power |

The connector photo cannot establish the present Mega endpoints or all connections on the opposite side of the module. Trace the three visible signal wires and verify that S0/S1 select an active frequency scale and OE is enabled before enabling the sensor flag. If these inputs are exposed elsewhere, configure or strap them according to the actual board rather than leaving them floating. An I²C address scan cannot find this module: it must be tested by checking frequency counts as the selected filter/paper changes. Capture `COLOR RED` and `COLOR GREEN` at the actual mounting height and illumination; changing the module, scale setting or illumination requires re-calibration.

The colour driver reserves **D32**, not Timer5. D30/D29 are ordinary GPIO filter controls; D28/D25 control scaling. The shield motor PWM and Serial3 Bluetooth interfaces are unchanged. The 128×32 OLED driver uses I²C and no reset GPIO. A compile guard rejects the hypothetical combination of a physical OLED reset on D28 and enabled colour scaling. The proposed application remains a separate program with its own unmodified pin assumptions.

`python3 tools/verify_core.py` includes instruction-level waveform checks of the sampler source, cycle spacing/termination, active-exposure normalization, missing/invalid frame recovery and pin-conflict checks. AVR builds also assemble the actual sampler. Those checks do not prove physical input waveform quality, serial reliability or control-loop timing; measure these with the connected board before a mission.

The default frequency interface matches the visible header; exact silicon identity remains an inference until markings or hardware checks establish it. The alternate I²C driver is retained for other hardware. Register/filter definitions follow the [TCS34725 datasheet](https://cdn-shop.adafruit.com/datasheets/TCS34725.pdf), [Adafruit driver definitions](https://github.com/adafruit/Adafruit_TCS34725/blob/master/Adafruit_TCS34725.h) and [ams TCS3200 datasheet](https://look.ams-osram.com/m/664723bdb31f55db/original/TCS3200-DS000107.pdf). Verify the actual module's supply/logic voltage and onboard level shifting before wiring; a bare chip and a breakout board are not interchangeable.

Ultrasonics are alternated with at least 40 ms between pings, 24 ms echo timeout, median-of-three valid readings and 350 ms stale invalidation. No echo is not a valid far-distance reading. Their usable return/reflector geometry at every required orientation must be measured. The MPU6050 startup bias calibration runs while all motors are stopped; keep the physical robot stationary throughout startup. Gyro hardware/scale/sign and loop timing require actual board validation.

The core uses local ultrasonic calibration instead of inventing field wall geometry. Read [core_platform.md](core_platform.md) for the coordinate convention, beacon model limitations, centre/turn/box captures, acceptance thresholds and Mac dashboard. Once the board is connected, first verify the remaining sensor models and trace the shield cables, then enable a verified configuration and capture real references.
