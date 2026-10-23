# Working car sketch and current wiring

On 23 October 2026 the user supplied `TestCode_2026/Car_Volt_Feedback_24A/Car_Volt_Feedback_24A.ino`, `IDP_Controller.aia` and `IDP_Controller.apk`. These are reference inputs, not replacements for the standalone competition state machine. The original files have not been edited, installed or uploaded by this session.

## Facts from the working sketch

| Item | Working configuration | Core implementation |
| --- | --- | --- |
| M1 PWM / direction | D12 / D34,D35 | Same |
| M2 PWM / direction | D8 / D37,D36 | Same |
| M3 PWM / direction | D9 / D43,D42 | Same |
| M4 PWM / direction | D5 / A4,A5 | Same updated-board profile; legacy D26/D27 is not this car's reference |
| Body-forward wheel polarity | A/C `FORWARD`, B/D `BACKOFF` | `MOTOR_SIGN={-1,+1,-1,+1}` |
| USB | Serial, 115200 baud | Same |
| Bluetooth | Serial3, 9600 baud; TX3 D14 / RX3 D15 | Default `BLUETOOTH_UART=3` |
| OLED | SSD1306, 128×32, I²C 0x3c | `mega2560_oled` now selects `OLED_DRIVER=3`, four text rows |
| OLED reset | Sketch names D28; user confirms no physical reset wire | Core uses no GPIO reset; D28 is available for colour S0 |
| Voltage stream | `analogRead(A0)` printed when it changes | Explains the observed numeric stream; raw ADC counts are not a calibrated battery voltage |
| Servo pins | Pan D48, tilt D47 | Servo control omitted from core/probe because these now belong to ultrasonics |

The old sketch must be adapted before reuse with the new sensor wiring: it attaches Servo outputs to D47/D48. Do not run those servo attachments into the ultrasonic ECHO/TRIG connections. The competition core and prepared diagnostic do not attach those servos.

## Current signal allocation

| Sensor signal | Port bit | Arduino pin |
| --- | --- | --- |
| Light A left output | PF6 | **A6** — user confirmed the physical move from A4 |
| Light A right output | PF3 | A3 |
| Light B left output | PF2 | A2 |
| Light B right output | PF1 | A1 |
| Left ultrasonic TRIG / ECHO | PG1 / PG0 | D40 / D41 |
| Right ultrasonic TRIG / ECHO | PL1 / PL2 | D48 / D47 |
| Colour S1 | PA3 | D25 |
| Colour S0 | PA6 | D28 |
| Colour S3 | PA7 | D29 |
| Colour S2 | PC7 | D30 |
| Colour OUT | PC5 | D32 |

The colour allocation supersedes the earlier D19/D44/D45 proposal. The direct-port sampler now reads PINC bit 5, retaining nine-cycle sample spacing and leaving timers unchanged. S0=HIGH and S1=LOW select 20% scaling. The four-wire OLED uses only power, ground, SDA and SCL. The MCU port conversions follow the [Arduino Mega pinout](https://docs.arduino.cc/resources/pinouts/A000067-full-pinout.pdf).

The user confirmed that light A's left signal has physically moved from A4 to A6 and that both light modules are powered. A4 remains reserved for M4. `MOTOR_WIRING_CONFIRMED=0` and `SENSOR_WIRING_CONFIRMED=0` remain defaults until commissioning. Four raw light channels still require the actual beacon-separation interface/model; `LIGHT_DRIVER=0` remains selected. Check A1 against any connected M4 encoder output too. The user also confirmed that **no gyro is connected yet**; missing responses at 0x68/0x69 are expected. Install and identify the gyro before validating heading or gyro-controlled turns.

## Phone-controller reference

The `.aia` archive was inspected as XML; the `.apk` was not run. Its button blocks contain A/B/C/E/G/H and Z, with literal backslash+n suffixes in the stored text. The core accepts that suffix only on known button letters, as well as actual newline or bare-letter packets. A/B/C/D/E/F/G/H now reproduce the working sketch's enabled-wheel polarity patterns. The sketch's printed diagonal labels are not a reliable description of its actual motor macros.

Core commands remain bounded and require `ARM` on the controlling port; there is no automatic arming. The supplied app has no ARM action, so its UI must gain an ARM message (or use a Bluetooth terminal that can send it) before it can control the core. The supplied APK is therefore not a verified drop-in controller. The core's MEGA3 telemetry also differs from the original app's plain A0-number stream.

## What was actually probed

The Mac enumerated `/dev/cu.wchusbserial210`, USB VID:PID `1A86:7523`. A passive 115200-baud read received numeric lines. A bootloader read confirmed ATmega2560 signature **0x1e9801**. Existing flash and EEPROM were backed up and independently verified against the board: 261,406 represented flash bytes (the tool omits the erased tail) and 4,096 EEPROM bytes. Full raw images and logs are retained under `.local-data/hardware-probe/2026-10-23/`.

The user approved installing and leaving the separate [hardware probe](../firmware/hardware_probe_mega/) on the board. Its upload verified **8,384 flash bytes**. The competition firmware has not been uploaded. The diagnostic holds the four motor PWM signals LOW, leaves direction/encoder and colour filter/scaling pins untouched, reads the four analogue inputs, triggers the reported ultrasonic pins, scans I²C and passively samples colour OUT D32. It contains no motion commands or EEPROM writes and does not refresh the OLED.

### First live capture

48 well-formed samples were retained on 23 October, 15:25 HKT (07:25 UTC):

| Check | Observed result | Interpretation / next check |
| --- | --- | --- |
| I²C scan | Only address 60 / 0x3c ACKed | Consistent with the supplied OLED; ACK alone is not a model identity |
| Gyro 0x68 / 0x69 | No response | User subsequently confirmed no gyro installed; expected absence |
| Light ADC A6 | 75–92 | Raw signal observed; user subsequently confirmed the physical wire move |
| Light ADC A3 / A2 / A1 | 0 / 0 / 0–2 | User subsequently confirmed both modules powered; controlled illumination response remains untested |
| Left ultrasonic | 0 valid returns out of 48 | Place a flat reflector 10–30 cm along its beam; check power/TRIG/ECHO if still absent |
| Right ultrasonic | 37 valid returns out of 48; valid median 92 mm | Echoes received, with intermittent misses; repeat against a stable reflector |
| Colour OUT D32 | 403–1,361 edges per 6.912 ms active exposure | Pulse activity observed; filter/scaling pins deliberately not driven, so no RGB classification or calibration established |

Logs: `.local-data/hardware-probe/2026-10-23/diagnostic-readings.jsonl` and `diagnostic-summary.json`. Motor PWM was held LOW throughout the diagnostic capture. The diagnostic remains installed; the original sketch was not restored because its D47/D48 Servo outputs conflict with current ultrasonic wiring. Original application-only, full flash and EEPROM recovery images remain saved locally.

### Follow-up after wiring and power confirmation

Another 48 well-formed samples were retained on 23 October, 15:35 HKT (07:35 UTC), after the user's confirmations above. These are ambient readings, not a controlled illumination or distance test.

| Check | Observed result |
| --- | --- |
| Light A left / A6 | 422–553 ADC counts |
| Light A right / A3 | 0 ADC counts throughout |
| Light B left / A2 | 0 ADC counts throughout |
| Light B right / A1 | 478–650 ADC counts |
| Left ultrasonic | 0 valid returns out of 48 |
| Right ultrasonic | 48 valid returns out of 48; median 210 mm |
| I²C scan | Only 0x3c ACKed; no gyro installed |
| Colour OUT D32 | 442–1,314 edges per 6.912 ms active exposure; filters/scaling still untouched |
| Motor PWM | LOW in all 48 samples |

Logs: `.local-data/hardware-probe/2026-10-23/diagnostic-followup-073524.jsonl` and `diagnostic-followup-073524-summary.json`. A6 and A1 now show changing signals. A3 and A2 remaining zero does not establish a failed module: test each detector separately by covering it, then illuminating it while keeping the robot stationary, and trace the signal/ground connections if it stays zero. Both modules being powered is user-reported, not a measured supply-voltage check. For the left ultrasonic, place a flat reflector 10–30 cm along its own angled beam before checking TRIG/ECHO/power. The right return is more consistent in this capture, but accuracy still needs comparison with a measured distance.

## Checks

`python3 tools/verify_core.py` checks forward wheel polarity and all eight course button patterns, literal suffix framing, exact colour pin constants, pin-conflict rejection, colour burst timing/frames, mission routes and calibration/storage behavior. Standard AVR builds and the separate probe build check compilation. Hardware direction, sensor response, timing and field accuracy remain commissioning work.
