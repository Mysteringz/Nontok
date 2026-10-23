# MEGA3 core protocol

The Mega performs every control/localization operation. No PC heartbeat or Jetson process is required after `START`. MEGA3 replaces MEGA2; the old three-range CSV format is not compatible. The separate proposed application's TEL/VEL protocol is unchanged.

## Transport

USB: **115200, 8N1**, ASCII uppercase commands terminated with newline. CR is ignored. Command lines are at most 143 bytes, excluding newline. Invalid, oversized, or timed-out partial packets stop/disarm into a fault. Stop is accepted from either command port. `!` stops immediately even inside a damaged packet; remaining bytes are discarded through the next newline. The dashboard sends `!STOP\n` for its stop button.

The supplied working car sketch confirms **Serial3 at 9600 baud** on D14/D15 for Bluetooth; `BLUETOOTH_UART=3` is now the core default. USB remains 115200.

`BLUETOOTH_UART=0` shares Serial0 with USB. Both have **one baud rate**: a COM0 Bluetooth module must be configured to 115200 for this firmware, and must not contend with USB RX. A factory 9600-baud COM0 module does not work at the new core baud. Prefer the verified COM2/COM3 header at 9600 and set `BLUETOOTH_UART=2/3` for simultaneous Bluetooth control and USB telemetry. Confirm physical wiring before moving the module.

The port that successfully sends `ARM` owns motion until `STOP`. The other port can read status and stop. Calibration requires `DISARMED`, so it cannot replace targets during a mission. `ARM` never starts motion itself.

## Commands

| Command | Result |
|---|---|
| `PING`, `STATUS`, `HELP` | Protocol/status/help; USB STATUS queues a telemetry row |
| `STOP` / `Z` / `!` | Stop, clear fault/finished latch, disarm |
| `ARM` | Enter IDLE only if motor wiring confirmed; acknowledge a prior fault with STOP first |
| `START` (`PARK` legacy alias) | Start the entire competition sequence; all sensors/profile/heading must be ready |
| `DEMO` | Six bounded movement pulses, scheduled on Mega; requires IDLE |
| `SPEED n` | Manual translation speed 1–150 nominal mm/s |
| `FWD/BACK/LEFT/RIGHT/CW/CCW [ms]` | Timed manual motion; 1–2000 ms, default 500 |
| `DRIVE vx vy wz ms` | Manual body velocities, translation magnitude ≤150 mm/s, yaw ≤45°/s, 1–2000 ms |
| `PROBE` | Disarmed-only I²C scan; reported decimal addresses are not a confirmed device identity |
| `HEADING degrees` | Set a known world heading against current gyro; disarmed, valid IMU required |
| `BEACON i x y height ambient signal distance exponent` | Configure one measured XY position and optical model; confirms its position bit; i=0 left, 1 middle, 2 right; distances mm, signal ambient-subtracted ADC |
| `COLOR RED` / `COLOR GREEN` | Capture normalized RGB paper exemplar from a valid settled sample |
| `TEACH point x y yaw` | Capture base ultrasonic readings at the measured chassis pose, clearing old derivatives for that point |
| `SAMPLE point X/Y/YAW offset` | Capture local range derivative after a measured offset from that point's original base |
| `CALIB` | Dump the current profile as a `# PROFILE {...}` JSON line; disarmed only |
| `SAVE` | Commit current profile, including incomplete commissioning work, to redundant CRC-protected EEPROM slots |
| `LOAD` | Reload last committed profile; invalidates the current heading reference |
| `DEFAULTS` | Reset RAM profile to the 1500 mm wall width / 300 mm marker facts, leaving side beacon XY unconfirmed and clearing sensor/point calibration; EEPROM changes only on SAVE |

Point names: `CENTER`, `TURN_LEFT`, `TURN_RIGHT`, `BOX_LEFT`, `BOX_RIGHT`. X/Y offsets must have magnitude 20–80 mm; YAW offsets 2–10°. See [the exact capture procedure](core_platform.md#capture-the-field-references) before collecting samples. Capturing a base records its raw ultrasonic returns. The known supplied XY/yaw and measured derivatives allow local fusion; the Mac does not compute them.

Working-sketch Bluetooth button patterns are retained for finite manual pulses and require ARM: A forward, E backward, B/H right/left strafe, C/G CCW/CW, D/F the working sketch's forward-left/forward-right wheel patterns, Z stop. These names describe the core kinematics; verify actual assembled directions. The supplied App Inventor XML stores literal backslash+n suffixes; the parser accepts those only after one known A–H/Z letter. One-letter packets without newline are accepted after a 50 ms gap. Ordinary text packets time out after 500 ms. Commands do not use dynamic `String` allocation. The supplied app has no ARM action; its UI needs one on the Bluetooth port before driving the core. Merely opening the app cannot arm the robot.

`CALIB` includes `field_width_mm=1500` (side-wall spacing) and `beacon_position_mask`. Bit 0 is left, bit 1 middle, bit 2 right. Default mask 2 defines only the origin; the two side lights' internal X=0 placeholders are unmeasured. All three positions must be explicitly configured before the position estimate can become valid. Beacons may have different X coordinates. EEPROM format is now v5, rejecting older calibration records.

Replies begin with `#`, e.g. `# OK ARM`, `# ERR START`, `# OK CALIBRATION`, `# ERR CALIBRATION`. A queued dashboard command is not an acknowledgment from the board: check the event log for the actual reply. After connection `# READY MEGA3` and the CSV header identify the firmware. Numeric I²C address 41 = 0x29, 60 = 0x3c, 104 = 0x68. The frequency colour interface visible in IMG_3849 does not have an I²C address; check its live RGB/clear counts instead.

## Telemetry

USB CSV is scheduled every 100 ms and pumped into the UART buffer without blocking motor timers. If output falls behind, rows are skipped; the controller continues independently. Optional Bluetooth Serial2/3 carries command replies/status, while full streaming telemetry remains on USB. A reply may terminate an incomplete CSV line; the decoder discards malformed rows.

```text
ms,mode,armed,left45_mm,right45_mm,gyro_deg,gyro_dps,imu_ok,light_left,light_middle,light_right,light_ok,red,green,blue,clear,color_ok,color,decision,x_mm,y_mm,yaw_deg,pose_ok,heading_ok,beacon_residual_mm,us_fused,us_residual_mm,reference,target,vx,vy,wz,calibrated,hardware,fault
```

| Fields | Meaning |
|---|---|
| `ms` | Unsigned 32-bit Mega uptime, wraps after about 49.7 days |
| `mode`, `armed`, `fault` | Mission state, motor-arm state, fault reason (`NONE` normally) |
| `left45_mm`, `right45_mm` | Actual two front ±45° returns; −1 = invalid/stale/no echo |
| `gyro_deg`, `gyro_dps`, `imu_ok` | Unwrapped integrated raw gyro yaw, rate and validity |
| `light_left/middle/right`, `light_ok` | Independently resolved beacon channels, normalized ADC counts |
| `red/green/blue/clear`, `color_ok` | Raw colour channels and validity; driver units differ (I²C RGBC counts vs D32/PC5 sampled edges normalized to 10 ms of active exposure, at selected frequency scaling) |
| `color`, `decision` | Current classification vs latched mission choice (`UNKNOWN`, `RED`, `GREEN`) |
| `x_mm`, `y_mm`, `pose_ok` | Estimated world XY and validity; ignore numeric XY if pose_ok=0 |
| `yaw_deg`, `heading_ok` | Fused world heading in ±180° and heading-reference validity |
| `beacon_residual_mm` | RMS optical-range fit error; meaningful only when pose_ok=1 |
| `us_fused`, `us_residual_mm`, `reference` | Local ultrasound model used, its pre-correction innovation RMS, and active taught reference |
| `target` | Current reference sought by the controller |
| `vx`, `vy`, `wz` | Commanded body-forward/left mm/s and CCW °/s; not encoder-measured velocity |
| `calibrated` | Whole profile validates; independent of physical hardware-confirmation flags |
| `hardware` | Bit mask: 1 motor wiring confirmed, 2 sensor wiring confirmed, 4 light driver selected, 8 colour driver selected. All bits = 15. Not proof of an actual connected/working device. |

State names: `DISARMED`, `IDLE`, `MANUAL`, `DEMO`, `TO_CENTER`, `READ_COLOR`, `TURN`, `TO_BOX`, `DOCK`, `FINISHED`, `FAULT`. A completed mission stays `FINISHED`, with `armed=0` and zero motor commands, until an explicit STOP/reset. Serial recording continues. Restarting the Mac cannot itself restart a completed mission.

The dashboard prefixes retained samples with its own `received_at` UTC timestamp, `elapsed_s` recording time and sequence. Use these times across a Mega reset; raw `ms` restarts at zero. Broken/nonfinite rows and unexpected field counts are ignored. A reset is reported in the event log and requires heading re-referencing before another autonomous start.


## Field-survey commands

All require the Mega to be disarmed. Arming/motion is blocked while the survey session or a capture is active. See [core_survey.md](core_survey.md) for required reference measurements and the exact workflow.

| Command | Operation |
|---|---|
| `SURVEY MOUNTS lf ll rf rl` | Measured sensor-face forward/left offsets, mm, each within ±500; set before collecting captures |
| `SURVEY BEGIN x y yaw` | Clear old captures, anchor the gyro to a known chassis pose and begin a disarmed survey |
| `SURVEY POSE x y` | Set the next tape-measured chassis position, preserving the session's heading reference |
| `SURVEY TAKE WALL` | Settle, then average eight fresh range updates per channel |
| `SURVEY TAKE POINT` | Range capture plus 16 fresh independent-beacon readings at a known floor pose |
| `SURVEY DARK` | Capture 16 fresh ambient light updates; the operator must turn the beacons off |
| `SURVEY OPTICS hl hm hr n` | Measured vertical separations in mm (0–3000) and chosen optical exponent (0.5–5) |
| `SURVEY SOLVE` | Fit wall planes and optional optical models on the Mega; finite calculation while motors remain disarmed |
| `SURVEY APPLY` | Apply valid results to RAM; does not write EEPROM or manufacture missing box/colour references |
| `SURVEY STATUS` | Report phase, sample count, staged settings and fitted results |
| `SURVEY DUMP` | Re-emit all current raw capture records, up to 40 |
| `SURVEY CANCEL`, `SURVEY END` | Cancel the active averaging window / exit the session while preserving captures |
| `SURVEY CLEAR` | Exit and clear the current dataset/results; staged mount/ambient/optical settings remain |

Coordinates are mm, known yaw is degrees. The station must have 0<X≤4000 and −750<Y<750. Side-light fits assume LEDs lie on the measured side walls; the middle light defines the origin. Those constraints belong to this photographed field, not a general SLAM system.

Replies `# SURVEY {...}` report status and fit quality. `# SURVEY_SAMPLE {...}` reports the index, Mega uptime, measured X/Y, gyro-referenced yaw, optical-point flag, averaged range pair and light channels. `CALIB` includes the applied `survey_map`. Existing MEGA3 CSV fields are unchanged. Fresh-sample sequence counters are internal; repeated loop reads cannot count as independent measurements.

`SOLVE` may take several seconds; wait for the final status before further commands. `APPLY` requires a valid wall fit. A failed attempted optical fit clears the corresponding ready bits rather than retaining stale models. A first/changed mount calibration clears local reference samples. Use `SAVE` afterward and re-teach centre/turn/box references as required. `STOP`/`!` exits a survey and reports the updated status.
