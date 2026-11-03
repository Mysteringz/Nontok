# REAL Run Prog

The program on the robot from 3 Nov 2026, about 13:05 (saved first as morning-calibrated-final): the morning-final firmware ("Fixed + adjust"), with the new field
calibration of 3 Nov 12:54-12:56 stored in the robot's EEPROM.

- `real-run-prog.hex`: the firmware (96,244 bytes). Identical to `../morning-final/morning-final.hex`; flashed and verified.
- `field-2026-11-03b.json`: the calibration stored on the robot (CALSAVE, read back and verified).
  - `field-2026-11-03b.jsonl`: the raw captures behind it.
- `firmware/` and `core_robot_mega/`: the source.

The calibration lives in EEPROM, not in the firmware: flashing a hex does not change it. To put it back on a robot, run
`python3 tools/upload_calibration.py "REAL Run Prog/field-2026-11-03b.json"` with the console on the robot.

Stored targets (front, side L, side R, 45 L, 45 R; 9999 = not used):

| pose | front | side L | side R | 45 L | 45 R |
|---|---|---|---|---|---|
| N | 257 | 675 | 642 | 644 | 677 |
| turned left | 625 | 9999 | 218 | 9999 | 706 |
| left box | 53 | 9999 | 216 | 9999 | 277 |
| turned right | 567 | 240 | 9999 | 596 | 9999 |
| right box | 47 | 203 | 9999 | 223 | 9999 |

- The left box's 45 L had no echo in 11 of 16 samples, so it is dropped (as the start-facing sides always are).
- Colour: green 857 (2 Nov green-pad capture; no green capture was taken on 3 Nov). Red 1558: the mean of the two 3 Nov N captures, 1.533 and 1.582 (was 1737). The red/green split is now 1.16; the bare floor reads about 1.07.
