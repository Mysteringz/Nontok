# Lab assistant serial protocol (Jetson ↔ Mega)

This protocol is used by the lab-assistant sketch, [firmware/lab_assistant_mega](../firmware/lab_assistant_mega/lab_assistant_mega.ino), and by [jetson/lab_assistant/hardware/protocol.py](../jetson/lab_assistant/hardware/protocol.py).

The Mega-only PlatformIO core speaks MEGA2, documented in [serial_protocol.md](serial_protocol.md). The two protocols are not compatible; Jetson control belongs only to this proposed application.

The link is USB serial at 115200 baud, with ASCII lines ending in `\n` and fields separated by spaces.

Units are mm, mm/s, degrees and deg/s. Heading is the raw gyro heading: counter-clockwise is positive and it is never wrapped. The Jetson converts it to the map frame.

## Jetson → Mega

| Command | Reply | Meaning |
|---|---|---|
| `VEL <vx> <vy> <wz>` | none | Body velocity: forward, left, counter-clockwise. **While moving, resend at least every 500 ms**, otherwise the watchdog stops the motors. While `wz` is 0, the Mega uses the gyro to hold the current heading. |
| `TURN <deg>` | `OK TURN`, then `DONE TURN` or `ERR TURN_TIMEOUT` | Closed-loop relative turn. `VEL` is ignored until the turn finishes. Afterwards the Mega holds the new heading. |
| `HOLD <deg>` | `OK HOLD` | Set the absolute gyro heading to hold. |
| `STOP` | `OK STOP` | Stop everything, including a turn. |
| `TILT <deg>` | `OK TILT` | Camera servo target. The servo moves 1° every 15 ms. |
| `ZERO` | `OK ZERO` | Reset the gyro heading to 0. Bench use only. |
| `CAL` | `OK CAL` | Re-measure the gyro bias. Takes ~1 s, and the robot must be still. |
| `PING` | `PONG` | Link check. |

## Mega → Jetson

| Line | When |
|---|---|
| `READY` | After a reset. Opening the port resets the Mega. |
| `TEL <t_ms> <heading_deg> <front_mm> <left_mm> <right_mm> <charge_mv> <turning> <tilt_deg>` | Every 50 ms. |
| `DONE TURN` | A `TURN` finished. |
| `ERR <reason>` | `UNKNOWN <cmd>`, `OVERFLOW`, `TURN_TIMEOUT`, `WATCHDOG`, `IMU_NOT_FOUND`. |

Notes:
- An ultrasonic sensor reads `4000` when nothing echoes back.
- `tilt_deg` is how far the servo has been stepped so far. The Jetson waits for it to reach the target before using camera frames.
- `charge_mv` is the receiver coil's output. A reading above `docking.charge_mv_min` means the robot is on the coil.
- Unlike MEGA2, there is no `ARM` step or speed cap in this legacy proposed-application protocol. If the two protocols are merged later, retain explicit arming and bounded motion (see the README).
