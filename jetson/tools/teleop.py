"""Drive the robot from the keyboard and watch the sensors (week 1-2 basic car demo).

    python -m tools.teleop

  w / s   forward / back        a / d   strafe left / right
  q / e   rotate left / right   space   stop
  t / g   tilt camera up / down x       quit
"""
import argparse
import select
import sys
import termios
import time
import tty

from lab_assistant.config import load_yaml
from lab_assistant.hardware.serial_link import RobotLink

KEYS = {"w": (1, 0, 0), "s": (-1, 0, 0), "a": (0, 1, 0), "d": (0, -1, 0), "q": (0, 0, 1), "e": (0, 0, -1)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config/robot.yaml")
    parser.add_argument("--speed", type=float, default=150, help="mm/s")
    parser.add_argument("--turn-rate", type=float, default=45, help="deg/s")
    args = parser.parse_args()

    cfg = load_yaml(args.config)
    link = RobotLink(**cfg["serial"])
    link.open()
    tilt = cfg["tilt"]["scan_deg"]
    link.tilt(tilt)
    vel = (0.0, 0.0, 0.0)
    print(__doc__)

    old_term = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        last_print = 0.0
        while True:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1).lower()
                if key == "x":
                    break
                if key == " ":
                    vel = (0.0, 0.0, 0.0)
                elif key in KEYS:
                    fx, fy, fw = KEYS[key]
                    vel = (fx * args.speed, fy * args.speed, fw * args.turn_rate)
                elif key in "tg":
                    tilt = max(0, min(180, tilt + (5 if key == "t" else -5)))
                    link.tilt(tilt)
            link.vel(*vel)  # resent every 0.1 s to keep the Mega's watchdog happy
            tel = link.telemetry
            if tel and time.monotonic() - last_print > 0.3:
                last_print = time.monotonic()
                print(f"\rheading {tel.heading_deg:7.1f}  front {tel.front_mm:5d}  left {tel.left_mm:5d}  "
                      f"right {tel.right_mm:5d}  charge {tel.charge_mv:5d} mV  tilt {tel.tilt_deg:3d}   ",
                      end="", flush=True)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term)
        link.stop()
        link.close()
        print()


if __name__ == "__main__":
    main()
