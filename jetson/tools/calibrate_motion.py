"""Measure motion.speed_scale (actual speed / commanded speed) with the ultrasonic sensors.

Forward:  face a wall 1.5-2 m away         python -m tools.calibrate_motion
Strafe:   put a wall 1.5-2 m to the right  python -m tools.calibrate_motion --strafe

Mecanum wheels usually strafe slower than they drive forward. If the two scales differ
by more than ~10 %, give strafing its own scale in Move / Localizer.
"""
import argparse
import statistics
import time

from lab_assistant.config import load_yaml
from lab_assistant.hardware.serial_link import RobotLink


def read_mm(link, side: str, seconds: float = 0.5) -> float:
    samples = []
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        tel = link.telemetry
        if tel:
            samples.append(getattr(tel, f"{side}_mm"))
        time.sleep(0.05)
    return statistics.median(samples)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config/robot.yaml")
    parser.add_argument("--speed", type=float, default=150)
    parser.add_argument("--seconds", type=float, default=4.0)
    parser.add_argument("--strafe", action="store_true", help="strafe right towards a wall")
    args = parser.parse_args()

    cfg = load_yaml(args.config)
    link = RobotLink(**cfg["serial"])
    link.open()
    time.sleep(1.0)
    side = "right" if args.strafe else "front"
    try:
        before = read_mm(link, side)
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            if args.strafe:
                link.vel(0, -args.speed, 0)
            else:
                link.vel(args.speed, 0, 0)
            time.sleep(0.05)
        link.stop()
        time.sleep(0.5)
        after = read_mm(link, side)
    finally:
        link.stop()
        link.close()

    actual = before - after
    commanded = args.speed * args.seconds
    print(f"{side} wall: {before:.0f} mm -> {after:.0f} mm, moved {actual:.0f} mm, commanded {commanded:.0f} mm")
    print(f"speed_scale = {actual / commanded:.3f}")


if __name__ == "__main__":
    main()
