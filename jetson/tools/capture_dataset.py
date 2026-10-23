"""Take photos for YOLO training (module 4, week 2).

    python -m tools.capture_dataset --label multimeter [--tilt 75]

SPACE saves a frame, a toggles auto-capture every --interval s, q quits.
Photos go to data/raw/<label>/. Vary distance, angle, lighting and background, and take
photos with the camera tilted down at the tray as well, since S5/S7 check the tray.
"""
import argparse
import time
from pathlib import Path

from lab_assistant.config import load_yaml
from lab_assistant.hardware.camera import Camera


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", required=True)
    parser.add_argument("--out", default="data/raw")
    parser.add_argument("--config", default="config/robot.yaml")
    parser.add_argument("--tilt", type=int, help="set the camera servo (needs the Mega connected)")
    parser.add_argument("--interval", type=float, default=1.0)
    args = parser.parse_args()

    import cv2

    cfg = load_yaml(args.config)
    if args.tilt is not None:
        from lab_assistant.hardware.serial_link import RobotLink

        link = RobotLink(**cfg["serial"])
        link.open()
        link.tilt(args.tilt)

    out = Path(args.out) / args.label
    out.mkdir(parents=True, exist_ok=True)
    camera = Camera(**cfg["camera"]).start()
    auto, last_auto, count = False, 0.0, 0
    try:
        while True:
            frame = camera.read()
            if frame is None:
                time.sleep(0.05)
                continue
            cv2.imshow("capture (SPACE save, a auto, q quit)", frame)
            key = cv2.waitKey(30) & 0xFF
            if key == ord("q"):
                break
            if key == ord("a"):
                auto = not auto
            now = time.monotonic()
            if key == ord(" ") or (auto and now - last_auto >= args.interval):
                last_auto = now
                path = out / f"{args.label}_{int(time.time() * 1000)}.jpg"
                cv2.imwrite(str(path), frame)
                count += 1
                print(f"saved {path} ({count})")
    finally:
        camera.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
