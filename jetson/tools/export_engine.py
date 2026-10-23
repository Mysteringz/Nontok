"""Build the TensorRT engine for YOLO. Run this ON THE JETSON: engines only work on
the device (and TensorRT version) they were built on.

    python -m tools.export_engine models/lab_equipment.pt --imgsz 416
"""
import argparse


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("weights", help="best.pt from training/train.py")
    parser.add_argument("--imgsz", type=int, default=416)
    args = parser.parse_args()

    from ultralytics import YOLO

    path = YOLO(args.weights).export(format="engine", half=True, imgsz=args.imgsz, device=0)
    print(f"Engine written to {path}; point detector.weights in config/robot.yaml at it")


if __name__ == "__main__":
    main()
