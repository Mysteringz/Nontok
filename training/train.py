"""Train the equipment detector on a PC or Colab with a GPU (not on the Nano).

    pip install ultralytics
    python training/split_dataset.py data/labelled data/lab_equipment
    python training/train.py --epochs 100

Then copy runs/<name>/weights/best.pt to jetson/models/lab_equipment.pt and build the
TensorRT engine on the Jetson:  python -m tools.export_engine models/lab_equipment.pt
"""
import argparse
import tempfile
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", type=Path, default=HERE / "dataset.yaml")
    parser.add_argument("--model", default="yolo11n.pt", help="small pretrained model: the Nano is slow")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=416)  # must match detector.imgsz on the robot
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--name", default="lab_equipment")
    args = parser.parse_args()

    from ultralytics import YOLO

    # Ultralytics resolves a relative `path` against its own datasets folder, not this repo.
    data = yaml.safe_load(args.data.read_text())
    data["path"] = str((ROOT / data["path"]).resolve())
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
        yaml.safe_dump(data, f)

    model = YOLO(args.model)
    model.train(data=f.name, epochs=args.epochs, imgsz=args.imgsz, batch=args.batch,
                project=str(ROOT / "runs"), name=args.name)
    metrics = model.val()
    print(f"mAP50 = {metrics.box.map50:.3f}, mAP50-95 = {metrics.box.map:.3f}")


if __name__ == "__main__":
    main()
