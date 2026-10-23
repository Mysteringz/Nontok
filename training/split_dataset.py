"""Split a labelled YOLO export into train/val folders.

    python training/split_dataset.py data/labelled data/lab_equipment --val 0.2

Input: <src>/images/*.jpg and <src>/labels/*.txt (YOLO format, as exported by
Label Studio / Roboflow / CVAT). Images without a label file are kept as background.
"""
import argparse
import random
import shutil
from pathlib import Path

IMAGE_EXTS = {".jpg", ".jpeg", ".png"}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("src", type=Path)
    parser.add_argument("dst", type=Path)
    parser.add_argument("--val", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    images = sorted(p for p in (args.src / "images").iterdir() if p.suffix.lower() in IMAGE_EXTS)
    random.Random(args.seed).shuffle(images)
    n_val = int(len(images) * args.val)
    splits = {"val": images[:n_val], "train": images[n_val:]}

    for split, files in splits.items():
        (args.dst / "images" / split).mkdir(parents=True, exist_ok=True)
        (args.dst / "labels" / split).mkdir(parents=True, exist_ok=True)
        for img in files:
            shutil.copy2(img, args.dst / "images" / split / img.name)
            label = args.src / "labels" / f"{img.stem}.txt"
            if label.exists():
                shutil.copy2(label, args.dst / "labels" / split / label.name)
        print(f"{split}: {len(files)} images")


if __name__ == "__main__":
    main()
