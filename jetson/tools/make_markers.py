"""Print-ready ArUco markers for the stations (ids in config/lab_map.yaml).

    python -m tools.make_markers --ids 0 1 2
"""
import argparse
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ids", type=int, nargs="+", default=[0, 1, 2])
    parser.add_argument("--dictionary", default="DICT_4X4_50")
    parser.add_argument("--px", type=int, default=600)
    parser.add_argument("--out", default="markers")
    args = parser.parse_args()

    import cv2

    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(getattr(aruco, args.dictionary))
    out = Path(args.out)
    out.mkdir(exist_ok=True)
    for marker_id in args.ids:
        if hasattr(aruco, "generateImageMarker"):  # OpenCV >= 4.7
            img = aruco.generateImageMarker(dictionary, marker_id, args.px)
        else:
            img = aruco.drawMarker(dictionary, marker_id, args.px)
        border = args.px // 8  # white quiet zone, needed for detection
        img = cv2.copyMakeBorder(img, border, border, border, border, cv2.BORDER_CONSTANT, value=255)
        path = out / f"marker_{marker_id}.png"
        cv2.imwrite(str(path), img)
        print(f"wrote {path}")


if __name__ == "__main__":
    main()
