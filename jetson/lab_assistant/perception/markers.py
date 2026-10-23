"""ArUco markers on the dock / storage / workstation (print them with tools/make_markers.py)."""
from dataclasses import dataclass
from typing import Dict


@dataclass
class MarkerObs:
    marker_id: int
    x_norm: float    # -1 (left edge of image) .. +1 (right edge)
    y_norm: float    # -1 (top) .. +1 (bottom)
    width_px: float  # grows as the robot gets closer


class MarkerDetector:
    def __init__(self, dictionary: str = "DICT_4X4_50"):
        import cv2

        aruco = cv2.aruco
        self._dict = aruco.getPredefinedDictionary(getattr(aruco, dictionary))
        if hasattr(aruco, "ArucoDetector"):  # OpenCV >= 4.7
            self._detector = aruco.ArucoDetector(self._dict, aruco.DetectorParameters())
        else:  # OpenCV 4.1 that ships with JetPack 4
            self._detector = None
            self._params = aruco.DetectorParameters_create()

    def detect(self, frame) -> Dict[int, MarkerObs]:
        import cv2
        import numpy as np

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self._detector is not None:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self._dict, parameters=self._params)
        found = {}
        if ids is None:
            return found
        h, w = gray.shape
        for c, marker_id in zip(corners, ids.flatten()):
            pts = c.reshape(4, 2)
            cx, cy = pts.mean(axis=0)
            found[int(marker_id)] = MarkerObs(
                marker_id=int(marker_id),
                x_norm=float((cx - w / 2) / (w / 2)),
                y_norm=float((cy - h / 2) / (h / 2)),
                width_px=float(np.linalg.norm(pts[0] - pts[1])),
            )
        return found
