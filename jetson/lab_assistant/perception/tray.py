"""Is anything on the tray? Used with the camera tilted down (S5 load, S7 unload)."""
from typing import Optional, Sequence


class TrayMonitor:
    def __init__(self, camera, detector, roi: Sequence[float] = (0, 0, 1, 1), min_conf: float = 0.4):
        self.camera = camera
        self.detector = detector
        self.roi = roi  # part of the image that shows the tray: [x1, y1, x2, y2] as fractions
        self.min_conf = min_conf

    def has_item(self) -> Optional[bool]:
        """True/False, or None when there is no camera frame yet.

        Only sees the classes YOLO was trained on. TODO (optional): compare against
        a photo of the empty tray to catch unknown objects too.
        """
        frame = self.camera.read()
        if frame is None:
            return None
        h, w = frame.shape[:2]
        x1, y1, x2, y2 = self.roi[0] * w, self.roi[1] * h, self.roi[2] * w, self.roi[3] * h
        for det in self.detector.detect(frame):
            cx, cy = det.center
            if det.conf >= self.min_conf and x1 <= cx <= x2 and y1 <= cy <= y2:
                return True
        return False
