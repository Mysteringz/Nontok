"""Module 4: YOLO detector for lab equipment, trained on our own photos (see training/)."""
from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass
class Detection:
    label: str
    conf: float
    box: Tuple[float, float, float, float]  # x1, y1, x2, y2 in pixels

    @property
    def center(self) -> Tuple[float, float]:
        x1, y1, x2, y2 = self.box
        return (x1 + x2) / 2, (y1 + y2) / 2


class EquipmentDetector:
    """Ultralytics YOLO. Use the .pt weights on a PC, and the TensorRT .engine on the Jetson
    (built with tools/export_engine.py on the Jetson itself)."""

    def __init__(self, weights: str, conf: float = 0.5, imgsz: int = 416):
        from ultralytics import YOLO

        self.conf = conf
        self.imgsz = imgsz
        self._model = YOLO(weights, task="detect")

    def detect(self, frame) -> List[Detection]:
        result = self._model.predict(frame, conf=self.conf, imgsz=self.imgsz, verbose=False)[0]
        detections = []
        for box in result.boxes:
            label = result.names[int(box.cls[0])]
            xyxy = tuple(float(v) for v in box.xyxy[0].tolist())
            detections.append(Detection(label, float(box.conf[0]), xyxy))
        return detections

    def find(self, frame, label: str) -> Optional[Detection]:
        hits = [d for d in self.detect(frame) if d.label == label]
        return max(hits, key=lambda d: d.conf) if hits else None
