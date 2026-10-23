"""USB camera on the tilt servo. A thread keeps grabbing so read() always returns a fresh frame."""
import threading
import time


class Camera:
    def __init__(self, device=0, width: int = 640, height: int = 480):
        self.device = device
        self.width = width
        self.height = height
        self._cap = None
        self._frame = None
        self._frame_time = 0.0
        self._running = False
        self._lock = threading.Lock()

    def start(self):
        import cv2

        self._cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if not self._cap.isOpened():
            raise RuntimeError(f"Cannot open camera {self.device}")
        self._running = True
        threading.Thread(target=self._grab, name="camera", daemon=True).start()
        return self

    def read(self):
        """Latest BGR frame (numpy array), or None before the first frame."""
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def frame_age(self) -> float:
        return time.monotonic() - self._frame_time

    def stop(self):
        self._running = False
        if self._cap is not None:
            self._cap.release()

    def _grab(self):
        while self._running:
            ok, frame = self._cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            with self._lock:
                self._frame = frame
                self._frame_time = time.monotonic()
