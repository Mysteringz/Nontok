"""S4 SEARCH: strafe along the shelf and tilt the camera until YOLO sees the item.

Always strafes back to where it started before finishing, so the route out of
storage starts from the right place.
"""
from enum import Enum, auto
from typing import List, Optional

from ..navigation.move import Move
from ..perception.detector import Detection


class SearchStatus(Enum):
    RUNNING = auto()
    FOUND = auto()
    NOT_FOUND = auto()


class ShelfSearch:
    def __init__(self, link, camera, detector, item: str, offsets_mm: List[float],
                 tilts_deg: List[int], cfg: dict):
        self.link = link
        self.camera = camera
        self.detector = detector
        self.item = item
        self._plan = [(o, t) for o in offsets_mm for t in tilts_deg]
        self._settle_s = cfg["search"]["settle_s"]
        self._frames_per_view = cfg["search"]["frames_per_view"]
        self._speed = cfg["motion"]["strafe_speed"]
        self._scale = cfg["motion"]["speed_scale"]
        self._offset = 0.0  # mm left of where the search started
        self._tilt = None
        self._phase = "next"
        self._timer = 0.0
        self._looks = 0
        self._move: Optional[Move] = None
        self.found: Optional[Detection] = None

    def cancel(self):
        """Stop searching; the robot still strafes back to the start."""
        self._plan.clear()

    def update(self, tel, dt: float) -> SearchStatus:
        self._timer += dt
        if self._phase == "next":
            if self.found is None and self._plan:
                offset, self._tilt = self._plan.pop(0)
                self.link.tilt(self._tilt)
                self._start_move(offset, "move")
            else:
                self._start_move(0.0, "return")
        elif self._phase == "move":
            if self._move.update(dt):
                self._set_phase("settle")
        elif self._phase == "settle":
            if self._timer >= self._settle_s and abs(tel.tilt_deg - self._tilt) <= 1:
                self._looks = 0
                self._set_phase("look")
        elif self._phase == "look":
            frame = self.camera.read()
            if frame is not None:
                self.found = self.detector.find(frame, self.item)
                self._looks += 1
                if self.found is not None or self._looks >= self._frames_per_view:
                    self._set_phase("next")
        elif self._phase == "return":
            if self._move.update(dt):
                return SearchStatus.FOUND if self.found else SearchStatus.NOT_FOUND
        return SearchStatus.RUNNING

    def _start_move(self, offset: float, phase: str):
        self._move = Move(self.link, dy_mm=offset - self._offset, speed=self._speed, speed_scale=self._scale)
        self._offset = offset
        self._set_phase(phase)

    def _set_phase(self, phase: str):
        self._phase = phase
        self._timer = 0.0
