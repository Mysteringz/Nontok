"""Open-loop body-frame move by a fixed distance (undock, strafes, small corrections)."""
import math


class Move:
    """Drives (dx forward, dy left) mm at `speed`, timed from the calibrated speed.

    Call update(dt) every tick; it returns True once the move is finished and the robot
    has been told to stop. The Mega holds the heading while it moves.
    """

    def __init__(self, link, dx_mm: float = 0.0, dy_mm: float = 0.0, speed: float = 100.0,
                 speed_scale: float = 1.0):
        self.link = link
        dist = math.hypot(dx_mm, dy_mm)
        self._remaining = dist
        self._actual_speed = speed * speed_scale
        self._vx = speed * dx_mm / dist if dist else 0.0
        self._vy = speed * dy_mm / dist if dist else 0.0
        self._started = False

    @property
    def done(self) -> bool:
        return self._remaining <= 0

    def update(self, dt: float) -> bool:
        if self.done:
            return True
        if self._started:
            self._remaining -= self._actual_speed * dt
            if self.done:
                self.link.vel(0, 0, 0)
                return True
        self._started = True
        self.link.vel(self._vx, self._vy, 0)
        return False
