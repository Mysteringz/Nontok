"""Dead reckoning: gyro heading + commanded velocity x a calibrated scale (no wheel encoders).

The map frame is fixed by the dock: every time the robot docks (S0) the pose is reset to
the dock's pose, which removes the drift built up during the last trip.
"""
import math
from typing import Tuple

from .geometry import Pose, wrap180


class Localizer:
    def __init__(self, speed_scale: float = 1.0):
        self.speed_scale = speed_scale  # actual speed / commanded speed
        self.pose = Pose()
        self._heading_offset = 0.0      # map heading - gyro heading

    def reset(self, pose: Pose, gyro_heading_deg: float):
        self.pose = Pose(pose.x_mm, pose.y_mm, pose.heading_deg)
        self._heading_offset = pose.heading_deg - gyro_heading_deg

    def map_heading(self, gyro_heading_deg: float) -> float:
        return wrap180(gyro_heading_deg + self._heading_offset)

    def update(self, commanded: Tuple[float, float, float], gyro_heading_deg: float, dt: float):
        vx, vy, _ = commanded
        self.pose.heading_deg = self.map_heading(gyro_heading_deg)
        h = math.radians(self.pose.heading_deg)
        vx *= self.speed_scale
        vy *= self.speed_scale
        self.pose.x_mm += (vx * math.cos(h) - vy * math.sin(h)) * dt
        self.pose.y_mm += (vx * math.sin(h) + vy * math.cos(h)) * dt
