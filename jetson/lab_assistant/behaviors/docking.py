"""S8 DOCK ALIGN (module 3): line up with the dock marker, creep to the wall, confirm charging.

  CENTER   strafe until the dock's ArUco marker is centred in the image
  FORWARD  creep until the front ultrasonic reads the dock's wall distance
  CONFIRM  wait for the receiver coil voltage
  SWEEP    not charging: strafe +-step, +-2 step, ... looking for the coil
Charging at any point means the robot is on the coil.
"""
from enum import Enum, auto
from typing import Optional

from ..navigation.move import Move


class DockStatus(Enum):
    RUNNING = auto()
    DOCKED = auto()
    FAILED = auto()


class DockAligner:
    def __init__(self, link, camera, markers, dock, cfg: dict):
        d = cfg["docking"]
        self.link = link
        self.camera = camera
        self.markers = markers
        self.marker_id = dock.marker_id
        self.wall_mm = dock.wall_mm
        self.tilt_deg = cfg["tilt"]["marker_deg"]
        self.d = d
        self._strafe_speed = cfg["motion"]["strafe_speed"]
        self._scale = cfg["motion"]["speed_scale"]
        step = d["sweep_step_mm"]
        # Relative strafes that visit +s, -s, +2s, -2s, ...
        self._sweep = [step * (i + 1) * (1 if i % 2 == 0 else -1) for i in range(2 * d["sweep_steps"])]
        self._phase = "center"
        self._timer = 0.0
        self._no_marker_s = 0.0
        self._charging_s = 0.0
        self._move: Optional[Move] = None

    def start(self):
        self.link.tilt(self.tilt_deg)

    def update(self, tel, dt: float) -> DockStatus:
        d = self.d
        self._timer += dt
        self._charging_s = self._charging_s + dt if tel.charge_mv >= d["charge_mv_min"] else 0.0
        if self._charging_s >= d["confirm_s"]:
            self.link.stop()
            return DockStatus.DOCKED

        if self._phase == "center":
            self._center(tel, dt)
        elif self._phase == "forward":
            if tel.front_mm <= self.wall_mm:
                self.link.vel(0, 0, 0)
                self._set_phase("confirm")
            elif self._timer > d["forward_timeout_s"]:
                self.link.stop()
                return DockStatus.FAILED
            else:
                self.link.vel(d["creep_speed"], 0, 0)
        elif self._phase == "confirm":
            if self._timer >= d["confirm_s"] * 1.5:
                self._set_phase("sweep")
        elif self._phase == "sweep":
            if self._move is None:
                if not self._sweep:
                    self.link.stop()
                    return DockStatus.FAILED
                self._move = Move(self.link, dy_mm=self._sweep.pop(0), speed=d["creep_speed"],
                                  speed_scale=self._scale)
            if self._move.update(dt):
                self._move = None
                self._set_phase("confirm")
        return DockStatus.RUNNING

    def _center(self, tel, dt: float):
        d = self.d
        if abs(tel.tilt_deg - self.tilt_deg) > 1:
            return  # camera still moving
        frame = self.camera.read()
        obs = self.markers.detect(frame).get(self.marker_id) if frame is not None else None
        if obs is None:
            self.link.vel(0, 0, 0)
            self._no_marker_s += dt
            if self._no_marker_s > d["marker_timeout_s"]:
                self._set_phase("forward")  # go in blind, the sweep will find the coil
            return
        self._no_marker_s = 0.0
        err = obs.x_norm - d["marker_center_offset"]
        if abs(err) <= d["center_tolerance"]:
            self.link.vel(0, 0, 0)
            self._set_phase("forward")
            return
        # Marker right of centre (err > 0) -> robot must move right (negative vy).
        vy = max(-self._strafe_speed, min(self._strafe_speed, -d["center_gain"] * err))
        self.link.vel(0, vy, 0)

    def _set_phase(self, phase: str):
        self._phase = phase
        self._timer = 0.0
