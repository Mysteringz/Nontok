"""Route following (S2 NAVIGATE) and obstacle avoidance (S3 AVOID OBSTACLE).

A route is a list of legs from lab_map.yaml. For each leg:
  TURN    closed-loop gyro turn to the leg heading (done on the Mega)
  DRIVE   drive forward; the leg ends by dead-reckoned distance, or by the front
          ultrasonic for a final approach (stop_at_front_mm)
  REJOIN  strafe back by however far obstacle avoidance moved the robot sideways
  SETTLE  wait for fresh ultrasonic readings
  CHECK   compare wall distances with the map: small error -> CORRECT by strafing,
          large error -> the robot is lost (T15)
"""
import logging
from enum import Enum, auto
from typing import List

from .geometry import wrap180
from .lab_map import Leg
from .move import Move

log = logging.getLogger(__name__)


class NavStatus(Enum):
    RUNNING = auto()
    ARRIVED = auto()
    OBSTACLE = auto()           # T9: something in front that the map does not explain
    CLEARED = auto()            # T10: avoid_update() got around it
    WALL_CHECK_FAILED = auto()  # T15
    STUCK = auto()              # T16, or a turn that never finished


class _Phase(Enum):
    TURN = auto()
    DRIVE = auto()
    REJOIN = auto()
    SETTLE = auto()
    CORRECT = auto()
    DONE = auto()


class Navigator:
    def __init__(self, link, localizer, cfg: dict):
        m = cfg["motion"]
        self.link = link
        self.localizer = localizer
        self.cruise = m["cruise_speed"]
        self.approach = m["approach_speed"]
        self.strafe = m["strafe_speed"]
        self.slow_zone = m["slow_zone_mm"]
        self.turn_timeout = m["turn_timeout_s"]
        self.safety = cfg["safety"]
        self._legs: List[Leg] = []
        self._index = 0
        self._phase = _Phase.DONE
        self._progress = 0.0  # mm driven along the current leg
        self._lateral = 0.0   # mm strafed sideways (+ left) to get around obstacles
        self._timer = 0.0
        self._turn_sent = False
        self._move = None
        self._avoid_dir = 1.0

    @property
    def scale(self) -> float:
        return self.localizer.speed_scale

    @property
    def leg(self) -> Leg:
        return self._legs[self._index]

    def start(self, legs: List[Leg]):
        self._legs = list(legs)
        self._index = -1
        self._next_leg()

    # ---------------- S2 NAVIGATE ----------------
    def update(self, tel, dt: float) -> NavStatus:
        self._timer += dt
        if self._phase is _Phase.TURN:
            return self._turn(tel)
        if self._phase is _Phase.DRIVE:
            return self._drive(tel, dt)
        if self._phase in (_Phase.REJOIN, _Phase.CORRECT):
            if self._move.update(dt):
                self._move = None
                if self._phase is _Phase.REJOIN:
                    self._lateral = 0.0
                    self._set_phase(_Phase.SETTLE)
                else:
                    self._next_leg()
            return NavStatus.RUNNING
        if self._phase is _Phase.SETTLE:
            if self._timer >= self.safety["check_settle_s"]:
                return self._check_walls(tel)
            return NavStatus.RUNNING
        return NavStatus.ARRIVED

    def _next_leg(self):
        self._index += 1
        self._progress = 0.0
        self._lateral = 0.0
        self._turn_sent = False
        self._move = None
        self._set_phase(_Phase.TURN if self._index < len(self._legs) else _Phase.DONE)

    def _set_phase(self, phase: _Phase):
        self._phase = phase
        self._timer = 0.0

    def _turn(self, tel) -> NavStatus:
        if not self._turn_sent:
            delta = wrap180(self.leg.heading_deg - self.localizer.map_heading(tel.heading_deg))
            if abs(delta) < 3.0:
                self._set_phase(_Phase.DRIVE)
                return NavStatus.RUNNING
            self.link.turn(delta)
            self._turn_sent = True
            self._timer = 0.0
            return NavStatus.RUNNING
        result = self.link.turn_result()
        if result is None:
            if self._timer > self.turn_timeout:
                self.link.stop()
                return NavStatus.STUCK
            return NavStatus.RUNNING
        if not result:
            return NavStatus.STUCK
        self._set_phase(_Phase.DRIVE)
        return NavStatus.RUNNING

    def _drive(self, tel, dt: float) -> NavStatus:
        s = self.safety
        leg = self.leg
        front = tel.front_mm
        self._progress += self.link.commanded[0] * self.scale * dt
        remaining = leg.distance_mm - self._progress

        if leg.stop_at_front_mm is not None:
            if front <= leg.stop_at_front_mm:
                return self._end_drive()
            if remaining < -s["final_approach_overrun_mm"]:
                self.link.stop()
                log.warning("Drove %.0f mm past the expected wall", -remaining)
                return NavStatus.WALL_CHECK_FAILED
            # The wall itself gets close at the end of the leg: only something much
            # closer than where the wall should be counts as an obstacle.
            expected_front = max(remaining, 0.0) + leg.stop_at_front_mm
            if front < s["obstacle_stop_mm"] and front < expected_front - s["obstacle_margin_mm"]:
                self.link.stop()
                return NavStatus.OBSTACLE
            slow = front < leg.stop_at_front_mm + self.slow_zone
        else:
            if remaining <= 0:
                return self._end_drive()
            if front < s["obstacle_stop_mm"]:
                self.link.stop()
                return NavStatus.OBSTACLE
            slow = remaining < self.slow_zone

        self.link.vel(self.approach if slow else self.cruise, 0, 0)
        return NavStatus.RUNNING

    def _end_drive(self) -> NavStatus:
        self.link.vel(0, 0, 0)
        if abs(self._lateral) > 1.0:
            self._move = Move(self.link, dy_mm=-self._lateral, speed=self.strafe, speed_scale=self.scale)
            self._set_phase(_Phase.REJOIN)
        else:
            self._set_phase(_Phase.SETTLE)
        return NavStatus.RUNNING

    def _check_walls(self, tel) -> NavStatus:
        s = self.safety
        readings = {"front": tel.front_mm, "left": tel.left_mm, "right": tel.right_mm}
        dx, dys = 0.0, []
        for side, expected in self.leg.expect.items():
            err = readings[side] - expected
            if abs(err) > s["wall_tolerance_mm"]:
                log.warning("Wall check failed: %s %d mm, expected %d", side, readings[side], expected)
                return NavStatus.WALL_CHECK_FAILED
            if side == "front":
                dx = err
            elif side == "left":
                dys.append(err)   # too far from the left wall -> move left
            else:
                dys.append(-err)  # too far from the right wall -> move right
        dy = sum(dys) / len(dys) if dys else 0.0
        if max(abs(dx), abs(dy)) >= s["min_correction_mm"]:
            self._move = Move(self.link, dx, dy, speed=self.approach, speed_scale=self.scale)
            self._set_phase(_Phase.CORRECT)
        else:
            self._next_leg()
        return NavStatus.RUNNING

    # ---------------- S3 AVOID OBSTACLE ----------------
    def begin_avoid(self, tel):
        """Strafe towards the side with more room. The leg resumes afterwards and the
        sideways offset is undone at the end of the leg (REJOIN), past the obstacle."""
        self.link.stop()
        self._avoid_dir = 1.0 if tel.left_mm >= tel.right_mm else -1.0
        self._move = None
        log.info("Obstacle at %d mm, strafing %s", tel.front_mm, "left" if self._avoid_dir > 0 else "right")

    def avoid_update(self, tel, dt: float) -> NavStatus:
        s = self.safety
        self._lateral += self.link.commanded[1] * self.scale * dt
        side_mm = tel.left_mm if self._avoid_dir > 0 else tel.right_mm
        if abs(self._lateral) > s["max_strafe_mm"] or side_mm < s["side_min_mm"]:
            self.link.stop()
            return NavStatus.STUCK
        if self._move is None:
            if tel.front_mm < s["obstacle_clear_mm"]:
                self.link.vel(0, self._avoid_dir * self.strafe, 0)
                return NavStatus.RUNNING
            # Front is clear: keep going a bit so the robot's corner clears too.
            self._move = Move(self.link, dy_mm=self._avoid_dir * s["avoid_extra_mm"],
                              speed=self.strafe, speed_scale=self.scale)
        if self._move.update(dt):
            self._move = None
            return NavStatus.CLEARED
        return NavStatus.RUNNING
