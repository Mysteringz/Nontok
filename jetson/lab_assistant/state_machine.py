"""Top-level behaviour: the S0-S9 state diagram from the proposal.

tick() runs the current state's handler once. Handlers never block: they send commands,
read telemetry/perception and return the next state (None = stay). Transition labels
T1..T16 match docs/state_machine.md.
"""
import logging
from dataclasses import dataclass
from enum import Enum, auto
from typing import Any, Optional

from .behaviors.docking import DockAligner, DockStatus
from .behaviors.search import SearchStatus, ShelfSearch
from .navigation.move import Move
from .navigation.navigator import Navigator, NavStatus
from .perception.tray import TrayMonitor
from .voice.commands import Command, Intent

log = logging.getLogger(__name__)


class State(Enum):
    DOCKED = "S0 DOCKED / CHARGING"
    UNDOCK = "S1 UNDOCK"
    NAVIGATE = "S2 NAVIGATE"
    AVOID_OBSTACLE = "S3 AVOID OBSTACLE"
    SEARCH = "S4 SEARCH"
    NOTIFY_AND_LOAD = "S5 NOTIFY & LOAD"
    WAIT_AT_WORKSTATION = "S6 WAIT AT WORKSTATION"
    WAIT_FOR_UNLOAD = "S7 WAIT FOR UNLOAD"
    DOCK_ALIGN = "S8 DOCK ALIGN"
    LOST = "S9 LOST / FAULT"


MOVING_STATES = {State.UNDOCK, State.NAVIGATE, State.AVOID_OBSTACLE, State.SEARCH, State.DOCK_ALIGN}


class Task(Enum):
    NONE = auto()
    FIND = auto()     # go to storage and search for mission.item
    DELIVER = auto()  # carry the found item to the workstation
    RETURN = auto()   # carry an item from the workstation back to storage
    HOME = auto()     # go back to the dock


@dataclass
class Mission:
    task: Task = Task.NONE
    item: Optional[str] = None
    location: str = "dock"  # station the robot last arrived at = start of the next route
    target: str = "dock"    # station it is heading to


@dataclass
class Robot:
    """Everything the state machine talks to. Tests pass fakes."""
    link: Any        # hardware.serial_link.RobotLink
    camera: Any      # hardware.camera.Camera
    detector: Any    # perception.detector.EquipmentDetector
    markers: Any     # perception.markers.MarkerDetector
    speaker: Any     # voice.speaker.Speaker
    localizer: Any   # navigation.localizer.Localizer
    lab_map: Any     # navigation.lab_map.LabMap
    cfg: dict


class StateMachine:
    def __init__(self, robot: Robot, navigator: Optional[Navigator] = None):
        self.r = robot
        self.cfg = robot.cfg
        self.nav = navigator or Navigator(robot.link, robot.localizer, robot.cfg)
        self.tray = TrayMonitor(robot.camera, robot.detector, **robot.cfg["tray"])
        self.state = State.DOCKED
        self.mission = Mission()
        self.time_in_state = 0.0
        self._entering = True
        self._behavior = None  # Move / ShelfSearch / DockAligner owned by the current state
        self._count = 0        # debounce counter for tray checks
        self._timer = 0.0
        self._lost_message = ""
        self._handlers = {
            State.DOCKED: self._docked,
            State.UNDOCK: self._undock,
            State.NAVIGATE: self._navigate,
            State.AVOID_OBSTACLE: self._avoid_obstacle,
            State.SEARCH: self._search,
            State.NOTIFY_AND_LOAD: self._notify_and_load,
            State.WAIT_AT_WORKSTATION: self._wait_at_workstation,
            State.WAIT_FOR_UNLOAD: self._wait_for_unload,
            State.DOCK_ALIGN: self._dock_align,
            State.LOST: self._lost,
        }

    # ------------------------------------------------------------------ main tick
    def tick(self, dt: float, command: Optional[Command] = None):
        tel = self.r.link.telemetry
        if tel is None:
            return  # nothing from the Mega yet
        self.r.localizer.update(self.r.link.commanded, tel.heading_deg, dt)

        if self.state in MOVING_STATES and self.r.link.telemetry_age() > self.cfg["safety"]["telemetry_timeout_s"]:
            self._go_lost("I lost contact with my motor controller.")

        if command is not None:
            command = self._global_command(command)

        entering, self._entering = self._entering, False
        nxt = self._handlers[self.state](tel, dt, command, entering)
        self.time_in_state += dt
        if nxt is not None and nxt is not self.state:
            self._change(nxt)

    def _change(self, nxt: State):
        log.info("%s -> %s  (task=%s, item=%s, target=%s)", self.state.value, nxt.value,
                 self.mission.task.name, self.mission.item, self.mission.target)
        self.state = nxt
        self._entering = True
        self.time_in_state = 0.0
        self._behavior = None
        self._count = 0
        self._timer = 0.0

    def _go_lost(self, message: str):
        self.r.link.stop()
        self._lost_message = message
        self._change(State.LOST)

    def _global_command(self, cmd: Command) -> Optional[Command]:
        """Commands that work in any state. Returns the command if the state should handle it."""
        if cmd.intent is Intent.STOP:
            self._go_lost("Stopped. Put me back on the dock when you are done.")
            return None
        if cmd.intent is not Intent.GO_BACK:
            return cmd
        # "go back": target = dock
        if self.state in (State.DOCKED, State.DOCK_ALIGN, State.LOST):
            return None
        self.mission.task = Task.HOME
        self.r.speaker.say("Going back to the dock")
        if self.state is State.UNDOCK:
            self._change(State.DOCK_ALIGN)
        elif self.state is State.SEARCH and self._behavior is not None:
            self._behavior.cancel()  # strafes back to the start, then heads home
        elif self.state in (State.NOTIFY_AND_LOAD, State.WAIT_AT_WORKSTATION, State.WAIT_FOR_UNLOAD):
            self._change(self._go_to("dock"))
        # NAVIGATE / AVOID: there is no route from the middle of a route, so the robot
        # finishes this route and _arrived() sends it home.
        return None

    # ------------------------------------------------------------------ helpers
    def _go_to(self, target: str) -> State:
        self.mission.target = target
        if target == self.mission.location:
            return self._arrived()
        try:
            self.nav.start(self.r.lab_map.route(self.mission.location, target))
        except KeyError as e:
            log.error("%s", e)
            self._lost_message = "I do not know the way."
            return State.LOST
        return State.NAVIGATE

    def _arrived(self) -> State:
        here = self.mission.location = self.mission.target
        if here == "dock":
            return State.DOCK_ALIGN                                      # T7
        if self.mission.task is Task.HOME:
            return self._go_to("dock")
        if here == "storage":
            if self.mission.task is Task.FIND:
                return State.SEARCH                                      # T3
            return State.WAIT_FOR_UNLOAD                                 # T13
        return State.WAIT_AT_WORKSTATION                                 # T11

    def _tray_check_due(self, dt: float) -> bool:
        """Rate-limits YOLO on the tray (it is slow on the Nano)."""
        self._timer += dt
        if self._timer < self.cfg["load"]["check_period_s"]:
            return False
        self._timer = 0.0
        return True

    def _tilt_settled(self, tel, deg: int) -> bool:
        return abs(tel.tilt_deg - deg) <= 1

    # ------------------------------------------------------------------ states
    def _docked(self, tel, dt, cmd, entering):
        if entering:
            self.r.link.stop()
            self.r.link.tilt(self.cfg["tilt"]["scan_deg"])
            self.r.localizer.reset(self.r.lab_map.stations["dock"].pose, tel.heading_deg)
            self.mission = Mission()
        if cmd and cmd.intent is Intent.FIND:
            self.mission = Mission(task=Task.FIND, item=cmd.item, location="dock", target="storage")
            self.r.speaker.say(f"Finding the {cmd.item}")
            return State.UNDOCK                                          # T1
        return None

    def _undock(self, tel, dt, cmd, entering):
        if entering:
            m = self.cfg["motion"]
            self._behavior = Move(self.r.link, dx_mm=-m["undock_mm"], speed=m["approach_speed"],
                                  speed_scale=m["speed_scale"])
        if self._behavior.update(dt):
            return self._go_to(self.mission.target)                      # T2
        return None

    def _navigate(self, tel, dt, cmd, entering):
        status = self.nav.update(tel, dt)
        if status is NavStatus.ARRIVED:
            return self._arrived()                                       # T3 / T7 / T11 / T13
        if status is NavStatus.OBSTACLE:
            return State.AVOID_OBSTACLE                                  # T9
        if status in (NavStatus.WALL_CHECK_FAILED, NavStatus.STUCK):
            self._lost_message = "I am lost. Please put me back on the dock."
            return State.LOST                                            # T15
        return None

    def _avoid_obstacle(self, tel, dt, cmd, entering):
        if entering:
            self.nav.begin_avoid(tel)
        status = self.nav.avoid_update(tel, dt)
        if status is NavStatus.CLEARED:
            return State.NAVIGATE                                        # T10
        if status is NavStatus.STUCK or self.time_in_state > self.cfg["safety"]["stuck_timeout_s"]:
            self._lost_message = "I am stuck. Please put me back on the dock."
            return State.LOST                                            # T16
        return None

    def _search(self, tel, dt, cmd, entering):
        if entering:
            storage = self.r.lab_map.stations["storage"]
            self._behavior = ShelfSearch(self.r.link, self.r.camera, self.r.detector, self.mission.item,
                                         storage.search_offsets_mm, self.cfg["search"]["tilts_deg"], self.cfg)
            if self.mission.task is Task.HOME:  # "go back" arrived before the search started
                self._behavior.cancel()
            else:
                self.r.speaker.say(f"Looking for the {self.mission.item}")
        status = self._behavior.update(tel, dt)
        if status is SearchStatus.FOUND:
            return State.NOTIFY_AND_LOAD                                 # T5
        if status is SearchStatus.NOT_FOUND:
            if self.mission.task is not Task.HOME:
                self.r.speaker.say(f"Sorry, I could not find the {self.mission.item}")
                self.mission.task = Task.HOME
            return self._go_to("dock")                                   # T4
        return None

    def _notify_and_load(self, tel, dt, cmd, entering):
        tray_deg = self.cfg["tilt"]["tray_deg"]
        if entering:
            self.r.speaker.say(f"{self.mission.item} found. Please put it on my tray.")
            self.r.link.tilt(tray_deg)
        if self.time_in_state > self.cfg["load"]["timeout_s"]:
            self.r.speaker.say("Nothing was put on my tray. Going back.")
            self.mission.task = Task.HOME
            return self._go_to("dock")
        if not self._tilt_settled(tel, tray_deg) or not self._tray_check_due(dt):
            return None
        self._count = self._count + 1 if self.tray.has_item() else 0
        if self._count >= self.cfg["load"]["confirm_checks"]:
            self.r.speaker.say("Taking it to the workstation")
            self.mission.task = Task.DELIVER
            return self._go_to("workstation")                            # T6
        return None

    def _wait_at_workstation(self, tel, dt, cmd, entering):
        if entering:
            self.r.speaker.say("Arrived")
            self.r.link.tilt(self.cfg["tilt"]["tray_deg"])
        if cmd is None:
            return None
        if cmd.intent is Intent.TAKE_TO and cmd.place == "storage":
            self.r.speaker.say("Taking it back to storage")
            self.mission.task = Task.RETURN
            return self._go_to("storage")                                # T12
        if cmd.intent is Intent.FIND:
            self.r.speaker.say(f"Finding the {cmd.item}")
            self.mission.task = Task.FIND
            self.mission.item = cmd.item
            return self._go_to("storage")
        return None

    def _wait_for_unload(self, tel, dt, cmd, entering):
        tray_deg = self.cfg["tilt"]["tray_deg"]
        if entering:
            self.r.speaker.say("Please take the item off my tray")
            self.r.link.tilt(tray_deg)
        if not self._tilt_settled(tel, tray_deg) or not self._tray_check_due(dt):
            return None
        self._count = self._count + 1 if self.tray.has_item() is False else 0
        if self._count >= self.cfg["load"]["confirm_checks"]:
            self.r.speaker.say("Thank you. Going back to the dock.")
            self.mission.task = Task.HOME
            return self._go_to("dock")                                   # T14
        return None

    def _dock_align(self, tel, dt, cmd, entering):
        if entering:
            self.mission.location = "dock"
            self._behavior = DockAligner(self.r.link, self.r.camera, self.r.markers,
                                         self.r.lab_map.stations["dock"], self.cfg)
            self._behavior.start()
        status = self._behavior.update(tel, dt)
        if status is DockStatus.DOCKED:
            self.r.speaker.say("Docked")
            return State.DOCKED                                          # T8
        if status is DockStatus.FAILED:
            self._lost_message = "I could not find the charging coil. Please put me on the dock."
            return State.LOST
        return None

    def _lost(self, tel, dt, cmd, entering):
        if entering:
            self.r.link.stop()
            self.r.speaker.say(self._lost_message or "I am lost. Please put me back on the dock.")
        # Ends when the user puts the robot on the dock: the coil starts charging.
        self._timer = self._timer + dt if tel.charge_mv >= self.cfg["docking"]["charge_mv_min"] else 0.0
        if self._timer >= self.cfg["docking"]["confirm_s"]:
            return State.DOCKED
        return None
