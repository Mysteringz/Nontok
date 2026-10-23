from fakes import (FakeCamera, FakeDetector, FakeLink, FakeMarkers, FakeNavigator, FakeSpeaker,
                   load_cfg, load_map)
from lab_assistant.navigation.localizer import Localizer
from lab_assistant.perception.detector import Detection
from lab_assistant.perception.markers import MarkerObs
from lab_assistant.state_machine import Robot, State, StateMachine
from lab_assistant.voice.commands import Command, Intent

DT = 0.1
MULTIMETER = Detection("multimeter", 0.9, (300, 300, 340, 400))  # inside the tray area


def make_sm():
    robot = Robot(link=FakeLink(), camera=FakeCamera(), detector=FakeDetector(), markers=FakeMarkers(),
                  speaker=FakeSpeaker(), localizer=Localizer(), lab_map=load_map(), cfg=load_cfg())
    nav = FakeNavigator()
    sm = StateMachine(robot, navigator=nav)
    sm.tick(DT)
    return sm, robot, nav


def run_until(sm, state, ticks=500):
    for _ in range(ticks):
        sm.tick(DT)
        if sm.state is state:
            return
    raise AssertionError(f"stuck in {sm.state}")


def test_find_deliver_return_and_dock():
    sm, r, nav = make_sm()
    assert sm.state is State.DOCKED

    sm.tick(DT, Command(Intent.FIND, item="multimeter"))                  # T1
    assert sm.state is State.UNDOCK
    run_until(sm, State.NAVIGATE)                                         # T2
    assert nav.routes[-1] == r.lab_map.route("dock", "storage")

    r.detector.detections = [MULTIMETER]
    run_until(sm, State.SEARCH)                                           # T3
    run_until(sm, State.NOTIFY_AND_LOAD)                                  # T5
    run_until(sm, State.WAIT_AT_WORKSTATION)                              # T6, T11
    assert nav.routes[-1] == r.lab_map.route("storage", "workstation")

    sm.tick(DT, Command(Intent.TAKE_TO, place="storage"))                 # T12
    run_until(sm, State.WAIT_FOR_UNLOAD)                                  # T13
    r.detector.detections = []
    run_until(sm, State.DOCK_ALIGN)                                       # T14, T7
    assert nav.routes[-1] == r.lab_map.route("storage", "dock")

    r.markers.obs = {0: MarkerObs(0, 0.0, 0.0, 80.0)}
    r.link.set(front_mm=100, charge_mv=5000)
    run_until(sm, State.DOCKED)                                           # T8


def test_item_not_found_goes_home():
    sm, r, nav = make_sm()
    sm.tick(DT, Command(Intent.FIND, item="screwdriver"))
    run_until(sm, State.SEARCH)
    run_until(sm, State.DOCK_ALIGN)                                       # T4
    assert any("could not find" in s for s in r.speaker.said)
    assert r.link.sent.count(("TILT", 75)) == 3                           # 3 shelf positions x 2 tilts


def test_go_back_from_workstation():
    sm, r, nav = make_sm()
    sm.tick(DT, Command(Intent.FIND, item="multimeter"))
    r.detector.detections = [MULTIMETER]
    run_until(sm, State.WAIT_AT_WORKSTATION)
    sm.tick(DT, Command(Intent.GO_BACK))
    run_until(sm, State.DOCK_ALIGN)
    assert nav.routes[-1] == r.lab_map.route("workstation", "dock")


def test_stop_then_put_back_on_dock():
    sm, r, nav = make_sm()
    sm.tick(DT, Command(Intent.FIND, item="multimeter"))
    sm.tick(DT, Command(Intent.STOP))
    assert sm.state is State.LOST
    assert r.link.commanded == (0, 0, 0)
    r.link.set(charge_mv=5000)
    run_until(sm, State.DOCKED)


def test_lost_when_route_fails():
    from lab_assistant.navigation.navigator import NavStatus

    sm, r, nav = make_sm()
    nav.update = lambda tel, dt: NavStatus.WALL_CHECK_FAILED
    sm.tick(DT, Command(Intent.FIND, item="multimeter"))
    run_until(sm, State.LOST)                                             # T15
