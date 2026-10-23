import math

from fakes import FakeLink, load_cfg
from lab_assistant.navigation.lab_map import Leg
from lab_assistant.navigation.localizer import Localizer
from lab_assistant.navigation.navigator import Navigator, NavStatus

DT = 0.1


def make_nav(**tel):
    link = FakeLink(**tel)
    loc = Localizer()
    loc.reset(Localizer().pose, link.telemetry.heading_deg)
    return Navigator(link, loc, load_cfg()), link


def run(nav, link, ticks=300, world=None):
    for _ in range(ticks):
        if world:
            world(link)
        status = nav.update(link.telemetry, DT)
        if status is not NavStatus.RUNNING:
            return status
    raise AssertionError("navigator never finished")


def test_drives_a_leg_and_arrives():
    nav, link = make_nav()
    nav.start([Leg(heading_deg=0, distance_mm=300)])
    assert run(nav, link) is NavStatus.ARRIVED
    driven = sum(vx * DT for kind, *v in link.sent if kind == "VEL" for vx in v[:1])
    assert 280 <= driven <= 330
    assert link.commanded == (0, 0, 0)


def test_turns_to_leg_heading_first():
    nav, link = make_nav()
    nav.start([Leg(heading_deg=90, distance_mm=100)])
    nav.update(link.telemetry, DT)
    assert link.sent[0] == ("TURN", 90)


def test_obstacle_in_front():
    nav, link = make_nav(front_mm=150)
    nav.start([Leg(heading_deg=0, distance_mm=1000)])
    assert run(nav, link) is NavStatus.OBSTACLE


def test_final_approach_wall_is_not_an_obstacle():
    nav, link = make_nav(front_mm=650)
    travelled = [0.0]

    def world(link):  # the wall gets closer as the robot drives
        travelled[0] += link.commanded[0] * DT
        link.set(front_mm=int(650 - travelled[0]))

    nav.start([Leg(heading_deg=0, distance_mm=500, stop_at_front_mm=150)])
    assert run(nav, link, world=world) is NavStatus.ARRIVED
    assert link.telemetry.front_mm <= 150


def test_wall_far_off_means_lost():
    nav, link = make_nav(left_mm=800)
    nav.start([Leg(heading_deg=0, distance_mm=100, expect={"left": 300})])
    assert run(nav, link) is NavStatus.WALL_CHECK_FAILED


def test_small_wall_error_is_corrected_by_strafing():
    nav, link = make_nav(left_mm=360)
    nav.start([Leg(heading_deg=0, distance_mm=100, expect={"left": 300})])
    assert run(nav, link) is NavStatus.ARRIVED
    assert any(kind == "VEL" and vy > 0 for kind, *rest in link.sent if kind == "VEL" for vy in rest[1:2])


def test_avoid_then_rejoin():
    nav, link = make_nav(left_mm=800, right_mm=300)
    nav.start([Leg(heading_deg=0, distance_mm=400)])
    link.set(front_mm=150)
    assert run(nav, link) is NavStatus.OBSTACLE

    nav.begin_avoid(link.telemetry)
    status = nav.avoid_update(link.telemetry, DT)
    assert status is NavStatus.RUNNING and link.commanded[1] > 0  # strafes left: more room
    link.set(front_mm=1500)
    for _ in range(50):
        status = nav.avoid_update(link.telemetry, DT)
        if status is not NavStatus.RUNNING:
            break
    assert status is NavStatus.CLEARED

    link.sent.clear()
    assert run(nav, link) is NavStatus.ARRIVED
    strafes = [rest[1] for kind, *rest in link.sent if kind == "VEL" and not math.isclose(rest[1], 0)]
    assert strafes and all(vy < 0 for vy in strafes)  # strafed back right to rejoin the route
