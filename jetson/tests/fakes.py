"""Stand-ins for the hardware so the logic can be tested on a laptop."""
from dataclasses import replace
from pathlib import Path

from lab_assistant.config import load_yaml
from lab_assistant.hardware.protocol import Telemetry
from lab_assistant.navigation.lab_map import LabMap
from lab_assistant.navigation.navigator import NavStatus

CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"


def load_cfg():
    return load_yaml(CONFIG_DIR / "robot.yaml")


def load_map():
    return LabMap.load(CONFIG_DIR / "lab_map.yaml")


class FakeLink:
    def __init__(self, **tel):
        values = dict(t_ms=0, heading_deg=0.0, front_mm=2000, left_mm=500, right_mm=500,
                      charge_mv=0, turning=False, tilt_deg=90)
        values.update(tel)
        self.telemetry = Telemetry(**values)
        self.commanded = (0.0, 0.0, 0.0)
        self.sent = []

    def set(self, **kw):
        self.telemetry = replace(self.telemetry, **kw)

    def telemetry_age(self):
        return 0.0

    def vel(self, vx, vy, wz=0.0):
        self.commanded = (vx, vy, wz)
        self.sent.append(("VEL", vx, vy, wz))

    def stop(self):
        self.commanded = (0.0, 0.0, 0.0)
        self.sent.append(("STOP",))

    def turn(self, deg):
        self.commanded = (0.0, 0.0, 0.0)
        self.set(heading_deg=self.telemetry.heading_deg + deg)  # turns finish instantly
        self.sent.append(("TURN", deg))

    def turn_result(self):
        return True

    def hold(self, deg):
        self.sent.append(("HOLD", deg))

    def tilt(self, deg):
        self.set(tilt_deg=int(deg))  # servo moves instantly
        self.sent.append(("TILT", deg))


class FakeFrame:
    shape = (480, 640, 3)


class FakeCamera:
    def read(self):
        return FakeFrame()


class FakeDetector:
    def __init__(self):
        self.detections = []

    def detect(self, frame):
        return list(self.detections)

    def find(self, frame, label):
        hits = [d for d in self.detections if d.label == label]
        return max(hits, key=lambda d: d.conf) if hits else None


class FakeMarkers:
    def __init__(self):
        self.obs = {}

    def detect(self, frame):
        return dict(self.obs)


class FakeSpeaker:
    speaking = False

    def __init__(self):
        self.said = []

    def say(self, text):
        self.said.append(text)


class FakeNavigator:
    """Arrives on the first update; records the routes it was given."""

    def __init__(self):
        self.routes = []

    def start(self, legs):
        self.routes.append(legs)

    def update(self, tel, dt):
        return NavStatus.ARRIVED

    def begin_avoid(self, tel):
        pass

    def avoid_update(self, tel, dt):
        return NavStatus.CLEARED
