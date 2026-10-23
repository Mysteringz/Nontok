"""Line protocol shared with firmware/lab_assistant_mega. See docs/lab_assistant_protocol.md."""
from dataclasses import dataclass
from typing import Optional, Tuple, Union


@dataclass
class Telemetry:
    t_ms: int
    heading_deg: float  # gyro heading, counter-clockwise positive, not wrapped
    front_mm: int
    left_mm: int
    right_mm: int
    charge_mv: int
    turning: bool
    tilt_deg: int


@dataclass
class Event:
    kind: str           # READY, PONG, OK, DONE, ERR
    args: Tuple[str, ...]


def parse_line(line: str) -> Optional[Union[Telemetry, Event]]:
    parts = line.strip().split()
    if not parts:
        return None
    if parts[0] == "TEL":
        if len(parts) != 9:
            return None
        try:
            return Telemetry(
                t_ms=int(parts[1]),
                heading_deg=float(parts[2]),
                front_mm=int(parts[3]),
                left_mm=int(parts[4]),
                right_mm=int(parts[5]),
                charge_mv=int(parts[6]),
                turning=parts[7] == "1",
                tilt_deg=int(parts[8]),
            )
        except ValueError:
            return None  # garbled line, e.g. right after the Mega resets
    return Event(parts[0], tuple(parts[1:]))


def vel(vx: float, vy: float, wz: float) -> str:
    return f"VEL {vx:.0f} {vy:.0f} {wz:.1f}"


def turn(rel_deg: float) -> str:
    return f"TURN {rel_deg:.1f}"


def hold(heading_deg: float) -> str:
    return f"HOLD {heading_deg:.1f}"


def tilt(deg: int) -> str:
    return f"TILT {int(deg)}"


STOP = "STOP"
ZERO = "ZERO"
CAL = "CAL"
PING = "PING"
