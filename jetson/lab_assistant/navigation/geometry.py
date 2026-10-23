from dataclasses import dataclass


@dataclass
class Pose:
    """Map frame: origin at the dock coil, +x = facing the dock wall, +y = left, heading CCW."""
    x_mm: float = 0.0
    y_mm: float = 0.0
    heading_deg: float = 0.0


def wrap180(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0
