"""Static lab map: station positions and fixed routes between them (config/lab_map.yaml)."""
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from .geometry import Pose


@dataclass
class Leg:
    heading_deg: float                        # turn to this map heading first
    distance_mm: float                        # then drive this far (dead reckoning)
    stop_at_front_mm: Optional[float] = None  # or: drive until the front wall is this close
    expect: Dict[str, float] = field(default_factory=dict)  # front/left/right wall distances at the end


@dataclass
class Station:
    name: str
    pose: Pose
    marker_id: Optional[int] = None
    wall_mm: Optional[float] = None  # front distance when parked at the station
    search_offsets_mm: List[float] = field(default_factory=lambda: [0.0])


class LabMap:
    def __init__(self, stations: Dict[str, Station], routes: Dict[str, List[Leg]]):
        self.stations = stations
        self.routes = routes

    @classmethod
    def load(cls, path) -> "LabMap":
        from ..config import load_yaml

        return cls.from_dict(load_yaml(path))

    @classmethod
    def from_dict(cls, data: dict) -> "LabMap":
        stations = {}
        for name, s in data["stations"].items():
            stations[name] = Station(
                name=name,
                pose=Pose(*s["pose"]),
                marker_id=s.get("marker_id"),
                wall_mm=s.get("wall_mm"),
                search_offsets_mm=s.get("search_offsets_mm", [0.0]),
            )
        routes = {}
        for key, legs in data["routes"].items():
            routes[key] = [
                Leg(
                    heading_deg=leg["heading"],
                    distance_mm=leg["distance_mm"],
                    stop_at_front_mm=leg.get("stop_at_front_mm"),
                    expect=leg.get("expect", {}),
                )
                for leg in legs
            ]
        return cls(stations, routes)

    def route(self, start: str, goal: str) -> List[Leg]:
        key = f"{start}->{goal}"
        if key not in self.routes:
            raise KeyError(f"No route {key} in lab_map.yaml")
        return self.routes[key]
