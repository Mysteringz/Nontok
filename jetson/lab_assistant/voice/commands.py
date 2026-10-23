"""Turns recognised text into robot commands: "find ...", "take this to ...", "go back", "stop"."""
import re
from dataclasses import dataclass
from enum import Enum, auto
from typing import Dict, List, Optional


class Intent(Enum):
    FIND = auto()
    TAKE_TO = auto()
    GO_BACK = auto()
    STOP = auto()


@dataclass(frozen=True)
class Command:
    intent: Intent
    item: Optional[str] = None   # YOLO class name
    place: Optional[str] = None  # station name in lab_map.yaml


# Spoken phrase -> YOLO class name. Values must match training/dataset.yaml.
ITEMS: Dict[str, str] = {
    "multimeter": "multimeter",
    "multi meter": "multimeter",
    "screwdriver": "screwdriver",
    "screw driver": "screwdriver",
    "wire": "wire",
    "wires": "wire",
    "sensor": "sensor",
}

# Spoken phrase -> station name in config/lab_map.yaml.
PLACES: Dict[str, str] = {
    "storage": "storage",
    "the storage": "storage",
    "workstation": "workstation",
    "work station": "workstation",
    "the workstation": "workstation",
    "my desk": "workstation",
}


def _lookup(text: str, table: Dict[str, str]) -> Optional[str]:
    for phrase in sorted(table, key=len, reverse=True):  # longest match first
        if re.search(rf"\b{re.escape(phrase)}\b", text):
            return table[phrase]
    return None


def parse_command(text: str) -> Optional[Command]:
    t = text.lower().strip()
    if not t:
        return None
    if re.search(r"\b(stop|halt)\b", t):
        return Command(Intent.STOP)
    if re.search(r"\bgo (back|home)\b", t):
        return Command(Intent.GO_BACK)
    m = re.search(r"\b(?:find|get|fetch|bring)\b(.*)", t)
    if m:
        item = _lookup(m.group(1), ITEMS)
        return Command(Intent.FIND, item=item) if item else None
    m = re.search(r"\btake (?:this|it)(?: back)? to\b(.*)", t)
    if m:
        place = _lookup(m.group(1), PLACES)
        return Command(Intent.TAKE_TO, place=place) if place else None
    return None


def grammar() -> List[str]:
    """Phrases for Vosk's restricted grammar: a small vocabulary is much more accurate."""
    phrases = ["stop", "go back", "go home"]
    for item in ITEMS:
        phrases += [f"find the {item}", f"get the {item}", f"bring me the {item}"]
    for place in PLACES:
        phrases += [f"take this to {place}", f"take it to {place}", f"take this back to {place}"]
    return phrases + ["[unk]"]
