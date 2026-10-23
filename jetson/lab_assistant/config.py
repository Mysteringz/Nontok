"""YAML config loading. Configs are plain dicts: cfg["motion"]["cruise_speed"]."""
from pathlib import Path


def load_yaml(path):
    import yaml

    with open(Path(path)) as f:
        return yaml.safe_load(f)
