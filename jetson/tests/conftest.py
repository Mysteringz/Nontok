import sys
from pathlib import Path

JETSON_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(JETSON_DIR))
