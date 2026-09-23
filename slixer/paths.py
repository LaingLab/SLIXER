"""Where Slixer keeps what it remembers.

Everything lives beside the code by default. Setting SLIXER_DATA (or `run.py --data DIR`) moves all of it
somewhere else at once -- which is how the tests run without touching your programs, your calibration or
your camera address.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

DATA = Path(os.environ.get("SLIXER_DATA") or Path(__file__).resolve().parent)

MAPPING = DATA / "mapping.json"  # how the model lines up with the arm
SETTINGS = DATA / "settings.json"  # the camera's address, and so on
PROGRAMS = DATA / "programs"
SCENE = DATA / "scene"  # imported parts
VISION = DATA / "vision.json"  # which model, and whether it is on
DATASETS = DATA / "datasets"  # pictures collected for training your own model

# Models are assets rather than state: beside the code, whatever SLIXER_DATA says, so tests and trial runs
# share the downloaded weights instead of fetching them again.
MODELS = Path(__file__).resolve().parent / "models"
# The Python the YOLO models run under: this one, once the vision extra is installed (uv sync --extra vision),
# unless SLIXER_VISION_PYTHON names another -- an environment of their own, say.
VISION_PYTHON = Path(os.environ.get("SLIXER_VISION_PYTHON") or sys.executable)
