"""The few things worth remembering between runs, so starting Slixer is one command with no arguments.

Kept in `settings.json` beside this file. Anything given on the command line wins for that run; anything
chosen in the page (the camera's address, say) is saved here for next time.
"""

from __future__ import annotations

import json
from pathlib import Path

import paths

SETTINGS_PATH = paths.SETTINGS

DEFAULTS = {
    "camera_host": None,  # the Pi beside the arm, e.g. "192.168.1.50"
    "camera_port": 50102,
}


def load() -> dict:
    settings = dict(DEFAULTS)
    try:
        stored = json.loads(SETTINGS_PATH.read_text())
    except (OSError, json.JSONDecodeError):
        return settings  # missing or half-written: the defaults are a fine place to start
    if isinstance(stored, dict):
        settings.update({key: stored[key] for key in DEFAULTS if key in stored})
    return settings


def save(**changes) -> dict:
    settings = load()
    settings.update({key: value for key, value in changes.items() if key in DEFAULTS})
    temporary = SETTINGS_PATH.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(settings, indent=2) + "\n")
    temporary.replace(SETTINGS_PATH)
    return settings
