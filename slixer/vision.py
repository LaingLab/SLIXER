"""What the camera sees, as something a program can wait for.

A YOLO model -- stock, open-vocabulary, or one trained on your own things -- runs on the camera in a process
of its own (detection.py). This keeps its settings in `vision.json`, so it comes back on after a restart
if it was on, and keeps the session's idea of what's in view up to date, which is what a program's "wait
for" step waits on.

    vision = Vision(session, camera=lambda: slixer.camera)
    vision.start()
    vision.set_model(True, "yoloe-26s-seg.pt", classes=["well plate"])
"""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import Callable

import paths
from detection import KEEP, ModelRunner

VISION_FILE = paths.VISION
RATE_HZ = 10.0  # for deciding, not for steering: ten times a second is plenty


class Vision:
    """The model's settings, and what it can see right now."""

    def __init__(self, session, camera: Callable[[], object | None], path: Path = VISION_FILE,
                 runner: ModelRunner | None = None):
        self.session = session
        self.camera = camera  # a function, because the camera can be swapped for another while running
        self.path = path
        self.model = runner or ModelRunner(camera)
        self.model_on = False
        self._running = False
        self._thread: threading.Thread | None = None
        self.load()

    # ---- the file ------------------------------------------------------------------------------------

    def load(self) -> None:
        try:
            raw = json.loads(self.path.read_text())
        except (OSError, json.JSONDecodeError):
            return
        self.model_on = bool(raw.get("model_on", False))
        saved = raw.get("model") or {}
        self.model.config.update({k: saved[k] for k in ("model", "conf", "classes") if k in saved})

    def save(self) -> None:
        temporary = self.path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps({"model_on": self.model_on, "model": self.model.config}, indent=2) + "\n")
        temporary.replace(self.path)

    # ---- running -------------------------------------------------------------------------------------

    def start(self) -> "Vision":
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="vision")
        self._thread.start()
        if self.model_on:
            try:
                problem = self.model.start()  # it was running last time: bring it back
            except ValueError as error:
                problem = str(error)
            if problem:
                self.model.state, self.model.problem = "failed", problem
        return self

    def stop(self) -> None:
        self._running = False
        self.model.stop()
        if self._thread:
            self._thread.join(timeout=2)

    def set_model(self, on: bool, model: str | None = None, conf: float | None = None, classes=KEEP) -> str:
        """Starts, restarts or stops the model. Returns why not, if it can't. What's left out stays as it was;
        `classes=None` means everything the model knows."""
        if not on:
            self.model_on = False
            self.model.stop()
            self.model.state = "off"
            self._refresh_seen()  # at once: a program must not act on something the model was just told to forget
            self.save()
            return ""
        same = (model is None or model == self.model.config["model"]) and self.model.state == "running"
        problem = self.model.retune(conf, classes) if same else self.model.start(model, conf, classes)
        if problem:
            return problem
        self.model_on = True
        self.save()
        return ""

    def _refresh_seen(self) -> None:
        """What counts as seen: anything the model is confident of, in a picture less than a second old."""
        self.session.seen = {hit["label"] for hit in self.model.current()} if self.model_on else set()

    def _loop(self) -> None:
        period = 1.0 / RATE_HZ
        while self._running:
            self._refresh_seen()
            time.sleep(period)

    # ---- for everyone else -----------------------------------------------------------------------------

    def labels(self) -> list[str]:
        """Everything a "wait for" step could wait on: the names the running model knows."""
        return sorted(self.model.describe()["names"]) if self.model_on else []

    def describe(self) -> dict:
        return {
            "model_on": self.model_on,
            "model": self.model.describe(),
            "found": self.model.current() if self.model_on else [],
        }
