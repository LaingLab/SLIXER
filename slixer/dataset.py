"""Collects camera pictures for training a model on your own things.

A stock model knows eighty everyday objects; it has never seen a 6-well plate. Teaching it takes a few
hundred labelled pictures of your things, taken the way the arm will see them, and this is what gathers
them: press capture (or leave it capturing every few seconds while you move things about), and each picture
is saved exactly as the camera sent it, alongside where the arm was at that moment.

If a model is running, what it sees is saved too, as a YOLO label file -- a first draft of the labels. In an
annotation tool you then correct drafts instead of drawing every outline from scratch. Once your own model
is any good, its drafts are good, and the next round of labelling is quicker still.

    datasets/<name>/images/<stamp>.jpg     the picture, byte for byte
    datasets/<name>/labels/<stamp>.txt     draft labels, YOLO format (outlines if the model segments)
    datasets/<name>/poses/<stamp>.json     the arm's joint angles and mode when it was taken
    datasets/<name>/classes.txt            one class name per line; a label's first number is its line
"""

from __future__ import annotations

import json
import re
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

import paths

SAFE_NAME = re.compile(r"^[A-Za-z0-9_-]{1,40}$")
SAME_PICTURE = 2.0  # mean difference, 0-255, below which an automatic capture is skipped as a repeat


def folder(name: str) -> Path:
    if not SAFE_NAME.match(name or ""):
        raise ValueError("a dataset name may only use letters, numbers, - and _")
    return paths.DATASETS / name


def thumbnail(jpeg: bytes) -> np.ndarray | None:
    image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_REDUCED_GRAYSCALE_8)
    return None if image is None else cv2.resize(image, (40, 30), interpolation=cv2.INTER_AREA).astype(np.float32)


def _not_empty(path: Path) -> bool:
    """Whether a label file has anything in it. One that vanishes while the folder is being counted (they
    get replaced wholesale with an annotation tool's export) simply isn't counted."""
    try:
        return path.stat().st_size > 0
    except OSError:
        return False


class Collector:
    def __init__(self, camera, session, vision):
        self.camera = camera  # functions, as elsewhere: these can be swapped while running
        self.session = session
        self.vision = vision
        self.name = "lab"
        self.prelabel = True
        self.auto_every = 0.0  # seconds between automatic captures; 0 is off
        self.last_note = ""
        self._last_thumb: np.ndarray | None = None
        self._auto: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()

    # ---- taking pictures -----------------------------------------------------------------------------------

    def capture(self, automatic: bool = False) -> str:
        """Saves the camera's newest picture. Returns what happened, as a sentence."""
        camera = self.camera()
        if camera is None:
            raise ValueError("no camera to take a picture with")
        jpeg, age = camera.read_jpeg()
        if jpeg is None or age > 1.0:
            raise ValueError("the camera hasn't sent a picture in the last second")
        with self._lock:
            thumb = thumbnail(jpeg)
            if automatic and thumb is not None and self._last_thumb is not None \
                    and float(np.mean(np.abs(thumb - self._last_thumb))) < SAME_PICTURE:
                self.last_note = "skipped: nothing has changed since the last one"
                return self.last_note
            self._last_thumb = thumb
            where = folder(self.name)
            for sub in ("images", "labels", "poses"):
                (where / sub).mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")[:-3]
            (where / "images" / f"{stamp}.jpg").write_bytes(jpeg)  # exactly as the camera made it
            (where / "poses" / f"{stamp}.json").write_text(json.dumps({
                "arm": self.session.current_arm_pose(),
                "mode": self.session.mode,
                "connected": self.session.connected,
                "time": time.time(),
            }))
            drafted = 0
            if self.prelabel and self.vision.model_on:
                drafted = self._draft_labels(where, stamp, self.vision.model.current())
            count = len(list((where / "images").glob("*.jpg")))
            self.last_note = f"saved picture {count} in {self.name}" + (f", with {drafted} draft labels" if drafted else "")
            return self.last_note

    def _draft_labels(self, where: Path, stamp: str, found: list[dict]) -> int:
        """Writes what the model saw in YOLO's format: an outline where there is one, else a box."""
        if not found:
            return 0
        classes_file = where / "classes.txt"
        classes = classes_file.read_text().split("\n") if classes_file.exists() else []
        classes = [c for c in classes if c.strip()]
        lines = []
        for hit in found:
            if hit["label"] not in classes:
                classes.append(hit["label"])  # new names go on the end, so existing numbers never change
            index = classes.index(hit["label"])
            if hit.get("polygon") and len(hit["polygon"]) >= 3:
                points = " ".join(f"{x:.5f} {y:.5f}" for x, y in hit["polygon"])
                lines.append(f"{index} {points}")
            else:
                x0, y0, x1, y1 = hit["box"]
                lines.append(f"{index} {(x0 + x1) / 2:.5f} {(y0 + y1) / 2:.5f} {x1 - x0:.5f} {y1 - y0:.5f}")
        classes_file.write_text("\n".join(classes) + "\n")
        (where / "labels" / f"{stamp}.txt").write_text("\n".join(lines) + "\n")
        return len(lines)

    # ---- capturing on a timer ------------------------------------------------------------------------------

    def set_auto(self, every: float) -> None:
        """Captures every `every` seconds until set to 0. Repeats of an unchanged scene are skipped."""
        self._stop.set()
        if self._auto is not None:
            self._auto.join(timeout=2)
        self.auto_every = max(0.0, min(3600.0, float(every)))
        if self.auto_every <= 0:
            return
        self._stop = threading.Event()
        stop = self._stop

        def run() -> None:
            while not stop.wait(self.auto_every):
                try:
                    self.capture(automatic=True)
                except ValueError as error:
                    self.last_note = f"automatic capture paused: {error}"

        self._auto = threading.Thread(target=run, daemon=True, name="capture")
        self._auto.start()

    def stop(self) -> None:
        self._stop.set()

    # ---- what's there --------------------------------------------------------------------------------------

    def describe(self) -> dict:
        datasets = []
        if paths.DATASETS.exists():
            for where in sorted(p for p in paths.DATASETS.iterdir() if p.is_dir()):
                images = len(list((where / "images").glob("*.jpg"))) if (where / "images").exists() else 0
                labelled = sum(_not_empty(p) for p in (where / "labels").glob("*.txt")) \
                    if (where / "labels").exists() else 0
                datasets.append({"name": where.name, "images": images, "labelled": labelled})
        return {"name": self.name, "prelabel": self.prelabel, "auto_every": self.auto_every,
                "note": self.last_note, "datasets": datasets}
