"""A YOLO model watching the camera, run in a separate process.

This side keeps `yolo_worker.py` running in a process of its own (it needs the vision extra: `uv sync --extra
vision`), hands it the camera's newest frame whenever it's ready for one, and keeps its latest answer. One frame is in flight at a time, so the
model always works on the freshest picture and a slow frame means a skipped frame, never a growing queue.

If the worker dies -- a CUDA error, a bad model file, an out-of-memory -- this notices, says why, and the
rest of Slixer carries on. The arm never waits on anything in here.

    runner = ModelRunner(camera=lambda: slixer.camera)
    runner.start("yolo26s-seg.pt", conf=0.35)
    runner.detections   # [{label, score, box, centre, polygon?}, ...], fractions of the picture
"""

from __future__ import annotations

import importlib.util
import json
import struct
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path
from typing import Callable

import paths

STOCK_MODELS = (  # downloaded on first use; the -seg ones outline what they find as well as boxing it
    "yolo26n-seg.pt", "yolo26s-seg.pt", "yolo26m-seg.pt",
    "yolo26n.pt", "yolo26s.pt", "yolo26m.pt",
    # Open vocabulary: finds things by name -- "well plate" -- with no training. -pf: a built-in vocabulary.
    "yoloe-26s-seg.pt", "yoloe-26m-seg.pt", "yoloe-26s-seg-pf.pt",
)
DEFAULT_MODEL = "yolo26s-seg.pt"  # on an RTX 4090, 14 ms a frame; the n and m sizes are within 3 ms of it
STALE_SECONDS = 1.0  # answers about a picture older than this are dropped: the scene has moved on
WORKER = Path(__file__).resolve().parent / "yolo_worker.py"


def open_vocabulary(name: str) -> bool:
    return name.startswith("yoloe") and "-pf" not in name


def available_models() -> list[dict]:
    """Every model that can be picked: the stock ones, and anything trained and put in the models folder."""
    found = {p.name for p in paths.MODELS.glob("*.pt")}
    models = [{"name": name, "stock": True, "downloaded": name in found, "open": open_vocabulary(name)}
              for name in STOCK_MODELS]
    models += [{"name": name, "stock": False, "downloaded": True, "open": False}
               for name in sorted(found) if name not in STOCK_MODELS]
    return models


KEEP = object()  # an argument left out: keep what was set last time, rather than clear it


class ModelRunner:
    def __init__(self, camera: Callable[[], object | None], python: Path | None = None):
        self.camera = camera
        self.python = Path(python or paths.VISION_PYTHON)
        self.state = "off"  # off, starting, running, failed
        self.problem = ""
        self.info: dict = {}  # what the worker said when the model loaded
        self.config: dict = {"model": DEFAULT_MODEL, "conf": 0.35, "classes": None}
        self.detections: list[dict] = []
        self.answered_at = 0.0
        self.ms = 0.0
        self.rate = 0.0
        self._process: subprocess.Popen | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._write = threading.Lock()  # stop() and the feeder can both write to the worker; never at once
        self._frame_times: deque = deque(maxlen=30)
        self._retune: dict | None = None  # new thresholds, handed to the worker between frames
        self.log_path = paths.DATA / "vision-worker.log"

    # ---- starting and stopping ------------------------------------------------------------------------

    @property
    def installed(self) -> bool:
        """Whether Ultralytics is there to run: in this environment, or in another one named for it."""
        if self.python == Path(sys.executable):
            return importlib.util.find_spec("ultralytics") is not None
        return self.python.exists()

    def start(self, model: str | None = None, conf: float | None = None, classes=KEEP) -> str:
        """Starts (or restarts) the model. Returns why it can't, or "" once it is on its way.

        Whatever is left out is as it was last time -- so a model that was on comes back after a restart
        still looking for the same things. `classes=None` means everything the model knows."""
        name = (model or self.config["model"]).strip()
        if "/" in name or "\\" in name or not name.endswith(".pt"):
            raise ValueError("a model is a .pt file name from the models folder")
        if name not in STOCK_MODELS and not (paths.MODELS / name).exists():
            raise ValueError(f"there's no {name} in {paths.MODELS}")
        if not self.installed:
            return "the vision extra isn't installed -- run: uv sync --extra vision"
        if classes is KEEP:
            classes = self.config["classes"]
        if open_vocabulary(name) and not classes:
            raise ValueError(f"{name} finds things by name: type what to look for first")
        self.stop()
        self.config = {
            "model": name,
            "conf": max(0.01, min(0.99, float(conf if conf is not None else self.config["conf"]))),
            "classes": [str(c) for c in classes] if classes else None,
        }
        self._stop.clear()
        self.state, self.problem, self.info = "starting", "", {}
        self._frame_times.clear()  # else the gap since the last model ran reads as a slow rate
        self.rate = 0.0
        self._thread = threading.Thread(target=self._run, daemon=True, name="model")
        self._thread.start()
        return ""

    def retune(self, conf: float | None = None, classes=KEEP) -> str:
        """New thresholds for the running model, without reloading it. Starts it if it isn't running."""
        if classes is KEEP:
            classes = self.config["classes"]
        if open_vocabulary(self.config["model"]) and not classes:
            raise ValueError("this model finds things by name: type what to look for")
        if conf is not None:
            self.config["conf"] = max(0.01, min(0.99, float(conf)))
        self.config["classes"] = [str(c) for c in classes] if classes else None
        if self.state != "running":
            return self.start()
        self._retune = dict(self.config)  # the feeder sends it: one conversation with the worker at a time
        return ""

    def stop(self) -> None:
        self._stop.set()
        process = self._process
        if process is not None and process.poll() is None:
            try:
                self._send(process, b"Q", b"")
            except OSError:
                pass
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=5)
        self._process = None
        with self._lock:
            self.detections = []
        if self.state != "failed":
            self.state = "off"

    # ---- talking to the worker -------------------------------------------------------------------------

    def _send(self, process, kind: bytes, payload: bytes) -> None:
        with self._write:
            process.stdin.write(struct.pack(">I", len(payload) + 1) + kind + payload)
            process.stdin.flush()

    @staticmethod
    def _receive(process) -> tuple[bytes, bytes]:
        header = process.stdout.read(4)
        if len(header) < 4:
            raise EOFError
        length = struct.unpack(">I", header)[0]
        body = process.stdout.read(length)
        if len(body) < length:
            raise EOFError
        return body[:1], body[1:]

    def _fail(self, why: str) -> None:
        self.state, self.problem = "failed", why
        with self._lock:
            self.detections = []

    def _log_tail(self) -> str:
        try:
            lines = [l for l in self.log_path.read_text(errors="replace").splitlines() if l.strip()]
            return lines[-1][:200] if lines else ""
        except OSError:
            return ""

    def _run(self) -> None:
        paths.MODELS.mkdir(parents=True, exist_ok=True)
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        # The worker's own chatter -- downloads, warnings -- goes to a log file. Left in a pipe nobody
        # reads, it would fill the pipe and freeze the worker mid-sentence.
        log = open(self.log_path, "ab")
        try:
            process = subprocess.Popen([str(self.python), str(WORKER)], stdin=subprocess.PIPE,
                                       stdout=subprocess.PIPE, stderr=log, cwd=str(paths.MODELS))
        except OSError as error:
            log.close()
            return self._fail(f"couldn't start the model: {error}")
        self._process = process
        try:
            self._send(process, b"C", json.dumps(self.config).encode())
            kind, payload = self._receive(process)  # loading, and downloading the first time
            if kind != b"R":
                return self._fail(json.loads(payload).get("error", "the model didn't load"))
            self.info = json.loads(payload)
            self.state = "running"
            self._feed(process)
        except (EOFError, BrokenPipeError, OSError, ValueError) as error:
            if not self._stop.is_set():
                tail = self._log_tail()
                self._fail(f"the model stopped{': ' + tail if tail else f' ({type(error).__name__})'}")
        finally:
            log.close()
            if process.poll() is None:
                process.kill()

    def _feed(self, process) -> None:
        last_seen = -1
        while not self._stop.is_set():
            camera = self.camera()
            if camera is None:
                self._forget_if_stale()
                time.sleep(0.1)
                continue
            if self._retune is not None:
                config, self._retune = self._retune, None
                self._send(process, b"C", json.dumps(config).encode())
                kind, payload = self._receive(process)
                if kind == b"R":
                    self.info = json.loads(payload)
            jpeg, age = camera.read_jpeg()
            number = camera.frames_received
            if jpeg is None or age > STALE_SECONDS or number == last_seen:
                self._forget_if_stale()
                time.sleep(0.005)
                continue
            last_seen = number
            captured = time.monotonic() - age
            self._send(process, b"F", struct.pack(">Q", number) + jpeg)
            kind, payload = self._receive(process)
            answer = json.loads(payload)
            if kind == b"E":
                self.problem = answer.get("error", "")
                continue
            with self._lock:
                self.detections = answer["detections"]
                self.answered_at = captured  # how old the answer is, measured from the picture it's about
            self.ms = answer["ms"]
            self.problem = ""
            self._frame_times.append(time.monotonic())
            if len(self._frame_times) >= 8:  # fewer, and two quick answers in a row read as a rate
                span = self._frame_times[-1] - self._frame_times[0]
                self.rate = (len(self._frame_times) - 1) / span if span > 0 else 0.0

    def _forget_if_stale(self) -> None:
        if self.detections and time.monotonic() - self.answered_at > STALE_SECONDS:
            with self._lock:
                self.detections = []
            self.rate = 0.0

    # ---- for everyone else -----------------------------------------------------------------------------

    def current(self) -> list[dict]:
        with self._lock:
            if time.monotonic() - self.answered_at > STALE_SECONDS:
                return []
            return list(self.detections)

    def describe(self) -> dict:
        return {
            "installed": self.installed,
            "state": self.state,
            "problem": self.problem,
            "model": self.config["model"],
            "conf": self.config["conf"],
            "classes": self.config["classes"],
            "task": self.info.get("task"),
            "device": self.info.get("device"),
            "names": sorted(set((self.info.get("names") or {}).values())),
            "open": open_vocabulary(self.config["model"]),
            "ms": self.ms,
            "rate": round(self.rate, 1),
        }
