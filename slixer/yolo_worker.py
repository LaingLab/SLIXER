"""Runs an Ultralytics YOLO model on camera frames, in a process of its own.

Slixer starts this with its own Python once the vision extra is installed (`uv sync --extra vision`), or with
SLIXER_VISION_PYTHON, and talks to it over stdin and stdout. It is a process of its own for the arm's sake:
Slixer's control loop runs fifty times a second in the main process, and a model loading, a slow frame, or
a CUDA error in here can't delay it or take it down. If this dies, the arm carries on.

Messages are framed as a 4-byte big-endian length, a 1-byte kind, then the payload:

    in   C  config JSON   {"model": path, "conf": 0.25, "iou": 0.5, "imgsz": 640, "classes": [names] | null}
    in   F  frame         8-byte frame number, then the camera's JPEG as it arrived
    in   Q  quit
    out  R  ready JSON    {"model", "task", "names", "device", "warmup_ms"}
    out  D  results JSON  {"frame", "ms", "size": [w, h], "detections": [...]}
    out  E  error JSON    {"error": "what went wrong"}

Every box and outline is in fractions of the picture (0-1), so it can be drawn over the picture at any size.
"""

from __future__ import annotations

import json
import os
import struct
import sys
import time
import traceback

# stdout carries the messages. Anything else that prints -- Ultralytics logs, a library's warning, a stray
# print -- must not land in the middle of one, so the real stdout is kept for messages and everything
# written to "stdout" from here on, at the file-descriptor level too, goes to stderr instead.
_messages = os.fdopen(os.dup(1), "wb", buffering=0)
os.dup2(2, 1)
sys.stdout = sys.stderr
_requests = os.fdopen(os.dup(0), "rb", buffering=0)

os.environ.setdefault("YOLO_VERBOSE", "False")
os.environ.setdefault("YOLO_OFFLINE", "True")  # no telemetry; weights still download on first use

from regions import patches, thin  # noqa: E402  (after the redirect, like everything that might print)

# What Slixer can show: things found in the picture, boxed or outlined, or the areas a semantic model marks.
SHOWN = ("detect", "segment", "pose", "semantic")
KINDS = {"classify": "classification", "obb": "rotated-box (OBB)", "depth": "depth"}


def send(kind: bytes, payload: bytes) -> None:
    _messages.write(struct.pack(">I", len(payload) + 1) + kind + payload)


def send_json(kind: bytes, value) -> None:
    send(kind, json.dumps(value, separators=(",", ":")).encode())


def read_exactly(count: int) -> bytes:
    chunks, got = [], 0
    while got < count:
        chunk = _requests.read(count - got)
        if not chunk:
            raise EOFError
        chunks.append(chunk)
        got += len(chunk)
    return b"".join(chunks)


def receive() -> tuple[bytes, bytes]:
    length = struct.unpack(">I", read_exactly(4))[0]
    body = read_exactly(length)
    return body[:1], body[1:]


def open_vocabulary(path: str) -> bool:
    """YOLOE models find whatever they're told to by name; the -pf ones have a vocabulary of their own."""
    name = os.path.basename(path)
    return name.startswith("yoloe") and "-pf" not in name


class Model:
    def __init__(self, config: dict):
        import torch
        from ultralytics import YOLO, YOLOE

        self.path = config["model"]
        self.conf = float(config.get("conf", 0.25))
        self.iou = float(config.get("iou", 0.5))
        self.device = 0 if torch.cuda.is_available() else "cpu"
        self.half = self.device == 0  # half precision on the GPU: twice as fast, same answers
        self.open = open_vocabulary(self.path)
        self.model = YOLOE(self.path) if self.open else YOLO(self.path)
        self.task = self.model.task
        if self.task not in SHOWN:
            raise ValueError(f"{os.path.basename(self.path)} is a {KINDS.get(self.task, self.task)} model, which "
                             "Slixer can't show: it runs detection and segmentation models")
        # The size of picture it learned from, unless told otherwise: a model trained on 1280-pixel pictures
        # finds things at that scale, and at 640 misses them or marks others.
        self.imgsz = config.get("imgsz") or self.model.overrides.get("imgsz") or 640
        self.words: list[str] = []
        self.retune(config)

        started = time.perf_counter()
        import numpy as np

        side = max(self.imgsz) if isinstance(self.imgsz, (list, tuple)) else int(self.imgsz)
        self.model.predict(np.zeros((side, side, 3), np.uint8), imgsz=self.imgsz, device=self.device, half=self.half,
                           verbose=False)  # the first run pays for CUDA start-up; better now than on a frame
        self.warmup_ms = (time.perf_counter() - started) * 1000
        self.device_name = torch.cuda.get_device_name(0) if self.device == 0 else "CPU"

    def retune(self, config: dict) -> None:
        self.conf = float(config.get("conf", self.conf))
        wanted = [str(w).strip() for w in (config.get("classes") or []) if str(w).strip()]
        if self.open:
            # Its vocabulary is whatever it's told to look for, turned into text embeddings. Only redone
            # when the words change: it takes a few hundred milliseconds.
            if not wanted:
                raise ValueError("this model finds things by name: type what to look for")
            if wanted != self.words:
                self.model.set_classes(wanted, self.model.get_text_pe(wanted))
                self.words = wanted
            self.classes = None
        else:
            by_name = {name: index for index, name in dict(self.model.names).items()}
            self.classes = [by_name[n] for n in wanted if n in by_name] if wanted else None
        self.names = dict(self.model.names)

    def run(self, frame) -> list[dict]:
        result = self.model.predict(frame, conf=self.conf, iou=self.iou, imgsz=self.imgsz, device=self.device,
                                    half=self.half, classes=self.classes, verbose=False)[0]
        if self.task == "semantic":  # a class for every pixel rather than a list of things: each patch is a find
            marked = result.semantic_mask
            return patches(marked.data.cpu().numpy(), self.names) if marked is not None else []
        height, width = result.orig_shape
        detections = []
        boxes = result.boxes
        outlines = result.masks.xyn if result.masks is not None else None
        for index in range(len(boxes)):
            x0, y0, x1, y1 = (float(v) for v in boxes.xyxy[index].tolist())
            cls = int(boxes.cls[index])
            found = {
                "label": self.names.get(cls, str(cls)),
                "cls": cls,
                "score": round(float(boxes.conf[index]), 3),
                "box": [x0 / width, y0 / height, x1 / width, y1 / height],
                "centre": [(x0 + x1) / 2 / width, (y0 + y1) / 2 / height],
            }
            if outlines is not None and index < len(outlines) and len(outlines[index]):
                found["polygon"] = thin(outlines[index])
            detections.append(found)
        return detections


def main() -> None:
    import cv2
    import numpy as np

    model = None
    last_error = ""
    while True:
        try:
            kind, payload = receive()
        except EOFError:
            return  # Slixer has gone: so do we
        try:
            if kind == b"Q":
                return
            if kind == b"C":
                config = json.loads(payload)
                if model is not None and config.get("model") == model.path:
                    model.retune(config)  # same model, new thresholds: no reload, no pause
                else:
                    model = None
                    model = Model(config)
                send_json(b"R", {"model": os.path.basename(model.path), "task": model.task,
                                 "names": model.names, "device": model.device_name, "imgsz": model.imgsz,
                                 "warmup_ms": round(model.warmup_ms), "open": model.open})
            elif kind == b"F":
                number = struct.unpack(">Q", payload[:8])[0]
                if model is None:
                    send_json(b"E", {"error": "no model loaded yet"})
                    continue
                frame = cv2.imdecode(np.frombuffer(payload[8:], np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    send_json(b"D", {"frame": number, "ms": 0, "size": [0, 0], "detections": []})
                    continue
                started = time.perf_counter()
                detections = model.run(frame)
                send_json(b"D", {"frame": number, "ms": round((time.perf_counter() - started) * 1000, 1),
                                 "size": [frame.shape[1], frame.shape[0]], "detections": detections})
                last_error = ""
        except Exception as error:  # report it and stay up: one bad frame or model mustn't need a restart
            said = f"{type(error).__name__}: {error}"
            if said != last_error:  # the same failure on every picture goes in the log once, not 30 times a second
                traceback.print_exc()
            last_error = said
            send_json(b"E", {"error": said})


if __name__ == "__main__":
    main()
