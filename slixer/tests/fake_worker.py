"""Stands in for yolo_worker.py, speaking its protocol, to test what Slixer does with a model's answers.

FAKE_WORKER_FAILS says how it answers pictures: "every" fails on each one, as a model Slixer can't read the
results of does; "sometimes" fails on every third, as bad frames might, and answers the rest.
"""

import json
import os
import struct
import sys

messages, requests = sys.stdout.buffer, sys.stdin.buffer
FOUND = [{"label": "thing", "cls": 0, "score": 0.9, "box": [0.1, 0.1, 0.2, 0.2], "centre": [0.15, 0.15]}]


def send(kind: bytes, value) -> None:
    payload = json.dumps(value).encode()
    messages.write(struct.pack(">I", len(payload) + 1) + kind + payload)
    messages.flush()


def receive() -> tuple[bytes, bytes]:
    header = requests.read(4)
    if len(header) < 4:
        raise EOFError
    body = requests.read(struct.unpack(">I", header)[0])
    return body[:1], body[1:]


fails, pictures = os.environ.get("FAKE_WORKER_FAILS", "every"), 0
while True:
    try:
        kind, payload = receive()
    except EOFError:
        break
    if kind == b"Q":
        break
    if kind == b"C":
        send(b"R", {"model": "fake.pt", "task": "detect", "names": {"0": "thing"}, "device": "fake", "warmup_ms": 0})
    elif kind == b"F":
        pictures += 1
        if fails == "every" or pictures % 3 == 0:
            send(b"E", {"error": "TypeError: object of type 'NoneType' has no len()"})
        else:
            send(b"D", {"frame": struct.unpack(">Q", payload[:8])[0], "ms": 1.0, "size": [640, 480], "detections": FOUND})
