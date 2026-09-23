"""A stand-in for the Pi's camera: a still picture, served exactly as the Pi streams (camera_stream.py's
framing), so Slixer can't tell it from the real one.

    python slixer/tests/fake_camera.py --port 50113
    python slixer/tests/fake_camera.py --port 50113 --picture photo.jpg   # something for a model to find
"""
from __future__ import annotations

import argparse
import socket
import struct
import threading
import time

import cv2
import numpy as np

HEADER = struct.Struct("<4sIdI")
MAGIC = b"S101"

# a few coloured blocks on a grey table: (x0, y0, x1, y1) as fractions of the picture, and a BGR colour
BLOCKS = (
    ((0.12, 0.30, 0.30, 0.62), (45, 45, 205)),
    ((0.42, 0.30, 0.58, 0.62), (60, 190, 40)),
    ((0.70, 0.30, 0.88, 0.62), (190, 60, 30)),
)


def picture(width: int = 640, height: int = 480) -> np.ndarray:
    frame = np.full((height, width, 3), 118, np.uint8)
    for (x0, y0, x1, y1), colour in BLOCKS:
        cv2.rectangle(frame, (int(x0 * width), int(y0 * height)), (int(x1 * width), int(y1 * height)), colour, -1)
    return frame


def serve(port: int, fps: float = 15.0, jpeg: bytes | None = None) -> None:
    if jpeg is None:
        ok, encoded = cv2.imencode(".jpg", picture(), [cv2.IMWRITE_JPEG_QUALITY, 92])
        jpeg = encoded.tobytes()
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", port))
    server.listen(4)

    def stream(conn):
        number = 0
        try:
            while True:
                conn.sendall(HEADER.pack(MAGIC, number, time.time(), len(jpeg)) + jpeg)
                number += 1
                time.sleep(1 / fps)
        except OSError:
            conn.close()

    while True:
        conn, _ = server.accept()
        threading.Thread(target=stream, args=(conn,), daemon=True).start()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", type=int, default=50113)
    parser.add_argument("--picture", help="serve this JPEG instead of the blocks")
    args = parser.parse_args()
    serve(args.port, jpeg=open(args.picture, "rb").read() if args.picture else None)
