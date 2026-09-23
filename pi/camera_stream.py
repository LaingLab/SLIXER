# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy", "opencv-python-headless"]
# ///
"""Streams camera frames from a Raspberry Pi next to the arm to a PC that does the vision work.

Run this on the Pi; the PC connects to it. Frames are JPEG, each stamped with the time it was captured so
the PC can tell how old a frame is and line it up with the arm's reported position. When the network or
the PC falls behind, frames are dropped rather than queued, because for control a fresh frame matters far
more than a complete sequence.

    uv run pi/camera_stream.py                       # USB camera at /dev/video0, 640x480, 30 fps
    uv run pi/camera_stream.py --width 1280 --height 720 --fps 15
    uv run pi/camera_stream.py --test-pattern        # no camera needed, for checking the link

`uv run` gives it only the two packages listed at the top of this file, not the rest of Slixer. Plain
`python3 pi/camera_stream.py` works too, with the Pi's own OpenCV (sudo apt install python3-opencv).
"""

from __future__ import annotations

import argparse
import shutil
import socket
import struct
import subprocess
import time

import cv2
import numpy as np

HEADER = struct.Struct("<4sIdI")  # magic, frame number, capture time, jpeg length
MAGIC = b"S101"
DEFAULT_PORT = 50102


def hold_frame_rate(device: str) -> None:
    """Stops the camera trading frame rate for exposure time in dim light, which is the usual reason a
    webcam quietly drops from 30 fps to 8 or 15. Frames may get darker; add light rather than let it slow."""
    if shutil.which("v4l2-ctl") is None:
        print("camera: install v4l-utils (sudo apt install v4l-utils) so this can hold 30 fps in dim light")
        return
    for control in ("exposure_dynamic_framerate", "exposure_auto_priority"):  # newer and older kernel names
        result = subprocess.run(["v4l2-ctl", "-d", device, "-c", f"{control}=0"], capture_output=True, text=True)
        if result.returncode == 0:
            print(f"camera: {control}=0, so it holds its frame rate in dim light")
            return
    print("camera: no exposure-priority control; it may slow down in dim light. Use --exposure to fix the")
    print("        frame rate, or add light. Its exposure controls:")
    listing = subprocess.run(["v4l2-ctl", "-d", device, "--list-ctrls"], capture_output=True, text=True).stdout
    for line in listing.splitlines():
        if "expo" in line.lower() or "gain" in line.lower():
            print("        " + line.strip())


def set_manual_exposure(device: str, exposure: int) -> None:
    """Manual exposure fixes the frame rate: the camera can no longer lengthen each frame to gather light.
    Units are 100 microseconds, so 330 or less leaves room for 30 fps. Lower is darker but sharper."""
    if shutil.which("v4l2-ctl") is None:
        raise SystemExit("--exposure needs v4l-utils: sudo apt install v4l-utils")
    for control in ("auto_exposure=1", f"exposure_time_absolute={exposure}"):
        result = subprocess.run(["v4l2-ctl", "-d", device, "-c", control], capture_output=True, text=True)
        if result.returncode != 0:
            print(f"camera: could not set {control}: {result.stderr.strip()}")
            return
    # Webcams step between fixed rates, and each frame needs readout time on top of its exposure, so
    # the usable exposure for a rate is a little under its frame period (about 280 for 30 fps).
    rate = 30 if exposure <= 280 else 15 if exposure <= 610 else 7.5
    print(f"camera: manual exposure {exposure / 10:.1f} ms, expect about {rate:g} fps"
          + ("" if rate == 30 else " (about 280 or less for 30 fps; add light or --gain to brighten)"))


def set_gain(device: str, gain: int) -> None:
    """Amplifies the sensor signal: brighter at a short exposure, at the cost of some noise."""
    if shutil.which("v4l2-ctl") is None:
        raise SystemExit("--gain needs v4l-utils: sudo apt install v4l-utils")
    result = subprocess.run(["v4l2-ctl", "-d", device, "-c", f"gain={gain}"], capture_output=True, text=True)
    print(f"camera: gain {gain}" if result.returncode == 0 else f"camera: could not set gain: {result.stderr.strip()}")


def describe_mode(camera: "cv2.VideoCapture") -> str:
    code = int(camera.get(cv2.CAP_PROP_FOURCC))
    fourcc = "".join(chr((code >> (8 * i)) & 0xFF) for i in range(4))
    width, height = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH)), int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
    return f"{width}x{height} {fourcc} at {camera.get(cv2.CAP_PROP_FPS):.0f} fps"


def raw_jpeg(frame) -> bytes | None:
    """The camera's own JPEG, when OpenCV hands back the undecoded buffer; otherwise None."""
    if frame is None or frame.dtype != np.uint8 or (frame.ndim == 2 and frame.shape[0] != 1) or frame.ndim > 2:
        return None
    data = frame.tobytes()
    if not data.startswith(b"\xff\xd8"):
        return None
    end = data.rfind(b"\xff\xd9")  # some drivers pad the buffer after the image
    return data[: end + 2] if end > 0 else None


def try_passthrough(camera: "cv2.VideoCapture") -> bool:
    """Asks OpenCV for the camera's compressed frames as-is, skipping a decode and re-encode that would
    blur detail and cost CPU. Falls back if this camera or driver can't provide them."""
    camera.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    for _ in range(5):  # the first frames after a mode change can be empty
        ok, frame = camera.read()
        if ok and raw_jpeg(frame) is not None:
            return True
    camera.set(cv2.CAP_PROP_CONVERT_RGB, 1)
    return False


def open_camera(args) -> "cv2.VideoCapture | None":
    if args.test_pattern:
        return None
    camera = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))  # let the camera do the compressing
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    camera.set(cv2.CAP_PROP_FPS, args.fps)
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # always hand us the newest frame
    if not camera.isOpened():
        raise SystemExit(f"could not open {args.device}: is the camera plugged in?")
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))  # some drivers only honour it after the size
    print(f"camera: {describe_mode(camera)} (what the camera agreed to; YUYV modes are often slow)")
    hold_frame_rate(args.device)
    if args.exposure:
        set_manual_exposure(args.device, args.exposure)
    if args.gain is not None:
        set_gain(args.device, args.gain)
    args.passthrough = try_passthrough(camera)
    print("camera: forwarding its own JPEG frames untouched" if args.passthrough
          else f"camera: re-encoding frames as JPEG quality {args.quality}")
    return camera


def test_frame(width: int, height: int, count: int) -> np.ndarray:
    """A moving pattern, so the link can be checked without a camera attached."""
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    x = int((count * 7) % max(1, width - 40))
    frame[:, :, 0] = 40
    cv2.rectangle(frame, (x, height // 3), (x + 40, height // 3 + 40), (0, 255, 0), -1)
    cv2.putText(frame, f"test {count}", (10, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    return frame


def serve(args) -> None:
    camera = open_camera(args)
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(("0.0.0.0", args.port))
    listener.listen(1)
    print(f"camera ready; waiting for a PC on port {args.port}")

    frames = 0
    while True:
        client, address = listener.accept()
        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        client.setblocking(False)
        print(f"streaming to {address[0]}")
        sent = dropped = captured_count = bad = 0
        reported = time.monotonic()
        try:
            while True:
                if camera is None:
                    ok, frame = True, test_frame(args.width, args.height, frames)
                    time.sleep(1.0 / args.fps)
                else:
                    ok, frame = camera.read()
                if not ok:
                    print("camera read failed; reopening")
                    camera.release()
                    camera = open_camera(args)
                    continue
                frames += 1
                captured_count += 1
                captured = time.time()
                if getattr(args, "passthrough", False):
                    jpeg = raw_jpeg(frame)
                    if jpeg is None:  # a partial frame from the camera: skip it, never re-encode compressed data
                        bad += 1
                        continue
                else:
                    try:
                        ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), args.quality])
                    except cv2.error:
                        ok = False
                    if not ok:
                        bad += 1
                        continue
                    jpeg = encoded.tobytes()
                payload = HEADER.pack(MAGIC, frames, captured, len(jpeg)) + jpeg
                try:
                    client.sendall(payload)
                    sent += 1
                except BlockingIOError:
                    dropped += 1  # PC or network behind: skip this frame, keep the next one fresh
                except (BrokenPipeError, ConnectionResetError, OSError):
                    raise StopIteration
                now = time.monotonic()
                if now - reported >= 5:
                    elapsed = now - reported
                    note = f", {bad} bad frames skipped" if bad else ""
                    print(f"{captured_count / elapsed:.1f} fps from the camera, {sent / elapsed:.1f} sent, {dropped} dropped{note}")
                    sent = dropped = captured_count = bad = 0
                    reported = now
        except StopIteration:
            print("PC disconnected; waiting again")
        finally:
            client.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--quality", type=int, default=90,
                        help="JPEG quality 1-100, used only when the camera's own frames can't be forwarded")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--test-pattern", action="store_true", help="stream a moving pattern, no camera")
    parser.add_argument("--list-modes", action="store_true", help="show what the camera supports, then exit")
    parser.add_argument("--exposure", type=int, default=0,
                        help="manual exposure in 100 us units; about 280 or less for 30 fps, 610 or less for 15")
    parser.add_argument("--gain", type=int, default=None,
                        help="sensor gain: brightens a short exposure at some cost in noise (range: see startup)")
    args = parser.parse_args()
    if args.list_modes:
        if shutil.which("v4l2-ctl") is None:
            raise SystemExit("install v4l-utils first: sudo apt install v4l-utils")
        subprocess.run(["v4l2-ctl", "-d", args.device, "--list-formats-ext"])
        return
    serve(args)


if __name__ == "__main__":
    main()
