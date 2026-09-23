"""Receives camera frames from the Raspberry Pi beside the arm.

Frames arrive in a background thread which keeps only the newest one, so a control loop always sees the
freshest view rather than working through a backlog. Each frame carries the time it was captured, so you
can tell how far behind you are. Reconnects by itself if the Pi restarts.

    uv run pc/camera_client.py PI_ADDRESS            # report frame rate and age
    uv run pc/camera_client.py PI_ADDRESS --show     # and display the video

In your own code:

    camera = ArmCamera("192.168.1.50")
    camera.start()
    frame, age = camera.read()
"""

from __future__ import annotations

import argparse
import http.server
import socket
import struct
import threading
import time

import cv2
import numpy as np

HEADER = struct.Struct("<4sIdI")
MAGIC = b"S101"
DEFAULT_PORT = 50102


class ArmCamera:
    """The camera at the arm, as seen from the PC."""

    def __init__(self, host: str, port: int = DEFAULT_PORT, decode: bool = True):
        self.host, self.port = host, port
        self.decode = decode  # off when only the JPEG is wanted: saves unpacking every frame for nothing
        self._lock = threading.Lock()
        self._frame: np.ndarray | None = None
        self._jpeg: bytes | None = None
        self._captured = 0.0  # the Pi's clock: only meaningful if the two clocks agree
        self._arrived = 0.0   # this machine's clock, which is what staleness is judged by
        self._running = False
        self._thread: threading.Thread | None = None
        self.frames_received = 0
        self.connected = False

    def start(self) -> "ArmCamera":
        self._running = True
        self._thread = threading.Thread(target=self._receive_forever, daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2)

    def read_jpeg(self) -> tuple[bytes | None, float]:
        """The newest frame still in the form the camera sent it, and how long ago it arrived.

        Showing this directly costs nothing and loses nothing. Decoding a frame and encoding it again
        puts a second generation of JPEG on top of the first, which is what makes a good camera look
        blocky on screen.
        """
        with self._lock:
            if self._jpeg is None:
                return None, float("inf")
            return self._jpeg, time.monotonic() - self._arrived

    def read(self) -> tuple[np.ndarray | None, float]:
        """The newest frame and how long ago it arrived, in seconds, or (None, inf) if none has.

        Measured on this machine's clock, so it doesn't depend on the Pi's clock being right (a Pi has no
        battery-backed clock and can be seconds out until it syncs).
        """
        with self._lock:
            if self._frame is None:
                return None, float("inf")
            return self._frame, time.monotonic() - self._arrived

    def _receive_forever(self) -> None:
        while self._running:
            try:
                with socket.create_connection((self.host, self.port), timeout=5) as link:
                    link.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    self.connected = True
                    self._read_frames(link)
            except OSError:
                pass
            self.connected = False
            if self._running:
                time.sleep(1.0)  # Pi not up yet, or the link dropped: try again

    def _read_frames(self, link: socket.socket) -> None:
        buffer = bytearray()
        while self._running:
            while len(buffer) < HEADER.size:
                if not self._fill(link, buffer):
                    return
            magic, _number, captured, length = HEADER.unpack(bytes(buffer[: HEADER.size]))
            if magic != MAGIC:  # out of step: resynchronise on the next magic
                start = bytes(buffer).find(MAGIC, 1)
                del buffer[: start if start > 0 else len(buffer)]
                continue
            while len(buffer) < HEADER.size + length:
                if not self._fill(link, buffer):
                    return
            jpeg = bytes(buffer[HEADER.size : HEADER.size + length])
            del buffer[: HEADER.size + length]
            frame = None
            if self.decode:
                frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    continue  # a truncated frame: skip it rather than pass rubbish to a control loop
            with self._lock:
                self._jpeg, self._captured, self._arrived = jpeg, captured, time.monotonic()
                if frame is not None:
                    self._frame = frame
            self.frames_received += 1

    def _fill(self, link: socket.socket, buffer: bytearray) -> bool:
        try:
            chunk = link.recv(65536)
        except OSError:
            return False
        if not chunk:
            return False
        buffer.extend(chunk)
        return True


class BrowserView:
    """Shows frames in a web browser, for OpenCV builds with no window support (lerobot's is headless).

    Serves only on this machine's localhost, so the camera isn't exposed to the rest of the network.
    With WSL's mirrored networking a Windows browser reaches it at the same address.
    """

    def __init__(self, port: int = 8080):
        self.port = port
        self._jpeg: bytes | None = None
        self._lock = threading.Lock()
        view = self

        class Handler(http.server.BaseHTTPRequestHandler):
            def log_message(self, *args):  # keep the terminal for the control loop's own output
                pass

            def do_GET(self):
                if self.path != "/stream":
                    # Native size: stretching a small frame to the window magnifies every compression artifact.
                    page = (b'<html><body style="margin:0;background:#111;display:flex;justify-content:center">'
                            b'<img src="/stream" style="max-width:100%;image-rendering:auto"></body></html>')
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.send_header("Content-Length", str(len(page)))
                    self.end_headers()
                    self.wfile.write(page)
                    return
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    while True:
                        with view._lock:
                            jpeg = view._jpeg
                        if jpeg is not None:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                            self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                            self.wfile.write(jpeg + b"\r\n")
                        time.sleep(0.05)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        for candidate in range(port, port + 10):  # take the first free port
            try:
                self._server = http.server.ThreadingHTTPServer(("127.0.0.1", candidate), Handler)
                self.port = candidate
                break
            except OSError:
                continue
        else:
            raise OSError(f"no free port for the browser view between {port} and {port + 9}")
        self._server.daemon_threads = True
        threading.Thread(target=self._server.serve_forever, daemon=True).start()

    def show(self, frame: np.ndarray) -> None:
        ok, jpeg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 95])  # local only: be generous
        if ok:
            with self._lock:
                self._jpeg = jpeg.tobytes()


class Display:
    """A window if this OpenCV can make one, otherwise the browser view. Never takes the caller down."""

    def __init__(self, title: str = "arm camera", browser_port: int = 8080):
        self._title = title
        self._browser_port = browser_port
        self._browser: BrowserView | None = None

    def show(self, frame: np.ndarray) -> bool:
        """Displays a frame. Returns False once the user asks to quit (q in the window)."""
        if self._browser is None:
            try:
                cv2.imshow(self._title, frame)
                return (cv2.waitKey(1) & 0xFF) != ord("q")
            except cv2.error:
                self._browser = BrowserView(self._browser_port)
                print(f"(this OpenCV has no window support: watch the camera at http://localhost:{self._browser.port})")
        self._browser.show(frame)
        return True


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("host", help="the Pi's address or hostname")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--show", action="store_true", help="display the video")
    parser.add_argument("--seconds", type=float, default=0.0, help="stop after this long (0 = run forever)")
    args = parser.parse_args()

    camera = ArmCamera(args.host, args.port).start()
    display = Display() if args.show else None
    started = last_report = time.monotonic()
    counted = 0
    try:
        while args.seconds <= 0 or time.monotonic() - started < args.seconds:
            frame, age = camera.read()
            if frame is not None:
                counted = camera.frames_received
                if display is not None and not display.show(frame):
                    break
            now = time.monotonic()
            if now - last_report >= 2.0:
                size = "-" if frame is None else f"{frame.shape[1]}x{frame.shape[0]}"
                status = "connected" if camera.connected else "waiting for the Pi"
                print(f"{status}: {counted} frames, {size}, newest arrived {age * 1000:.0f} ms ago")
                last_report = now
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        camera.stop()


if __name__ == "__main__":
    main()
