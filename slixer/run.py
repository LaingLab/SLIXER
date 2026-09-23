"""Starts Slixer.

    uv run slixer/run.py                          # http://localhost:8000, camera as last used
    uv run slixer/run.py --camera 192.168.1.50    # a different camera (and remember it)
    uv run slixer/run.py --host 0.0.0.0           # let the rest of the lab reach it (by IP address,
                                                  # or this PC's name; --allow-host adds another name)
    uv run slixer/run.py --arm-port 50199         # listen somewhere other than the real arm (testing)
    uv run slixer/run.py --data /tmp/try          # keep programs, calibration etc. somewhere else
"""

from __future__ import annotations

import argparse
import ipaddress
import os
import socket
import sys
from pathlib import Path

import uvicorn

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "pc"))

from so101_link import PORT  # noqa: E402
from version import VERSION  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1", help="0.0.0.0 to serve the whole network")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--camera", default=None, help="address of the Pi streaming the camera")
    parser.add_argument("--camera-port", type=int, default=None)
    parser.add_argument("--arm-port", type=int, default=PORT,
                        help=f"UDP port the arm reports on (default {PORT}; anything else can't reach the real arm)")
    parser.add_argument("--data", default=None, help="folder for programs, calibration and settings")
    parser.add_argument("--allow-host", action="append", default=[], metavar="NAME",
                        help="another name this PC is reached by, with --host 0.0.0.0 (IP addresses always work)")
    parser.add_argument("--version", action="version", version=f"Slixer {VERSION}")
    args = parser.parse_args()

    # Before anything that remembers things is imported: each works out where its file lives on import.
    if args.data:
        Path(args.data).mkdir(parents=True, exist_ok=True)
        os.environ["SLIXER_DATA"] = str(Path(args.data).resolve())
    import settings  # noqa: PLC0415
    from server import create_app  # noqa: PLC0415

    remembered = settings.load()
    camera = args.camera or remembered["camera_host"]
    camera_port = args.camera_port or remembered["camera_port"]
    if args.camera:
        settings.save(camera_host=args.camera, camera_port=camera_port)

    if args.host not in ("127.0.0.1", "localhost"):
        print("serving to the whole network: anyone who can reach this port can drive the arm")
    if args.arm_port != PORT:
        print(f"listening for an arm on UDP {args.arm_port}, not the real arm's {PORT}")

    where = "localhost" if args.host in ("0.0.0.0", "127.0.0.1") else args.host
    print(f"Slixer {VERSION} at http://{where}:{args.port}" + (f"  (camera {camera}:{camera_port})" if camera else "  (no camera set)"))
    uvicorn.run(
        create_app(camera, camera_port, arm_port=args.arm_port, hosts=allowed_hosts(args.host, args.allow_host)),
        host=args.host,
        port=args.port,
        log_level="warning",  # the page reports what matters; uvicorn's per-request lines just scroll
        # A page that vanishes without closing -- a laptop put to sleep, a network gone -- is noticed within
        # about ten seconds, and if it was the last one, the arm is let go. uvicorn's own default is forty.
        ws_ping_interval=5.0,
        ws_ping_timeout=5.0,
        # Ctrl-C should stop it. By default uvicorn waits for every connection to finish first, and the
        # camera stream never finishes -- so with a tab open it would wait for ever, holding the arm's
        # port. Two seconds' grace, then it goes.
        timeout_graceful_shutdown=2,
    )


def allowed_hosts(bind: str, extra: list[str]) -> set[str]:
    """The names a browser may use for this server, besides IP addresses (which always work).

    Served to this PC only, that's just "localhost". Served to the network, it's this PC's own names too,
    plus any given with --allow-host. Anything else is refused: see server.SameOrigin.
    """
    names = {"localhost"}
    try:
        loopback = ipaddress.ip_address(bind).is_loopback
    except ValueError:
        loopback = bind == "localhost"
        if not loopback:
            names.add(bind.lower())  # served under a name: that name, then
    if not loopback:
        here = socket.gethostname().lower()
        names |= {here, f"{here}.local", socket.getfqdn().lower()}
    return names | {name.strip().lower() for name in extra if name.strip()}


if __name__ == "__main__":
    main()
