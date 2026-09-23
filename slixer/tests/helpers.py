"""Small things the tests share. Kept out of conftest.py, which pytest imports in its own way: importing
conftest by name from a test file runs it a second time."""

from __future__ import annotations

import socket
import time

REAL_ARM_PORT = 50101


def free_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    assert port != REAL_ARM_PORT
    return port


def wait_for(condition, seconds: float, why: str) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if condition():
            return
        time.sleep(0.02)
    raise AssertionError(why)
