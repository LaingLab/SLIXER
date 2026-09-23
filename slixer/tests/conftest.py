"""Shared set-up for Slixer's tests.

Two things every test relies on, arranged here before anything else is imported:

  * SLIXER_DATA points at a throwaway folder, so no test can touch your programs, your calibration or your
    camera address. It has to be set before the modules are imported, because each works out where its
    file lives on import.
  * Arms and cameras are imitations on ports picked fresh for each test. The real arm reports on UDP 50101;
    nothing here ever listens there, so nothing here can move it.

    python -m pytest slixer/tests -q
"""

from __future__ import annotations

import os
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
# One throwaway folder for the whole run, even if this file is somehow imported twice.
if not os.environ.get("SLIXER_TEST_DATA"):
    os.environ["SLIXER_TEST_DATA"] = tempfile.mkdtemp(prefix="slixer-tests-")
_DATA = os.environ["SLIXER_TEST_DATA"]
os.environ["SLIXER_DATA"] = _DATA

sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parent.parent / "pc"))

import pytest  # noqa: E402

from helpers import free_udp_port, wait_for  # noqa: E402


@pytest.fixture
def data_dir() -> Path:
    return Path(_DATA)


@pytest.fixture
def rig():
    """An imitation follower and a Session listening for it -- a complete arm, with no hardware."""
    from fake_follower import FakeFollower
    from mapping import Mapping
    from session import Session

    pc_port, arm_port = free_udp_port(), free_udp_port()
    arm = FakeFollower(listen_port=arm_port, report_to=("127.0.0.1", pc_port)).start()
    session = Session(mapping=Mapping(), arm_port=pc_port).start()
    wait_for(lambda: session.hearing_follower, 3.0, "the imitation arm never reported")
    yield arm, session
    session.stop()
    session.arm._socket.close()
    arm.stop()
