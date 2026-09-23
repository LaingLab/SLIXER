"""Regression tests for what an independent review of Slixer found on 2026-09-22.

Each test is one finding, written the way the reviewer demonstrated it, so that if the bug ever comes back
the test that fails says exactly what it was.
"""

from __future__ import annotations

import math
import struct
import time

import numpy as np
import pytest

from fake_follower import FakeFollower, K_MAX_RES, POSE_FORMAT, POSE_MAGIC
from helpers import free_udp_port, wait_for
from kinematics import Chain
from mapping import JointMap, Mapping
from program import Program, Runner, Step
from session import MAX_DEGREES_PER_SECOND, Session

# The arm's own recorded rest pose from 2026-09-22: lift and wrist sit past the 3D model's own limits.
REST = [-3.4, -104.5, 95.3, -99.2, -1.8, 2.9]
# An imitation arm whose calibrated range contains that rest pose, as the real one's does.
WIDE = [(900, 3200), (500, 3600), (700, 3500), (600, 3500), (100, 3995), (2000, 3400)]


def spy(session):
    sent, real = [], session.arm.send_pose

    def record(degrees, percent):
        sent.append(list(degrees) + [percent])
        real(degrees, percent)

    session.arm.send_pose = record
    return sent


def arm_at(pose, ranges=WIDE, priority=True):
    """An imitation follower already standing at `pose` (the arm's own numbers)."""
    pc_port, arm_port = free_udp_port(), free_udp_port()
    arm = FakeFollower(listen_port=arm_port, report_to=("127.0.0.1", pc_port), ranges=list(ranges),
                       priority=priority)
    for j, degrees in enumerate(pose[:5]):
        low, high = ranges[j]
        arm.positions[j] = arm.goals[j] = (low + high) / 2 + degrees * K_MAX_RES / 360.0
    arm.start()
    session = Session(mapping=Mapping(), arm_port=pc_port).start()
    wait_for(lambda: session.hearing_follower, 3, "no arm")
    return arm, session


def shut(arm, session):
    session.stop()
    session.arm._socket.close()
    arm.stop()


# 1 ---------------------------------------------------------------------------------------------------------

def test_1_lining_up_is_refused_while_driving(rig):
    arm, session = rig
    session.set_mode("drive")
    session.set_target_arm([40, 0, 0, 0, 0, 50])
    time.sleep(0.8)
    problem = session.remap(lambda: session.mapping.joints.__setitem__("shoulder_pan", JointMap(sign=-1)))
    assert "Drive" in problem
    assert session.mapping.joints["shoulder_pan"].sign == 1


def test_1_lining_up_elsewhere_keeps_what_the_arm_would_be_told(rig):
    arm, session = rig
    session.set_mode("plan")
    session.set_target_arm([40, -10, 20, 5, 0, 50])
    wait_for(lambda: abs(session.current_arm_pose()[0] - 40) < 0.1, 3, "never arrived")
    before = session._to_arm_list(session.commanded)
    session.remap(lambda: session.mapping.joints.__setitem__("shoulder_pan", JointMap(sign=-1, offset_deg=12)))
    assert session._to_arm_list(session.commanded) == pytest.approx(before)


# 2 ---------------------------------------------------------------------------------------------------------

# The imitation arm's calibrated range, as model radians under the default mapping.
WIDE_LIMITS = [tuple(math.radians(v) for v in sorted(((lo - (lo + hi) / 2) * 360 / K_MAX_RES,
                                                        (hi - (lo + hi) / 2) * 360 / K_MAX_RES)))
               for lo, hi in WIDE[:5]] + [(0.0, math.radians(90))]


def test_2_dragging_never_changes_the_arms_shape():
    """From the real rest pose, small drags in any direction bend the arm; they never refold it."""
    chain = Chain.load()
    rest = Mapping().to_model(REST[:5], REST[5])
    rng = np.random.default_rng(11)
    tip = chain.tip_position(rest)
    worst = 0.0
    for _ in range(200):
        aim = tip + rng.normal(0, 0.02, 3)  # about 2 cm, as a drag between two updates
        angles, _, _ = chain.solve_ik(aim, rest, restarts=0, limits=WIDE_LIMITS, max_change=math.radians(15))
        worst = max(worst, max(abs(math.degrees(a - b)) for a, b in zip(angles[:5], rest[:5])))
    assert worst < 45, f"a 2 cm drag asked a joint to swing {worst:.0f} degrees"


def test_2_reaching_prefers_the_nearest_shape_and_drive_refuses_a_big_swing(rig):
    arm, session = rig
    chain = session.chain
    session.set_mode("drive")
    far = chain.tip_position(Mapping().to_model([80, -60, 70, 60, 0], 50))  # somewhere else entirely
    _, _, _, problem = session.solve_to(far.tolist(), reach=True)
    assert "Plan" in problem


# 3 ---------------------------------------------------------------------------------------------------------

def test_3_stop_holds_even_with_a_leader_waiting():
    arm, session = arm_at([0, 0, 0, 0, 0, 50], priority=True)
    try:
        stop_leader = arm.run_leader([0.0] * 5)
        time.sleep(0.5)
        session.set_mode("drive")
        session.set_target_arm([45, 0, 0, 0, 0, 50])
        time.sleep(1.5)
        session.stop_everything()
        time.sleep(1.5)
        assert arm.degrees()[0] == pytest.approx(45, abs=1.5)  # held, not handed to the leader at 0
        session.release()  # letting go is its own, deliberate step
        wait_for(lambda: abs(arm.degrees()[0]) < 1.5, 4, "the leader never got the arm back")
        stop_leader.set()
    finally:
        shut(arm, session)


# 4 ---------------------------------------------------------------------------------------------------------

def _leader_with_wifi_copies(arm, seconds, gap_every=1.0, gap=0.3):
    """A leader on the radio, also sending copies over Wi-Fi that drop out now and then, as Wi-Fi does."""
    started = time.monotonic()
    seq = 0
    while time.monotonic() - started < seconds:
        seq = (seq + 1) & 0xFF
        packet = struct.pack(POSE_FORMAT, POSE_MAGIC, seq, 0, 0, 0, 0, 0, 0)
        arm.deliver(packet, via_wifi=False)
        since = (time.monotonic() - started) % gap_every
        if since > gap:  # the Wi-Fi copy, except during a dropout
            arm.deliver(packet, via_wifi=True)
        time.sleep(0.02)


def test_4_a_leaders_own_wifi_copies_never_make_teleop_stutter():
    arm, session = arm_at([0, 0, 0, 0, 0, 50], priority=True)
    try:
        _leader_with_wifi_copies(arm, 1.0)
        glides = arm.glides
        _leader_with_wifi_copies(arm, 6.0)  # six Wi-Fi dropouts of 300 ms
        assert arm.glides == glides  # no source flips, so no glides
    finally:
        shut(arm, session)


# 5 ---------------------------------------------------------------------------------------------------------

def test_5_the_control_loop_survives_a_network_error(rig):
    arm, session = rig
    session.set_mode("drive")
    calls = {"n": 0}
    real = session.arm.send_pose

    def flaky(degrees, percent):
        calls["n"] += 1
        if calls["n"] == 5:
            raise OSError("Network is unreachable")
        real(degrees, percent)

    session.arm.send_pose = flaky
    time.sleep(0.5)
    assert session._thread.is_alive() and calls["n"] > 10


@pytest.mark.parametrize("bad", [[0.1] * 5, [0.1, 0.1, float("nan"), 0.1, 0.1, 0.1]])
def test_5_malformed_targets_are_refused(rig, bad):
    arm, session = rig
    session.set_mode("plan")
    with pytest.raises(ValueError):
        session.set_target(bad)


# 6 ---------------------------------------------------------------------------------------------------------

def test_6_entering_drive_at_the_real_rest_pose_moves_nothing():
    arm, session = arm_at(REST)
    try:
        sent = spy(session)
        session.set_mode("drive")
        time.sleep(0.4)
        first = sent[0]
        assert max(abs(a - b) for a, b in zip(first[:5], REST[:5])) < 0.2  # not clamped to -100 / -95
    finally:
        shut(arm, session)


def test_6_a_program_reaches_the_real_rest_pose():
    arm, session = arm_at([0, -60, 60, -40, 0, 50])
    try:
        session.set_mode("drive")
        session.run_program(Program(steps=[Step(kind="move", arm=list(REST), speed=60)]))
        wait_for(lambda: session.runner is None, 8, "never finished")
        time.sleep(0.5)
        assert arm.degrees() == pytest.approx(REST[:5], abs=0.4)
    finally:
        shut(arm, session)


# 7 ---------------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("speed", [90, 120, 180])
def test_7_fast_moves_arrive_before_the_next_step(speed):
    """Plays a program through the same slew limit Session applies, on a clock that isn't real."""
    program = Program(steps=[Step(kind="move", arm=[90, 0, 0, 0, 0, 50], speed=speed),
                             Step(kind="gripper", percent=0)])
    mapping = Mapping()
    runner = Runner(program.resolved(mapping), start_pose=mapping.to_model([0] * 5, 50))
    runner.max_speed = MAX_DEGREES_PER_SECOND
    commanded = mapping.to_model([0] * 5, 50)
    ceiling = math.radians(MAX_DEGREES_PER_SECOND) / 50
    t = 0.0
    while runner.index == 0 and t < 10:
        wanted = runner.tick(t, commanded)
        commanded = [c + max(-ceiling, min(ceiling, w - c)) for c, w in zip(commanded, wanted)]
        t += 0.02
    assert math.degrees(commanded[0]) == pytest.approx(90, abs=0.6)


# 8 ---------------------------------------------------------------------------------------------------------

def test_8_saving_never_silently_replaces_a_program(data_dir):
    from fastapi.testclient import TestClient
    from server import create_app

    with TestClient(create_app(arm_port=free_udp_port())) as client:
        with client.websocket_connect("/ws") as socket:
            def reply(kind):
                while True:
                    message = socket.receive_json()
                    if message["type"] == kind:
                        return message
            first = {"name": "precious", "loop": False, "steps": [{"kind": "wait", "seconds": 1}]}
            socket.send_json({"do": "program.save", "program": first})
            reply("programs")
            second = {"name": "precious", "loop": False, "steps": [{"kind": "wait", "seconds": 9}]}
            socket.send_json({"do": "program.save", "program": second})
            assert reply("program.exists")["name"] == "precious"
            assert Program.load("precious").steps[0].seconds == 1  # untouched
            socket.send_json({"do": "program.save", "program": second, "replace": True})
            reply("programs")
            assert Program.load("precious").steps[0].seconds == 9


# 12 --------------------------------------------------------------------------------------------------------

def test_12_stop_answers_at_once_even_behind_a_slow_request(data_dir, monkeypatch):
    from fastapi.testclient import TestClient
    from dataset import Collector
    from server import create_app

    monkeypatch.setattr(Collector, "capture", lambda self, automatic=False: time.sleep(1.0) or "captured")
    with TestClient(create_app(arm_port=free_udp_port())) as client:
        with client.websocket_connect("/ws") as socket:
            # A camera that doesn't exist, which the old code waited on, and a request that takes a second,
            # then STOP straight after.
            socket.send_json({"do": "camera", "host": "10.255.255.1"})
            socket.send_json({"do": "dataset.capture"})
            started = time.monotonic()
            socket.send_json({"do": "stop"})
            while True:
                message = socket.receive_json()
                if message["type"] == "note" and message["text"].startswith("stopped"):
                    break
            assert time.monotonic() - started < 0.5
