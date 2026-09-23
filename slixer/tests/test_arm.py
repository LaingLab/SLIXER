"""Slixer against an imitation of the real follower: modes, safety rules, and two senders sharing an arm.

Every arm here is `fake_follower.FakeFollower`, which runs the follower firmware's own pose filter, freeze
and glide logic, on a port the real arm never uses.
"""

from __future__ import annotations

import math
import struct
import time

import pytest

from helpers import free_udp_port, wait_for
from fake_follower import FakeFollower, K_MAX_RES
from mapping import JointMap, Mapping
from program import Program, Step
from session import Session
from so101_link import AHEAD_BY, HELLO_MAGIC, POSE_FORMAT, Arm


def pan_degrees(arm: FakeFollower, sample) -> float:
    low, high = arm.ranges[0]
    return (sample.positions[0] - (low + high) / 2) * 360 / K_MAX_RES


def sent_by(session: Session) -> list:
    """Records everything the session sends the arm, and still sends it."""
    sent, real = [], session.arm.send_pose

    def spy(degrees, percent):
        sent.append(list(degrees) + [percent])
        real(degrees, percent)

    session.arm.send_pose = spy
    return sent


# ---- the three modes -------------------------------------------------------------------------------------

def test_watching_sends_nothing_and_refuses_to_move(rig):
    arm, session = rig
    sent = sent_by(session)
    pose, problem = session.set_target([0.5] * 6)
    assert problem and "watching" in problem
    time.sleep(0.3)
    assert sent == []


def test_planning_moves_a_virtual_arm_and_sends_nothing(rig):
    arm, session = rig
    sent = sent_by(session)
    assert session.set_mode("plan") == ""
    session.set_target_arm([30, -20, 40, 10, 0, 80])
    wait_for(lambda: abs(session.current_arm_pose()[0] - 30) < 0.1, 3, "the virtual arm never got there")
    session.run_program(Program(steps=[Step(kind="move", arm=[-30, 0, 0, 0, 0, 50], speed=90)]))
    time.sleep(1.2)
    assert sent == []
    assert arm.degrees() == pytest.approx([0.0] * 5, abs=0.05)  # the real one never moved


def test_drive_needs_the_follower(data_dir):
    session = Session(mapping=Mapping(), arm_port=free_udp_port())  # nobody on this port
    try:
        assert "no arm to drive" in session.set_mode("drive")
        assert session.mode == "watch"
    finally:
        session.arm._socket.close()


def test_switching_to_drive_never_moves_the_arm(rig):
    arm, session = rig
    sent = sent_by(session)
    assert session.set_mode("drive") == ""
    time.sleep(0.5)
    reported = session.state.degrees[:5]
    assert all(max(abs(a - b) for a, b in zip(s[:5], reported)) < 0.1 for s in sent)


def test_commands_walk_rather_than_jump(rig):
    arm, session = rig
    session.set_mode("drive")
    time.sleep(0.2)
    start = time.monotonic()
    session.set_target_arm([80, 0, 0, 0, 0, 50])  # inside the imitation arm's calibrated +/-92 degrees
    wait_for(lambda: session._to_arm_list(session.commanded)[0] > 79.5, 5, "never arrived")
    took = time.monotonic() - start
    assert took == pytest.approx(80 / 90.0, rel=0.15)  # 90 degrees a second, not all at once


def test_stop_ends_the_program_and_holds_the_arm(rig):
    arm, session = rig
    session.set_mode("drive")
    session.run_program(Program(steps=[Step(kind="move", arm=[40, 0, 0, 0, 0, 50], speed=20)]))
    time.sleep(0.5)
    session.stop_everything()
    stopped_at = arm.degrees()[0]
    assert session.mode == "drive" and session.runner is None  # still sending: holding, not letting go
    time.sleep(1.0)
    assert arm.degrees()[0] == pytest.approx(stopped_at, abs=1.0)


def test_a_program_drives_the_arm_where_it_was_told(rig):
    arm, session = rig
    session.set_mode("drive")
    time.sleep(0.3)
    arm.history.clear()
    session.run_program(Program(steps=[Step(kind="move", arm=[35, -10, 20, 0, 0, 50], speed=60)]))
    wait_for(lambda: session.runner is None, 5, "the program never finished")
    time.sleep(0.4)
    assert arm.degrees() == pytest.approx([35, -10, 20, 0, 0], abs=0.3)


def test_recalibrating_mid_session_changes_nothing_the_arm_is_told(rig):
    arm, session = rig
    session.mapping.joints["shoulder_pan"] = JointMap(sign=-1, offset_deg=12.0)
    session.mapping.joints["wrist_flex"] = JointMap(sign=-1, offset_deg=0.0)
    session.set_mode("drive")
    time.sleep(0.2)
    session.run_program(Program(steps=[Step(kind="move", arm=[25, 0, 0, -15, 0, 50], speed=60)]))
    wait_for(lambda: session.runner is None, 5, "the program never finished")
    time.sleep(0.4)
    assert arm.degrees() == pytest.approx([25, 0, 0, -15, 0], abs=0.3)


# ---- two senders ------------------------------------------------------------------------------------------

def _drive_with_leader(priority: bool) -> float:
    """How far the arm gets on a +40 degree move while a still leader arm is on the radio.

    Both send at 50 Hz, so the gap between their sequence counters barely moves, and on the old firmware
    whoever starts ahead wins outright -- a coin toss per session, which is worse than a steady fight. The
    leader here starts about 64 ahead of Slixer's first packet: the losing side of the toss.
    """
    pc_port, arm_port = free_udp_port(), free_udp_port()
    arm = FakeFollower(listen_port=arm_port, report_to=("127.0.0.1", pc_port), priority=priority).start()
    session = Session(mapping=Mapping(), arm_port=pc_port).start()
    try:
        wait_for(lambda: session.hearing_follower, 3, "no arm")
        stop_leader = arm.run_leader([0.0] * 5, start_seq=20)
        time.sleep(0.6)
        session.set_mode("drive")
        time.sleep(0.3)
        here = session._to_arm_list(session.commanded)
        goal = list(here)
        goal[0] += 40
        arm.history.clear()
        session.run_program(Program(steps=[Step(kind="move", arm=goal, speed=40), Step(kind="wait", seconds=0.8)]))
        time.sleep(2.2)
        reached = max(pan_degrees(arm, s) for s in arm.history)
        stop_leader.set()
        return reached
    finally:
        session.stop()
        session.arm._socket.close()
        arm.stop()


def test_with_old_firmware_a_radio_leader_wins_the_fight():
    """Documents the bug the new firmware fixes: this is what the arm did on 2026-09-22."""
    assert _drive_with_leader(priority=False) < 30


def test_with_new_firmware_the_program_has_the_arm():
    assert _drive_with_leader(priority=True) == pytest.approx(40, abs=1.0)


def test_handing_back_to_the_leader_glides_rather_than_lunges():
    pc_port, arm_port = free_udp_port(), free_udp_port()
    arm = FakeFollower(listen_port=arm_port, report_to=("127.0.0.1", pc_port), priority=True).start()
    session = Session(mapping=Mapping(), arm_port=pc_port).start()
    try:
        wait_for(lambda: session.hearing_follower, 3, "no arm")
        stop_leader = arm.run_leader([0.0] * 5)
        time.sleep(0.5)
        session.set_mode("drive")
        session.set_target_arm([40, 0, 0, 0, 0, 50])
        time.sleep(1.2)
        arm.history.clear()
        session.set_mode("watch")  # stop sending: the leader should take the arm back
        time.sleep(2.0)
        samples = list(arm.history)
        peak = max(abs(pan_degrees(arm, b) - pan_degrees(arm, a)) / (b.t - a.t)
                   for a, b in zip(samples, samples[1:]) if b.t > a.t)
        assert pan_degrees(arm, samples[-1]) == pytest.approx(0, abs=0.5)
        assert peak < 90  # the firmware's glide (about 67 deg/s at its peak), never a 300 deg/s lunge
        stop_leader.set()
    finally:
        session.stop()
        session.arm._socket.close()
        arm.stop()


def test_a_leader_steering_is_noticed_while_watching():
    pc_port, arm_port = free_udp_port(), free_udp_port()
    arm = FakeFollower(listen_port=arm_port, report_to=("127.0.0.1", pc_port)).start()
    session = Session(mapping=Mapping(), arm_port=pc_port).start()
    try:
        wait_for(lambda: session.hearing_follower, 3, "no arm")
        assert not session.leader_steering
        stop_leader = arm.run_leader([0.0] * 5)
        wait_for(lambda: session.leader_steering, 4, "the leader was never noticed")
        stop_leader.set()
    finally:
        session.stop()
        session.arm._socket.close()
        arm.stop()


def test_an_arm_that_will_not_follow_raises_a_warning():
    pc_port, arm_port = free_udp_port(), free_udp_port()
    arm = FakeFollower(listen_port=arm_port, report_to=("127.0.0.1", pc_port), priority=False).start()
    session = Session(mapping=Mapping(), arm_port=pc_port).start()
    try:
        wait_for(lambda: session.hearing_follower, 3, "no arm")
        stop_leader = arm.run_leader([0.0] * 5, start_seq=20)  # ahead of Slixer: the losing side
        time.sleep(0.5)
        session.set_mode("drive")
        session.set_target_arm([45, 0, 0, 0, 0, 50])
        wait_for(lambda: session.warning, 5, "no warning for an arm that isn't following")
        assert "isn't following" in session.warning
        session.set_mode("watch")
        time.sleep(0.1)
        assert session.warning == ""
        stop_leader.set()
    finally:
        session.stop()
        session.arm._socket.close()
        arm.stop()


# ---- the link itself ---------------------------------------------------------------------------------------

class _Recorder:
    def __init__(self):
        self.sequences = []

    def sendto(self, data, where):
        self.sequences.append(struct.unpack(POSE_FORMAT, data)[1])

    def settimeout(self, t):
        pass


def _newer(a: int, b: int) -> bool:
    difference = (a - b) & 0xFF
    return difference != 0 and difference < 128


@pytest.mark.parametrize("offset", range(0, 256, 5))
def test_our_sequence_always_reads_as_newer_than_a_leader_we_can_hear(offset):
    arm = Arm(port=free_udp_port())
    arm._socket.close()
    arm._socket = recorder = _Recorder()
    theirs = offset
    for _ in range(60):
        theirs = (theirs + 2) & 0xFF
        arm.other_sequence, arm.other_sequence_seen = theirs, time.monotonic()
        arm.send_pose([0] * 5, 0)
        assert _newer(recorder.sequences[-1], theirs)


def test_sequence_is_left_alone_when_nothing_else_is_sending():
    arm = Arm(port=free_udp_port())
    arm._socket.close()
    arm._socket = recorder = _Recorder()
    for _ in range(5):
        arm.send_pose([0] * 5, 0)
    assert recorder.sequences == [1, 2, 3, 4, 5]
    arm.other_sequence, arm.other_sequence_seen = 9, time.monotonic() - 5.0  # heard long ago
    arm.send_pose([0] * 5, 0)
    assert recorder.sequences[-1] == 6
    assert AHEAD_BY == 64 and HELLO_MAGIC == 0x5D


def test_greeting_is_a_packet_no_board_acts_on_and_is_not_repeated_at_once():
    """A board sends to whoever has spoken to it; Slixer answers what it hears so it gets the full rate."""
    import socket

    board = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    board.bind(("127.0.0.1", 0))
    board.settimeout(0.5)
    arm = Arm(port=free_udp_port())
    try:
        arm._greet(board.getsockname())
        arm._greet(board.getsockname())  # again straight away: once is enough
        data, _ = board.recvfrom(64)
        assert data == bytes((HELLO_MAGIC, 1))  # two bytes: matches neither a pose (14) nor a command
        with pytest.raises(socket.timeout):
            board.recvfrom(64)
    finally:
        arm._socket.close()
        board.close()
