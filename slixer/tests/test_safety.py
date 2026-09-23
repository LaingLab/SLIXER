"""Regression tests for the second review (2026-09-23).

Each of these was a way the arm could move when it shouldn't, keep moving after STOP, or be driven by the
wrong page. All found by review, reproduced against the imitation follower, and fixed. Everything here runs
against fake_follower.FakeFollower, on ports the real arm never uses.
"""

from __future__ import annotations

import io
import json
import threading
import time

import pytest
import trimesh

import fake_follower
from fake_follower import WAITING, FakeFollower
from helpers import TEST_HOSTS, free_udp_port, wait_for
from mapping import GripperMap, Mapping
from program import Program
from session import StaleRequest
from so101_link import ArmState

PROGRAM_TO_40 = {"name": "to forty", "loop": False, "steps": [{"kind": "move", "arm": [40, 0, 0, 0, 0, 50], "speed": 40}]}


def pan(arm: FakeFollower) -> float:
    return arm.degrees()[0]


def drive_to(session, arm, pan_degrees: float, tolerance: float = 1.0, seconds: float = 5.0) -> None:
    """Drives the arm's pan there, and waits until it has arrived and Slixer has heard so: until a report says
    it's there, "where the arm actually is" (what STOP and Plan hold) is still somewhere on the way."""
    assert session.set_target_arm([pan_degrees, 0, 0, 0, 0, 50])[1] == ""
    wait_for(lambda: abs(pan(arm) - pan_degrees) < tolerance, seconds, f"the arm never reached pan {pan_degrees}")
    wait_for(lambda: (state := session.follower_state) is not None and abs(state.degrees[0] - pan_degrees) < tolerance,
             seconds, f"no report of the arm at pan {pan_degrees}")


class Server:
    """create_app and a TestClient around an imitation follower, torn down in the right order."""

    def __init__(self):
        from fastapi.testclient import TestClient
        from server import create_app

        pc_port, arm_port = free_udp_port(), free_udp_port()
        self.arm = FakeFollower(listen_port=arm_port, report_to=("127.0.0.1", pc_port)).start()
        self.app = create_app(arm_port=pc_port, hosts=TEST_HOSTS)
        self.slixer = self.app.state.slixer
        self.session = self.slixer.session
        self.client = TestClient(self.app)

    def __enter__(self) -> "Server":
        self.client.__enter__()
        wait_for(lambda: self.session.hearing_follower, 3, "the imitation arm never reported")
        return self

    def __exit__(self, *exc) -> None:
        self.client.__exit__(*exc)
        self.arm.stop()


def reply(socket, containing: str = "") -> dict:
    """The next note, skipping state messages; with `containing`, the next note that says it."""
    while True:
        message = socket.receive_json()
        if message["type"] == "note" and containing in message.get("text", ""):
            return message


# ---- STOP, and everything that should stop with it -----------------------------------------------------

def test_a_move_asked_for_before_stop_is_refused_after_it(rig):
    arm, session = rig
    assert session.set_mode("drive") == ""
    before = session.epoch
    session.stop_everything()
    program = Program.from_json(PROGRAM_TO_40)
    for late in (lambda: session.set_target([0.2] * 6, before),
                 lambda: session.set_target_arm([30, 0, 0, 0, 0, 50], before),
                 lambda: session.solve_to([0.2, 0.0, 0.2], epoch=before),
                 lambda: session.run_program(program, epoch=before)):
        with pytest.raises(StaleRequest):
            late()
    assert session.set_target_arm([30, 0, 0, 0, 0, 50], session.epoch)[1] == ""  # asked for after STOP: fine


def test_moves_queued_behind_a_slow_request_are_dropped_by_stop(data_dir, monkeypatch):
    from dataset import Collector

    monkeypatch.setattr(Collector, "capture", lambda self, automatic=False: time.sleep(1.0) or "captured")
    with Server() as rig, rig.client.websocket_connect("/ws") as socket:
        socket.send_json({"do": "mode", "mode": "drive"})
        wait_for(lambda: rig.session.mode == "drive", 2, "not driving")
        start = pan(rig.arm)
        socket.send_json({"do": "dataset.capture"})  # holds this page's queue for a second
        socket.send_json({"do": "target_arm", "arm": [40, 0, 0, 0, 0, 50]})
        socket.send_json({"do": "program.run", "program": PROGRAM_TO_40})
        socket.send_json({"do": "stop"})
        reply(socket, "stopped")
        time.sleep(2.5)  # the slow request finishes, and what was queued behind it is dealt with
        assert rig.session.runner is None
        assert abs(pan(rig.arm) - start) < 2.0, "a move asked for before STOP was carried out after it"


def test_watch_lets_go_at_once_even_behind_a_slow_request(data_dir, monkeypatch):
    from dataset import Collector

    monkeypatch.setattr(Collector, "capture", lambda self, automatic=False: time.sleep(2.0) or "captured")
    with Server() as rig, rig.client.websocket_connect("/ws") as socket:
        socket.send_json({"do": "mode", "mode": "drive"})
        wait_for(lambda: rig.session.mode == "drive", 2, "not driving")
        socket.send_json({"do": "dataset.capture"})
        socket.send_json({"do": "mode", "mode": "watch"})
        wait_for(lambda: rig.session.mode == "watch", 0.5, "Watch waited behind a slow request")


def test_stop_holds_the_arm_where_it_is_not_where_it_was_sent(rig, monkeypatch):
    arm, session = rig
    monkeypatch.setattr(fake_follower, "SERVO_DEG_PER_SEC", 20.0)  # an arm well behind its command
    assert session.set_mode("drive") == ""
    session.set_target_arm([60, 0, 0, 0, 0, 50])
    time.sleep(0.8)
    at_stop = pan(arm)
    assert session._to_arm_list(session.commanded)[0] - at_stop > 20  # it really was lagging
    session.stop_everything()
    time.sleep(1.5)
    assert abs(pan(arm) - at_stop) < 5, "after STOP the arm carried on to where it had been sent"


def test_a_page_whose_task_failed_still_lets_go_when_it_closes(data_dir, monkeypatch):
    import server

    async def broken(socket, slixer):
        raise RuntimeError("the page went away mid-send")

    monkeypatch.setattr(server, "_push", broken)
    with Server() as rig:
        with rig.client.websocket_connect("/ws") as socket:
            socket.send_json({"do": "mode", "mode": "drive"})
            reply(socket, "driving")
        wait_for(lambda: rig.slixer.viewers == 0 and rig.session.mode == "watch", 2,
                 "closing the page didn't let go of the arm")


def test_state_keeps_coming_when_a_snapshot_fails(data_dir):
    with Server() as rig:
        real, calls = rig.slixer.datasets, []

        def vanishing():  # a label file deleted while the pictures are being counted
            calls.append(1)
            if len(calls) == 3:
                raise FileNotFoundError("labels/0001.txt")
            return real()

        rig.slixer.datasets = vanishing
        with rig.client.websocket_connect("/ws"):
            time.sleep(1.0)  # twenty snapshots' worth
        assert len(calls) > 10, "the page's updates stopped at the failed snapshot"


def test_a_label_file_that_vanishes_mid_count_is_not_counted(data_dir):
    from dataset import _not_empty

    assert _not_empty(data_dir / "no" / "such" / "labels.txt") is False


# ---- a program waits for the arm ------------------------------------------------------------------------

def test_a_program_waits_for_the_real_arm_at_each_waypoint(rig, monkeypatch):
    arm, session = rig
    monkeypatch.setattr(fake_follower, "SERVO_DEG_PER_SEC", 20.0)
    assert session.set_mode("drive") == ""
    program = Program.from_json({"name": "waypoints", "loop": False, "steps": [
        {"kind": "move", "arm": [40, 0, 0, 0, 0, 50], "speed": 90},
        {"kind": "gripper", "percent": 0}]})
    assert session.run_program(program) == ""
    wait_for(lambda: session.runner is None or session.runner.index >= 1, 10, "the program never moved on")
    # Within ARM_ARRIVED (3 degrees) of the waypoint, give or take a report; the command alone gets there
    # while the arm is still up to 8 degrees behind.
    assert pan(arm) > 40 - 4.0, f"the next step began with the arm only at pan {pan(arm):.1f}"


def test_a_program_waits_while_the_arm_is_out_of_touch(rig):
    arm, session = rig
    real_deliver, dropping = arm.deliver, threading.Event()
    arm.deliver = lambda data, via_wifi=True: None if dropping.is_set() else real_deliver(data, via_wifi)
    assert session.set_mode("drive") == ""
    program = Program.from_json({"name": "slow", "loop": False, "steps": [
        {"kind": "move", "arm": [50, 0, 0, 0, 0, 50], "speed": 30}]})  # about 1.7 s
    assert session.run_program(program) == ""
    time.sleep(0.3)
    dropping.set()  # the arm stops hearing Slixer, and freezes
    time.sleep(2.2)
    assert session.runner is not None, "the program finished while the arm couldn't hear it"
    # ...and it didn't run on without the arm: when the arm hears it again, it carries on from about where it
    # froze, instead of cutting straight across to wherever the program had got to.
    ahead = session._to_arm_list(session.commanded)[0] - pan(arm)
    assert ahead < 20, f"the program ran {ahead:.0f} degrees ahead of an arm that couldn't hear it"
    dropping.clear()
    wait_for(lambda: session.runner is None, 8, "the program never finished once the arm was back")
    assert abs(pan(arm) - 50) < 3


class Late:
    """The imitation arm's socket, with everything it sends arriving `delay` seconds late -- as reports do
    from a real arm over Wi-Fi, a report's worth behind the servos."""

    def __init__(self, sock, delay: float):
        self.sock, self.delay = sock, delay

    def sendto(self, data, address):
        threading.Timer(self.delay, self.sock.sendto, (data, address)).start()

    def __getattr__(self, name):
        return getattr(self.sock, name)


def test_a_fast_program_runs_smoothly_despite_the_links_delay(rig):
    arm, session = rig
    arm._socket = Late(arm._socket, 0.12)  # the arm reported ~12 degrees behind at 90 degrees a second
    sent, real_send = [], session.arm.send_pose
    session.arm.send_pose = lambda degrees, percent: (sent.append(degrees[0]), real_send(degrees, percent))
    assert session.set_mode("drive") == ""
    program = Program.from_json({"name": "fast", "loop": False, "steps": [
        {"kind": "move", "arm": [80, 0, 0, 0, 0, 50], "speed": 100}]})
    assert session.run_program(program) == ""
    wait_for(lambda: session.runner is None, 10, "the program never finished")
    assert session.program_note == "finished"
    middle = [pan for pan in sent if 10 < pan < 70]
    stalls = sum(1 for a, b in zip(middle, middle[1:]) if abs(b - a) < 0.05)
    assert stalls == 0, f"the command stood still {stalls} times mid-move: the arm would move in steps"


def test_a_program_stops_if_the_arm_never_reaches_a_waypoint(rig):
    arm, session = rig
    real_move = arm._move_servos

    def blocked(dt):  # something in the way of the shoulder at pan 20
        real_move(dt)
        low, high = arm.ranges[0]
        arm.positions[0] = min(arm.positions[0], (low + high) / 2 + 20 * 4095 / 360)

    arm._move_servos = blocked
    assert session.set_mode("drive") == ""
    program = Program.from_json({"name": "blocked", "loop": False, "steps": [
        {"kind": "move", "arm": [60, 0, 0, 0, 0, 50], "speed": 90},
        {"kind": "gripper", "percent": 0}]})
    assert session.run_program(program) == ""
    wait_for(lambda: session.runner is None, 10, "the program waited for ever")
    assert "didn't get there" in session.program_note  # it stopped, rather than moving on to the gripper
    assert abs(session._to_arm_list(session.target)[0] - 20) < 3  # and holds the arm where it is


# ---- only an arm that's ready is driven -----------------------------------------------------------------

def test_drive_is_refused_while_the_follower_checks_itself(rig):
    arm, session = rig
    arm.fail(zeroed=True)  # just powered up, servos not answering: everything reported as zero
    wait_for(lambda: session.state.state == "checking arm", 2, "the fault never arrived")
    assert "isn't ready" in session.set_mode("drive")
    assert session.follower_state is None  # its zeros aren't mirrored, or clamped to
    assert session.real_pose() is None


def test_driving_stops_when_the_follower_stops_taking_poses(rig):
    arm, session = rig
    assert session.set_mode("drive") == ""
    drive_to(session, arm, 30)
    arm.fail()
    wait_for(lambda: session.mode == "watch", 2, "still driving an arm that is checking itself")
    assert "stopped sending" in session.snapshot()["warning"]
    time.sleep(0.3)  # nothing more is sent, so nothing is stored up for it...
    arm.recover()
    time.sleep(0.6)
    assert arm.state == WAITING  # ...and it doesn't glide anywhere when it's ready again


def test_an_uncalibrated_report_is_recognised():
    zeros = ArmState(state="checking arm", fault="no reply from servo", fault_joint="gripper", poses_received=0,
                     position_steps=(0,) * 6, range_min=(0,) * 6, range_max=(0,) * 6, version=2)
    assert not zeros.calibrated and not zeros.ready


# ---- Plan after Drive holds the arm ----------------------------------------------------------------------

def test_leaving_drive_for_plan_keeps_holding_the_arm(rig):
    arm, session = rig
    leader = arm.run_leader([0, 0, 0, 0, 0])  # a powered leader, resting at zero
    try:
        wait_for(lambda: abs(pan(arm)) < 1, 5, "not following the leader")
        assert session.set_mode("drive") == ""
        drive_to(session, arm, 45)
        assert session.set_mode("plan") == ""
        time.sleep(1.5)
        assert abs(pan(arm) - 45) < 2, "leaving Drive for Plan let the leader take the arm"
        assert session.snapshot()["holding"]
        assert session.set_mode("watch") == ""
        wait_for(lambda: abs(pan(arm)) < 2, 8, "Watch should let go, back to the leader")
    finally:
        leader.set()


def test_closing_every_page_lets_go_of_an_arm_held_in_plan(data_dir):
    with Server() as rig:
        with rig.client.websocket_connect("/ws") as socket:
            socket.send_json({"do": "mode", "mode": "drive"})
            reply(socket, "driving")
            socket.send_json({"do": "mode", "mode": "plan"})
            assert "held where it is" in reply(socket, "planning")["text"]
        wait_for(lambda: not rig.session.sending, 2, "the arm is still held with no page open")


# ---- only this server's own page ------------------------------------------------------------------------

def test_other_pages_and_other_names_are_refused(data_dir):
    from starlette.websockets import WebSocketDisconnect

    with Server() as rig:
        for origin in ("https://elsewhere.example", "http://localhost:3000", "null"):
            with pytest.raises(WebSocketDisconnect):
                with rig.client.websocket_connect("/ws", headers={"origin": origin}):
                    pass
        with rig.client.websocket_connect("/ws", headers={"origin": "http://testserver"}) as socket:
            assert socket.receive_json()["type"] == "state"  # this server's own page
        # DNS rebinding: another site's name, pointed at this machine
        assert rig.client.get("/api/model", headers={"host": "rebind.attacker.example"}).status_code == 403
        assert rig.client.get("/api/model", headers={"host": "127.0.0.1:8000"}).status_code == 200
        upload = {"file": ("part.stl", b"solid x\nendsolid x\n", "model/stl")}
        assert rig.client.post("/api/scene/upload", headers={"origin": "https://elsewhere.example"},
                               files=upload).status_code == 403


def test_the_names_the_server_answers_to():
    from run import allowed_hosts

    assert allowed_hosts("127.0.0.1", []) == {"localhost"}
    network = allowed_hosts("0.0.0.0", ["Arm-PC.lab.example"])
    assert {"localhost", "arm-pc.lab.example"} <= network and len(network) > 2


# ---- importing a part doesn't hold anything up ----------------------------------------------------------

def big_stl() -> bytes:
    buffer = io.BytesIO()
    trimesh.creation.icosphere(subdivisions=7, radius=50).export(buffer, file_type="stl")  # 327,680 faces
    return buffer.getvalue()


def test_importing_a_big_part_doesnt_starve_the_control_loop(data_dir):
    from scene import Scene

    data, gaps, done = big_stl(), [], threading.Event()

    def tick():
        last = time.monotonic()
        while not done.is_set():
            time.sleep(0.005)
            now = time.monotonic()
            gaps.append(now - last)
            last = now

    ticker = threading.Thread(target=tick)
    ticker.start()
    try:
        item = Scene().add(data, "big")
    finally:
        done.set()
        ticker.join()
    assert "thinned" in item.note
    assert max(gaps) < 0.15, f"a thread stood still for {max(gaps) * 1000:.0f} ms while a part was imported"


def test_stop_is_answered_at_once_while_a_part_is_imported(data_dir):
    data = big_stl()
    with Server() as rig, rig.client.websocket_connect("/ws") as socket:
        importing, real_add = threading.Event(), rig.slixer.scene.add

        def add(*args):
            importing.set()
            return real_add(*args)

        rig.slixer.scene.add = add
        uploaded = []
        upload = threading.Thread(target=lambda: uploaded.append(
            rig.client.post("/api/scene/upload", files={"file": ("big.stl", data, "model/stl")})))
        upload.start()
        assert importing.wait(30), "the import never started"
        time.sleep(0.2)  # well into it
        started = time.monotonic()
        socket.send_json({"do": "stop"})
        reply(socket, "stopped")
        waited = time.monotonic() - started
        upload.join(60)
        assert uploaded and uploaded[0].status_code == 200
        assert waited < 0.3, f"STOP waited {waited:.2f} s for a part to import"


# ---- the gripper's calibration ---------------------------------------------------------------------------

def test_a_gripper_whose_open_and_shut_are_the_same_is_refused(data_dir):
    with pytest.raises(ValueError):
        GripperMap(closed_deg=90.0, open_deg=90.0)
    GripperMap(closed_deg=90.0, open_deg=0.0)  # reversed is fine
    with Server() as rig, rig.client.websocket_connect("/ws") as socket:
        before = rig.session.mapping.gripper
        socket.send_json({"do": "mapping", "save": False, "joints": {},
                          "gripper": {"closed_deg": 90, "open_deg": 90}})
        assert "apart" in reply(socket, "apart")["text"]
        assert rig.session.mapping.gripper == before


def test_an_unusable_saved_gripper_falls_back_to_the_default(tmp_path):
    path = tmp_path / "mapping.json"
    path.write_text(json.dumps({"gripper": {"closed_deg": 90, "open_deg": 90}}))
    assert Mapping.load(path).gripper == GripperMap()
