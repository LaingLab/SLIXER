"""The server, as a browser sees it: pages, uploads, and the live link."""

from __future__ import annotations

import io
import time

import pytest
import trimesh
from fastapi.testclient import TestClient

from helpers import TEST_HOSTS, free_udp_port, wait_for
from fake_follower import FakeFollower
from server import create_app


@pytest.fixture
def client(data_dir):
    with TestClient(create_app(arm_port=free_udp_port(), hosts=TEST_HOSTS)) as client:
        yield client


def states(socket, count: int = 1):
    """The next `count` state messages, skipping any other replies."""
    got = []
    while len(got) < count:
        message = socket.receive_json()
        if message["type"] == "state":
            got.append(message)
    return got[-1]


def reply(socket, of_type: str = "note"):
    while True:
        message = socket.receive_json()
        if message["type"] == of_type:
            return message


# ---- pages ----------------------------------------------------------------------------------------------

def test_the_page_and_the_model(client):
    assert "<title>Slixer</title>" in client.get("/").text
    model = client.get("/api/model").json()
    assert model["motors"][0] == "shoulder_pan"
    assert "gripper_frame_link" not in model["meshes"]  # a position, not a shape: never asked for
    assert "base_link" in model["meshes"]
    mesh = client.get("/static/models/base_link.stl", headers={"accept-encoding": "gzip"})
    assert mesh.status_code == 200


@pytest.mark.parametrize("path", ["/static/models/../../server.py", "/api/scene/mesh/../../server.py",
                                  "/static/models/nope.stl", "/api/scene/mesh/nope.stl"])
def test_nothing_outside_the_mesh_folders_can_be_fetched(client, path):
    response = client.get(path)
    assert response.status_code == 404 or "def create_app" not in response.text


def test_uploading_a_part(client):
    buffer = io.BytesIO()
    trimesh.creation.box(extents=[120, 80, 40]).export(buffer, file_type="stl")
    response = client.post("/api/scene/upload", files={"file": ("bracket.stl", buffer.getvalue())})
    assert response.status_code == 200
    item = response.json()["item"]
    assert item["name"] == "bracket" and item["scale"] == 0.001
    assert client.get(f"/api/scene/mesh/{item['id']}.stl").status_code == 200
    assert client.post("/api/scene/upload", files={"file": ("notes.txt", b"hello")}).status_code == 400


# ---- the live link ----------------------------------------------------------------------------------------

def test_state_arrives_and_bad_instructions_do_not_drop_the_link(client):
    with client.websocket_connect("/ws") as socket:
        state = states(socket)
        assert state["mode"] == "watch" and not state["movable"]
        for bad in ({"do": "target"}, {"do": "ik", "xyz": "nope"}, {"do": "nonsense"},
                    {"do": "program.load", "name": "../../etc/passwd"}, {"do": "mode", "mode": "fly"}):
            socket.send_json(bad)
            assert reply(socket)["text"]
        assert states(socket)["type"] == "state"  # still connected


def test_watch_refuses_to_move_and_plan_allows_it(client):
    with client.websocket_connect("/ws") as socket:
        socket.send_json({"do": "target_arm", "arm": [20, 0, 0, 0, 0, 50]})
        assert "Plan" in reply(socket)["text"]
        socket.send_json({"do": "mode", "mode": "plan"})
        assert "planning" in reply(socket)["text"]
        socket.send_json({"do": "target_arm", "arm": [20, 0, 0, 0, 0, 50]})
        assert reply(socket, "accepted")["pose"]
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            if abs(states(socket)["arm_pose"][0] - 20) < 0.2:
                break
        else:
            pytest.fail("the virtual arm never reached its target")


def test_drive_is_refused_with_no_arm(client):
    with client.websocket_connect("/ws") as socket:
        socket.send_json({"do": "mode", "mode": "drive"})
        assert "no arm" in reply(socket)["text"]
        assert states(socket)["mode"] == "watch"


def test_a_bad_mapping_changes_nothing(client):
    with client.websocket_connect("/ws") as socket:
        before = states(socket)["mapping"]
        socket.send_json({"do": "mapping", "joints": {
            "shoulder_pan": {"sign": -1, "offset_deg": 10},
            "elbow_flex": {"sign": 1, "offset_deg": float("nan")},  # json can't carry NaN...
        }})
        # ...so it arrives as null and is refused as not-a-number, before shoulder_pan is touched
        assert reply(socket)["text"]
        assert states(socket)["mapping"] == before


def test_programs_round_trip_through_the_link(client):
    program = {"name": "link test", "loop": False, "steps": [
        {"kind": "move", "arm": [10, 20, 30, 40, 50, 60], "speed": 45, "note": "first"},
        {"kind": "wait", "seconds": 1.5, "note": ""}]}
    with client.websocket_connect("/ws") as socket:
        socket.send_json({"do": "program.save", "program": program})
        assert "link test" in reply(socket, "programs")["names"]
        socket.send_json({"do": "program.load", "name": "link test"})
        loaded = reply(socket, "program")["program"]
        assert loaded["steps"][0]["arm"] == [10, 20, 30, 40, 50, 60] and loaded["steps"][0]["note"] == "first"
        socket.send_json({"do": "program.delete", "name": "link test"})
        assert "link test" not in reply(socket, "programs")["names"]


def test_closing_the_page_while_driving_stops_driving(data_dir):
    pc_port, arm_port = free_udp_port(), free_udp_port()
    arm = FakeFollower(listen_port=arm_port, report_to=("127.0.0.1", pc_port)).start()
    app = create_app(arm_port=pc_port, hosts=TEST_HOSTS)
    try:
        with TestClient(app) as client:
            session = app.state.slixer.session
            wait_for(lambda: session.hearing_follower, 3, "no arm")
            with client.websocket_connect("/ws") as socket:
                socket.send_json({"do": "mode", "mode": "drive"})
                assert "driving" in reply(socket)["text"]
                assert session.mode == "drive"
            wait_for(lambda: session.mode == "watch", 2, "still driving after the page went away")
    finally:
        arm.stop()


def test_the_camera_address_is_remembered(data_dir):
    import settings

    with TestClient(create_app(arm_port=free_udp_port(), hosts=TEST_HOSTS)) as client:
        with client.websocket_connect("/ws") as socket:
            socket.send_json({"do": "camera", "host": "10.9.8.7"})
            assert "remembered" in reply(socket)["text"]
    assert settings.load()["camera_host"] == "10.9.8.7"
    assert str(settings.SETTINGS_PATH).startswith(str(data_dir))  # the test folder, never yours
