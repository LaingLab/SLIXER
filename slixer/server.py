"""Serves Slixer: the 3D arm, the camera, and the controls, to a browser.

Everything the page needs comes from here. The model's shape is read from the URDF so the viewer and the
maths agree; the camera is passed through untouched; and the arm is driven through one `Session`, which
is the only thing in the project that sends it anything.

    uv run slixer/run.py                          # http://localhost:8000, arm found by itself
    uv run slixer/run.py --camera 192.168.1.50    # with the Pi's camera
    uv run slixer/run.py --host 0.0.0.0           # reachable from the rest of the lab

The default is deliberately localhost-only. This page can move a robot, and a robot that anyone on the
network can move is a robot that will eventually be moved by someone who didn't mean to.
"""

from __future__ import annotations

import asyncio
import contextlib
import gzip
import json
import math
import sys
import threading
import time
import traceback
from pathlib import Path

from fastapi import FastAPI, File, HTTPException, Request, Response, UploadFile, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "pc"))

from camera_client import ArmCamera  # noqa: E402

from kinematics import Chain  # noqa: E402
from mapping import GripperMap, JointMap, Mapping  # noqa: E402
from program import Program  # noqa: E402
import settings  # noqa: E402
from scene import MESH_DIR, Scene  # noqa: E402
from session import Session  # noqa: E402
from version import VERSION  # noqa: E402
from vision import Vision  # noqa: E402
from dataset import Collector  # noqa: E402
from detection import available_models  # noqa: E402
from so101_link import PORT as ARM_PORT  # noqa: E402

MAX_UPLOAD = 64 * 1024 * 1024  # an STL bigger than this is a mistake, not a part

WEB = HERE / "web"
UPDATE_HZ = 20.0  # how often the browser is told where the arm is
FRESH = {"Cache-Control": "no-cache"}  # always check for a newer copy (cheap: unchanged files answer 304)


class Static(StaticFiles):
    """The page's own scripts and styles, which must never be served from an old cache.

    A browser that pairs a new index.html with yesterday's app.js has a broken page, and heuristic caching
    will do exactly that to files served without instructions. three.js and the meshes are big and never
    change, so they are left to cache.
    """

    async def get_response(self, path, scope):
        response = await super().get_response(path, scope)
        if path.endswith((".js", ".css", ".html")) and not path.startswith("vendor/"):
            response.headers.update(FRESH)
        return response


class Slixer:
    """Holds the pieces the request handlers need."""

    def __init__(self, camera_host: str | None = None, camera_port: int = 50102, arm_port: int = ARM_PORT):
        self.chain = Chain.load()
        self.session = Session(chain=self.chain, arm_port=arm_port)
        self.scene = Scene()
        self.camera: ArmCamera | None = None
        self.vision = Vision(self.session, camera=lambda: self.camera)
        self.collector = Collector(camera=lambda: self.camera, session=self.session, vision=self.vision)
        self._datasets: tuple[float, dict] = (0.0, {})
        self.camera_host = camera_host
        self.camera_port = camera_port
        self.viewers = 0
        if camera_host:
            self.open_camera(camera_host, camera_port)

    def open_camera(self, host: str, port: int = 50102) -> None:
        if self.camera is not None:
            # The old one can take a couple of seconds to hang up; let it, somewhere else.
            threading.Thread(target=self.camera.stop, daemon=True).start()
        # Frames are passed on exactly as they arrive -- to the browser, to the model, into training
        # pictures -- so there's no reason to unpack them here.
        self.camera = ArmCamera(host, port, decode=False).start()
        self.camera_host, self.camera_port = host, port

    def datasets(self) -> dict:
        """The collector's state, counted at most once a second: it walks folders of pictures."""
        now = time.monotonic()
        if now - self._datasets[0] > 1.0:
            self._datasets = (now, self.collector.describe())
        return self._datasets[1]

    def camera_status(self) -> dict:
        if self.camera is None:
            return {"configured": False, "connected": False, "frames": 0, "age": None, "host": None}
        _, age = self.camera.read_jpeg()
        return {
            "configured": True,
            "connected": self.camera.connected,
            "frames": self.camera.frames_received,
            "age": None if age == float("inf") else round(age, 3),
            "host": f"{self.camera.host}:{self.camera.port}",
        }


def create_app(camera_host: str | None = None, camera_port: int = 50102, arm_port: int = ARM_PORT) -> FastAPI:
    slixer = Slixer(camera_host, camera_port, arm_port)
    squashed: dict[str, bytes] = {}  # meshes, compressed once and kept

    @contextlib.asynccontextmanager
    async def lifespan(_app):
        slixer.session.start()
        slixer.vision.start()
        try:
            yield
        finally:
            slixer.collector.stop()
            slixer.vision.stop()
            slixer.session.stop()
            if slixer.camera:
                slixer.camera.stop()

    app = FastAPI(title="Slixer", lifespan=lifespan)
    app.state.slixer = slixer

    # ---- the page -----------------------------------------------------------

    @app.get("/")
    def index() -> FileResponse:
        return FileResponse(WEB / "index.html", headers=FRESH)

    @app.get("/api/model")
    def model() -> JSONResponse:
        """The arm's shape. Fetched once: it only changes if you swap the URDF."""
        description = slixer.chain.describe()
        # Which links have a mesh at all, so the page doesn't ask for frames that are only a position,
        # like the gripper's tip.
        description["meshes"] = sorted(p.stem for p in (WEB / "models").glob("*.stl"))
        description["version"] = VERSION
        return JSONResponse(description)

    @app.get("/api/programs")
    def programs() -> JSONResponse:
        return JSONResponse({"names": Program.saved()})

    @app.get("/api/models")
    def models() -> JSONResponse:
        """The models that can be picked: stock ones (fetched on first use) and any you've trained."""
        return JSONResponse({"models": available_models(), "installed": slixer.vision.model.installed})

    @app.get("/api/scene")
    def scene() -> JSONResponse:
        return JSONResponse(slixer.scene.describe())

    @app.post("/api/scene/upload")
    async def upload(file: UploadFile = File(...)) -> JSONResponse:
        """Takes an STL and puts it in the scene."""
        if not file.filename.lower().endswith(".stl"):
            raise HTTPException(status_code=400, detail="only .stl files, for now")
        data = await file.read(MAX_UPLOAD + 1)
        if len(data) > MAX_UPLOAD:
            raise HTTPException(status_code=413, detail="that file is over 64 MB")
        try:
            item = slixer.scene.add(data, Path(file.filename).stem)
        except Exception as error:
            raise HTTPException(status_code=400, detail=f"couldn't read that STL: {error}") from error
        return JSONResponse({"item": item.to_json(), "items": slixer.scene.describe()["items"]})

    @app.get("/api/scene/mesh/{name}")
    def scene_mesh(name: str, request: Request) -> Response:
        """An imported part's mesh, compressed the same way the robot's own are."""
        path = (MESH_DIR / name).resolve()
        if path.parent != MESH_DIR.resolve() or not path.is_file():
            raise HTTPException(status_code=404, detail="no such part")
        if "gzip" not in request.headers.get("accept-encoding", ""):
            return FileResponse(path)
        # Not cached in memory like the robot's own meshes: these come and go as you import and delete,
        # and a few milliseconds of compression once per page load is cheaper than a stale part on screen.
        return Response(gzip.compress(path.read_bytes(), 6), media_type="model/stl",
                        headers={"Content-Encoding": "gzip", "Cache-Control": "no-cache"})

    @app.get("/static/models/{name}")
    def mesh(name: str, request: Request) -> Response:
        """A link's shape, compressed.

        The meshes are the bulk of what a browser fetches, and they squash to about a third of their
        size, which is worth having over a lab network. Compressing here rather than with a blanket
        middleware keeps it well away from the camera: gzipping a stream that never ends would sit on
        frames waiting for a buffer to fill.
        """
        folder = (WEB / "models").resolve()
        path = (folder / name).resolve()
        if path.parent != folder or not path.is_file():
            raise HTTPException(status_code=404, detail="no such mesh")
        if "gzip" not in request.headers.get("accept-encoding", ""):
            return FileResponse(path)
        if name not in squashed:
            squashed[name] = gzip.compress(path.read_bytes(), 6)
        return Response(
            squashed[name],
            media_type="model/stl",
            headers={"Content-Encoding": "gzip", "Cache-Control": "public, max-age=3600"},
        )

    app.mount("/static", Static(directory=WEB), name="static")

    # ---- the camera ---------------------------------------------------------

    @app.get("/camera.mjpg")
    async def camera_stream() -> StreamingResponse:
        """The camera, passed straight through as the multipart stream browsers already understand."""

        async def frames():
            last_sent = -1.0
            while True:
                camera = slixer.camera
                if camera is None:
                    await asyncio.sleep(0.5)
                    continue
                jpeg, age = camera.read_jpeg()
                if jpeg is not None and age != last_sent:
                    last_sent = age
                    yield b"--frame\r\nContent-Type: image/jpeg\r\n"
                    yield f"Content-Length: {len(jpeg)}\r\n\r\n".encode()
                    yield jpeg + b"\r\n"
                await asyncio.sleep(0.02)

        return StreamingResponse(frames(), media_type="multipart/x-mixed-replace; boundary=frame")

    # ---- the live link ------------------------------------------------------

    @app.websocket("/ws")
    async def live(socket: WebSocket) -> None:
        await socket.accept()
        slixer.viewers += 1
        queue: asyncio.Queue = asyncio.Queue()
        pusher = asyncio.create_task(_push(socket, slixer))
        worker = asyncio.create_task(_work(socket, slixer, queue))
        try:
            while True:
                message = json.loads(await socket.receive_text())
                if isinstance(message, dict) and message.get("do") == "stop":
                    # Straight away, here, never queued behind anything slower: connecting a camera
                    # or starting a model can take a second, and STOP must not wait that second.
                    await socket.send_text(json.dumps(handle(slixer, message)))
                else:
                    await queue.put(message)
        except (WebSocketDisconnect, json.JSONDecodeError, RuntimeError):
            pass
        finally:
            for task in (pusher, worker):
                task.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await task
            slixer.viewers -= 1
            if slixer.viewers <= 0 and slixer.session.driving:
                # Nobody is watching any more. Whoever was driving has closed the tab, lost their network,
                # or put the laptop to sleep; none of those should leave a program running. The arm is let
                # go: the firmware freezes it, or with the leader on, the leader glides it back to itself.
                slixer.session.release()

    return app


async def _work(socket: WebSocket, slixer: Slixer, queue: asyncio.Queue) -> None:
    """Carries out one connection's instructions in order, each on a worker thread.

    On a thread, so a slow one (connecting to a camera that isn't answering) holds up only this queue --
    never the event loop, the state going out to every page, or STOP.
    """
    while True:
        message = await queue.get()
        try:
            reply = await asyncio.to_thread(handle, slixer, message)
        except (KeyError, ValueError) as error:
            # The expected kind: a bad number, a name that doesn't exist. Said as a sentence.
            reply = {"type": "note", "text": str(error.args[0]) if error.args else "that didn't work"}
        except Exception as error:
            # Anything at all. Losing this connection is what lets go of the arm, so a bug in one
            # instruction must not take it down mid-move. It is reported to the page and printed here.
            traceback.print_exc()
            reply = {"type": "note", "text": f"something went wrong: {error!r}"}
        if reply is not None:
            await socket.send_text(json.dumps(reply))


async def _push(socket: WebSocket, slixer: Slixer) -> None:
    """Sends the arm's state to the browser, forever."""
    period = 1.0 / UPDATE_HZ
    while True:
        snapshot = slixer.session.snapshot()
        snapshot["camera"] = slixer.camera_status()
        snapshot["vision"] = slixer.vision.describe()
        snapshot["dataset"] = slixer.datasets()
        snapshot["type"] = "state"
        await socket.send_text(json.dumps(snapshot))
        await asyncio.sleep(period)


def _finite(value, what: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{what} must be a real number")
    return number


def handle(slixer: Slixer, message: dict) -> dict | None:
    """One instruction from the browser. Returns a reply when there is something to say."""
    session = slixer.session
    what = message.get("do")

    if what == "target":
        pose, problem = session.set_target(message["pose"])
        return {"type": "note", "text": problem} if problem else {"type": "accepted", "pose": pose}

    if what == "target_arm":
        pose, problem = session.set_target_arm(message["arm"])
        return {"type": "note", "text": problem} if problem else {"type": "accepted", "pose": pose}

    if what == "ik":
        pose, error, reached, problem = session.solve_to(message["xyz"])
        if problem:
            return {"type": "note", "text": problem}
        return {"type": "ik", "pose": pose, "error": error, "reached": reached}

    if what == "mode":
        problem = session.set_mode(str(message.get("mode", "")))
        return {"type": "note", "text": problem or MODE_NOTES[session.mode]}

    if what == "drive":  # the older on/off form, still accepted
        problem = session.set_driving(bool(message.get("on")))
        return {"type": "note", "text": problem or MODE_NOTES[session.mode]}

    if what == "stop":
        session.stop_everything()
        if session.driving:
            return {"type": "note", "text": "stopped: holding the arm here. Switch to Watch to let go of it"}
        return {"type": "note", "text": "stopped"}

    if what == "program.run":
        start = int(message.get("start", 0))
        problem = session.run_program(Program.from_json(message["program"]), start_index=start)
        return {"type": "note", "text": problem or ("rehearsing" if session.planning else "running")}

    if what == "program.stop":
        session.stop_program()
        return {"type": "note", "text": "program stopped"}

    if what == "program.save":
        program = Program.from_json(message["program"])
        if program.name in Program.saved() and not message.get("replace"):
            return {"type": "program.exists", "name": program.name}
        program.save()
        return {"type": "programs", "names": Program.saved(), "text": f"saved {program.name!r}"}

    if what == "program.load":
        program = Program.load(message["name"])
        return {"type": "program", "program": program.to_json()}

    if what == "program.delete":
        Program.delete(message["name"])
        return {"type": "programs", "names": Program.saved(), "text": "deleted"}

    if what == "mapping":
        joints = {}
        for name, values in (message.get("joints") or {}).items():
            if name in session.mapping.joints:
                joints[name] = JointMap(
                    sign=1 if int(values.get("sign", 1)) >= 0 else -1,
                    offset_deg=_finite(values.get("offset_deg", 0.0), f"{name}'s offset"),
                )
        gripper = None
        if "gripper" in message:
            gripper = GripperMap(
                closed_deg=_finite(message["gripper"].get("closed_deg", 0.0), "the gripper's shut angle"),
                open_deg=_finite(message["gripper"].get("open_deg", 90.0), "the gripper's open angle"),
            )
        # Checked in full before anything is changed, so a bad number can't leave half a mapping in use.
        def change():
            session.mapping.joints.update(joints)
            if gripper is not None:
                session.mapping.gripper = gripper

        problem = session.remap(change)
        if problem:
            return {"type": "note", "text": problem}
        # Changes show at once, so the model can be watched moving into line; they are only kept when
        # asked. Programs don't depend on this -- they store the arm's own numbers -- so trying things
        # here is safe.
        if message.get("save"):
            session.mapping.save()
            session.mapping_dirty = False
            return {"type": "note", "text": "saved: the model will be lined up like this from now on"}
        session.mapping_dirty = True
        return None

    if what == "mapping.zero":
        if session.hearing_follower and session.state is not None:
            degrees = session.state.degrees[:5]
        elif session.hearing_leader:
            degrees = session.arm.leader_pose()[0]
        else:
            return {"type": "note", "text": "no arm reporting, so there is no pose to take as zero"}
        problem = session.remap(lambda: session.mapping.zero_here(degrees))
        if problem:
            return {"type": "note", "text": problem}
        session.mapping_dirty = True
        return {"type": "note", "text": "this pose is now the model's zero -- press save to keep it"}

    if what == "scene.update":
        item = slixer.scene.update(str(message["id"]), message.get("changes") or {})
        return {"type": "scene", "items": slixer.scene.describe()["items"], "changed": item.to_json()}

    if what == "scene.remove":
        slixer.scene.remove(str(message["id"]))
        return {"type": "scene", "items": slixer.scene.describe()["items"]}

    if what == "model.set":
        on = bool(message.get("on"))
        classes = message.get("classes")
        if isinstance(classes, str):
            classes = [c.strip() for c in classes.split(",") if c.strip()] or None
        problem = slixer.vision.set_model(on, message.get("model"), message.get("conf"), classes)
        if problem:
            return {"type": "note", "text": problem}
        return {"type": "note", "text": f"starting {slixer.vision.model.config['model']} -- "
                                        "the first time, it downloads" if on else "model stopped"}

    if what == "dataset.set":
        collector = slixer.collector
        if "name" in message:
            dataset_folder = message["name"].strip()
            from dataset import folder
            folder(dataset_folder)  # refuses unsafe names
            collector.name = dataset_folder
        if "prelabel" in message:
            collector.prelabel = bool(message["prelabel"])
        if "auto_every" in message:
            collector.set_auto(_finite(message["auto_every"], "the capture interval"))
        slixer._datasets = (0.0, {})
        return {"type": "note", "text": (f"capturing every {collector.auto_every:g} s into {collector.name}"
                                         if collector.auto_every else f"collecting into {collector.name}")}

    if what == "dataset.capture":
        text = slixer.collector.capture()
        slixer._datasets = (0.0, {})
        return {"type": "note", "text": text}

    if what == "camera":
        host = str(message.get("host", "")).strip()
        if not host:
            return {"type": "note", "text": "no camera address given"}
        port = int(message.get("port", 50102))
        slixer.open_camera(host, port)
        settings.save(camera_host=host, camera_port=port)  # so next time it just starts
        return {"type": "note", "text": f"looking for the camera at {host} (remembered for next time)"}

    return {"type": "note", "text": f"don't know how to {what!r}"}


MODE_NOTES = {
    "watch": "watching: the model mirrors the arm and nothing is sent",
    "plan": "planning: the model is a virtual arm, and the real one is left alone",
    "drive": "driving: the model now moves the real arm",
}


app_files = WEB
