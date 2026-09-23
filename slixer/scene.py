"""Things in the world around the arm: the bench, a fixture, the box you are picking out of.

Import the STL you printed the part from and put it where the real one sits, and the viewer stops being a
picture of an arm in empty space. You can see what the gripper has to reach past, and place a waypoint
against the object rather than guessing coordinates.

Imported meshes are kept in `scene/meshes/` and where they sit is kept in `scene/scene.json`, so a scene
survives a restart and can be copied to another machine or committed alongside a program.

One thing is done for you, because it catches everyone: CAD and slicers work in millimetres while the
robot model works in metres. A part exported at full size arrives a thousand times too big, so anything
that turns up implausibly large is scaled down and the guess is reported rather than hidden.
"""

from __future__ import annotations

import concurrent.futures
import json
import math
import multiprocessing
import threading
import uuid
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
import trimesh

import paths

SCENE_DIR = paths.SCENE
MESH_DIR = SCENE_DIR / "meshes"
SCENE_FILE = SCENE_DIR / "scene.json"

TARGET_FACES = 8000  # imported parts can be far heavier than they need to be on screen
MIN_FACES = 200
BIGGEST_SENSIBLE_METRES = 3.0  # a part bigger than this on a 0.4 m arm was almost certainly in millimetres
COLOURS = ("#4fd1ff", "#46d18a", "#ff9d3f", "#c98bff", "#ff6b9d", "#e0c860")


@dataclass
class Item:
    """One imported object: which mesh, where it sits, and how it is drawn."""

    id: str
    name: str
    position: list[float] = field(default_factory=lambda: [0.25, 0.0, 0.0])  # metres, the arm's own frame
    rotation: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])  # radians, roll-pitch-yaw
    scale: float = 1.0
    colour: str = COLOURS[0]
    visible: bool = True
    note: str = ""  # how the mesh was interpreted on import, shown in the panel

    def to_json(self) -> dict:
        return asdict(self)


def prepare(data: bytes) -> tuple[bytes, float, int, int]:
    """The slow part of importing: reads an STL, tidies the mesh, and centres it on its own origin.

    Returns the tidied mesh as STL bytes, how big it is, and how many triangles it had before and after
    thinning. Scene.add runs this in a process of its own.
    """
    mesh = trimesh.load_mesh(trimesh.util.wrap_as_stream(data), file_type="stl", process=False)
    if not isinstance(mesh, trimesh.Trimesh) or len(mesh.faces) == 0:
        raise ValueError("that file has no triangles in it -- is it really an STL?")

    # Welding first, for the same reason the robot's own meshes need it: an STL is loose triangles,
    # and thinning loose triangles shatters the model instead of simplifying it.
    mesh.merge_vertices()
    faces_before = len(mesh.faces)
    if faces_before > TARGET_FACES:
        try:
            mesh = mesh.simplify_quadric_decimation(face_count=max(MIN_FACES, TARGET_FACES))
        except Exception:
            pass  # full detail still draws, just more slowly

    # Sit the part on its own origin, so placing it is about where you want it rather than wherever
    # the CAD package happened to put 0,0,0.
    mesh.apply_translation(-mesh.bounding_box.centroid)
    size = float(np.max(mesh.extents)) if mesh.extents is not None else 0.0
    return mesh.export(file_type="stl"), size, faces_before, len(mesh.faces)


def _in_own_process(function, *args):
    """Runs `function` in a fresh process, and returns what it returns or raises what it raises.

    Thinning a big mesh holds Python's interpreter lock for a second or more. Done in this process, that
    would starve the 50 Hz loop that keeps the arm fed, and the firmware would freeze the arm mid-move. A
    fresh process -- spawned, not forked from this one and all its threads -- takes a moment to start, and
    parts are imported rarely.
    """
    context = multiprocessing.get_context("spawn")
    with concurrent.futures.ProcessPoolExecutor(max_workers=1, mp_context=context) as pool:
        return pool.submit(function, *args).result()


class Scene:
    """Everything imported, and the file it is remembered in."""

    def __init__(self) -> None:
        self.items: list[Item] = []
        self._lock = threading.RLock()  # parts are imported on one thread while another may be moving one
        self.load()

    # ---- the file ------------------------------------------------------------

    def load(self) -> None:
        if not SCENE_FILE.exists():
            return
        try:
            raw = json.loads(SCENE_FILE.read_text())
        except (json.JSONDecodeError, OSError):
            return  # a half-written scene shouldn't stop the arm coming up
        self.items = []
        for entry in raw.get("items", []):
            if not (MESH_DIR / f"{entry.get('id', '')}.stl").exists():
                continue  # the mesh was deleted from under us; drop the entry rather than break the viewer
            self.items.append(
                Item(
                    id=str(entry["id"]),
                    name=str(entry.get("name", "part"))[:64],
                    position=[float(v) for v in entry.get("position", [0, 0, 0])][:3],
                    rotation=[float(v) for v in entry.get("rotation", [0, 0, 0])][:3],
                    scale=float(entry.get("scale", 1.0)),
                    colour=str(entry.get("colour", COLOURS[0]))[:16],
                    visible=bool(entry.get("visible", True)),
                    note=str(entry.get("note", ""))[:120],
                )
            )

    def save(self) -> None:
        SCENE_DIR.mkdir(parents=True, exist_ok=True)
        temporary = SCENE_FILE.with_suffix(".json.tmp")
        temporary.write_text(json.dumps({"items": [i.to_json() for i in self.items]}, indent=2) + "\n")
        temporary.replace(SCENE_FILE)

    # ---- importing -----------------------------------------------------------

    def add(self, data: bytes, name: str) -> Item:
        """Takes the bytes of an STL, tidies the mesh, and puts it in the scene in front of the arm."""
        stl, size, faces_before, faces_after = _in_own_process(prepare, data)
        scale, note = self._guess_units(size, faces_before, faces_after)
        MESH_DIR.mkdir(parents=True, exist_ok=True)
        with self._lock:
            item = Item(id=uuid.uuid4().hex[:12], name=name[:64] or "part", scale=scale, note=note,
                        colour=COLOURS[len(self.items) % len(COLOURS)])
            (MESH_DIR / f"{item.id}.stl").write_bytes(stl)
            self.items.append(item)
            self.save()
        return item

    @staticmethod
    def _guess_units(size: float, before: int, after: int) -> tuple[float, str]:
        thinned = f", thinned {before} to {after} triangles" if after < before else ""
        if size <= 0:
            return 1.0, f"empty bounding box{thinned}"
        if size > BIGGEST_SENSIBLE_METRES:
            # Read as millimetres. Stated plainly, because a wrong guess is obvious on screen and the
            # scale box is right there to correct it.
            return 0.001, f"{size:.0f} units across, read as millimetres ({size:.0f} mm){thinned}"
        return 1.0, f"{size * 1000:.0f} mm across{thinned}"

    # ---- changing ------------------------------------------------------------

    def find(self, item_id: str) -> Item | None:
        return next((i for i in self.items if i.id == item_id), None)

    def update(self, item_id: str, changes: dict) -> Item:
        with self._lock:
            item = self.find(item_id)
            if item is None:
                raise KeyError(f"no object {item_id!r} in the scene")
            if "name" in changes:
                item.name = str(changes["name"])[:64]
            if "position" in changes:
                item.position = self._three(changes["position"], "a position")
            if "rotation" in changes:
                item.rotation = self._three(changes["rotation"], "a rotation")
            if "scale" in changes:
                item.scale = max(1e-4, min(1000.0, self._real(changes["scale"])))
            if "colour" in changes:
                item.colour = str(changes["colour"])[:16]
            if "visible" in changes:
                item.visible = bool(changes["visible"])
            self.save()
            return item

    @classmethod
    def _three(cls, values, what: str) -> list[float]:
        if not isinstance(values, (list, tuple)) or len(values) != 3:
            raise ValueError(f"{what} is three numbers")
        return [cls._real(v) for v in values]

    @staticmethod
    def _real(value) -> float:
        number = float(value)
        if not math.isfinite(number):
            raise ValueError("that isn't a number")
        return number

    def remove(self, item_id: str) -> None:
        with self._lock:
            item = self.find(item_id)
            if item is None:
                return
            self.items = [i for i in self.items if i.id != item_id]
            (MESH_DIR / f"{item.id}.stl").unlink(missing_ok=True)
            self.save()

    def describe(self) -> dict:
        with self._lock:
            return {"items": [i.to_json() for i in self.items]}
