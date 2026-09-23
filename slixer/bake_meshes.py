"""Turns the SO-101's printable STLs into something a browser can load quickly.

The published model is 16 MB across 13 meshes, several of which are the same servo repeated. A browser
on the far side of the lab network doesn't want to fetch that on every reload, and it doesn't need to:
at the size the arm appears on screen, a tenth of the triangles look identical.

So each link's visual meshes are merged into one, in that link's own frame, and thinned down. The viewer
then loads one file per link and only has to apply the joint angles.

    uv run slixer/bake_meshes.py            # rebuild web/models from assets/so101

The baked meshes are in the repository, so this is only needed to swap in different ones. The original STLs
aren't (16 MB): put TheRobotStudio/SO-ARM100's `Simulation/SO101/assets/` folder at `assets/so101/assets/`.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent
URDF = HERE / "assets" / "so101" / "so101.urdf"
OUT = HERE / "web" / "models"

TARGET_FACES = 4000  # per link: plenty at screen size, and small enough to load instantly
MIN_FACES = 300  # don't thin simple brackets into rubble


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF's fixed-axis roll-pitch-yaw, as a 3x3 rotation."""
    cr, sr, cp, sp, cy, sy = (
        math.cos(roll), math.sin(roll),
        math.cos(pitch), math.sin(pitch),
        math.cos(yaw), math.sin(yaw),
    )
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def origin_transform(element: ET.Element | None) -> np.ndarray:
    """The 4x4 an <origin xyz rpy> stands for. A missing origin means identity, per the URDF spec."""
    transform = np.eye(4)
    if element is None:
        return transform
    xyz = [float(v) for v in (element.get("xyz") or "0 0 0").split()]
    rpy = [float(v) for v in (element.get("rpy") or "0 0 0").split()]
    transform[:3, :3] = rpy_to_matrix(*rpy)
    transform[:3, 3] = xyz
    return transform


def bake_link(link: ET.Element, root: Path) -> trimesh.Trimesh | None:
    """Every visual mesh on one link, placed in the link's frame and merged into a single body."""
    pieces = []
    for visual in link.findall("visual"):
        mesh_element = visual.find("geometry/mesh")
        if mesh_element is None:
            continue  # boxes and cylinders: the SO-101 doesn't use them, so nothing to do
        path = root / mesh_element.get("filename")
        if not path.exists():
            print(f"  missing {path.name}: skipped")
            continue
        piece = trimesh.load_mesh(path, process=False)
        scale = mesh_element.get("scale")
        if scale:
            piece.apply_scale([float(v) for v in scale.split()])
        piece.apply_transform(origin_transform(visual.find("origin")))
        pieces.append(piece)
    if not pieces:
        return None
    return trimesh.util.concatenate(pieces)


def thin(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """Fewer triangles, same silhouette. Returns the mesh untouched if it is already small.

    An STL is a heap of loose triangles: neighbours repeat the corners they share rather than pointing
    at one copy. Thinning that directly shatters the model, because nothing tells the algorithm which
    triangles are joined, so each collapses on its own and the surface falls apart. Welding matching
    corners together first turns the heap back into a surface, and then it thins properly.
    """
    mesh.merge_vertices()
    if len(mesh.faces) <= TARGET_FACES:
        return mesh
    try:
        return mesh.simplify_quadric_decimation(face_count=max(MIN_FACES, TARGET_FACES))
    except Exception as error:  # a fresh install without fast-simplification: full detail still works
        print(f"  (not thinned: {error})")
        return mesh


def main() -> None:
    if not URDF.exists():
        raise SystemExit(f"no model at {URDF} -- see the README for where to download it")
    OUT.mkdir(parents=True, exist_ok=True)
    root = URDF.parent
    tree = ET.parse(URDF).getroot()

    total_before = total_after = 0
    for link in tree.findall("link"):
        name = link.get("name")
        merged = bake_link(link, root)
        if merged is None:
            continue  # a frame with no geometry, such as the gripper tip
        before = len(merged.faces)
        merged = thin(merged)
        destination = OUT / f"{name}.stl"
        merged.export(destination)
        size = destination.stat().st_size
        total_before += before
        total_after += len(merged.faces)
        print(f"{name:<26} {before:>7} -> {len(merged.faces):>6} faces  {size / 1024:>7.0f} KB")

    served = sum(p.stat().st_size for p in OUT.glob("*.stl"))
    print(f"\n{total_before} -> {total_after} faces, {served / 1024 / 1024:.1f} MB in {OUT}")


if __name__ == "__main__":
    main()
