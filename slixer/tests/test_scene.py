"""Imported parts."""

from __future__ import annotations

import io

import pytest
import trimesh

from scene import MESH_DIR, Scene


def stl(mesh) -> bytes:
    buffer = io.BytesIO()
    mesh.export(buffer, file_type="stl")
    return buffer.getvalue()


# ---- parts -----------------------------------------------------------------------------------------------

def test_a_part_exported_in_millimetres_is_scaled_and_says_so(data_dir):
    scene = Scene()
    item = scene.add(stl(trimesh.creation.box(extents=[120, 80, 40])), "bracket")
    assert item.scale == 0.001
    assert "millimetres" in item.note


def test_a_part_exported_in_metres_is_left_alone(data_dir):
    item = Scene().add(stl(trimesh.creation.box(extents=[0.12, 0.08, 0.04])), "bracket")
    assert item.scale == 1.0


def test_a_heavy_part_is_thinned_without_breaking_its_surface(data_dir):
    item = Scene().add(stl(trimesh.creation.icosphere(subdivisions=6, radius=0.05)), "ball")
    stored = trimesh.load_mesh(MESH_DIR / f"{item.id}.stl", process=False)
    stored.merge_vertices()
    assert len(stored.faces) <= 8000
    assert len(stored.faces) / len(stored.vertices) == pytest.approx(2.0, abs=0.05)  # still one closed skin


def test_parts_persist_move_and_go(data_dir):
    scene = Scene()
    item = scene.add(stl(trimesh.creation.box(extents=[50, 50, 50])), "cube")
    scene.update(item.id, {"position": [0.3, 0.1, 0.025], "rotation": [0, 0, 1.57], "name": "fixture"})
    again = Scene()
    found = again.find(item.id)
    assert found.name == "fixture" and found.position == [0.3, 0.1, 0.025]
    again.remove(item.id)
    assert Scene().find(item.id) is None
    assert not (MESH_DIR / f"{item.id}.stl").exists()


@pytest.mark.parametrize("data", [b"not an stl", b""])
def test_rubbish_is_refused(data_dir, data):
    with pytest.raises(ValueError):
        Scene().add(data, "rubbish")


def test_bad_placements_are_refused(data_dir):
    scene = Scene()
    item = scene.add(stl(trimesh.creation.box(extents=[50, 50, 50])), "cube")
    with pytest.raises(ValueError):
        scene.update(item.id, {"position": [float("nan"), 0, 0]})
    with pytest.raises(ValueError):
        scene.update(item.id, {"position": [0.1, 0.2]})  # three numbers or nothing
    with pytest.raises(KeyError):
        scene.update("nope", {"scale": 2})
