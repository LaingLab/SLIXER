"""The YOLO model and the training pictures, against the real worker on this machine's GPU.

These need the vision extra (uv sync --extra vision) and a stock model already downloaded, and are
skipped without them rather than fetching anything.
"""

from __future__ import annotations

import os
import time
from pathlib import Path

import pytest

import paths
from dataset import Collector, folder
from detection import ModelRunner, available_models
from helpers import wait_for

BUS = None
if paths.VISION_PYTHON.exists():
    candidates = list(paths.VISION_PYTHON.parent.parent.glob("lib/python*/site-packages/ultralytics/assets/bus.jpg"))
    BUS = candidates[0] if candidates else None
MODEL = "yolo26n-seg.pt"
needs_model = pytest.mark.skipif(not (BUS and (paths.MODELS / MODEL).exists()),
                                 reason="no vision extra, or yolo26n-seg.pt not downloaded")


class Camera:
    """ArmCamera's interface, serving one JPEG as a fresh frame on every read."""

    def __init__(self, jpeg: bytes):
        self.jpeg, self.decode, self.frames_received = jpeg, False, 0
        self.host, self.port, self.connected = "test", 0, True

    def read_jpeg(self):
        self.frames_received += 1
        return self.jpeg, 0.01

    def read(self):
        return None, float("inf")


class Seen:
    seen: set = set()
    mode = "watch"
    connected = False

    def current_arm_pose(self):
        return [1.0, 2.0, 3.0, 4.0, 5.0, 50.0]


def test_the_model_list_offers_stock_and_trained_models():
    names = [m["name"] for m in available_models()]
    assert "yolo26s-seg.pt" in names and "yolo26n.pt" in names


def test_a_model_name_cannot_reach_outside_the_models_folder():
    runner = ModelRunner(lambda: None)
    for bad in ("../../etc/passwd", "sub/dir.pt", "notamodel.txt", "missing-custom.pt"):
        with pytest.raises(ValueError):
            runner.start(bad)


def test_without_the_vision_environment_it_says_so():
    runner = ModelRunner(lambda: None, python=Path("/nonexistent/python"))
    assert "uv sync --extra vision" in runner.start(MODEL)


def test_settings_saved_by_the_colour_version_still_load(data_dir):
    import json

    from vision import VISION_FILE, Vision

    VISION_FILE.write_text(json.dumps({
        "enabled": True, "detectors": [{"id": "a1", "label": "red", "hue": 177.0}],  # taught colours: gone now
        "model_on": False, "model": {"model": "yolo26n-seg.pt", "conf": 0.5, "classes": ["cup"]},
    }))
    vision = Vision(Seen(), camera=lambda: None)
    assert vision.model.config == {"model": "yolo26n-seg.pt", "conf": 0.5, "classes": ["cup"]}
    vision.save()
    assert set(json.loads(VISION_FILE.read_text())) == {"model_on", "model"}


def test_switching_the_model_off_forgets_what_it_saw_at_once(data_dir):
    from vision import Vision

    session = Seen()
    session.seen = {"cup"}
    vision = Vision(session, camera=lambda: None)
    vision.model_on = True
    vision.set_model(False)
    assert session.seen == set()  # a program waiting for "cup" must not go on a sighting from before
    assert vision.labels() == []


def test_a_model_that_was_on_comes_back_looking_for_the_same_things(data_dir):
    import json

    from vision import VISION_FILE, Vision

    class Idle(ModelRunner):
        installed = True  # as if the vision extra were there

        def _run(self):  # no worker: this is about what it would be started with
            pass

    VISION_FILE.write_text(json.dumps({"model_on": True, "model": {
        "model": "yoloe-26s-seg.pt", "conf": 0.4, "classes": ["well plate"]}}))
    runner = Idle(lambda: None)
    vision = Vision(Seen(), camera=lambda: None, runner=runner)
    vision.start()  # as Slixer does when it starts
    try:
        assert runner.state == "starting", runner.problem
        assert runner.config == {"model": "yoloe-26s-seg.pt", "conf": 0.4, "classes": ["well plate"]}
        runner.retune(conf=0.6)  # a new threshold alone keeps the words
        assert runner.config["classes"] == ["well plate"]
        assert vision.set_model(True, "yolo26n-seg.pt", 0.4, None) == ""  # while None means: everything
        assert runner.config["classes"] is None
    finally:
        vision.stop()


def test_a_model_that_was_on_but_cannot_start_says_why(data_dir):
    import json

    from vision import VISION_FILE, Vision

    VISION_FILE.write_text(json.dumps({"model_on": True, "model": {"model": "yolo26n-seg.pt"}}))
    vision = Vision(Seen(), camera=lambda: None, runner=ModelRunner(lambda: None, python=Path("/nonexistent/python")))
    vision.start()
    try:
        assert vision.model.state == "failed"
        assert "uv sync --extra vision" in vision.model.problem
    finally:
        vision.stop()


@needs_model
def test_the_worker_finds_and_outlines_on_the_gpu():
    camera = Camera(BUS.read_bytes())  # one camera, as Slixer has: a new one per read would reset its count
    runner = ModelRunner(lambda: camera)
    try:
        assert runner.start(MODEL, conf=0.3) == ""
        wait_for(lambda: runner.state == "failed" or runner.current(), 60, "no answer from the model")
        assert runner.state == "running", runner.problem
        found = runner.current()
        labels = {hit["label"] for hit in found}
        assert {"bus", "person"} <= labels
        assert all(hit.get("polygon") for hit in found)  # a -seg model outlines what it finds
        assert all(0 <= v <= 1 for hit in found for v in hit["box"])  # fractions of the picture
    finally:
        runner.stop()


@needs_model
def test_a_restarted_model_does_not_count_the_time_it_was_off_in_its_rate():
    camera = Camera(BUS.read_bytes())  # a fresh frame on every read: the rate is the model's own, tens a second
    runner = ModelRunner(lambda: camera)
    try:
        runner.start(MODEL)
        wait_for(lambda: runner.rate > 0, 60, "no rate from the model")
        runner.stop()
        time.sleep(2)
        before = camera.frames_received
        runner.start(MODEL)
        rates = []
        # Every rate shown after the restart, past the thirtieth new answer -- until then, answers from
        # before the stop would still be in the average if the restart didn't clear them.
        wait_for(lambda: rates.append(runner.rate) or camera.frames_received >= before + 32
                 or runner.state == "failed", 60, "no answers after the restart")
        assert runner.state == "running", runner.problem
        shown = [rate for rate in rates if rate > 0]
        assert shown, "no rate shown after the restart"
        assert min(shown) > 15, f"a rate of {min(shown):.1f} a second counted the time the model was off"
    finally:
        runner.stop()


@needs_model
def test_retuning_changes_the_threshold_without_a_reload():
    camera = Camera(BUS.read_bytes())
    runner = ModelRunner(lambda: camera)
    try:
        runner.start(MODEL, conf=0.25)
        wait_for(lambda: runner.current(), 60, "no answer from the model")
        process = runner._process
        loose = runner.current()
        runner.retune(conf=0.85)
        time.sleep(1.5)
        strict = runner.current()
        assert runner._process is process  # the same worker: nothing was reloaded
        assert strict, "no answers after retuning"  # not vacuous: there must be something to check
        assert all(hit["score"] >= 0.85 for hit in strict)
        assert len(strict) < len(loose)  # something below 85% was dropped
    finally:
        runner.stop()


@needs_model
def test_a_capture_saves_the_picture_its_pose_and_draft_labels(data_dir):
    from vision import Vision

    jpeg = BUS.read_bytes()
    camera = Camera(jpeg)
    session = Seen()
    vision = Vision(session, camera=lambda: camera, runner=ModelRunner(lambda: camera))
    collector = Collector(camera=lambda: camera, session=session, vision=vision)
    try:
        assert vision.set_model(True, MODEL, 0.4) == ""
        wait_for(lambda: vision.model.current(), 60, "no answer from the model")
        collector.name = "pytest"
        note = collector.capture()
        assert "draft labels" in note
        where = folder("pytest")
        image = next((where / "images").glob("*.jpg"))
        assert image.read_bytes() == jpeg  # exactly as the camera sent it
        labels = (where / "labels" / f"{image.stem}.txt").read_text().splitlines()
        classes = (where / "classes.txt").read_text().split()
        assert "bus" in classes and "person" in classes
        first = labels[0].split()
        assert int(first[0]) < len(classes) and len(first) > 5  # an outline: class, then x y pairs
        assert (where / "poses" / f"{image.stem}.json").exists()
        # An unchanging scene isn't captured twice automatically.
        assert "skipped" in collector.capture(automatic=True)
    finally:
        vision.model.stop()


def test_dataset_names_are_kept_inside_the_datasets_folder():
    for bad in ("../x", "a/b", "", "x" * 41, "dot.name"):
        with pytest.raises(ValueError):
            folder(bad)


OPEN = "yoloe-26s-seg.pt"
needs_open = pytest.mark.skipif(not (BUS and (paths.MODELS / OPEN).exists()), reason="yoloe-26s-seg.pt not downloaded")


@needs_open
def test_an_open_vocabulary_model_finds_what_it_is_told_to_and_can_be_retold():
    camera = Camera(BUS.read_bytes())
    runner = ModelRunner(lambda: camera)
    try:
        with pytest.raises(ValueError):
            runner.start(OPEN)  # it needs words
        runner.start(OPEN, conf=0.3, classes=["bus", "person"])
        wait_for(lambda: runner.current() or runner.state == "failed", 90, "no answer")
        assert {h["label"] for h in runner.current()} == {"bus", "person"}
        runner.retune(classes=["bus"])
        wait_for(lambda: {h["label"] for h in runner.current()} == {"bus"}, 10, "never re-told")
    finally:
        runner.stop()


# ---- semantic models, and models that fail ---------------------------------------------------------------

def test_the_patches_a_semantic_model_marks_become_outlined_finds():
    import cv2
    import numpy as np

    from regions import patches

    marked = np.zeros((720, 1280), np.uint8)  # a model of one class marks it 1 on a background of 0
    cv2.circle(marked, (300, 200), 60, 1, -1)
    cv2.rectangle(marked, (800, 400), (900, 470), 1, -1)
    marked[700:703, 10:13] = 1  # a speck, not a thing
    found = patches(marked, {0: "Class 0"})
    assert [hit["label"] for hit in found] == ["Class 0", "Class 0"]
    circle, square = found  # the biggest first
    assert circle["score"] is None and circle["cls"] == 0  # a class map has no confidence to give
    assert circle["centre"] == pytest.approx([300 / 1280, 200 / 720], abs=0.002)
    assert square["box"] == pytest.approx([800 / 1280, 400 / 720, 901 / 1280, 471 / 720], abs=0.002)
    assert all(0 <= v <= 1 for hit in found for point in hit["polygon"] for v in point)
    assert 4 <= len(square["polygon"]) <= 80 and len(circle["polygon"]) <= 80


def test_background_and_pixels_outside_the_classes_asked_for_are_not_finds():
    import numpy as np

    from regions import patches

    marked = np.zeros((480, 640), np.uint8)  # a model of several classes, the first of them the background
    marked[50:150, 50:200] = 1
    marked[300:400, 400:500] = 2
    marked[0:40, 500:640] = 255  # Ultralytics' mark for a class that wasn't asked for
    found = patches(marked, {0: "background", 1: "plate", 2: "tip"})
    assert sorted((hit["label"], hit["cls"]) for hit in found) == [("plate", 1), ("tip", 2)]


class Runnable(ModelRunner):
    installed = True  # the stand-in worker needs no vision extra


def stand_in(monkeypatch, fails: str) -> tuple[Camera, ModelRunner]:
    """A ModelRunner whose worker is tests/fake_worker.py, failing on `fails` pictures."""
    import sys

    import detection

    monkeypatch.setattr(detection, "WORKER", Path(__file__).resolve().parent / "fake_worker.py")
    monkeypatch.setenv("FAKE_WORKER_FAILS", fails)
    camera = Camera(b"a picture")
    return camera, Runnable(lambda: camera, python=Path(sys.executable))


def test_a_model_that_fails_on_every_picture_stops_and_says_why(data_dir, monkeypatch):
    camera, runner = stand_in(monkeypatch, "every")
    try:
        assert runner.start(MODEL) == ""
        wait_for(lambda: runner.state == "failed", 10, "a model failing on every picture was left running")
        assert "in a row" in runner.problem and "NoneType" in runner.problem  # why: not "loading" for ever
        assert runner.current() == []
    finally:
        runner.stop()


def test_a_bad_picture_now_and_then_does_not_stop_a_model(data_dir, monkeypatch):
    camera, runner = stand_in(monkeypatch, "sometimes")
    try:
        assert runner.start(MODEL) == ""
        wait_for(lambda: camera.frames_received > 90 or runner.state == "failed", 10, "the model stopped answering")
        assert runner.state == "running", runner.problem  # a third of them failed, but never twenty in a row
        assert [hit["label"] for hit in runner.current()] == ["thing"]
    finally:
        runner.stop()
