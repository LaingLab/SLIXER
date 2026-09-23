"""The parts with no network in them: the arm's geometry, lining the model up, and programs."""

from __future__ import annotations

import json
import math

import numpy as np
import pytest

from kinematics import Chain
from mapping import GripperMap, JointMap, Mapping
from program import Program, Runner, Step, legacy_arm


@pytest.fixture(scope="module")
def chain() -> Chain:
    return Chain.load()


# ---- kinematics ------------------------------------------------------------------------------------------

def test_the_model_is_the_so101(chain):
    assert chain.motors == ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
    tip = chain.tip_position([0] * 6)
    assert np.allclose(tip, [0.3914, 0.0, 0.2265], atol=1e-3)  # stretched out flat: about 45 cm of reach


def test_ik_reaches_nearly_everything_reachable_and_stays_in_limits(chain):
    rng = np.random.default_rng(3)
    limits = chain.limits()
    reached = 0
    for _ in range(120):
        truth = [rng.uniform(lo * 0.85, hi * 0.85) for lo, hi in limits]
        seed = [rng.uniform(lo * 0.5, hi * 0.5) for lo, hi in limits]
        angles, error, ok = chain.solve_ik(chain.tip_position(truth), seed)
        reached += ok
        assert all(lo - 1e-9 <= a <= hi + 1e-9 for a, (lo, hi) in zip(angles, limits))
        assert angles[5] == pytest.approx(seed[5])  # IK never touches the gripper, even after a restart
    assert reached >= 114  # 95 %; measured 99 %


def test_ik_on_an_unreachable_point_fails_calmly(chain):
    angles, error, ok = chain.solve_ik(np.array([2.0, 2.0, 2.0]), [0] * 6)
    assert not ok and error > 1.0
    assert all(math.isfinite(a) for a in angles)


# ---- lining the model up ---------------------------------------------------------------------------------

def test_mapping_round_trips_through_signs_and_offsets():
    mapping = Mapping()
    mapping.joints["shoulder_pan"] = JointMap(sign=-1, offset_deg=12.5)
    mapping.joints["elbow_flex"] = JointMap(sign=1, offset_deg=-30.0)
    mapping.gripper = GripperMap(closed_deg=-5.0, open_deg=80.0)
    arm = [10.0, -20.0, 35.0, 5.0, -60.0]
    model = mapping.to_model(arm, 42.0)
    back, percent = mapping.to_arm(model)
    assert back == pytest.approx(arm)
    assert percent == pytest.approx(42.0)
    assert math.degrees(model[0]) == pytest.approx(-10.0 + 12.5)  # flipped, then shifted


def test_zero_here_makes_the_present_pose_the_models_zero():
    mapping = Mapping()
    mapping.joints["wrist_flex"] = JointMap(sign=-1, offset_deg=0.0)
    pose = [15.0, -40.0, 60.0, 22.0, 5.0]
    mapping.zero_here(pose)
    assert mapping.to_model(pose, 0)[:5] == pytest.approx([0.0] * 5, abs=1e-12)


def test_mapping_survives_a_broken_file(data_dir):
    from mapping import CONFIG_PATH

    CONFIG_PATH.write_text("{ this is not json")
    assert Mapping.load().joints["shoulder_pan"].sign == 1  # defaults, rather than refusing to start
    CONFIG_PATH.unlink()


# ---- programs --------------------------------------------------------------------------------------------

def test_moves_store_the_arms_own_numbers_not_the_models():
    step = Step.from_json({"kind": "move", "arm": [10, 20, 30, 40, 50, 60], "speed": 40})
    assert step.to_json()["arm"] == [10, 20, 30, 40, 50, 60]
    assert "pose" not in step.to_json()


def test_programs_saved_by_the_first_version_convert_exactly():
    # The first Slixer stored model radians under the default mapping; nothing else ever existed.
    arm = [-3.4, -104.5, 95.3, -99.2, -1.8, 2.9]
    saved = Mapping().to_model(arm[:5], arm[5])
    assert legacy_arm(saved) == pytest.approx(arm, abs=1e-9)
    assert Step.from_json({"kind": "move", "pose": saved}).arm == pytest.approx(arm, abs=1e-9)


def test_recalibrating_cannot_change_what_a_program_tells_the_arm():
    """The reason moves store the arm's numbers: flip a joint after recording and the arm must not care."""
    program = Program(steps=[Step(kind="move", arm=[25.0, 0.0, 0.0, -15.0, 0.0, 50.0])])
    for mapping in (Mapping(), _flipped()):
        resolved = program.resolved(mapping)
        degrees, percent = mapping.to_arm(resolved.steps[0].pose)
        assert list(degrees) + [percent] == pytest.approx([25.0, 0.0, 0.0, -15.0, 0.0, 50.0])


def _flipped() -> Mapping:
    mapping = Mapping()
    mapping.joints["shoulder_pan"] = JointMap(sign=-1, offset_deg=12.0)
    mapping.joints["wrist_flex"] = JointMap(sign=-1, offset_deg=0.0)
    return mapping


@pytest.mark.parametrize("bad", [
    {"kind": "move", "arm": [1, 2, 3]},
    {"kind": "move", "arm": [1, 2, 3, 4, 5, float("nan")]},
    {"kind": "move"},
    {"kind": "teleport"},
])
def test_bad_steps_are_refused(bad):
    with pytest.raises(ValueError):
        Step.from_json(bad)


@pytest.mark.parametrize("name", ["../../etc/passwd", "a/b", "", "x" * 65, "dot.json"])
def test_program_names_cannot_escape_the_folder(name):
    with pytest.raises(ValueError):
        Program(name=name).save()


def test_programs_save_and_load(data_dir):
    program = Program(name="round trip", loop=True, steps=[
        Step(kind="move", arm=[1, 2, 3, 4, 5, 6], speed=55),
        Step(kind="wait_for", label="red block", seconds=0),
    ])
    program.save()
    again = Program.load("round trip")
    assert again.to_json() == program.to_json()
    assert "round trip" in Program.saved()
    Program.delete("round trip")
    assert "round trip" not in Program.saved()


def _run(program: Program, start, seen=None, seconds: float = 20.0, hz: float = 50.0):
    """Plays a program on a clock that isn't real, and returns every pose it asked for."""
    runner = Runner(program.resolved(Mapping()), start_pose=list(start))
    here, poses, t = list(start), [], 0.0
    while t < seconds:
        wanted = runner.tick(t, here, seen)
        if wanted is None:
            break
        poses.append(list(wanted))
        here = wanted
        t += 1.0 / hz
    return runner, poses, t


def test_a_move_eases_in_and_out_and_arrives():
    program = Program(steps=[Step(kind="move", arm=[40, 0, 0, 0, 0, 50], speed=40)])
    runner, poses, took = _run(program, Mapping().to_model([0] * 5, 50))
    pans = [math.degrees(p[0]) for p in poses]
    assert pans[-1] == pytest.approx(40.0)
    assert took == pytest.approx(1.0, abs=0.05)  # 40 degrees at 40 degrees a second
    speeds = [abs(b - a) * 50 for a, b in zip(pans, pans[1:])]
    assert speeds[0] < 5 and speeds[-1] < 5  # still at both ends
    assert max(speeds) == pytest.approx(60.0, rel=0.05)  # smoothstep peaks at 1.5x the average


def test_wait_for_goes_on_seeing_and_gives_up_after_its_time():
    move = Step(kind="move", arm=[10, 0, 0, 0, 0, 50], speed=60)
    watching = Program(steps=[Step(kind="wait_for", label="red", seconds=0), move])
    runner, _, _ = _run(watching, [0] * 6, seen={"red"})
    assert runner.finished and runner.message == "finished"

    runner, _, took = _run(Program(steps=[Step(kind="wait_for", label="purple", seconds=2.0), move]),
                           [0] * 6, seen={"red"})
    assert "gave up" in runner.message and took == pytest.approx(2.0, abs=0.05)


def test_a_looping_program_goes_round():
    program = Program(loop=True, steps=[Step(kind="wait", seconds=0.2)])
    runner, _, _ = _run(program, [0] * 6, seconds=1.0)
    assert not runner.finished and runner.laps >= 3


def test_duration_estimate():
    program = Program(steps=[
        Step(kind="move", arm=[0, 0, 0, 0, 0, 0], speed=40),
        Step(kind="move", arm=[60, 0, 0, 0, 0, 0], speed=40),  # 1.5 s
        Step(kind="gripper", percent=100),  # 0.6 s
        Step(kind="wait", seconds=1.0),
    ])
    assert program.duration() == pytest.approx(3.1)


def test_saved_programs_from_this_afternoon_still_load(data_dir):
    """The four programs recorded on 2026-09-22, as they were saved, still open and still make sense."""
    from pathlib import Path

    real = Path(__file__).resolve().parent.parent / "programs"
    backup = real / ".before-arm-units-2026-09-22"
    source = backup if backup.exists() else real
    for path in sorted(source.glob("*.json")):
        raw = json.loads(path.read_text())
        program = Program.from_json(raw)
        assert program.steps, path.name
        for step in program.steps:
            if step.kind == "move":
                assert all(-200 < v < 200 for v in step.arm[:5]) and 0 <= step.arm[5] <= 100


def test_the_version_shown_is_the_version_released():
    import re
    from pathlib import Path

    from version import VERSION

    pyproject = (Path(__file__).resolve().parents[2] / "pyproject.toml").read_text()
    assert re.search(r'^version = "([^"]+)"', pyproject, re.M).group(1) == VERSION

