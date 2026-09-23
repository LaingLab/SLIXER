"""Sequences of moves the arm can play back on its own.

A program is a list of plain steps -- go here, wait, open the gripper, repeat -- kept as JSON so you can
read it, diff it, and write one by hand if you'd rather. The interesting part is how a move is played:
not by sending the destination and letting the servos bolt to it, but by walking the arm there over a
sensible number of seconds, easing in and out so nothing lurches at either end.

A move stores **the arm's own numbers**: five joint angles in degrees from the middle of each joint's
calibrated travel, and the gripper's opening in percent -- the same numbers the arm reports and the leader
prints. Not the 3D model's angles. The model's angles depend on how the model has been lined up with the
arm (Setup), and that can change after a program is written; a program stored in model angles would then
send a joint somewhere new, possibly to its mirror image. The arm's own numbers mean the same thing for
ever, so a program replays exactly as recorded whatever happens to the model.

Nothing here talks to the arm or to the clock on its own. `Session` ticks a `Runner` and gets back the
pose the arm should be in at that instant, which keeps all the timing in one place and makes a program
testable without any hardware attached.

    runner = Runner(program.resolved(mapping), start_pose=here)
    pose = runner.tick(now, here)   # model radians; None once it has finished
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from mapping import Mapping

import paths

PROGRAM_DIR = paths.PROGRAMS
DEFAULT_SPEED = 40.0  # degrees per second, per joint: a gentle walking pace for these servos
MIN_DURATION = 0.15  # even a tiny move gets a moment, so the arm is never asked to teleport
SAFE_NAME = re.compile(r"^[A-Za-z0-9 _-]{1,64}$")


def legacy_arm(pose) -> list[float]:
    """The arm's numbers for a move saved by the first version of Slixer, which stored model angles.

    Every program saved that way was recorded before the model could be lined up with the arm -- there
    was no Setup tab -- so the model's angles were the default mapping's: joint degrees unchanged, and the
    gripper's 0-100 % spread over 0-90 degrees. Undoing exactly that recovers what the arm was told.
    """
    if not isinstance(pose, list) or len(pose) != 6:
        raise ValueError("a move step needs a pose of six joint angles")
    body = [math.degrees(float(v)) for v in pose[:5]]
    gripper = math.degrees(float(pose[5])) / 90.0 * 100.0
    return body + [max(0.0, min(100.0, gripper))]


def ease(fraction: float) -> float:
    """Smoothstep: starts still, ends still. The servos see acceleration rather than a step change."""
    fraction = max(0.0, min(1.0, fraction))
    return fraction * fraction * (3.0 - 2.0 * fraction)


@dataclass
class Step:
    """One instruction. `kind` decides which of the other fields matter."""

    kind: str  # "move", "wait", "gripper" or "wait_for"
    arm: list[float] | None = None  # for "move": five joint degrees then gripper percent, the arm's own
    seconds: float = 1.0  # for "wait", and how long a "wait_for" gives up after (0: never)
    percent: float = 50.0  # for "gripper"
    speed: float = DEFAULT_SPEED  # degrees per second
    label: str = ""  # for "wait_for": what the camera should be seeing
    note: str = ""  # free text, shown in the editor
    pose: list[float] | None = field(default=None, repr=False)  # model radians, worked out to run it

    @classmethod
    def from_json(cls, raw: dict) -> "Step":
        kind = str(raw.get("kind", "move"))
        if kind not in ("move", "wait", "gripper", "wait_for"):
            raise ValueError(f"unknown step type: {kind!r}")
        arm = raw.get("arm")
        if kind == "move":
            if arm is None and raw.get("pose") is not None:
                arm = legacy_arm(raw["pose"])
            if not isinstance(arm, list) or len(arm) != 6:
                raise ValueError("a move step needs six numbers: five joint angles and the gripper")
            if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in arm):
                raise ValueError("a move step's joint angles must be real numbers")
            arm = [max(-327.0, min(327.0, float(v))) for v in arm[:5]] + [max(0.0, min(100.0, float(arm[5])))]
        return cls(
            kind=kind,
            arm=arm,
            seconds=max(0.0, min(3600.0, float(raw.get("seconds", 1.0)))),
            percent=max(0.0, min(100.0, float(raw.get("percent", 50.0)))),
            speed=max(1.0, min(180.0, float(raw.get("speed", DEFAULT_SPEED)))),
            label=str(raw.get("label", ""))[:64],
            note=str(raw.get("note", ""))[:200],
        )

    def to_json(self) -> dict:
        return {
            "kind": self.kind,
            "arm": self.arm,
            "seconds": self.seconds,
            "percent": self.percent,
            "speed": self.speed,
            "label": self.label,
            "note": self.note,
        }


@dataclass
class Program:
    name: str = "untitled"
    steps: list[Step] = field(default_factory=list)
    loop: bool = False

    @classmethod
    def from_json(cls, raw: dict) -> "Program":
        return cls(
            name=str(raw.get("name", "untitled"))[:64],
            steps=[Step.from_json(s) for s in raw.get("steps", [])],
            loop=bool(raw.get("loop", False)),
        )

    def to_json(self) -> dict:
        return {"name": self.name, "loop": self.loop, "steps": [s.to_json() for s in self.steps]}

    def resolved(self, mapping: "Mapping") -> "Program":
        """A copy ready to run: every move's target worked out in the model's angles, as they stand now."""
        steps = []
        for step in self.steps:
            copy = Step(**{**step.__dict__})
            if step.kind == "move" and step.arm is not None:
                copy.pose = mapping.to_model(step.arm[:5], step.arm[5])
            steps.append(copy)
        return Program(name=self.name, steps=steps, loop=self.loop)

    def duration(self) -> float:
        """Roughly how long one pass takes, in seconds. Waits for the camera count as nothing."""
        total, here = 0.0, None
        for step in self.steps:
            if step.kind == "move" and step.arm is not None:
                if here is not None:
                    biggest = max(abs(a - b) for a, b in zip(step.arm[:5], here[:5]))
                    total += max(MIN_DURATION, biggest / step.speed)
                here = step.arm
            elif step.kind == "wait":
                total += step.seconds
            elif step.kind == "gripper":
                total += 0.6
        return total

    # ---- saved programs -----------------------------------------------------

    @staticmethod
    def _path(name: str) -> Path:
        # Names become filenames, so anything that could climb out of the folder is refused outright.
        if not SAFE_NAME.match(name):
            raise ValueError("a program name may only contain letters, numbers, spaces, - and _")
        return PROGRAM_DIR / f"{name}.json"

    def save(self) -> Path:
        PROGRAM_DIR.mkdir(parents=True, exist_ok=True)
        path = self._path(self.name)
        # Written beside and moved into place, so a crash mid-save can't leave half a program.
        temporary = path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(self.to_json(), indent=2) + "\n")
        temporary.replace(path)
        return path

    @classmethod
    def load(cls, name: str) -> "Program":
        return cls.from_json(json.loads(cls._path(name).read_text()))

    @staticmethod
    def saved() -> list[str]:
        if not PROGRAM_DIR.exists():
            return []
        return sorted(p.stem for p in PROGRAM_DIR.glob("*.json"))

    @classmethod
    def delete(cls, name: str) -> None:
        cls._path(name).unlink(missing_ok=True)


class Runner:
    """Plays a program, one tick at a time.

    Hand it the clock and the pose being commanded -- and, driving a real arm, where that arm actually is
    -- and it hands back where the arm should be. It never assumes the arm arrived: each move is planned
    from the pose the step started in, so a program that gets nudged part way through carries on from
    where it is, and a move is over only once the command has got there and the real arm has too.
    """

    def __init__(self, program: Program, start_pose: list[float]):
        self.program = program
        self.index = 0
        self.laps = 0
        self.finished = not program.steps
        self.message = ""
        self._from = list(start_pose)
        self._started: float | None = None
        self._duration = 0.0
        self._waiting_for = ""
        self._target: list[float] | None = None
        self._arrive_by = 0.0
        self.stalled = False  # stopped because the real arm didn't reach a waypoint
        self._closest = math.inf  # how near the real arm has come to this move's target, and when
        self._closest_at = 0.0

    # Set by Session: the fastest the command may move, and the joint limits it will be held to. A move is
    # never planned faster than the one allows, and "arrived" is judged against a target the other has
    # clamped -- otherwise a pose just outside the limits could never be reached, and the program would
    # wait for it for ever.
    max_speed: float | None = None
    clamp = staticmethod(lambda pose: list(pose))
    ARRIVED = math.radians(0.5)
    GIVE_UP_AFTER = 3.0  # seconds past its time: a move that still hasn't arrived is let go, not waited on
    # The real arm, when there is one, is judged more loosely: servos carrying a load settle a degree or two
    # short of where they're sent. It has arrived within ARM_ARRIVED -- or within ARM_NEAR once it has
    # stopped getting closer for ARM_SETTLE seconds. A real arm that gets no closer for GIVE_UP_AFTER seconds
    # stops the program, rather than letting it carry on from the wrong place: something is in the way. One
    # that is slow but still coming is waited for.
    ARM_ARRIVED = math.radians(3.0)
    ARM_NEAR = math.radians(8.0)
    ARM_SETTLE = 0.5
    ARM_PROGRESS = math.radians(0.3)  # closer by less than this isn't getting closer

    @property
    def step(self) -> Step | None:
        if self.finished or self.index >= len(self.program.steps):
            return None
        return self.program.steps[self.index]

    def status(self) -> dict:
        step = self.step
        return {
            "running": not self.finished,
            "index": self.index,
            "count": len(self.program.steps),
            "laps": self.laps,
            "kind": step.kind if step else "",
            "waiting_for": self._waiting_for,
            "message": self.message,
        }

    def resume(self) -> None:
        """After a pause (the arm out of touch), starts the current move again from where the command is.

        Carrying on from the clock would restart the move at full speed from a standstill: a lurch.
        """
        step = self.step
        if step is not None and step.kind in ("move", "gripper"):
            self._started = None

    def _advance(self, now: float, here: list[float]) -> None:
        self.index += 1
        self._started = None
        self._from = list(here)
        self._waiting_for = ""
        if self.index >= len(self.program.steps):
            if self.program.loop:
                self.index = 0
                self.laps += 1
            else:
                self.finished = True
                self.message = "finished"

    def tick(self, now: float, here: list[float], seen: set[str] | None = None,
             actual: list[float] | None = None) -> list[float] | None:
        """Where the arm should be now, or None once the program is done.

        `here` is the pose being commanded. `seen` is what the camera currently recognises, which is what a
        "wait_for" step watches; passing nothing means nothing is recognised, so such a step simply waits.
        `actual` is where the real arm is, when one is being driven: a move isn't over until it gets there.
        """
        step = self.step
        if step is None:
            return None

        if self._started is None:
            self._started = now
            self._from = list(here)
            self._closest, self._closest_at = math.inf, 0.0
            if step.kind in ("move", "gripper"):
                if step.kind == "move":
                    target = list(step.pose)
                    speed = step.speed
                else:
                    target = list(self._from)
                    target[5] = self._gripper_radians(step.percent)
                    speed = 150.0  # a gripper is quick; the speed cap still has the last word
                self._target = self.clamp(target)
                biggest = max(abs(math.degrees(a - b)) for a, b in zip(self._target, self._from))
                duration = biggest / speed
                if self.max_speed:
                    # Easing in and out peaks at one and a half times the average speed; plan the move so
                    # that peak stays inside the cap, or the command would lag behind the plan and the
                    # next step would start short of this one.
                    duration = max(duration, 1.5 * biggest / self.max_speed)
                self._duration = max(MIN_DURATION, duration)
                self._arrive_by = self._duration + self.GIVE_UP_AFTER
            else:
                self._duration = step.seconds

        elapsed = now - self._started

        if step.kind in ("move", "gripper") and self._target is not None:
            fraction = 1.0 if self._duration <= 0 else ease(elapsed / self._duration)
            pose = [a + (b - a) * fraction for a, b in zip(self._from, self._target)]
            if elapsed >= self._duration:
                # A move is over when the arm has got there, not when the clock says it should have.
                arrived = max(abs(a - b) for a, b in zip(here, self._target)) <= self.ARRIVED
                if actual is None:
                    if arrived or elapsed >= self._arrive_by:
                        self._advance(now, self._target)
                    return pose
                # The real arm too, not only the command sent to it. Its gripper is left out: one closed on
                # something is meant to stop short.
                off = max(abs(a - b) for a, b in zip(actual[:5], self._target[:5]))
                if off < self._closest - self.ARM_PROGRESS:
                    self._closest, self._closest_at = off, elapsed  # still getting closer
                stuck_for = elapsed - self._closest_at
                if arrived and (off <= self.ARM_ARRIVED or (off <= self.ARM_NEAR and stuck_for >= self.ARM_SETTLE)):
                    self._advance(now, self._target)
                elif stuck_for >= self.GIVE_UP_AFTER:
                    self.finished = self.stalled = True
                    self.message = f"stopped at step {self.index + 1}: the arm didn't get there -- is something in the way?"
            return pose

        if step.kind == "wait":
            if elapsed >= self._duration:
                self._advance(now, self._from)
            return list(self._from)  # hold still, don't go limp

        if step.kind == "wait_for":
            self._waiting_for = step.label
            if seen and step.label in seen:
                self._advance(now, self._from)
            elif step.seconds > 0 and elapsed >= step.seconds:
                self.finished = True
                self.message = f"gave up waiting for {step.label!r}"
            return list(self._from)

        self._advance(now, self._from)
        return list(self._from)

    # The gripper's percentage has to become an angle to be interpolated with the rest of the pose.
    # Session installs the real conversion; on its own a Runner assumes the model's own 0-90 degrees.
    gripper_radians = staticmethod(lambda percent: math.radians(percent * 0.9))

    def _gripper_radians(self, percent: float) -> float:
        return float(self.gripper_radians(percent))
