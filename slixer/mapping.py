"""Lines up the 3D model with your actual arm.

The model and the hardware measure the same joints differently. The URDF has its own zero for each joint,
decided by whoever built the model. Your arm reports degrees either side of the middle of the range you
recorded during calibration, and which way is positive depends on how that servo happened to be seated
when it was assembled. Nobody can know your signs and offsets in advance -- they have to be looked at.

So this holds one sign and one offset per joint, saved to a file you can edit from the viewer: flip a
joint that moves the wrong way, nudge one that sits at an angle, and the model lines up with the arm.
Until it does, the model is a pretty picture; after it does, it is a measurement.

    numbers = Mapping.load()
    radians = numbers.to_model(state.degrees[:5], state.gripper_percent)
    degrees, percent = numbers.to_arm(radians)
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path

import paths

CONFIG_PATH = paths.MAPPING
BODY = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")


@dataclass
class JointMap:
    """One joint: which way round it turns, and where its zero sits.

    model_radians = radians(sign * arm_degrees + offset_deg)
    """

    sign: int = 1
    offset_deg: float = 0.0

    def to_model(self, arm_degrees: float) -> float:
        return math.radians(self.sign * arm_degrees + self.offset_deg)

    def to_arm(self, model_radians: float) -> float:
        return (math.degrees(model_radians) - self.offset_deg) / self.sign


@dataclass
class GripperMap:
    """The gripper is a percentage, not an angle: 0% shut, 100% wide open."""

    closed_deg: float = 0.0
    open_deg: float = 90.0

    def to_model(self, percent: float) -> float:
        fraction = max(0.0, min(100.0, percent)) / 100.0
        return math.radians(self.closed_deg + fraction * (self.open_deg - self.closed_deg))

    def to_arm(self, model_radians: float) -> float:
        span = self.open_deg - self.closed_deg
        if abs(span) < 1e-6:
            return 0.0
        return max(0.0, min(100.0, (math.degrees(model_radians) - self.closed_deg) / span * 100.0))


@dataclass
class Mapping:
    """How all six joints line up. Saved beside this file so it survives a restart."""

    joints: dict[str, JointMap] = field(default_factory=lambda: {name: JointMap() for name in BODY})
    gripper: GripperMap = field(default_factory=GripperMap)

    @classmethod
    def load(cls, path: Path = CONFIG_PATH) -> "Mapping":
        if not path.exists():
            return cls()
        try:
            raw = json.loads(path.read_text())
        except (json.JSONDecodeError, OSError):
            return cls()  # a half-written file shouldn't stop the arm coming up
        joints = {
            name: JointMap(
                sign=1 if int(raw.get("joints", {}).get(name, {}).get("sign", 1)) >= 0 else -1,
                offset_deg=float(raw.get("joints", {}).get(name, {}).get("offset_deg", 0.0)),
            )
            for name in BODY
        }
        gripper = GripperMap(
            closed_deg=float(raw.get("gripper", {}).get("closed_deg", 0.0)),
            open_deg=float(raw.get("gripper", {}).get("open_deg", 90.0)),
        )
        return cls(joints=joints, gripper=gripper)

    def save(self, path: Path = CONFIG_PATH) -> None:
        # Written to one side and moved into place, so an interrupted save can't leave a broken file.
        temporary = path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(asdict(self), indent=2) + "\n")
        temporary.replace(path)

    def to_model(self, body_degrees, gripper_percent: float) -> list[float]:
        """Five joint angles and a gripper percentage from the arm, as six model radians."""
        return [self.joints[name].to_model(float(d)) for name, d in zip(BODY, body_degrees)] + [
            self.gripper.to_model(float(gripper_percent))
        ]

    def to_arm(self, model_radians) -> tuple[list[float], float]:
        """Six model radians, as the five degrees and gripper percentage the arm understands."""
        body = [self.joints[name].to_arm(float(r)) for name, r in zip(BODY, model_radians[:5])]
        return body, self.gripper.to_arm(float(model_radians[5]))

    def zero_here(self, body_degrees) -> None:
        """Takes the arm's present pose to be the model's zero pose.

        Point the real arm at whatever the model shows with every slider at zero, then call this: the
        offsets shift so the two agree. Signs are left alone -- a wrong sign shows up as a joint moving
        the wrong way, which no offset can fix.
        """
        for name, degrees in zip(BODY, body_degrees):
            self.joints[name].offset_deg = -self.joints[name].sign * float(degrees)

    def describe(self) -> dict:
        return asdict(self)
