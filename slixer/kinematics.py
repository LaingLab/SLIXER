"""Where the arm's parts are, for any set of joint angles.

This reads the same URDF the viewer draws, so the 3D model and the maths can't drift apart: the browser
is told the shape of the arm by this module, and asks this module where the joints should put it.

Angles here are URDF radians -- the model's own convention. Turning your arm's calibrated degrees into
these is `mapping.py`'s job, because that depends on how your servos were assembled and calibrated.

    chain = Chain.load()
    places = chain.link_poses([0, 0, 0, 0, 0, 0])   # 4x4 for every link
    where = chain.tip_position([0, 0, 0, 0, 0, 0])  # the gripper tip in metres
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np

URDF_PATH = Path(__file__).resolve().parent / "assets" / "so101" / "so101.urdf"

# The six motors, in the order every other part of this project uses.
MOTORS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
TIP_LINK = "gripper_frame_link"  # the frame at the gripper's tip, which is what IK aims


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


def _origin(element: ET.Element | None) -> np.ndarray:
    transform = np.eye(4)
    if element is None:
        return transform
    transform[:3, :3] = rpy_to_matrix(*[float(v) for v in (element.get("rpy") or "0 0 0").split()])
    transform[:3, 3] = [float(v) for v in (element.get("xyz") or "0 0 0").split()]
    return transform


def axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rotation about an arbitrary axis through the origin (Rodrigues), as a 4x4."""
    x, y, z = axis
    c, s, t = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    transform = np.eye(4)
    transform[:3, :3] = np.array([
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ])
    return transform


@dataclass
class Joint:
    """One connection between two links: where it sits, which way it turns, and how far."""

    name: str
    kind: str  # "revolute", "continuous", "prismatic" or "fixed"
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float

    @property
    def moves(self) -> bool:
        return self.kind != "fixed"

    def transform(self, angle: float) -> np.ndarray:
        """Where this joint puts its child link, relative to its parent."""
        if not self.moves:
            return self.origin
        if self.kind == "prismatic":
            slide = np.eye(4)
            slide[:3, 3] = self.axis * angle
            return self.origin @ slide
        return self.origin @ axis_rotation(self.axis, angle)

    def clamp(self, angle: float) -> float:
        return angle if self.kind == "continuous" else max(self.lower, min(self.upper, angle))


class Chain:
    """The arm's shape, and the sums that turn joint angles into positions."""

    def __init__(self, joints: list[Joint], links: list[str], root: str):
        self.joints = {j.name: j for j in joints}
        self.order = [j.name for j in joints]  # parents always before children
        self.links = links
        self.root = root
        self.motors = [name for name in MOTORS if name in self.joints]

    @classmethod
    def load(cls, path: Path = URDF_PATH) -> "Chain":
        tree = ET.parse(path).getroot()
        links = [link.get("name") for link in tree.findall("link")]

        parsed: dict[str, Joint] = {}
        for joint in tree.findall("joint"):
            axis_element = joint.find("axis")
            axis = np.array([float(v) for v in (axis_element.get("xyz") if axis_element is not None else "0 0 1").split()])
            norm = float(np.linalg.norm(axis))
            limit = joint.find("limit")
            name = joint.get("name")
            parsed[name] = Joint(
                name=name,
                kind=joint.get("type"),
                parent=joint.find("parent").get("link"),
                child=joint.find("child").get("link"),
                origin=_origin(joint.find("origin")),
                axis=axis / norm if norm else np.array([0.0, 0.0, 1.0]),
                lower=float(limit.get("lower")) if limit is not None and limit.get("lower") else -math.pi,
                upper=float(limit.get("upper")) if limit is not None and limit.get("upper") else math.pi,
            )

        children = {j.child for j in parsed.values()}
        roots = [name for name in links if name not in children]
        if len(roots) != 1:
            raise ValueError(f"expected one base link, found {roots}")

        # Sort so a joint always comes after the one that carries it: then one pass computes every pose.
        by_parent: dict[str, list[Joint]] = {}
        for joint in parsed.values():
            by_parent.setdefault(joint.parent, []).append(joint)
        ordered: list[Joint] = []
        queue = [roots[0]]
        while queue:
            for joint in by_parent.get(queue.pop(0), []):
                ordered.append(joint)
                queue.append(joint.child)
        if len(ordered) != len(parsed):
            raise ValueError("the model is not one connected tree: some links are unreachable from the base")
        return cls(ordered, links, roots[0])

    def angle_map(self, angles) -> dict[str, float]:
        """Motor angles, in this project's order, as a lookup every joint can be asked about."""
        return dict(zip(self.motors, angles))

    def link_poses(self, angles) -> dict[str, np.ndarray]:
        """A 4x4 for every link, in the base's frame. This is both the drawing and the forward kinematics."""
        values = self.angle_map(angles)
        poses = {self.root: np.eye(4)}
        for name in self.order:
            joint = self.joints[name]
            poses[joint.child] = poses[joint.parent] @ joint.transform(values.get(name, 0.0))
        return poses

    def tip_position(self, angles) -> np.ndarray:
        return self.link_poses(angles)[TIP_LINK][:3, 3]

    def limits(self) -> list[tuple[float, float]]:
        return [(self.joints[name].lower, self.joints[name].upper) for name in self.motors]

    def clamp(self, angles) -> list[float]:
        return [self.joints[name].clamp(float(a)) for name, a in zip(self.motors, angles)]

    # ---- inverse kinematics -------------------------------------------------
    #
    # Five joints steer the tip, so the arm can reach a point in space but not at every approach angle.
    # Asking only for a position keeps the problem honest; asking for a full orientation as well would
    # make it unsolvable in most places. The gripper is left out: it opens and shuts, it doesn't reach.

    def _jacobian(self, angles: np.ndarray, arm: list[str]) -> np.ndarray:
        """How the tip moves for a small turn of each joint, worked out from the joint axes."""
        poses = self.link_poses(angles)
        tip = poses[TIP_LINK][:3, 3]
        columns = []
        for name in arm:
            joint = self.joints[name]
            frame = poses[joint.parent] @ joint.origin
            axis_world = frame[:3, :3] @ joint.axis
            if joint.kind == "prismatic":
                columns.append(axis_world)
            else:
                columns.append(np.cross(axis_world, tip - frame[:3, 3]))
        return np.column_stack(columns)

    def solve_ik(
        self,
        target: np.ndarray,
        seed,
        iterations: int = 60,
        tolerance: float = 5e-4,
        restarts: int = 4,
        limits: list[tuple[float, float]] | None = None,
        max_change: float | None = None,
    ) -> tuple[list[float], float, bool]:
        """Joint angles that put the tip on `target`, starting from `seed`.

        Returns the angles, how far the tip ends up from the target in metres, and whether it got there.
        Damped least squares: near a singularity -- arm straight out, or folded -- an undamped solve asks
        for enormous joint moves to gain a millimetre, which on real hardware is a lurch. The damping
        trades a little accuracy for staying calm, and the result is always clamped to the joint limits.

        A five-joint arm has poses it can't climb out of, where every small move makes things worse even
        though a perfectly good answer exists elsewhere. So a failed solve is tried again from a few
        scattered starting points. Solving takes about a millisecond, and the arm's own pose is always
        tried first, so this costs nothing in the normal case of dragging the tip a short distance.

        `limits` replaces the model's own joint limits -- the real arm's calibrated range is often wider, and
        solving inside the model's would bend a real pose to fit. `max_change` keeps every joint within that
        many radians of the seed: near a folded pose the maths is badly conditioned and an unfenced solve
        can wander a hundred degrees to gain a centimetre, which a real arm would then do.
        """
        box = self._box(seed, limits, max_change)
        best_angles, best_error, reached = self._solve_once(target, seed, iterations, tolerance, box)
        if reached or restarts <= 0:
            return best_angles, best_error, reached

        # Search from scattered starts, but prefer the answer that changes the arm's shape least: the
        # first one found might be the arm folded the other way round, and a real arm would swing through
        # everything in between to get there.
        def swing(angles) -> float:
            return max(abs(a - b) for a, b in zip(angles[:5], seed[:5]))

        rng = np.random.default_rng(abs(hash(tuple(np.round(target, 6)))) % (2**32))
        reaching = []
        for _ in range(restarts):
            # Scatter the reaching joints only: the gripper isn't part of the solve, so a restart must
            # leave it exactly as the caller set it rather than randomly opening the hand.
            scattered = [rng.uniform(low, high) for low, high in self.limits()]
            for index, name in enumerate(self.motors):
                if name == "gripper":
                    scattered[index] = float(seed[index])
            angles, error, ok = self._solve_once(target, scattered, iterations, tolerance, box)
            if ok:
                reaching.append((swing(angles), angles, error))
        if reaching:
            _, angles, error = min(reaching, key=lambda r: r[0])
            return angles, error, True
        return best_angles, best_error, False  # none reach: the nearest from where the arm is, not a leap

    def _box(self, seed, limits, max_change) -> list[tuple[float, float]]:
        """Where each joint may go during a solve: within its limits, and within max_change of the seed."""
        box = []
        for index, name in enumerate(self.motors):
            joint = self.joints[name]
            low, high = limits[index] if limits else (joint.lower, joint.upper)
            if joint.kind == "continuous" and not limits:
                low, high = -math.inf, math.inf
            if max_change is not None:
                low, high = max(low, seed[index] - max_change), min(high, seed[index] + max_change)
                if low > high:  # the seed sits outside its limits: stay put rather than jump inside them
                    low = high = float(seed[index])
            box.append((low, high))
        return box

    @staticmethod
    def _fence(angles, box) -> list[float]:
        return [max(low, min(high, float(a))) for a, (low, high) in zip(angles, box)]

    def _solve_once(self, target, seed, iterations: int, tolerance: float, box) -> tuple[list[float], float, bool]:
        arm = [name for name in self.motors if name != "gripper"]
        angles = np.array([float(v) for v in seed], dtype=float)
        indexes = [self.motors.index(name) for name in arm]
        target = np.asarray(target, dtype=float)

        best = angles.copy()
        best_error = float(np.linalg.norm(target - self.tip_position(angles)))
        for _ in range(iterations):
            error = target - self.tip_position(angles)
            distance = float(np.linalg.norm(error))
            if distance < best_error:
                best, best_error = angles.copy(), distance
            if distance < tolerance:
                return self._fence(angles, box), distance, True
            jacobian = self._jacobian(angles, arm)
            damping = 0.02 + 0.5 * distance  # firmer when far away, gentler as it closes in
            step = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + damping**2 * np.eye(3), error
            )
            largest = float(np.max(np.abs(step))) if step.size else 0.0
            if largest > 0.2:  # never swing a joint more than ~11 degrees in one step
                step *= 0.2 / largest
            for index, delta in zip(indexes, step):
                low, high = box[index]
                angles[index] = max(low, min(high, angles[index] + delta))

        return self._fence(best, box), best_error, best_error < tolerance

    # ---- what the browser needs to draw it ----------------------------------

    def describe(self) -> dict:
        """The model's shape, as JSON, so the viewer doesn't need its own URDF parser."""
        return {
            "root": self.root,
            "links": self.links,
            "motors": self.motors,
            "tip": TIP_LINK,
            "joints": [
                {
                    "name": j.name,
                    "type": j.kind,
                    "parent": j.parent,
                    "child": j.child,
                    "origin": j.origin.flatten().tolist(),  # column-major is applied browser-side
                    "axis": j.axis.tolist(),
                    "lower": j.lower,
                    "upper": j.upper,
                    "actuated": j.name in self.motors,
                }
                for j in (self.joints[n] for n in self.order)
            ],
        }


if __name__ == "__main__":
    chain = Chain.load()
    print(f"{chain.root} -> {len(chain.joints)} joints, driving {chain.motors}")
    rest = [0.0] * len(chain.motors)
    print(f"tip at rest: {np.round(chain.tip_position(rest), 4)} m")
    for name, (low, high) in zip(chain.motors, chain.limits()):
        print(f"  {name:<15} {math.degrees(low):+7.1f} to {math.degrees(high):+7.1f} deg")
