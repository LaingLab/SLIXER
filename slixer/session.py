"""The one place that talks to the arm.

Everything that wants to move the arm -- a slider, a dragged gripper, a running program -- goes through
here, so there is a single loop deciding what the arm is told and a single set of rules about what it is
allowed to be told. Two things sending poses independently is how you get a fight at 30 Hz.

There are three modes, and which one you are in decides what the model on screen means:

  watch  The model mirrors the real arm. Nothing is sent, and nothing you do moves anything. This is where
         you teach by demonstration: move the leader by hand and record where you put it.
  plan   The model comes loose from the real arm and becomes a virtual one. Pose it, record poses, play a
         program on it -- all without sending a single packet. This is where programs get written and
         rehearsed, against imported parts if you like, with the real arm left exactly where it is.
  drive  The model commands the real arm.

The rules for drive, in short:

  * Nothing is sent until you choose it. Opening the page cannot move your arm.
  * It starts from wherever the arm already is, so it never jumps on the first packet.
  * Every command is clamped to the joint limits the arm itself reported from its calibration.
  * Commands move at a walking pace: a big drag in the viewer becomes a smooth travel, not a lurch.
  * If every browser goes away, driving stops. The firmware then freezes the arm rather than letting go.
"""

from __future__ import annotations

import math
import sys
import threading
import time
import traceback
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "pc"))

from so101_link import PORT, Arm, ArmState  # noqa: E402

from kinematics import Chain  # noqa: E402
from mapping import CONFIG_PATH, Mapping  # noqa: E402
from program import Program, Runner  # noqa: E402

CONTROL_HZ = 50.0  # the leader's own rate: smaller steps, so the servos glide rather than hop
MAX_DEGREES_PER_SECOND = 90.0  # how fast a command is allowed to chase the target
ARM_TIMEOUT = 1.5  # no report for this long and we call the arm gone
LAG_LIMIT_DEG = 8.0  # the arm this far behind the command...
LAG_LIMIT_SECONDS = 1.0  # ...for this long means it isn't following, whatever the reason
STEERED_POSES_PER_SECOND = 5.0  # the follower acting on this many poses we didn't send: someone is steering
MAX_REACH_SWING = 60.0  # degrees: in Drive, a reach for a part that needs more is refused -- rehearse it first
DRAG_STEP_DEGREES = 15.0  # the most any joint may change for one update of a drag (about 25 a second)
MODES = ("watch", "plan", "drive")


class Session:
    """The arm, the model, and the loop that keeps them in step."""

    def __init__(self, chain: Chain | None = None, mapping: Mapping | None = None, arm_port: int = PORT):
        self.chain = chain or Chain.load()
        self.mapping = mapping or Mapping.load()
        self.arm = Arm(port=arm_port)

        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None

        self.mode = "watch"
        self.mapping_dirty = False  # the lining-up in use differs from what's saved
        self.state: ArmState | None = None
        self.last_report = 0.0

        rest = [0.0] * len(self.chain.motors)
        self.commanded = list(rest)  # what the model shows, and in drive mode what the arm is told
        self.target = list(rest)  # what was asked for; commanded walks towards it
        self.virtual = list(rest)  # the arm in plan mode, or when there is no real one to mirror

        self.runner: Runner | None = None
        self.program = Program(name="untitled")
        self.program_note = ""  # why the last program stopped, kept after its runner is gone
        self.rate = 0.0
        self._ticks = 0
        self._rate_mark = time.monotonic()
        self.seen: set[str] = set()  # what the camera recognises, for "wait_for" steps

        # Whether the arm is doing what it's told. See _check_following and _note_poses.
        self.lagging_since: float | None = None
        self.warning = ""
        self.loop_error = ""
        self._last_error_log = 0.0
        self._pose_counts: list[tuple[float, int]] = []
        self._last_poses: int | None = None

    # ---- lifecycle ----------------------------------------------------------

    def start(self) -> "Session":
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="arm-control")
        self._thread.start()
        return self

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)

    # ---- which mode -----------------------------------------------------------

    @property
    def driving(self) -> bool:
        return self.mode == "drive"

    @property
    def planning(self) -> bool:
        return self.mode == "plan"

    def set_mode(self, mode: str) -> str:
        """Switches mode. Returns a reason if it can't, and changes nothing in that case.

        Every switch starts from where the real arm is, when there is one: planning begins from the
        arm's own pose, and driving begins from it too, so neither ever makes anything jump.
        """
        if mode not in MODES:
            return f"there is no {mode!r} mode"
        if mode == "drive" and not self.hearing_follower:
            if self.hearing_leader:
                return "only the leader is on the network -- the follower can't be sent anything"
            return "no arm to drive -- use Plan to try things out without one"
        with self._lock:
            if mode != self.mode:
                here = self.measured_pose()
                self.commanded = list(here)
                self.target = list(here)
                self.virtual = list(here)
                self.runner = None
            self.mode = mode
        return ""

    def set_driving(self, on: bool) -> str:
        """The old on/off switch, kept for scripts: on is drive, off is watch."""
        return self.set_mode("drive" if on else "watch")

    # ---- where things are ---------------------------------------------------

    @property
    def hearing_follower(self) -> bool:
        """The arm itself is reporting: position, calibrated limits, faults. The full picture."""
        return self.state is not None and (time.monotonic() - self.last_report) < ARM_TIMEOUT

    @property
    def hearing_leader(self) -> bool:
        """Only the leader is on the network, so all we have is where it is telling the arm to go."""
        return self.arm.leader_pose() is not None

    @property
    def connected(self) -> bool:
        return self.hearing_follower or self.hearing_leader

    @property
    def source(self) -> str:
        if self.hearing_follower:
            return "follower"
        return "leader" if self.hearing_leader else "simulated"

    def real_pose(self) -> list[float] | None:
        """Where the real arm is, in model radians, or None if nothing is reporting.

        The follower is the truth and is used whenever it can be heard. Failing that, the leader's own
        poses are the next best thing: for a leader you are moving by hand, where it says to go is where
        the arm is, give or take how well the servos are keeping up. It is worth having, because only
        one of the two boards needs to be on the network for the model to move with your hands.
        """
        if self.hearing_follower and self.state is not None:
            return self.mapping.to_model(self.state.degrees[:5], self.state.gripper_percent)
        leader = self.arm.leader_pose()
        if leader is not None:
            degrees, gripper = leader
            return self.mapping.to_model(degrees, gripper)
        return None

    def measured_pose(self) -> list[float]:
        """The real arm if there is one, otherwise the virtual one."""
        real = self.real_pose()
        return real if real is not None else list(self.virtual)

    def current_arm_pose(self) -> list[float]:
        """The pose that counts right now, in the arm's own numbers: what a recorded move stores.

        Taken straight from the arm when it is the real arm -- no calibration in between to get wrong --
        and turned into the arm's numbers through the current calibration only for the virtual one.
        """
        if not self.planning:
            if self.hearing_follower and self.state is not None:
                return list(self.state.degrees[:5]) + [self.state.gripper_percent]
            leader = self.arm.leader_pose()
            if leader is not None:
                return list(leader[0]) + [leader[1]]
        degrees, percent = self.mapping.to_arm(self.current_pose())
        return list(degrees) + [percent]

    def current_pose(self) -> list[float]:
        """The pose that counts right now: what gets recorded, and what the readouts show.

        In plan mode that is the virtual arm, even with a real one on the network -- planning is exactly
        the job of not being the real arm.
        """
        return list(self.virtual) if self.planning else self.measured_pose()

    def effective_limits(self) -> list[tuple[float, float]]:
        """How far each joint may be sent: the range your arm was calibrated over, when it's reporting.

        That range is the real constraint -- it's what the servos were taught, and the firmware enforces it
        too. The 3D model's own limits are only a drawing's guess, and taking them as well would clip real
        poses: your arm rests at a shoulder lift of -104.5 degrees, past the model's -100. They are used
        only when there is no arm to ask. Worked out through the mapping, so the range in the arm's own
        numbers is the same whatever the model's lining-up says.
        """
        limits = list(self.chain.limits())
        if not self.hearing_follower or self.state is None:
            return limits
        for index, (low, high) in enumerate(self.state.limit_degrees[:5]):
            joint = self.mapping.joints[self.chain.motors[index]]
            limits[index] = tuple(sorted((joint.to_model(low), joint.to_model(high))))
        shut, wide = self.mapping.gripper.to_model(0.0), self.mapping.gripper.to_model(100.0)
        limits[5] = (min(shut, wide), max(shut, wide))
        return limits

    def clamp(self, pose) -> list[float]:
        return [max(low, min(high, float(v))) for v, (low, high) in zip(pose, self.effective_limits())]

    # ---- asking it to move --------------------------------------------------

    def _movable(self) -> str:
        """Why the model can't be moved right now, or "" if it can.

        Watch mode never moves anything, arm or no arm. One rule is easier to trust than a rule with an
        exception, and Plan is a single click away.
        """
        if self.mode == "watch":
            return "watching -- switch to Plan to pose the model, or Drive to move the arm"
        return ""

    def set_target_arm(self, arm) -> tuple[list[float], str]:
        """Ask for a pose given in the arm's own numbers, as a program stores it."""
        values = [float(v) for v in arm]
        if len(values) != 6 or not all(math.isfinite(v) for v in values):
            raise ValueError("a pose is five joint angles and the gripper, as real numbers")
        return self.set_target(self.mapping.to_model(values[:5], values[5]))

    def set_target(self, pose) -> tuple[list[float], str]:
        """Ask for a pose, in model radians. Returns what was accepted after clamping, and any refusal."""
        values = [float(v) for v in pose]
        if len(values) != len(self.chain.motors) or not all(math.isfinite(v) for v in values):
            # A NaN would clamp to a joint's limit and send it there; five numbers would crash the loop.
            raise ValueError("a pose is six joint angles, as real numbers")
        problem = self._movable()
        if problem:
            return list(self.commanded), problem
        with self._lock:
            self.target = self.clamp(values)
            return list(self.target), ""

    def solve_to(self, xyz, reach: bool = False) -> tuple[list[float], float, bool, str]:
        """Point the gripper tip at a place in space, keeping the gripper opening as it is.

        Dragging the handle (reach=False) only ever bends the arm on from the pose it's in: if the point is
        out of reach it gets as close as it can, and it never jumps to a different way of holding itself.
        A deliberate reach for a part (reach=True) may search further, but picks the answer nearest the
        present pose -- and in Drive refuses one that would swing any joint more than MAX_REACH_SWING, since
        a big change of shape is something to rehearse in Plan before a real arm does it.
        """
        problem = self._movable()
        if problem:
            return list(self.commanded), 0.0, False, problem
        target = [float(v) for v in xyz]
        if len(target) != 3 or not all(math.isfinite(v) for v in target):
            raise ValueError("a point is three numbers, in metres")
        with self._lock:
            seed = list(self.commanded)
        pose, error, reached = self.chain.solve_ik(
            np.asarray(target), seed, restarts=4 if reach else 0, limits=self.effective_limits(),
            max_change=None if reach else math.radians(DRAG_STEP_DEGREES))
        pose[5] = seed[5]
        swing = max(abs(math.degrees(a - b)) for a, b in zip(pose[:5], seed[:5]))
        if reach and self.driving and swing > MAX_REACH_SWING:
            return list(self.commanded), error, False, (
                f"reaching there means swinging a joint {swing:.0f} degrees -- try it in Plan first")
        accepted, _ = self.set_target(pose)
        return accepted, error, reached, ""

    def stop_everything(self) -> None:
        """The big red button: abandon any program and hold the arm exactly where it is.

        It stays in Drive, sending the pose it has reached, because stopping sending isn't the same as
        stopping: with the leader powered, the follower firmware would hand the arm back to the leader a
        quarter of a second later, and the arm would glide off to wherever the leader is. Switching to
        Watch is how the arm is let go, deliberately.
        """
        with self._lock:
            self.runner = None
            self.target = list(self.commanded)

    def release(self) -> None:
        """Stops sending altogether: the page has gone. The firmware freezes, or the leader takes over."""
        with self._lock:
            self.runner = None
            if self.mode == "drive":
                self.mode = "watch"

    def remap(self, change) -> str:
        """Changes the model's lining-up (`change` does it) without changing anything the arm is told.

        Poses here are in model angles, so a new mapping would reinterpret them: flip a joint mid-drive and
        the next packet sends it to its mirror image. So it's refused while driving, and otherwise every
        pose held here is carried across in the arm's own numbers, which the mapping can't touch.
        """
        if self.driving:
            return "switch out of Drive before changing how the model lines up -- it would move the arm"
        with self._lock:
            held = [self._to_arm_list(p) for p in (self.commanded, self.target, self.virtual)]
            change()
            self.commanded, self.target, self.virtual = (
                self.mapping.to_model(arm[:5], arm[5]) for arm in held)
            self.runner = None  # a running program's targets were worked out under the old mapping
        return ""

    # ---- programs -----------------------------------------------------------

    def run_program(self, program: Program, start_index: int = 0) -> str:
        if not program.steps:
            return "that program has no steps"
        if self.mode == "watch":
            return "switch to Plan to rehearse it, or Drive to run it on the arm"
        if not 0 <= start_index < len(program.steps):
            return "there is no such step to start from"
        with self._lock:
            runner = Runner(program.resolved(self.mapping), start_pose=list(self.commanded))
            runner.index = start_index
            runner.gripper_radians = self.mapping.gripper.to_model
            runner.max_speed = MAX_DEGREES_PER_SECOND
            runner.clamp = self.clamp
            self.program = program
            self.runner = runner
            self.program_note = ""
        return ""

    def stop_program(self) -> None:
        with self._lock:
            if self.runner is not None:
                self.runner = None
                self.program_note = "stopped"
                self.target = list(self.commanded)  # stay where the program left it

    # ---- the loop -----------------------------------------------------------

    def _loop(self) -> None:
        period = 1.0 / CONTROL_HZ
        next_tick = time.monotonic()
        while self._running:
            try:
                state = self.arm.poll(timeout=0.005)  # doubles as the loop's pacing when the arm is live
            except OSError:
                state = None
                time.sleep(0.005)
            if state is not None:
                self.state = state
                self.last_report = time.monotonic()
                self._note_poses(self.last_report, state.poses_received)

            now = time.monotonic()
            if now < next_tick:
                continue
            next_tick = max(now, next_tick + period)
            try:
                self._tick(now, period)
                self.loop_error = ""
            except Exception as error:  # a network blip or a bad value: keep the loop, and say so
                self.loop_error = f"{type(error).__name__}: {error}"
                if now - self._last_error_log > 5.0:
                    self._last_error_log = now
                    traceback.print_exc()

            self._ticks += 1
            if now - self._rate_mark >= 1.0:
                self.rate = self._ticks / (now - self._rate_mark)
                self._ticks, self._rate_mark = 0, now

    def _tick(self, now: float, period: float) -> None:
        with self._lock:
            if self.mode == "watch":
                real = self.real_pose()
                if real is not None:
                    # A reflection, exactly and without lag. Slew limiting is for commands the servos
                    # have to follow; a reflection that trailed 90 degrees a second behind would be
                    # useless for seeing what the arm is doing.
                    self.target = list(real)
                    self.commanded = list(real)
                    self.virtual = list(real)
                    self.lagging_since, self.warning = None, ""  # nothing is being told anything
                    return

            if self.runner is not None:
                wanted = self.runner.tick(now, list(self.commanded), self.seen)
                if wanted is None or self.runner.finished:
                    self.program_note = self.runner.message
                    self.runner = None
                else:
                    self.target = self.clamp(wanted)

            # Walk the command towards the target instead of jumping to it. This is what turns a flung
            # slider into a move the servos can actually follow.
            ceiling = math.radians(MAX_DEGREES_PER_SECOND) * period
            stepped = []
            for now_at, wanted in zip(self.commanded, self.target):
                delta = wanted - now_at
                stepped.append(now_at + max(-ceiling, min(ceiling, delta)))
            # Not clamped here: the target already is. Clamping the command itself would snap an arm that
            # starts outside its range straight back in, the moment Drive began.
            self.commanded = stepped
            command = list(self.commanded)
            driving = self.mode == "drive"
            if not driving:
                self.virtual = command  # plan mode, or nothing to mirror: the virtual arm follows

        if driving:
            degrees, percent = self.mapping.to_arm(command)
            self.arm.send_pose(degrees, percent)
        self._check_following(now, driving, command)

    # ---- is the arm doing what it's told? --------------------------------------

    def _note_poses(self, now: float, count: int) -> None:
        """Keeps the follower's own count of poses it acted on, which it resets every two seconds."""
        if self._last_poses is not None and count >= self._last_poses:
            self._pose_counts.append((now, count - self._last_poses))
        self._last_poses = count
        self._pose_counts = [(t, n) for t, n in self._pose_counts if now - t < 2.0]

    @property
    def follower_pose_rate(self) -> float:
        """How many poses a second the follower is acting on, from anyone."""
        if not self.hearing_follower or not self._pose_counts:
            return 0.0
        return sum(n for _, n in self._pose_counts) / 2.0

    @property
    def leader_steering(self) -> bool:
        """The follower is following something that isn't us. Only knowable while we are not sending."""
        return self.mode != "drive" and self.follower_pose_rate > STEERED_POSES_PER_SECOND

    def _check_following(self, now: float, driving: bool, command: list[float]) -> None:
        """Notices when the arm stops obeying, whatever the cause.

        A leader still steering a follower that has older firmware, a joint jammed against something,
        a model lined up the wrong way round: all look the same from here -- the arm is somewhere other
        than where it's being told to go, and staying there. Saying so beats leaving you to work out why
        the arm is ignoring the screen.
        """
        real = self.real_pose() if driving and self.hearing_follower else None
        if real is None:
            self.lagging_since, self.warning = None, ""
            return
        worst = max(abs(math.degrees(a - b)) for a, b in zip(real[:5], command[:5]))
        if worst < LAG_LIMIT_DEG:
            self.lagging_since, self.warning = None, ""
        elif self.lagging_since is None:
            self.lagging_since = now
        elif now - self.lagging_since > LAG_LIMIT_SECONDS:
            self.warning = (
                f"the arm isn't following -- it's {worst:.0f} degrees from where it's being told to go. "
                "If the leader is on, power it down (or update the follower's firmware); "
                "otherwise check nothing is blocking it"
            )

    # ---- for the browser ----------------------------------------------------

    def _to_arm_list(self, pose) -> list[float]:
        degrees, percent = self.mapping.to_arm(pose)
        return list(degrees) + [percent]

    def _limits_arm(self) -> list[list[float]]:
        """The joint limits in the arm's own numbers, so a slider can show the arm's range directly."""
        limits = []
        for index, (low, high) in enumerate(self.effective_limits()):
            if index < 5:
                joint = self.mapping.joints[self.chain.motors[index]]
                limits.append(sorted((joint.to_arm(low), joint.to_arm(high))))
            else:
                limits.append([0.0, 100.0])
        return limits

    def snapshot(self) -> dict:
        with self._lock:
            commanded = list(self.commanded)
            target = list(self.target)
            mode = self.mode
            runner = self.runner
        pose = self.current_pose()
        real = self.real_pose()
        state = self.state
        return {
            "connected": self.connected,
            "simulated": not self.connected,
            "source": self.source,
            "mode": mode,
            "driving": mode == "drive",
            "movable": mode != "watch",
            "address": self.arm.address or self.arm.leader_address,
            "arm_state": (
                state.state if state and self.hearing_follower
                else "watching the leader arm" if self.hearing_leader
                else "no arm on the network"
            ),
            "fault": (state.fault if state and self.hearing_follower else "") or "",
            "fault_joint": (state.fault_joint if state and self.hearing_follower else "") or "",
            "pose": pose,
            "arm_pose": self.current_arm_pose(),
            "commanded_arm": self._to_arm_list(commanded),
            "limits_arm": self._limits_arm(),
            "real": real,
            "real_degrees": list(state.degrees[:5]) if state and self.hearing_follower else None,
            "commanded": commanded,
            "target": target,
            "limits": [list(pair) for pair in self.effective_limits()],
            "tip": self.chain.tip_position(pose).tolist(),
            "tip_commanded": self.chain.tip_position(commanded).tolist(),
            "rate": round(self.rate, 1),
            "program": runner.status() if runner else {"running": False, "message": self.program_note},
            "mapping": self.mapping.describe(),
            "mapping_saved": CONFIG_PATH.exists() and not self.mapping_dirty,
            "mapping_dirty": self.mapping_dirty,
            "seen": sorted(self.seen),
            "follower_pose_rate": round(self.follower_pose_rate, 1),
            "leader_steering": self.leader_steering,
            "warning": self.warning or (f"the control loop hit a problem: {self.loop_error}" if self.loop_error else ""),
        }
