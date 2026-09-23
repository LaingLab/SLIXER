"""Talk to the SO-101 arm over the network: see what it is doing, and tell it where to go.

The follower board broadcasts its state, so this finds the arm without being told an address. Joint targets
use the leader's packet layout. A follower running protocol 2 (it says so in its status) is sent them marked
as coming from a program, and lets a program steer while it's sending, holding the leader arm back until a
quarter of a second after it stops. An older follower can't tell a program from the leader; for that one
the sequence number is stepped clear of any leader this can hear, so the program's poses read as newest.

    uv run pc/so101_link.py              # watch the arm
    uv run pc/so101_link.py --hold       # hold it where it is: proves the control path end to end
    uv run pc/so101_link.py --wiggle     # move one joint gently back and forth

Run it on the same network as the boards. Under WSL that needs mirrored networking (see the README),
otherwise run it from Windows.
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import time
from dataclasses import dataclass

PORT = 50101
POSE_MAGIC, STATUS_MAGIC = 0xA5, 0x5B
PROGRAM_MAGIC = 0xA7  # a pose from a program; a follower of protocol 2 lets it take over from the leader
HELLO_MAGIC = 0x5D  # "I'm here": matches no message either board acts on, so it is safely ignored
AHEAD_BY = 64  # how far clear of another sender's sequence to step; see Arm.send_pose
POSE_FORMAT = "<BB5hH"  # magic, sequence, five joints in 0.01 deg, gripper in 0.01 %
STATUS_FORMAT = "<BBBBB6h6h6h"  # magic, state, fault, joint, poses, positions, range min, range max
STATUS_V2_FORMAT = STATUS_FORMAT + "B"  # ...and the follower's protocol version
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
STATES = ("checking arm", "waiting for leader", "gliding", "following", "holding", "recording ranges")
FAULTS = ("", "no reply from servo", "no usable calibration", "outside calibrated range", "torque refused")
STEPS_PER_TURN = 4095


@dataclass
class ArmState:
    """One report from the follower: what it is doing, where it is, and how far it can go."""

    state: str
    fault: str
    fault_joint: str
    poses_received: int
    position_steps: tuple[int, ...]
    range_min: tuple[int, ...]
    range_max: tuple[int, ...]
    version: int = 1  # the follower's protocol: 2 and up understand PROGRAM_MAGIC

    @property
    def degrees(self) -> tuple[float, ...]:
        """Each joint's angle from the middle of its own travel, the same units targets use."""
        return tuple(
            (pos - (lo + hi) / 2) * 360.0 / STEPS_PER_TURN
            for pos, lo, hi in zip(self.position_steps, self.range_min, self.range_max)
        )

    @property
    def limit_degrees(self) -> tuple[tuple[float, float], ...]:
        """How far each joint can travel either side of its middle."""
        return tuple(
            (-(hi - lo) / 2 * 360.0 / STEPS_PER_TURN, (hi - lo) / 2 * 360.0 / STEPS_PER_TURN)
            for lo, hi in zip(self.range_min, self.range_max)
        )

    @property
    def gripper_percent(self) -> float:
        lo, hi = self.range_min[5], self.range_max[5]
        return 0.0 if hi == lo else max(0.0, min(100.0, (self.position_steps[5] - lo) * 100.0 / (hi - lo)))

    @property
    def ready(self) -> bool:
        return self.state in ("following", "holding", "waiting for leader", "gliding")


class Arm:
    """The follower arm, as seen from a PC."""

    def __init__(self, port: int = PORT):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._socket.bind(("0.0.0.0", port))
        self._port = port
        self._sequence = 0
        self.address: str | None = None  # learnt from the arm's own broadcasts
        self._peer_port = port
        self.last_state: ArmState | None = None

        # The leader's own packets, which are worth having even when the follower is out of reach: they
        # say where the arm is being told to go, which for a leader you are moving by hand is where the
        # arm is. See `leader_pose`.
        self.leader_degrees: tuple[float, ...] | None = None
        self.leader_gripper = 0.0
        self.leader_address: str | None = None
        self.leader_seen = 0.0
        self._greeted: dict[str, float] = {}

        # The newest sequence number seen from anything else driving the arm. The follower keeps only the
        # newest pose and judges "newest" by this number, so two senders each counting from their own zero
        # fight: whichever happens to be higher wins and the other is discarded as stale. See `send_pose`.
        self.other_sequence: int | None = None
        self.other_sequence_seen = 0.0

    def _greet(self, sender) -> None:
        """Says hello to a board that has just been heard from, so it sends to us directly.

        A board broadcasts once a second so it can be found, but sends to everyone it has heard from at
        the full rate. Until we say something we are nobody, and one packet a second is no use for
        watching an arm move. The greeting is two bytes that match no message either board acts on: it
        exists to be received, not understood.
        """
        now = time.monotonic()
        if now - self._greeted.get(sender[0], 0.0) < 2.0:
            return
        self._greeted[sender[0]] = now
        try:
            self._socket.sendto(bytes((HELLO_MAGIC, 1)), sender)
        except OSError:
            pass  # the board went away between hearing it and answering; the next packet will retry

    def poll(self, timeout: float = 0.5) -> ArmState | None:
        """Waits up to `timeout` for a report from the arm. Returns None if none arrived.

        Poses seen from the leader are recorded along the way, so a caller that gets None can still ask
        `leader_pose()` where the arm is being driven.
        """
        self._socket.settimeout(timeout)
        deadline = time.monotonic() + timeout
        while True:
            try:
                data, sender = self._socket.recvfrom(256)
            except socket.timeout:
                return None
            if len(data) in (struct.calcsize(STATUS_FORMAT), struct.calcsize(STATUS_V2_FORMAT)) \
                    and data[0] == STATUS_MAGIC:
                self.address, self._peer_port = sender
                self._greet(sender)
                v2 = len(data) == struct.calcsize(STATUS_V2_FORMAT)
                fields = struct.unpack(STATUS_V2_FORMAT if v2 else STATUS_FORMAT, data)
                self.last_state = ArmState(
                    state=STATES[fields[1]] if fields[1] < len(STATES) else "?",
                    fault=FAULTS[fields[2]] if fields[2] < len(FAULTS) else "?",
                    fault_joint=JOINTS[fields[3]] if fields[3] < len(JOINTS) else "?",
                    poses_received=fields[4],
                    position_steps=fields[5:11],
                    range_min=fields[11:17],
                    range_max=fields[17:23],
                    version=fields[23] if v2 else 1,
                )
                return self.last_state
            if len(data) == struct.calcsize(POSE_FORMAT) and data[0] == POSE_MAGIC:
                self._greet(sender)
                fields = struct.unpack(POSE_FORMAT, data)
                self.leader_degrees = tuple(v / 100.0 for v in fields[2:7])
                self.leader_gripper = fields[7] / 100.0
                self.leader_address = sender[0]
                self.leader_seen = time.monotonic()
                self.other_sequence = fields[1]
                self.other_sequence_seen = self.leader_seen
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            self._socket.settimeout(remaining)

    def leader_pose(self, fresh_for: float = 1.0) -> tuple[tuple[float, ...], float] | None:
        """Where the leader last said to go, or None if it hasn't said recently.

        Same units as `send_pose` takes: degrees from the middle of each joint's travel, gripper 0-100%.
        """
        if self.leader_degrees is None or time.monotonic() - self.leader_seen > fresh_for:
            return None
        return self.leader_degrees, self.leader_gripper

    def send_pose(self, degrees, gripper_percent: float) -> None:
        """Sends joint targets: degrees from the middle of each joint's travel, gripper 0-100%.

        The arm clamps anything beyond its own limits and ignores targets it stops hearing, holding
        position instead, so a crashed script leaves the arm still rather than limp.

        If a leader arm is also powered, it is sending its own pose fifty times a second and counting from
        its own zero. The arm keeps whichever pose is newest and judges that by the sequence number, so
        without care the two streams fight and the arm jerks between them. Stepping clear of whatever the
        other sender last used settles it: while a program is driving, it drives. Stop sending and the
        leader has the arm back within half a second.
        """
        if len(degrees) != 5:
            raise ValueError("expected five joint angles: " + ", ".join(JOINTS[:5]))
        # A follower that understands program poses keeps them apart from the leader's and gives them the
        # arm, so there is nothing to fight over. An older one can't tell the two apart, so there the
        # sequence number has to win the argument instead.
        program = self.last_state is not None and self.last_state.version >= 2
        self._sequence = (self._sequence + 1) & 0xFF
        if not program and self.other_sequence is not None and time.monotonic() - self.other_sequence_seen < 1.0:
            # Step clear unless ours already reads as strictly newer than theirs. Half a counter apart is
            # neither newer nor older -- the arm's comparison can't tell -- so that counts as not newer.
            ahead = (self._sequence - self.other_sequence) & 0xFF
            if not 1 <= ahead < 128:
                self._sequence = (self.other_sequence + AHEAD_BY) & 0xFF
        target = "255.255.255.255" if self.address is None else self.address
        packet = struct.pack(
            POSE_FORMAT,
            PROGRAM_MAGIC if program else POSE_MAGIC,
            self._sequence,
            *[int(round(max(-327.0, min(327.0, d)) * 100)) for d in degrees],
            int(round(max(0.0, min(100.0, gripper_percent)) * 100)),
        )
        self._socket.sendto(packet, (target, self._peer_port))


SHORT_NAMES = ("pan", "lift", "elbow", "wrist", "roll")  # the five body joints; the gripper reads as a %


def _describe(state: ArmState) -> str:
    joints = " ".join(f"{name}={deg:+6.1f}" for name, deg in zip(SHORT_NAMES, state.degrees[:5]))
    fault = f" [{state.fault} ({state.fault_joint})]" if state.fault else ""
    return f"{state.state:<20}{fault} | {joints} | gripper {state.gripper_percent:3.0f}%"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--hold", action="store_true", help="hold the arm where it is")
    parser.add_argument("--wiggle", action="store_true", help="rock the shoulder gently +/- 10 degrees")
    args = parser.parse_args()

    arm = Arm()
    print(f"listening on UDP {PORT}; power the arm and it will announce itself...")
    started = time.monotonic()
    while True:
        state = arm.poll(timeout=1.0)
        if state is None:
            continue
        print(f"{arm.address:<15} {_describe(state)}")
        if not (args.hold or args.wiggle) or not state.ready:
            continue
        target = list(state.degrees[:5])
        if args.wiggle:
            target[0] = 10.0 * math.sin((time.monotonic() - started) * 0.8)
        for _ in range(20):  # keep sending: the arm freezes if targets stop arriving
            arm.send_pose(target, state.gripper_percent)
            time.sleep(0.02)


if __name__ == "__main__":
    main()
