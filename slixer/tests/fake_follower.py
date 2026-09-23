"""An imitation follower arm, for testing Slixer without the real one.

It runs the follower firmware's own logic -- the stale-pose filter, the freeze after 250 ms of silence,
the glide on resume, the 20 Hz position reports -- with servos that move at a believable speed. Slixer
can't tell it from the real arm, and it listens on a port the real arm never uses, so nothing done to it
can reach hardware.

    uv run slixer/tests/fake_follower.py              # an arm on UDP 50198, reporting to 50199
    uv run slixer/run.py --arm-port 50199 --port 8001 # Slixer, watching it

Every constant below is copied from so101_follower.ino / teleop_math.h. If the firmware changes, change
these with it, or the tests will be testing an arm that no longer exists.
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import threading
import time
from dataclasses import dataclass, field

# ---- from the firmware ------------------------------------------------------------------------------
K_MAX_RES = 4095  # teleop_math.h: positions run 0..4095
K_TURN = 4096
K_SEAM_MARGIN = 100
K_STALE_MS = 250  # no leader data for this long -> freeze
K_TELEMETRY_MS = 50  # position reports, 20 Hz
K_GLIDE_DEG_PER_SEC = 45.0
K_NO_GLIDE_TICKS = 60  # ~5 degrees: closer than this, follow without gliding
K_GAP_ACCEPT_MS = 500  # after a silence this long, any sequence number is accepted

POSE_FORMAT = "<BB5hH"
STATUS_FORMAT = "<BBBBB6h6h6h"
POSE_MAGIC, STATUS_MAGIC = 0xA5, 0x5B
PROGRAM_MAGIC = 0xA7  # protocol 2: a pose from a program, which takes the arm from the leader

WAITING, GLIDING, FOLLOWING, FROZEN = 1, 2, 3, 4  # the state codes the status packet carries
STATE_NAMES = {WAITING: "waiting for leader", GLIDING: "gliding", FOLLOWING: "following", FROZEN: "holding"}

# Believable STS3215s at 12 V: about 300 degrees a second flat out, and quick to get there.
SERVO_DEG_PER_SEC = 300.0
SERVO_DEG_PER_SEC2 = 2000.0

DEFAULT_RANGES = [(1000, 3100), (900, 3000), (1100, 3200), (950, 3050), (100, 3995), (2000, 3400)]


def mid(r) -> float:
    return (r[0] + r[1]) * 0.5


def safe_min(r) -> int:
    return r[0] if r[0] > K_SEAM_MARGIN else K_SEAM_MARGIN


def safe_max(r) -> int:
    return r[1] if r[1] < K_MAX_RES - K_SEAM_MARGIN else K_MAX_RES - K_SEAM_MARGIN


def goal_from_centidegrees(cdeg: int, r) -> int:
    pos = cdeg * K_MAX_RES / 36000.0 + mid(r)
    return max(safe_min(r), min(safe_max(r), int(round(pos))))


def goal_from_centipercent(cpct: int, r) -> int:
    frac = min(cpct, 10000) / 10000.0
    return max(safe_min(r), min(safe_max(r), int(round(r[0] + frac * (r[1] - r[0])))))


def to_centidegrees(pos: float, r) -> int:
    return int(round(max(-32767.0, min(32767.0, (pos - mid(r)) * 36000.0 / K_MAX_RES))))


@dataclass
class Sample:
    t: float
    positions: list[float]
    state: int


@dataclass
class FakeFollower:
    listen_port: int = 50198
    report_to: tuple[str, int] = ("127.0.0.1", 50199)
    ranges: list[tuple[int, int]] = field(default_factory=lambda: list(DEFAULT_RANGES))
    record: bool = True
    priority: bool = True  # protocol 2: program poses (0xA7) take the arm from the leader (0xA5)

    def __post_init__(self) -> None:
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind(("127.0.0.1", self.listen_port))
        self._socket.settimeout(0.05)
        self._lock = threading.Lock()
        self._running = False
        self._threads: list[threading.Thread] = []

        self.state = WAITING
        self.positions = [mid(r) for r in self.ranges]  # arm resting mid-range
        self.velocities = [0.0] * 6
        self.goals = list(self.positions)
        self.glide_from = list(self.positions)
        self.glide_start = 0.0
        self.glide_ms = 1000.0

        self.latest: tuple | None = None  # the newest accepted pose packet's fields
        self.latest_ms = -1e9
        self.have_packet = False
        self.latest_from_program = False
        self.program_pose_ms = None
        self.following_program = False
        self.yielded = 0
        self.accepted = 0
        self.stale = 0
        self.glides = 0
        self.poses_since_report = 0
        self._last_report = 0.0
        self._last_telemetry = 0.0
        self.history: list[Sample] = []

    # ---- lifecycle ----------------------------------------------------------------------------------

    def start(self) -> "FakeFollower":
        self._running = True
        self._started = time.monotonic()
        for target, name in ((self._receive, "fake-rx"), (self._control, "fake-ctl")):
            thread = threading.Thread(target=target, daemon=True, name=name)
            thread.start()
            self._threads.append(thread)
        return self

    def stop(self) -> None:
        self._running = False
        for thread in self._threads:
            thread.join(timeout=1)
        self._socket.close()

    def _ms(self) -> float:
        return (time.monotonic() - self._started) * 1000.0

    # ---- the radio: onLeaderPose -------------------------------------------------------------------

    def _receive(self) -> None:
        while self._running:
            try:
                data, _ = self._socket.recvfrom(256)
            except (socket.timeout, OSError):
                continue
            self.deliver(data, via_wifi=True)

    def deliver(self, data: bytes, via_wifi: bool = True) -> None:
        """A packet arriving by either route. The route doesn't matter to the firmware: the marker does."""
        if len(data) != struct.calcsize(POSE_FORMAT):
            return  # greetings and anything else: ignored, exactly as the firmware does
        from_program = data[0] == PROGRAM_MAGIC and self.priority  # an old follower doesn't know 0xA7
        if not from_program and data[0] != POSE_MAGIC:
            return
        fields = struct.unpack(POSE_FORMAT, data)
        arrived = self._ms()
        with self._lock:
            if self.priority:
                if from_program:
                    self.program_pose_ms = arrived
                elif self.program_pose_ms is not None and arrived - self.program_pose_ms < K_STALE_MS:
                    self.yielded += 1
                    return
            newer_by = (fields[1] - (self.latest[1] if self.latest else 0)) & 0xFF
            other_sender = self.priority and self.have_packet and from_program != self.latest_from_program
            if (not self.have_packet or other_sender or arrived - self.latest_ms > K_GAP_ACCEPT_MS
                    or 0 < newer_by < 128):
                self.latest = fields
                self.latest_ms = arrived
                self.latest_from_program = from_program
                self.have_packet = True
                self.accepted += 1
                self.poses_since_report += 1
            else:
                self.stale += 1

    # ---- the main loop -----------------------------------------------------------------------------

    def _targets(self, fields) -> list[int]:
        body = [goal_from_centidegrees(fields[2 + j], self.ranges[j]) for j in range(5)]
        return body + [goal_from_centipercent(fields[7], self.ranges[5])]

    def _start_glide(self, now: float, fields) -> None:
        present = [round(p) for p in self.positions]
        target = self._targets(fields)
        farthest = max(abs(t - p) for t, p in zip(target, present))
        self.glide_from = list(present)
        self.goals = list(present)
        if farthest < K_NO_GLIDE_TICKS:
            self.state = FOLLOWING
            return
        seconds = farthest * 360.0 / K_TURN / K_GLIDE_DEG_PER_SEC
        self.glide_ms = max(500.0, min(5000.0, seconds * 1000.0))
        self.glide_start = now
        self.state = GLIDING
        self.glides += 1

    def _control(self) -> None:
        period = 1 / 200.0
        last = time.monotonic()
        while self._running:
            time.sleep(period)
            now_s = time.monotonic()
            dt, last = now_s - last, now_s
            now = self._ms()
            with self._lock:
                fields, fresh = self.latest, self.have_packet and now - self.latest_ms < K_STALE_MS
                from_program = self.latest_from_program
                if self.state in (WAITING, FROZEN):
                    if fresh:
                        self.following_program = from_program
                        self._start_glide(now, fields)
                elif self.state in (GLIDING, FOLLOWING):
                    if not fresh:
                        self.state = FROZEN
                    elif self.priority and from_program != self.following_program:
                        self.following_program = from_program  # a change of hands: glide, don't lunge
                        self._start_glide(now, fields)
                    else:
                        target = self._targets(fields)
                        if self.state == GLIDING:
                            u = (now - self.glide_start) / self.glide_ms
                            if u >= 1.0:
                                u, self.state = 1.0, FOLLOWING
                            s = u * u * (3.0 - 2.0 * u)
                            self.goals = [f + (t - f) * s for f, t in zip(self.glide_from, target)]
                        else:
                            self.goals = list(target)
                if self.state != WAITING:  # torque is off until the first glide
                    self._move_servos(dt)
                if self.record:
                    self.history.append(Sample(now / 1000.0, list(self.positions), self.state))
                if now - self._last_telemetry >= K_TELEMETRY_MS:
                    self._last_telemetry = now
                    self._send_status()
                if now - self._last_report >= 2000:
                    self._last_report = now
                    self.poses_since_report = 0

    def _move_servos(self, dt: float) -> None:
        """Each joint heads for its goal, limited in speed and in how hard it can speed up or slow down."""
        top = SERVO_DEG_PER_SEC * K_MAX_RES / 360.0
        accel = SERVO_DEG_PER_SEC2 * K_MAX_RES / 360.0
        for j in range(6):
            error = self.goals[j] - self.positions[j]
            wanted = math.copysign(min(top, math.sqrt(2 * accel * abs(error))), error)
            change = max(-accel * dt, min(accel * dt, wanted - self.velocities[j]))
            self.velocities[j] += change
            step = self.velocities[j] * dt
            if abs(step) >= abs(error):
                self.positions[j], self.velocities[j] = float(self.goals[j]), 0.0
            else:
                self.positions[j] += step

    def _send_status(self) -> None:
        fields = (STATUS_MAGIC, self.state, 0, 0, min(255, self.poses_since_report),
                  *[int(round(p)) for p in self.positions], *[r[0] for r in self.ranges], *[r[1] for r in self.ranges])
        # Protocol 2 adds its version at the end; an old follower's status stops short of it.
        packet = struct.pack(STATUS_FORMAT + "B", *fields, 2) if self.priority else struct.pack(STATUS_FORMAT, *fields)
        try:
            self._socket.sendto(packet, self.report_to)
        except OSError:
            pass

    # ---- for tests ---------------------------------------------------------------------------------

    def degrees(self) -> list[float]:
        """Joint angles as the arm itself would report them: degrees from the middle of each range."""
        with self._lock:
            return [(p - mid(r)) * 360.0 / K_MAX_RES for p, r in zip(self.positions[:5], self.ranges[:5])]

    def run_leader(self, degrees, rate_hz: float = 50.0, start_seq: int = 200) -> threading.Event:
        """A second sender, like a powered leader arm sitting still: its pose, over the radio, forever.

        Returns an event; set it to switch the leader off.
        """
        stop = threading.Event()

        def leader() -> None:
            seq = start_seq
            while not stop.is_set() and self._running:
                seq = (seq + 1) & 0xFF
                body = [int(round(d * 100)) for d in degrees]
                self.deliver(struct.pack(POSE_FORMAT, POSE_MAGIC, seq, *body, 0), via_wifi=False)
                stop.wait(1.0 / rate_hz)

        threading.Thread(target=leader, daemon=True, name="fake-leader").start()
        return stop


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--listen", type=int, default=50198, help="port it takes poses on")
    parser.add_argument("--report-to", type=int, default=50199, help="port Slixer listens on (--arm-port)")
    parser.add_argument("--leader", action="store_true", help="also run a still leader arm, to test contention")
    parser.add_argument("--old-firmware", action="store_true",
                        help="protocol 1: no program marker, so the newest sequence number wins")
    args = parser.parse_args()
    arm = FakeFollower(listen_port=args.listen, report_to=("127.0.0.1", args.report_to), record=False,
                       priority=not args.old_firmware).start()
    if args.leader:
        arm.run_leader([0.0] * 5)
    print(f"fake follower on UDP {args.listen}, reporting to {args.report_to}. Ctrl-C to stop.")
    try:
        while True:
            time.sleep(2)
            degrees = " ".join(f"{d:+6.1f}" for d in arm.degrees())
            print(f"{STATE_NAMES.get(arm.state, arm.state):<18} {degrees} | {arm.accepted} poses, {arm.stale} stale, "
                  f"{arm.glides} glides, {arm.yielded} leader poses held back")
    except KeyboardInterrupt:
        pass
    finally:
        arm.stop()


if __name__ == "__main__":
    main()
