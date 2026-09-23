"""Camera frames in, joint targets out: the loop your vision code lives in.

Everything around `decide()` is already handled: receiving frames from the Pi, reading the arm's real
position, sending targets, and stopping safely when something goes quiet. Write your vision code in
`decide()` and run it.

    uv run pc/vision_control.py PI_ADDRESS            # watch only: frames and the arm's real position
    uv run pc/vision_control.py PI_ADDRESS --show     # and show what the camera sees
    uv run pc/vision_control.py PI_ADDRESS --drive    # let decide() actually move the arm

Without --drive nothing is sent, so you can watch the arm being teleoperated by hand. With it, this program
has the arm while it is sending: a powered leader arm is held back, and gets the arm back a moment after this
stops. Grabbing the leader does not take over -- stopping this program (Ctrl-C) does.

Safety, without you having to think about it:
  * targets are only sent while the frames are fresh and the arm is ready;
  * if frames stop, or this program stops, the arm freezes where it is rather than going limp;
  * every target is clamped to the joint travel the arm reports;
  * stopping this program hands the arm back: it holds still, or follows the leader again if one is on.
"""

from __future__ import annotations

import argparse
import time

from camera_client import ArmCamera, Display
from so101_link import Arm, ArmState

STALE_FRAME_S = 0.5  # older than this and we stop driving
LOOP_HZ = 30


def decide(frame, state: ArmState, elapsed: float) -> tuple[list[float], float]:
    """Return the five joint angles to move to, and how far open the gripper should be (0-100%).

    Angles are degrees from the middle of each joint's own travel, the same units the leader arm sends.
    `state.degrees` is where the arm is now, and `state.limit_degrees` how far each joint can go.

    This default holds the arm still. Replace it with your own: find something in `frame`, work out where
    the arm should be, and return that. Move in small steps from `state.degrees` rather than jumping.
    """
    return list(state.degrees[:5]), state.gripper_percent


def clamp_to_limits(targets: list[float], state: ArmState) -> list[float]:
    return [max(lo, min(hi, target)) for target, (lo, hi) in zip(targets, state.limit_degrees[:5])]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("host", help="the Pi's address or hostname")
    parser.add_argument("--show", action="store_true", help="display the camera view")
    parser.add_argument("--drive", action="store_true",
                        help="actually send targets to the arm (without this it only watches)")
    parser.add_argument("--arm-port", type=int, default=50101, help="UDP port the arm reports on")
    parser.add_argument("--camera-port", type=int, default=50102, help="port the Pi streams frames on")
    args = parser.parse_args()

    camera = ArmCamera(args.host, args.camera_port).start()
    arm = Arm(port=args.arm_port)
    display = Display() if args.show else None
    print("waiting for the camera and the arm...")
    started = time.monotonic()
    last_report = 0.0
    driving = False

    try:
        while True:
            loop_started = time.monotonic()
            state = arm.poll(timeout=1.0 / LOOP_HZ)
            if state is None:
                state = arm.last_state  # keep going on the last report between updates
            frame, age = camera.read()

            ready = state is not None and state.ready and frame is not None and age < STALE_FRAME_S and args.drive
            if ready:
                targets, gripper = decide(frame, state, loop_started - started)
                arm.send_pose(clamp_to_limits(targets, state), gripper)
            if ready != driving:
                print("driving the arm" if ready else
                      ("watching only: pass --drive to move the arm" if not args.drive
                       else "not driving: waiting for fresh frames and a ready arm"))
                driving = ready

            if display is not None and frame is not None and not display.show(frame):
                break

            if loop_started - last_report >= 2.0:
                last_report = loop_started
                where = "?" if state is None else " ".join(f"{d:+.0f}" for d in state.degrees[:5])
                print(f"frame {age * 1000:6.0f} ms old | arm: {'?' if state is None else state.state} | {where}")

            time.sleep(max(0.0, 1.0 / LOOP_HZ - (time.monotonic() - loop_started)))
    except KeyboardInterrupt:
        print("\nstopped: the arm will hold its position")
    finally:
        camera.stop()


if __name__ == "__main__":
    main()
