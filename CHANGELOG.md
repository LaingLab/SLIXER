# Changelog

All notable changes to Slixer are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres
to [Semantic Versioning](https://semver.org/spec/v2.0.0.html). Versions are written the Python way
([PEP 440](https://peps.python.org/pep-0440/)): 0.1.0b1 is 0.1.0-beta.1.

## [Unreleased]

### Added

- **Semantic segmentation models**, such as one trained from `yolo26n-sem` on the Ultralytics Platform. Each
  patch of a class the model marks is outlined as a find, and can be waited for like any other.

### Changed

- Each model runs at the picture size it was trained at, rather than always at 640.

### Fixed

- **A model Slixer can't read the results of** no longer looks as if it's running, or still loading: after
  20 pictures in a row that fail it stops, saying why. A classification, rotated-box or depth model is
  refused when it loads, and a repeated error goes in the worker's log once, not for every picture.
- "Loading … (the first time, it downloads)" only mentions downloading for a stock model not yet downloaded.

Found by a review of 0.1.0b1, each one a way the arm could move when it shouldn't, keep moving after
STOP, or be driven by the wrong page. Every fix has a test in `slixer/tests/test_safety.py`.

- **STOP** drops moves still on their way from the page, and holds the arm where it actually is, not where
  it was last told to go. STOP, changing mode and stopping a program are carried out at once, even behind
  a slow request.
- **Esc during a drag** ends the drag: neither the handle nor a held slider sends anything more.
- **A lost connection** is noticed within 3 seconds and said plainly. A STOP pressed while it's down is
  sent the moment it's back.
- **Closing the last page** lets go of the arm even if one of its tasks has failed, and one bad state
  update no longer ends the updates.
- **Importing a big STL** no longer holds up STOP: it's prepared in a process of its own.
- **Programs wait for the real arm** in Drive: a move ends when the arm gets there, the program pauses
  while the arm is out of touch, and it stops, holding the arm, if the arm gets no closer for 3 seconds.
- **Drive** waits for a follower that's ready and calibrated, and ends, saying why, if the follower stops
  taking poses.
- **Switching from Drive to Plan** holds the real arm where it is, as Plan promises, instead of letting it
  glide back to the leader. Watch lets go.
- **Other websites can't drive the arm** through your browser: Slixer answers only its own page, at its
  own names. `--allow-host` adds a name.
- **A gripper map whose shut and open are less than 5° apart** is refused. It closed the gripper in Drive.
- Firmware: the follower reports where the arm really is while it waits for the leader, and keeps checking
  its servos while it holds the arm frozen.
- Firmware: `H` and `R` switch the follower's torque off first, so support the arm. Re-centring a servo
  that was still holding a pose would send it somewhere else.
- Firmware: a sweep across the encoder's 0/4095 seam is refused rather than saved as a nearly full-circle
  range, and a range like that already in a servo is reported at start-up. A refused save keeps recording.

## [0.1.0b1] - 2026-09-22

The first public beta.

### Added

- **A live 3D model of the SO-101**, from TheRobotStudio's URDF, that mirrors the real arm as it moves.
- **Three modes.** **Watch** mirrors the arm and never sends anything. **Plan** is a virtual arm to pose
  and rehearse on, with the real arm drawn as a ghost. **Drive** moves the real arm: never with a jump,
  at up to 90°/s, and within the range the arm was calibrated over.
- **STOP** (or Esc, even while typing) ends a program and holds the arm where it is. Switching to Watch,
  or closing every Slixer tab, lets go of it.
- **Programs** of move, wait, gripper and wait-for steps: recorded by demonstration (move the leader,
  press R), or posed by dragging the model or sliding each joint. Saved as readable JSON in the arm's own
  units, so re-lining the model up can never change what a program does.
- **Parts.** Import STL files (millimetre exports are recognised and scaled), place them by dragging or by
  numbers, lay them flat, and double-click one to send the gripper to it.
- **The camera**: a Raspberry Pi's stream, in its own tab or floating over the model.
- **YOLO detection and segmentation** on the GPU, in a process of its own so the arm never waits on it:
  stock models, open-vocabulary YOLOE that finds whatever you type, or your own. A program can wait until
  the camera sees something. Installed with `uv sync --extra vision`.
- **Training pictures**, each saved with the arm's pose and draft labels from the running model, and
  `slixer/train.py` to train a model on them.
- **Setup**: line the model up with your arm (flip, offset, or set every zero from one pose), with the
  arm's and the model's readings side by side, live.
- **Firmware** for two XIAO ESP32-C3 boards: leader-follower teleoperation over ESP-NOW with no computer;
  Wi-Fi, to reach a PC; joint travel re-recorded from the Serial Monitor; and protocol 2, in which a
  program's poses take the arm from the leader while they keep coming.
- `pc/so101_link.py` and `pc/camera_client.py`, for driving the arm and reading the camera from your own
  code, and `pi/camera_stream.py`, the camera streamer.

[Unreleased]: https://github.com/LaingLab/SLIXER/compare/v0.1.0b1...HEAD
[0.1.0b1]: https://github.com/LaingLab/SLIXER/releases/tag/v0.1.0b1
