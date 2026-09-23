# Changelog

All notable changes to Slixer are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres
to [Semantic Versioning](https://semver.org/spec/v2.0.0.html). Versions are written the Python way
([PEP 440](https://peps.python.org/pep-0440/)): 0.1.0b1 is 0.1.0-beta.1.

## [Unreleased]

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
