<div align="center">

# Slixer

**A browser workbench for the SO-101 robot arm**

See it, program it, and let the camera tell it when.

[![License: MIT](https://img.shields.io/badge/license-MIT-blue?style=flat-square)](LICENSE)
[![Python](https://img.shields.io/badge/python-3.10%20%7C%203.11%20%7C%203.12%20%7C%203.13-blue?style=flat-square)](https://www.python.org/)
[![Platform](https://img.shields.io/badge/platform-Linux%20%7C%20WSL2-lightgrey?style=flat-square)](#prerequisites)
[![Status](https://img.shields.io/badge/status-beta-orange?style=flat-square)](https://github.com/LaingLab/SLIXER/releases)

[**Quick start**](#quick-start) ·
[**First run**](slixer/FIRST_RUN.md) ·
[**User guide**](slixer/README.md) ·
[**Vision**](slixer/VISION.md)

![Slixer's 3D view of the arm beside the real SO-101, moving together](docs/arm.gif)

</div>

---

Slixer runs on a PC beside an [SO-101](https://github.com/TheRobotStudio/SO-ARM100) arm and serves one page to
a browser. On it:

- **A live 3D model of the arm**, built from the SO-101's own CAD, that mirrors the real arm as you move it.
- **Programs**, recorded by demonstration (move the leader arm, press R) or posed by dragging the model, then
  rehearsed on a virtual arm and played smoothly on the real one.
- **Your bench in 3D.** Import STL files of fixtures and labware, place them where they really are, and
  double-click one to send the gripper to it.
- **The camera**, with [Ultralytics](https://github.com/ultralytics/ultralytics) YOLO detecting and outlining
  what it sees on your GPU, including open-vocabulary YOLOE ("well plate, pipette tip") that needs no
  training. A program can wait until the camera sees something. Slixer also collects training pictures, with
  draft labels, for teaching a model your own things.

Three modes keep it clear what the model means: **Watch** mirrors the arm and never sends anything, **Plan**
is a virtual arm to rehearse on, and **Drive** moves the real one, speed-limited, with **STOP** (or Esc)
always a keypress away.

## How it fits together

```
 leader arm ──radio──▶ follower arm ◀──Wi-Fi──▶ PC running Slixer ◀──▶ browser
  (XIAO ESP32-C3)       (XIAO ESP32-C3)              ▲
                                                     │ camera frames
                                          Raspberry Pi + USB camera
```

The two arms teleoperate each other with no computer at all, over their own radio link. Slixer joins over
Wi-Fi: it watches the follower, and can drive it. It doesn't use lerobot's USB connection; the arms run the
firmware in this repository.

## Prerequisites

1. [uv](https://docs.astral.sh/uv/getting-started/installation/), which manages Python and the environment:
   `curl -LsSf https://astral.sh/uv/install.sh | sh`
2. **The arms**: an SO-101 leader and follower, each on a Seeed *Bus Servo Driver Board for XIAO* with a
   XIAO ESP32-C3, flashed from `so101_leader/` and `so101_follower/`. See the
   [firmware guide](so101_leader/README.md).
3. **Linux or WSL2.** Under WSL2, turn on mirrored networking so the arm's packets reach it (the firmware
   guide says how).
4. For the camera: a Raspberry Pi with a USB camera, running `pi/camera_stream.py`. For YOLO: an NVIDIA GPU
   with a recent driver (CUDA 13). Without one it runs on the CPU, slowly.

## Quick start

```bash
git clone https://github.com/LaingLab/SLIXER.git
cd SLIXER
uv sync --extra vision      # or plain `uv sync` without YOLO: the extra brings PyTorch, a few GB
uv run slixer/run.py        # then open http://localhost:8000
```

On the camera Pi: `uv run pi/camera_stream.py`. Then tell Slixer where it is, once:
`uv run slixer/run.py --camera <the Pi's address>`.

[**FIRST_RUN.md**](slixer/FIRST_RUN.md) walks through the first session in order: flashing, lining the
model up with your arm, a first program, and the camera.

> **Keep `--extra vision` on every `uv sync`.** A plain `uv sync` makes the environment match the lockfile
> exactly, so it removes PyTorch and Ultralytics again. `uv run` never removes anything.

## Documentation

| | |
| --- | --- |
| [slixer/FIRST_RUN.md](slixer/FIRST_RUN.md) | the first session, step by step |
| [slixer/README.md](slixer/README.md) | the user guide: modes, programs, parts, camera, lining up |
| [slixer/VISION.md](slixer/VISION.md) | YOLO models, open-vocabulary detection, and training your own |
| [so101_leader/README.md](so101_leader/README.md) | the arms' firmware: wiring, flashing, Wi-Fi, calibration |
| [CHANGELOG.md](CHANGELOG.md) | what's changed |

## What's in here

| folder | |
| --- | --- |
| `slixer/` | the app: a FastAPI server and the page it serves (three.js) |
| `so101_leader/`, `so101_follower/` | firmware for the two arms' XIAO ESP32-C3 boards |
| `pc/` | Python for your own code: `so101_link.py` talks to the arm, `camera_client.py` receives the camera |
| `pi/` | the camera streamer, for the Raspberry Pi |

## Safety

Slixer moves a robot. It's a beta, so go gently the first time it drives your arm.

- **STOP** (or **Esc**, which works even while you're typing) ends a program, drops anything still on its
  way, and holds the arm where it actually is. Switching to **Watch**, or closing every Slixer tab, lets go
  of it; switching to **Plan** keeps holding it.
- In **Drive**, moves are speed-limited and held to the range the arm was calibrated over. Drive waits for
  a follower that says it's ready, and a program waits for the arm: it pauses while the arm is out of
  touch, and stops, holding it, if the arm can't get where it's sent.
- **If the page loses Slixer**, it says so, and until it reconnects it can't send anything, STOP included.
  A STOP pressed meanwhile is sent the moment it's back; if the arm must stop sooner, switch off its power.
- **Parts are drawn, not felt.** The arm doesn't know an imported part is there, and will drive through it.
- **The page is served to this PC only**, unless you start it with `--host 0.0.0.0`, and only Slixer's own
  page can drive the arm, not another website open in the same browser. The arm's Wi-Fi link has no
  password of its own: anything on the same network can send it poses. Keep the arm on a network you
  trust. See [SECURITY.md](SECURITY.md).

## Development

```bash
uv run pytest               # about 160 tests, two minutes, no hardware needed
```

The tests run against an imitation follower that runs the firmware's own logic (`slixer/tests/fake_follower.py`)
and an imitation camera (`slixer/tests/fake_camera.py`), on ports the real arm never uses. The YOLO tests run
when the vision extra is installed and the models are downloaded, and are skipped otherwise.

## Licence and credits

MIT, © 2026 Laing Lab. See [LICENSE](LICENSE).

- The SO-101 model (URDF and meshes) is [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100),
  under the Apache License 2.0: see [slixer/assets/so101/](slixer/assets/so101/).
- [three.js](https://threejs.org), vendored in `slixer/web/vendor/`, is MIT.
- The vision extra installs [Ultralytics](https://github.com/ultralytics/ultralytics), which is AGPL-3.0 (as are
  its YOLO weights). It isn't part of this repository, and the rest of Slixer runs without it.

If Slixer helps your research, please cite it: see [CITATION.cff](CITATION.cff).
