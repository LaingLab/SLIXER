# First run

In order. The first two steps need doing once; after that, starting Slixer is one command.

## 1. Flash both boards (once)

The [firmware guide](../so101_leader/README.md) covers the hardware side: calibrating the arms, switching
their driver boards to XIAO mode, and fitting the antennas. Then:

1. In **both** `so101_leader/` and `so101_follower/`, copy `wifi_config.example.h` to `wifi_config.h` and
   put your network's name and password in it. (`wifi_config.h` stays out of git.)
2. Open each sketch in the Arduino IDE 2 and upload it: `so101_follower` to the follower's XIAO,
   `so101_leader` to the leader's. Pick the board by hand each time, **Tools → Board → esp32 →
   XIAO_ESP32C3**: the IDE likes to guess "Ozobot circuit kit", which is the wrong chip.

Power both arms. The follower should glide to the leader's pose and follow it. The leader's Serial Monitor
shows both boards' state every two seconds, the follower's for example:

```
[follower] following on radio + wifi (192.168.1.135, -41 dBm) | 812 poses (0 stale ignored, 0 leader poses held back for a program) | goals: ...
```

If it says `radio only (...)` instead, the bracket says why. Press **W** in either board's Serial Monitor to
list the networks it can hear. A follower that hears *no* networks at all while the arm's 12 V supply is
on, but does with it off, is being drowned by supply noise: move its antenna away from the servo board and
its power wires.

## 2. Install and start

```bash
git clone https://github.com/LaingLab/SLIXER.git
cd SLIXER
uv sync --extra vision
```

On the camera Pi (with its own copy of the repository): `uv run pi/camera_stream.py`

On the PC:

```bash
uv run slixer/run.py --camera 192.168.1.50     # your Pi's address: only needed the first time
```

Open **http://localhost:8000**. Top left should read something like **following the leader arm** (or
**holding**) and **camera**. If the arm pill says **no arm on the network**, the follower isn't on Wi-Fi
(step 1), or this PC can't hear it: under WSL2, turn on mirrored networking (see the firmware guide), and
`uv run pc/so101_link.py` prints whatever arrives.

## 3. Line the model up (about five minutes, once)

**Setup** tab, in **Watch**. Move one joint of the leader at a time; the follower copies it, and the model
should too.

- A joint turning the **wrong way**: **flip**.
- Turning the right way but **sitting at an angle**: type an **offset** until it matches.
- **save**.

The **arm** column is the arm's own reading; the **model** column is the model's. Programs don't depend
on any of this (they store the arm's own numbers), so you can record programs before lining up if you
like, but dragging the gripper and reaching for parts need it right.

## 4. A first program, by demonstration

1. **Program** tab, **Watch** mode.
2. Move the leader to a pose. Press **R**. Move, **R**. Three or four poses.
3. **+ gripper** between them if you like. Select any move to see it in amber.
4. Switch to **Plan** and press **▶ play**: it rehearses on the virtual arm. The real one doesn't move.
5. Hand near **STOP** (or Esc). Switch to **Drive**, press **▶ play**. The follower runs it. **STOP**
   holds the arm where it is; switch to **Watch** to let go of it.
6. **save** it with a name.

You can leave the leader on: it's held back while Slixer drives, and has the arm back a quarter of a second
after you switch to Watch. If the arm stops following, a red banner says so, with the likely reason.

## 5. Things to try

- **Plan mode, orange handle**: drag the gripper around; the sliders pose each joint.
- **Scene**: drop in an STL of something on your bench, place it, then (in Plan) **double-click its
  surface**: the gripper goes there, 20 mm short. **R** records it. A part that comes in standing on its
  edge (exported Y-up) needs **lie flat**.
- **Camera → Model**: pick `yoloe-26s-seg.pt`, type what to look for ("well plate, pipette tip"), tick
  **run**: it outlines them with no training. Or `yolo26s-seg.pt` for everyday things. A program's
  **+ wait for** step can then wait for any of those names. [VISION.md](VISION.md) has the rest, including
  training a model on your own things.
- **camera** (top right of the 3D view) floats the picture over the model; drag it about, pull its corner.

## If something's wrong

| you see | likely because |
| --- | --- |
| a joint of the model turns the wrong way | not lined up yet: Setup, flip |
| red banner "the arm isn't following" | the leader is on and the follower has older firmware (flash it), or something is blocking the arm |
| arm pill "only the leader is on Wi-Fi" | the follower isn't on Wi-Fi; Drive is refused, since it couldn't hear you |
| Drive refused: "no arm to drive" | no follower reports reaching this PC |
| camera pill "camera not answering" | `camera_stream.py` isn't running on the Pi, or the address is wrong (Camera tab) |
| Camera tab: "Models need the vision extra" | run `uv sync --extra vision`, then restart Slixer |

## A beta

Slixer is tested end to end against an imitation follower that runs the firmware's own logic, and an
imitation camera: every mode, programs, parts, YOLO models on the GPU, training pictures, calibration and
saving. Real arms differ, in joint directions, lighting, and Wi-Fi, so go gently the first time in Drive,
and please report what you find: https://github.com/LaingLab/SLIXER/issues
