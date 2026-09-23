# Slixer

A 3D model of your SO-101, the camera beside it, and a place to write what it should do — in one page in
a browser. The model is the real SO-101 geometry; its six joints are your six servos.

```bash
uv run slixer/run.py                          # then open http://localhost:8000
uv run slixer/run.py --camera 192.168.1.50    # tell it where the camera Pi is (remembered after that)
```

`./slixer/start.sh` does the same from any folder. New here? Read [FIRST_RUN.md](FIRST_RUN.md): it walks
through the first session in order.

## Three modes

The switch at the top right decides what the model on screen *means*, and the edge of the view is
coloured so you can't mistake which one you're in.

| mode | the model is | sends to the arm | edge |
| --- | --- | --- | --- |
| **Watch** | a mirror of the real arm | never | none |
| **Plan** | a virtual arm you pose and rehearse on | never | blue |
| **Drive** | what you are telling the real arm | yes | orange |

**Watch** is where it starts, and where you teach by demonstration: move the leader by hand, the model
follows, press **record pose**. Nothing you do in Watch moves anything.

**Plan** is where programs get written. The model comes loose from the arm: drag the orange handle,
use the sliders, double-click a part, play a program — the real arm stays exactly where it is, drawn as a
blue ghost so you can see it. Plans are still held to the arm's own calibrated joint limits.

**Drive** moves the real arm. Switching in never makes it jump (it starts from where the arm is),
commands walk at up to 90°/s rather than snapping, and everything is held to the range the arm was
calibrated over. **STOP** or **Esc** — which works even while you're typing — ends the program and
**holds the arm where it is**, still in Drive. Switching to **Watch** is how you let go of it: the arm
then freezes, or, with the leader on, glides back to the leader. Closing every Slixer tab lets go too.
Dragging the orange handle only ever bends the arm on from the pose it's in — it never refolds it — and
lining the model up in Setup is refused while driving, since it would move the arm.

## Programs

A program is a list of steps: **move**, **wait**, **gripper**, and **wait for** (until the camera sees
something). New steps go in after the selected one. Select a move and it's drawn in amber — see where it
goes without going there. Double-click a move (or press **go there**) to send the model there in Plan,
or the arm in Drive. **set to current pose** re-records a step. **▶ from here** plays from the selected
step. Programs save to `slixer/programs/` as readable JSON.

**Moves store the arm's own numbers** — five joint angles in degrees from the middle of each joint's
calibrated travel, plus the gripper in percent: the same numbers the leader prints on its serial port.
Not the 3D model's angles. That means re-lining the model up in Setup can never change what a saved
program does to the arm.

Keys: **R** records a pose from any tab. On the Program tab, **↑/↓** select, **Delete** removes a step.

## Parts

The **Scene** tab imports STL files — the bench, a fixture, the box you pick from. Put each where the real
one sits (drag it, or type millimetres; **sit on the floor** drops it flat, whichever way it's turned).
Then, in Plan or Drive, **double-click any part's surface** and the gripper reaches for it, stopping a set
distance short along the surface — press R and that's a waypoint.

CAD and slicers export in millimetres; the model is in metres. Anything implausibly large is read as
millimetres and scaled, and the panel says so. **Parts are drawn, not felt**: the arm doesn't know they
exist and will drive straight through one.

## Camera and models

The Camera tab shows the Pi's stream exactly as it arrives. Give it the Pi's address once; it's
remembered. **camera** (top right of the view) floats it over the model; drag to move, pull the corner to
resize.

A YOLO model can run on the picture, on this PC's GPU, in a process of its own: everyday things,
**anything you type** (open-vocabulary YOLOE: "well plate, glove"), or a model you've trained on your own
things. What it finds is outlined on the picture, and a program's **wait for** step can wait for any of
those names. The Camera tab also collects training pictures, with draft labels, for training your own.
It needs the vision extra (`uv sync --extra vision`). All of it is in **[VISION.md](VISION.md)**.

## Setup: lining the model up

The URDF has its own zero for each joint, and which way your servos count depends on how they were
assembled — so the first time, the model may turn some joints the wrong way. The Setup tab shows each
joint's reading from the arm beside the model's, live. Move a joint by hand: **flip** any that turn the
wrong way, **offset** any that sit at an angle, then **save**. Changes show at once but are only kept
when you press save.

## The leader arm and Drive

The leader can stay powered while Slixer drives. Slixer's poses carry a marker saying they come from a
program, and the follower gives them the arm while they keep coming, holding the leader back. A quarter
of a second after Slixer stops sending (you switch to Watch), the arm glides back to the leader. So you
can teach with the leader, play, and carry on teaching, without powering anything down. It follows that
grabbing the leader doesn't take over from a running program: **STOP** does.

That needs the follower firmware in this repository (protocol 2, which the follower reports in its
status). With older firmware, the leader and Slixer fight over the arm, so power the leader down before
Drive. Either way, if the arm stops following what Slixer tells it, a red banner says so.

## On the network

The default is localhost only: this page can move a robot. `--host 0.0.0.0` opens it to the lab, with no
login. The arm's own link is plain UDP on port 50101 with no password: anything on the same network can
send it poses, so keep it on a network you trust. Under WSL the arm's UDP only arrives with mirrored
networking on.

## Tests

```bash
uv run pytest     # ~140 tests, about two minutes, no hardware
```

They run against `tests/fake_follower.py`, which runs the follower firmware's own logic (both the old and
the new), and `tests/fake_camera.py`, in a throwaway data folder on ports the real arm never uses. The
YOLO tests run when the vision extra is installed and the models are downloaded, and are skipped
otherwise. `--data DIR` and `--arm-port N` do the same for a hands-on session with no arm:

```bash
uv run slixer/tests/fake_follower.py --listen 50198 --report-to 50199 &
uv run slixer/tests/fake_camera.py --port 50113 &
uv run slixer/run.py --arm-port 50199 --camera 127.0.0.1 --camera-port 50113 --port 8001 --data /tmp/slixer-try
```

## The pieces

| file | what it does |
| --- | --- |
| `run.py`, `start.sh` | start everything |
| `server.py` | the page, the camera, the live link |
| `session.py` | the only thing that sends to the arm: modes, one loop, all the safety rules |
| `kinematics.py` | reads the URDF; forward kinematics and the IK behind dragging and reaching |
| `mapping.py` | model angles ↔ the arm's own numbers |
| `program.py` | steps, and how a move is played out over time |
| `scene.py` | imported parts |
| `vision.py` | the model's settings → what a program can wait for |
| `detection.py`, `yolo_worker.py` | the YOLO model, in a process of its own |
| `dataset.py`, `train.py` | training pictures; training your own model |
| `settings.py`, `paths.py`, `version.py` | what's remembered, and where; which version this is |
| `web/` | the page; `three.js` is vendored so it works without internet |

The model is from [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)
(`Simulation/SO101`, Apache-2.0: see `assets/so101/`); `bake_meshes.py` turns its STLs into what the page
loads.
