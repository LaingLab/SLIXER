# Vision: detecting and outlining things

Slixer can run Ultralytics YOLO models on the camera, on this PC's GPU. On an RTX 4090 that's about 14 ms
a picture for segmentation, so it keeps up with the camera with room to spare. What a model finds is
outlined on the picture, and its names count for a program's **wait for** steps.

Everything here is on the **Camera** tab.

## How it's set up

Ultralytics and PyTorch are an optional extra, because they're a few gigabytes:

```bash
uv sync --extra vision
```

Keep `--extra vision` on every `uv sync` after that: a plain `uv sync` makes the environment match the
lockfile exactly, which removes them again. On Linux, PyTorch comes as its CUDA 13 build, which needs a
recent NVIDIA driver; without a usable GPU the models run on the CPU, slowly.

The model runs in a **process of its own**, so the arm can't be affected. Slixer's control loop runs 50
times a second in the main process; a model loading, a slow picture, or a CUDA error in the model's
process can't delay it or crash it. If the model's process dies, the Camera tab says why and the arm
carries on.

Ultralytics asks for the full `opencv-python`. Slixer uses `opencv-python-headless` (the same OpenCV
without a desktop's GUI libraries), and the two can't be installed together without overwriting each
other's files, so `pyproject.toml` leaves the full one out. Ultralytics works the same with either.

To upgrade Ultralytics: `uv lock --upgrade-package ultralytics && uv sync --extra vision`. To run the
models in some other environment, point `SLIXER_VISION_PYTHON` at its Python.

Models live in `slixer/models/`; stock ones download there the first time they're picked.

## Three kinds of model

| model | finds | when to use it |
| --- | --- | --- |
| `yolo26s-seg.pt` (n, s, m) | the 80 everyday COCO things — person, cup, bottle, … — outlined | trying things out |
| `yoloe-26s-seg.pt` | **whatever you type**: "well plate, pipette tip, glove" — outlined | your own things, today |
| `<yours>.pt` | exactly what you trained it on | once you've trained one |

`-seg` models outline what they find; the others only box it. The `yoloe-…-pf` model has a large built-in
vocabulary instead of being told what to look for.

**Start with YOLOE.** It finds things by name with no training at all: pick `yoloe-26s-seg.pt`, type what
to look for in the box, tick **run**. Whether it finds a 6-well plate well enough depends on the plate and
the light — try it. Where it's good enough, you're done; where it isn't, it still makes useful draft
labels for training your own model (below). Changing the words takes a moment, not a restart.

**sure enough at** is the confidence below which a find is ignored. Higher means fewer false alarms and
more misses.

## Teaching a model your own things

When a stock or open-vocabulary model isn't reliable enough, train one. The loop:

**1. Collect pictures** — Camera tab → *Training pictures*. Name the dataset (e.g. `plates`), then press
**capture** (or **C**), or set **every 2 s** and move things about. Each picture is saved exactly as the
camera sent it, with the arm's pose at that moment. With a model running and **draft labels** ticked,
what the model saw is saved as a first draft of the labels. An unchanged scene isn't saved twice.

Aim for a couple of hundred pictures to start, varied the way the arm will really see things: different
positions, angles, lighting, clutter, the gripper in shot, partly hidden objects, and a few with none of
your things in (they teach it what *isn't* there).

**2. Correct the labels** in an annotation tool. Outlines, not boxes, for a segmentation model.

- [CVAT](https://www.cvat.ai) or [Label Studio](https://labelstud.io) (both free; run locally or online),
  or [Roboflow](https://roboflow.com) (quickest to start).
- Import `slixer/datasets/<name>/images/` with the draft labels in `labels/` (YOLO format, class names
  in `classes.txt`), fix and finish them, and export as **YOLO / Ultralytics segmentation**.
- Put the exported `labels/` back over the dataset's `labels/`, and its class list in `classes.txt`
  (one name per line, in the same order as the class numbers) — or keep the tool's export whole and
  train from its `data.yaml` (next step).

Draft labels from a stock model use its names ("cup"); rename them to yours in the tool. YOLOE drafts
already carry the names you typed.

**3. Train.** On an RTX 4090, a few hundred pictures take minutes:

```bash
uv run slixer/train.py plates                          # a dataset Slixer collected
uv run slixer/train.py ~/Downloads/export/data.yaml    # or a tool's export
```

It splits the labelled pictures 80/20 into learning and checking, trains from `yolo26s-seg.pt`, and copies
the best result to `slixer/models/plates-<date>.pt`. `--model yolo26s.pt` trains a box-only model;
`--epochs` sets how long.

**4. Use it** — reload the Camera tab, pick `plates-<date>.pt`, tick **run**.

**5. Go round again.** Run your model while capturing more pictures: its drafts are now about your things,
so each round of correcting is faster, and the next model better.

## Using what it sees

- **In a program**: `+ wait for` → type or pick a name. The step waits until the model sees that name,
  or gives up after the time you set.
- **Where**: every find carries a box, a centre and (for `-seg`) an outline, all as fractions of the
  picture, in Slixer's state (`slixer.state.vision.found` in the browser console).

Not done yet: turning a position *in the picture* into a position *for the gripper*. That needs the
camera calibrated: its lens, and where it sits relative to the arm (on the wrist, where it points changes
as the arm moves). The arm's pose saved with every training picture is the start of that; it's the
natural next step once detection is reliable.

## If something's wrong

| the Camera tab says | because |
| --- | --- |
| *Models need the vision extra* | run `uv sync --extra vision`, then restart Slixer |
| *Loading … (the first time, it downloads)* | fetching the weights: a few seconds to a minute |
| *… finds things by name: type what to look for* | YOLOE needs words in the **look for** box |
| *Stopped: …* | the model process died; the reason follows, and `vision-worker.log` has the rest |
| it runs but finds nothing | lower **sure enough at**; check the words (YOLOE) or the model |
