"""Trains a YOLO model on your own pictures, and hands it to Slixer.

    uv run slixer/train.py lab                          # a dataset Slixer collected
    uv run slixer/train.py ~/Downloads/export/data.yaml  # an annotation tool's YOLO export
        --model yolo26s-seg.pt   what to start from (default): outlines. yolo26s.pt for boxes only.
        --epochs 100             passes over the pictures; more helps up to a point
        --imgsz 640

For a dataset Slixer collected, correct the draft labels first (see VISION.md). Pictures with no label file
are left out; a picture whose label file is empty is kept as a picture of nothing, which teaches the model
what isn't there -- worth a few of those.

The finished model is copied into slixer/models/ as <dataset>-<date>.pt, and shows up in Slixer's list of
models straight away. On an RTX 4090 a few hundred pictures train in minutes. Needs the vision extra
(uv sync --extra vision).
"""

from __future__ import annotations

import argparse
import hashlib
import shutil
import sys
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import paths  # noqa: E402

MIN_PICTURES = 10


def slixer_dataset(name: str) -> tuple[Path, dict]:
    """Writes a data.yaml for a dataset Slixer collected, splitting it 80/20 into training and checking."""
    where = paths.DATASETS / name
    if not (where / "images").exists():
        raise SystemExit(f"no dataset called {name!r} in {paths.DATASETS}")
    classes = [c.strip() for c in (where / "classes.txt").read_text().splitlines() if c.strip()] \
        if (where / "classes.txt").exists() else []
    if not classes:
        raise SystemExit(f"{where / 'classes.txt'} is missing or empty: one class name per line, in label order")

    labelled, outlines = [], False
    for image in sorted((where / "images").glob("*.jpg")):
        label = where / "labels" / f"{image.stem}.txt"
        if not label.exists():
            continue
        labelled.append(image)
        for line in label.read_text().splitlines():
            parts = line.split()
            if parts and int(parts[0]) >= len(classes):
                raise SystemExit(f"{label.name} uses class {parts[0]}, but classes.txt only has {len(classes)}")
            outlines |= len(parts) > 5
    if len(labelled) < MIN_PICTURES:
        raise SystemExit(f"only {len(labelled)} labelled pictures in {name}; label at least {MIN_PICTURES} first")

    # The same picture always lands in the same half, so the checking set doesn't drift between runs.
    def checking(image: Path) -> bool:
        return int(hashlib.sha1(image.name.encode()).hexdigest(), 16) % 5 == 0

    train = [str(p) for p in labelled if not checking(p)]
    val = [str(p) for p in labelled if checking(p)] or train[-1:]
    (where / "train.txt").write_text("\n".join(train) + "\n")
    (where / "val.txt").write_text("\n".join(val) + "\n")
    yaml = where / "data.yaml"
    names = "\n".join(f"  {i}: {n}" for i, n in enumerate(classes))
    yaml.write_text(f"path: {where}\ntrain: train.txt\nval: val.txt\nnames:\n{names}\n")
    return yaml, {"pictures": len(labelled), "train": len(train), "val": len(val), "classes": classes,
                  "outlines": outlines}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("dataset", help="a dataset name from slixer/datasets, or a path to a data.yaml")
    parser.add_argument("--model", default="yolo26s-seg.pt")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=16)
    args = parser.parse_args()

    if args.dataset.endswith((".yaml", ".yml")):
        yaml, summary = Path(args.dataset).expanduser().resolve(), None
        name = yaml.parent.name
    else:
        yaml, summary = slixer_dataset(args.dataset)
        name = args.dataset
        print(f"{summary['pictures']} labelled pictures ({summary['train']} to learn from, {summary['val']} to "
              f"check against), classes: {', '.join(summary['classes'])}")
        if args.model.endswith("-seg.pt") and not summary["outlines"]:
            raise SystemExit("the labels are boxes, but the model outlines things: use --model yolo26s.pt, "
                             "or draw outlines rather than boxes when labelling")

    from ultralytics import YOLO

    stamp = datetime.now().strftime("%Y%m%d-%H%M")
    base = paths.MODELS / args.model if (paths.MODELS / args.model).exists() else args.model
    model = YOLO(str(base))
    results = model.train(data=str(yaml), epochs=args.epochs, imgsz=args.imgsz, batch=args.batch,
                          project=str(yaml.parent / "runs"), name=stamp, exist_ok=False)
    best = Path(results.save_dir) / "weights" / "best.pt"
    if not best.exists():
        raise SystemExit(f"training finished without a best.pt in {best.parent}")
    paths.MODELS.mkdir(parents=True, exist_ok=True)
    finished = paths.MODELS / f"{name}-{stamp}.pt"
    shutil.copy2(best, finished)
    print(f"\ntrained: {finished}\npick {finished.name} on Slixer's Camera tab (reload the page if it's open)")


if __name__ == "__main__":
    main()
