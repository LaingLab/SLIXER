"""Regions of a picture, as Slixer draws them: outlines thinned for drawing, and the patches a semantic model marks.

An instance model (the -seg ones) reports each thing it finds, with a box, an outline and a confidence. A
semantic model (yolo26n-sem, say, trained on the Ultralytics Platform) reports no things at all: it gives every
pixel of the picture a class -- this is plate, that is background. Slixer draws finds and waits for them, so
each patch of a class becomes one here: its outline, the box around it, and its middle.

A class map carries no confidence, so these finds have none (a score of None), and a model's "sure enough at"
threshold doesn't change them.
"""

from __future__ import annotations

import cv2
import numpy as np

MAX_POINTS = 80  # an outline this detailed is plenty to draw, and keeps each message small
SMALLEST_PATCH = 0.0002  # of the picture; smaller is a speck, not a thing (14 x 14 pixels at 1280 x 720)
MOST_PATCHES = 30  # the biggest this many, per picture
IGNORED = 255  # Ultralytics' mark for a pixel outside the classes asked for
BACKGROUND = {"background", "bg", "_background_", "unlabeled", "unlabelled", "void"}


def thin(points, limit: int = MAX_POINTS) -> list:
    """At most `limit` points of an outline, evenly spaced along it."""
    if len(points) <= limit:
        return [[round(float(x), 4), round(float(y), 4)] for x, y in points]
    step = len(points) / limit
    return [[round(float(points[int(i * step)][0]), 4), round(float(points[int(i * step)][1]), 4)]
            for i in range(limit)]


def patches(class_map: np.ndarray, names: dict[int, str]) -> list[dict]:
    """Each patch of each class in a semantic model's class map, as a find: the biggest first.

    `class_map` holds a class for every pixel, at the picture's own size. A model of one class marks it 1 on a
    background of 0. A model of several gives each pixel its class's number, and a class named like
    "background" isn't a find.
    """
    height, width = class_map.shape[:2]
    if len(names) == 1:
        wanted = {1: (0, next(iter(names.values())))}
    else:
        wanted = {int(cls): (int(cls), name) for cls, name in names.items()
                  if str(name).strip().lower() not in BACKGROUND}
    smallest = SMALLEST_PATCH * width * height
    found = []
    for value in np.unique(class_map).tolist():
        if value == IGNORED or value not in wanted:
            continue
        cls, label = wanted[value]
        contours, _ = cv2.findContours((class_map == value).astype(np.uint8), cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < smallest:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            moments = cv2.moments(contour)
            middle = (moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]) if moments["m00"] \
                else (x + w / 2, y + h / 2)
            found.append((area, {
                "label": str(label),
                "cls": cls,
                "score": None,
                "box": [round(x / width, 4), round(y / height, 4), round((x + w) / width, 4), round((y + h) / height, 4)],
                "centre": [round(middle[0] / width, 4), round(middle[1] / height, 4)],
                "polygon": thin(contour[:, 0, :] / (width, height)),
            }))
    found.sort(key=lambda pair: -pair[0])
    return [hit for _, hit in found[:MOST_PATCHES]]
