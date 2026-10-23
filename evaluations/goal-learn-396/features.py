"""
evaluations/goal-learn-396/features.py -- 3.96 A: the room features, and the
simulator's stand-in room labeller.

What the ten geometric features (`brain/goal_policy.py` FEATURES) cannot
say: which room a candidate is in, whether the robot has stood there, and
how likely the target is in a room of that type. Computed from the brain's
own map (`segment()`), its pose trail, and room labels. They live here for
step A's separability check; step B moves them into `brain/goal_policy.py`
if criterion 1 is met.
"""
import math
import re

import numpy as np

from brain.frontier import _cell_of
from brain.room_labels import segment
from world.interface import CELL_UNKNOWN

# Appended after FEATURES, in this order.
ROOM_FEATURES = (
    "room_new",      # the candidate's room (a `segment()` piece) never stood in
    "room_area",     # that room's known floor, / 20 m2
    "room_other",    # a different room from the robot's
    "room_share",    # share of the pool's candidates in the same room
    "type_known",    # a room label names the candidate's room type
    "prior",         # P(target | that type); 0 when the type is unknown
    "prior_new",     # prior x room_new
)
ROOM_AREA_M2 = 20.0
# The stand-in labeller names a room once this share of its floor is known
# on the brain's map (amendment 1).
KNOWN_SHARE = 0.25


def _seg_at(seg, m: dict, x_m: float, y_m: float) -> int:
    """The segment id under a point, or of the nearest floor cell within one
    cell (a goal can sit on a frontier's unknown edge); -1 if none."""
    h, w = seg.shape
    cx, cy = _cell_of(m, x_m, y_m)
    for dx, dy in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (1, -1), (-1, 1)):
        i, j = cx + dx, cy + dy
        if 0 <= i < w and 0 <= j < h and seg[j, i] >= 0:
            return int(seg[j, i])
    return -1


def room_features(m: dict, pose: dict, goals: list, *, trail: list, types: list,
                  prior: dict) -> list:
    """Each candidate's ROOM_FEATURES. `goals`: the pool's (x_m, y_m) in
    order; `trail`: house-frame points the robot has stood on; `types`: each
    goal's room type from the labels, or None; `prior`: type -> P(target)."""
    seg, _ = segment(m)
    res2 = m["resolution_m"] ** 2
    ids, counts = np.unique(seg[seg >= 0], return_counts=True)
    area = dict(zip(ids.tolist(), counts.tolist()))
    here = _seg_at(seg, m, pose["x_m"], pose["y_m"])
    stood = {_seg_at(seg, m, x, y) for x, y in trail} | {here}
    rooms = [_seg_at(seg, m, x, y) for x, y in goals]
    out = []
    for r, t in zip(rooms, types):
        new = 1.0 if r >= 0 and r not in stood else 0.0
        p = float(prior.get(t, 0.0)) if t is not None else 0.0
        out.append([
            new,
            (area.get(r, 0) * res2 / ROOM_AREA_M2) if r >= 0 else 0.0,
            1.0 if r >= 0 and r != here else 0.0,
            (sum(x == r for x in rooms) / len(rooms)) if r >= 0 else 0.0,
            0.0 if t is None else 1.0,
            p,
            p * new,
        ])
    return out


def room_type(name: str) -> str:
    """'bedroom 2' -> 'bedroom'."""
    return re.sub(r" \d+$", "", name)


class Labeller:
    """The stand-in: the true room's type under a point, once KNOWN_SHARE of
    that room's floor is known on the brain's map; None before that, or off
    every room. `drop`: room names whose labels are withheld (criterion 3)."""

    def __init__(self, grid, cell_m: float, drop=frozenset()):
        self.cell_m = cell_m
        self.room_of = {}
        self.cells = {}
        for name, cells in grid.rooms.items():
            self.cells[name] = list(cells)
            for c in cells:
                self.room_of[c] = name
        self.drop = drop

    def known_rooms(self, m: dict) -> set:
        w, h = m["width"], m["height"]
        out = set()
        for name, cells in self.cells.items():
            if name in self.drop or not cells:
                continue
            known = 0
            for (i, j) in cells:
                mx, my = _cell_of(m, (i + 0.5) * self.cell_m, (j + 0.5) * self.cell_m)
                if 0 <= mx < w and 0 <= my < h and m["cells"][my * w + mx] != CELL_UNKNOWN:
                    known += 1
            if known >= KNOWN_SHARE * len(cells):
                out.add(name)
        return out

    def types(self, m: dict, goals: list, known=None) -> list:
        known = self.known_rooms(m) if known is None else known
        out = []
        for x, y in goals:
            cx, cy = math.floor(x / self.cell_m), math.floor(y / self.cell_m)
            name = self.room_of.get((cx, cy))
            if name is None:
                near = [self.room_of[(cx + dx, cy + dy)] for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                        if (cx + dx, cy + dy) in self.room_of]
                name = near[0] if near else None
            out.append(room_type(name) if name in known else None)
        return out
