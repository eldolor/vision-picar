"""
evaluations/goal-learn-396/features.py -- 3.96 A: the room features, and the
simulator's stand-in room labeller.

What the ten geometric features (`brain/goal_policy.py` FEATURES) cannot
say: which room a candidate is in, whether the robot has stood there, and
how likely the target is in a room of that type. Computed from the brain's
own map (`segment()`), its pose trail, and room labels. Since 3.96.C the
room features live in `brain/goal_policy.py` (what the brain runs) and are
re-exported here; the stand-in labeller stays here.
"""
import math

from brain.frontier import _cell_of
from brain.goal_policy import (ROOM_AREA_M2, ROOM_FEATURES, ROOM_RES_M,  # noqa: F401
                               _seg_at, coarse, room_features, room_type)
from world.interface import CELL_UNKNOWN

# The stand-in labeller names a room once this share of its floor is known
# on the brain's map (amendment 1).
KNOWN_SHARE = 0.25


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
