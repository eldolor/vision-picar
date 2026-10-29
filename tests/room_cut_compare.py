"""3.118's helpers: maps to cut, and the comparison against the frozen
`tests/room_cut_reference.py` (exact: the same room ids, the clearance bit
for bit)."""
import random

import numpy as np

from brain import room_labels as rl
from sim.maps import build_world
from tests import room_cut_reference as ref
from world.interface import CELL_FREE, CELL_OCCUPIED, CELL_UNKNOWN

# Two neck ratios beside the shipped one, to reach other merge decisions.
RATIOS = (rl.NECK_RATIO, 0.5, 0.9)


def house_map(name: str, k: int = 1, pad: int = 0) -> dict:
    """A sim house fully mapped: walls and furniture occupied, the rest
    free, each 0.30 m cell drawn as k x k (k = 6 is the car's 0.05 m), with
    `pad` cells of unknown round it (3.70's car-resolution maps: k 6, pad
    20)."""
    w = build_world(name)
    g = np.full((w.height, w.width), CELL_OCCUPIED, dtype=np.int64)
    for y, row in enumerate(w.layout):
        for x, ch in enumerate(row):
            if ch != "#" and (x, y) not in w.solid_cells:
                g[y, x] = CELL_FREE
    g = np.kron(g, np.ones((k, k), dtype=np.int64))
    g = np.pad(g, pad, constant_values=CELL_UNKNOWN)
    h, wd = g.shape
    return {"width": int(wd), "height": int(h), "resolution_m": 0.30 / k,
            "cells": g.ravel().tolist()}


def random_map(rng: random.Random, max_side: int) -> dict:
    """A random grid: its side lengths, its mix of free, occupied and
    unknown, and blocky or speckled cells, all drawn from `rng`."""
    w, h = rng.randint(1, max_side), rng.randint(1, max_side)
    kind = rng.choice(("mix", "free", "occupied", "unknown", "blocks", "rooms"))
    if kind in ("free", "occupied", "unknown"):
        v = {"free": CELL_FREE, "occupied": CELL_OCCUPIED, "unknown": CELL_UNKNOWN}[kind]
        cells = [v] * (w * h)
    elif kind == "mix":
        weights = [rng.random() for _ in range(3)]
        cells = rng.choices((CELL_FREE, CELL_OCCUPIED, CELL_UNKNOWN), weights, k=w * h)
    else:
        # Mostly floor with walls and unknown patches: room-like shapes.
        g = np.full((h, w), CELL_FREE, dtype=np.int64)
        for _ in range(rng.randint(0, 12)):
            j, i = rng.randrange(h), rng.randrange(w)
            v = CELL_OCCUPIED if kind == "rooms" or rng.random() < 0.6 else CELL_UNKNOWN
            if rng.random() < 0.5:
                g[j, i:i + rng.randint(1, w)] = v
            else:
                g[j:j + rng.randint(1, h), i] = v
        cells = g.ravel().tolist()
    return {"width": w, "height": h, "resolution_m": rng.choice((0.05, 0.1, 0.3)),
            "cells": cells}


def differences(m: dict, ratios=RATIOS) -> list:
    """Where the new cut disagrees with the frozen one, per ratio: empty
    when the room ids are equal and the clearance is equal bit for bit."""
    bad = []
    for r in ratios:
        s0, d0 = ref.segment(m, r)
        s1, d1 = rl.segment(m, r)
        if not np.array_equal(s0, s1):
            bad.append((r, "seg", int((s0 != s1).sum())))
        if d0.dtype != d1.dtype or not np.array_equal(d0, d1):
            bad.append((r, "clearance", int((d0 != d1).sum())))
    return bad
