"""3.120's helpers: seeds to walk from, and the comparison of `_walk_dist()`
against the frozen `tests/walk_reference.py` (exact: the same distances,
`inf` included, bit for bit)."""
import random

import numpy as np

from brain import room_labels as rl
from tests import walk_reference as ref

# Two door costs beside the shipped one: none, and less than a step's worth
# of the shipped spacing.
DOORS = (rl.DOOR_M, 0.0, 0.7)


def seeds(rng: random.Random, seg: np.ndarray, n: int) -> list:
    """`n` seeds as `_wants()` passes them: mostly floor cells, some on walls
    or unknown (they snap or drop), some off the map (`(-1, -1)`, a seed from
    a larger map), and repeats."""
    h, w = seg.shape
    floor = np.argwhere(seg >= 0)
    out = []
    for _ in range(n):
        r = rng.random()
        if out and r < 0.1:
            out.append(rng.choice(out))
        elif r < 0.2 or not floor.size:
            out.append((-1, -1))
        elif r < 0.4 and h and w:
            out.append((rng.randrange(h), rng.randrange(w)))
        else:
            out.append(tuple(int(v) for v in floor[rng.randrange(len(floor))]))
    return out


def differences(seg: np.ndarray, sources: list, res: float, doors=DOORS) -> list:
    """The door costs at which `_walk_dist()` differs from the reference."""
    bad = []
    for door in doors:
        d0, _ = ref._walk(seg, sources, res, door)
        d1 = rl._walk_dist(seg, sources, res, door)
        if not (d0.dtype == d1.dtype and np.array_equal(d0, d1)):
            bad.append(door)
    return bad


def map_differences(rng: random.Random, m: dict, counts=(1, 5, 20), doors=DOORS) -> list:
    """Cut `m`, then walk from `counts` seeds each: (count, door) pairs that
    differ."""
    seg, _ = rl.segment(m)
    return [(n, door) for n in counts
            for door in differences(seg, seeds(rng, seg, n), m["resolution_m"], doors)]
