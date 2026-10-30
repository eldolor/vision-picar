"""Frozen copy of `brain/room_labels.py`'s `_walk()` and `_floor_near()`
as they were before 3.120 (dev at 4d843a2), verbatim. 3.120 added
`_walk_dist()`, the same distances in numpy; `tests/test_walk_dist.py`
checks it against this one. Do not edit or "simplify" it: it is the
definition of the right answer, the way `tests/room_cut_reference.py` is
for 3.118.
"""

from __future__ import annotations

import heapq
from typing import Optional

import numpy as np

SNAP_M = 0.3


def _floor_near(seg: np.ndarray, c: tuple, r: int) -> Optional[tuple]:
    """`c` if it is floor in the cut, else the nearest floor cell within `r`
    cells -- a seed is kept in metres, and a SLAM correction or a later cut
    can turn the cell it stood on into a wall or unknown; None if none."""
    h, w = seg.shape
    j0, i0 = c
    if not (0 <= j0 < h and 0 <= i0 < w):
        # Off this map (a seed from a larger one: `cut.cell() or (-1, -1)`):
        # clamped, the search planted it on floor at the corner (3.76 L2).
        return None
    best = None
    for j in range(max(0, j0 - r), min(h, j0 + r + 1)):
        for i in range(max(0, i0 - r), min(w, i0 + r + 1)):
            if seg[j, i] >= 0:
                dd = (j - j0) ** 2 + (i - i0) ** 2
                if best is None or dd < best[0]:
                    best = (dd, j, i)
    return (best[1], best[2]) if best else None


def _walk(seg: np.ndarray, sources: list, res: float, door_m: float):
    """Multi-source walking distance over a cut's floor (4-connected), in
    metres, where every step from one of the cut's rooms into another costs
    `door_m` more; and which source reached each cell first. Unreached:
    inf and -1."""
    h, w = seg.shape
    d = np.full((h, w), np.inf)
    owner = np.full((h, w), -1, dtype=int)
    heap = []
    snap = max(1, int(round(SNAP_M / res)))
    for k, src in enumerate(sources):
        c = _floor_near(seg, src, snap)
        if c is not None and d[c] > 0:
            d[c], owner[c] = 0.0, k
            heap.append((0.0, c[0], c[1], k))
    heapq.heapify(heap)
    while heap:
        dist, j, i, k = heapq.heappop(heap)
        if dist > d[j, i]:
            continue
        here = seg[j, i]
        for nj, ni in ((j + 1, i), (j - 1, i), (j, i + 1), (j, i - 1)):
            if 0 <= nj < h and 0 <= ni < w and seg[nj, ni] >= 0:
                nd = dist + res + (door_m if seg[nj, ni] != here else 0.0)
                if nd < d[nj, ni]:
                    d[nj, ni], owner[nj, ni] = nd, k
                    heapq.heappush(heap, (nd, nj, ni, k))
    return d, owner
