"""Frozen copy of `brain/room_labels.py`'s `clearance()` and `segment()` as
they were before 3.118 (dev at 73a7725), verbatim. 3.118 rewrote both for
speed with exactly the old answers; `tests/test_room_cut.py` checks the new
code against this one. Do not edit or "simplify" it: it is the definition
of the right answer, the way `tests/frontier_reference.py` is for 3.77.
"""

from __future__ import annotations

import heapq
from collections import Counter

import numpy as np

from world.interface import CELL_FREE, CELL_OCCUPIED

NECK_RATIO = 0.75
MIN_ROOM_M2 = 0.5


_N8 = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
       (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142))


def clearance(free: np.ndarray, occ: np.ndarray) -> np.ndarray:
    """Each free cell's distance, in cells, to the nearest occupied cell or
    the map's edge (an octile approximation of the Euclidean distance);
    0 off the floor. Unknown cells are neither: distance passes through
    them, so a room half seen still has its middle."""
    h, w = free.shape
    d = np.full((h, w), np.inf)
    heap = []
    for j, i in zip(*np.nonzero(occ)):
        d[j, i] = 0.0
        heap.append((0.0, int(j), int(i)))
    for j in range(h):
        for i in (0, w - 1):
            if d[j, i] > 0.5:
                d[j, i] = 0.5
                heap.append((0.5, j, i))
    for i in range(w):
        for j in (0, h - 1):
            if d[j, i] > 0.5:
                d[j, i] = 0.5
                heap.append((0.5, j, i))
    heapq.heapify(heap)
    while heap:
        dist, j, i = heapq.heappop(heap)
        if dist > d[j, i]:
            continue
        for dj, di, c in _N8:
            nj, ni = j + dj, i + di
            if 0 <= nj < h and 0 <= ni < w and dist + c < d[nj, ni]:
                d[nj, ni] = dist + c
                heapq.heappush(heap, (dist + c, nj, ni))
    return np.where(free, d, 0.0)


def segment(m: dict, neck_ratio: float = NECK_RATIO, min_room_m2: float = MIN_ROOM_M2):
    """Cut a map's free floor into rooms: a watershed of clearance. Free
    cells are added widest first; a cell that joins two pieces merges them
    unless it is a neck -- its clearance under `neck_ratio` x the lower of
    the two pieces' peaks -- in which case it goes to the piece with the
    higher peak and the two stay apart. Returns `(seg, peak_d)`: `seg` an
    int array (height x width) of room ids, -1 off the floor; `peak_d` each
    cell's own clearance, so a caller can tell a room's middle from its
    doorway."""
    w, h, res = m["width"], m["height"], m["resolution_m"]
    cells = np.asarray(m["cells"]).reshape(h, w)
    free = cells == CELL_FREE
    occ = cells == CELL_OCCUPIED
    d = clearance(free, occ)
    parent: dict = {}
    peak: dict = {}
    size: dict = {}

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    label = np.full((h, w), -1, dtype=int)
    js, is_ = np.nonzero(free)
    order = np.argsort(-d[js, is_], kind="stable")
    nxt = 0
    for k in order:
        j, i = int(js[k]), int(is_[k])
        here = d[j, i]
        roots = set()
        for nj, ni in ((j + 1, i), (j - 1, i), (j, i + 1), (j, i - 1)):
            if 0 <= nj < h and 0 <= ni < w and label[nj, ni] >= 0:
                roots.add(find(label[nj, ni]))
        if not roots:
            parent[nxt], peak[nxt], size[nxt] = nxt, here, 0
            roots = {nxt}
            nxt += 1
        roots = sorted(roots, key=lambda r: -peak[r])
        keep = roots[0]
        for r in roots[1:]:
            if here >= neck_ratio * min(peak[keep], peak[r]):
                parent[r] = keep
                size[keep] += size[r]
        label[j, i] = keep
        size[keep] += 1
    seg = np.full((h, w), -1, dtype=int)
    if nxt:
        roots = np.array([find(a) for a in range(nxt)])
        seg[free] = roots[label[free]]
    # Tiny pieces join the neighbour they touch most.
    min_cells = max(1, int(round(min_room_m2 / (res * res))))
    ids, counts = np.unique(seg[seg >= 0], return_counts=True)
    for r, n in zip(ids, counts):
        if n >= min_cells:
            continue
        touch = Counter()
        for j, i in zip(*np.nonzero(seg == r)):
            for nj, ni in ((j + 1, i), (j - 1, i), (j, i + 1), (j, i - 1)):
                if 0 <= nj < h and 0 <= ni < w and seg[nj, ni] >= 0 and seg[nj, ni] != r:
                    touch[int(seg[nj, ni])] += 1
        if touch:
            seg[seg == r] = touch.most_common(1)[0][0]
    # Renumber 0..n-1.
    ids = np.unique(seg[seg >= 0])
    lut = np.full(int(seg.max()) + 2, -1, dtype=int)
    lut[ids] = np.arange(len(ids))
    seg = np.where(seg >= 0, lut[seg], -1)
    return seg, d
