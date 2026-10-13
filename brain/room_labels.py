"""
brain/room_labels.py -- 3.70 S2: rooms labelled during a survey.

A survey asks for a room's name ONCE per room, not once per frame (the
user's decision, 2026-10-11, conflict 6): on the car each ask is a paid
cloud call. So the map is first cut into rooms by its own shape, and a room
is asked about only when the robot stands well inside one that no label
covers yet.

**Rooms from the map's shape** (morphological segmentation, the method
`ipa_room_segmentation` calls "morphological"):

1. *Cores.* Free cells farther than `core_clear_m` from anything occupied.
   A doorway is narrower than twice that, so it has no core, and the cores
   of two rooms joined by a door are separate pieces.
2. *Growth.* Every core spreads over the free floor, breadth first and all
   at once, for at most `grow_m`; a cell goes to the core that reaches it
   first. A doorway is split between its two rooms.
3. *Leftovers.* Free floor no core reached -- a hall too narrow for a core
   -- is cut into its connected pieces, each a room of its own.

Unknown cells are neither: they do not close a core (a room half seen is
still a room) and nothing grows into them.

**Labels.** A label is a *seed*: a name and the house-frame point the robot
stood on when it was asked. Seeds are kept by position, never by room id,
because the map grows and every segmentation numbers its rooms afresh. A
room takes the name most of its seeds carry; a room with no seed is
unlabelled. `layer()` writes that as the map's `room_labels` block,
`{"names": [...], "cells": [index into names, or -1, per map cell]}` -- the
shape `brain/explore.py` reads (3.70 E3).

**When to ask.** Only from inside a room's core: a robot in a doorway may
be looking at the room it is leaving. One ask per room; an answer of
"unclear" is kept as a seed too (it labels nothing, and stops the room being
asked again). At `max_calls` asks the rest of the house stays unlabelled --
the survey goes on (the cap ends labelling, never the survey).

numpy only: the Jetson's install has no scipy.
"""

from __future__ import annotations

import heapq
import time
from collections import Counter, deque
from typing import Callable, Optional

import numpy as np

from world.interface import CELL_FREE, CELL_OCCUPIED

# Two basins of clearance are one room unless the narrowest point between
# them (the saddle) is clearly narrower than both: saddle < NECK_RATIO x the
# lower of the two peaks. A doorway's saddle is ~0.45 m against rooms' 1 m+
# peaks; a sofa splitting a room leaves a wide saddle.
NECK_RATIO = 0.75
# Pieces smaller than this are noise -- a gap behind a sofa -- not rooms.
MIN_ROOM_M2 = 0.5

UNCLEAR = "unclear"
# What a label source returns when it cannot answer for this frame:
# NO_SOURCE -- there is no source at all (a real camera frame carries no
# `room` and no cloud source is wired): nothing is counted, and the labeller
# stops asking for the rest of the mission, so it fetches no more frames.
# NOT_HERE -- this frame has nothing to name (a sim, teleop or replay frame
# whose `room` is "unknown"): nothing counted, asked again after RETRY_S.
NO_SOURCE = object()
NOT_HERE = object()
# The map's rooms are re-cut at most this often (on the labeller's clock):
# a cut is pure Python, ~0.2-0.9 s on a 200-400 cell square map, and a SLAM
# map changes -- size and origin too -- about once a second while the robot
# explores. Between cuts a place is looked up by POSITION in the last cut's
# own grid, so a grown map never forces a cut; floor the last cut has not
# seen is never asked about until the next.
RESEGMENT_S = 5.0
# After an ask that got no answer (NOT_HERE, or the source raised), wait
# this long before asking again; doubled after each failure in a row, up
# to RETRY_MAX_S. A cloud outage then spends a few calls, not the cap.
RETRY_S = 10.0
RETRY_MAX_S = 300.0


def frame_room(frame: dict):
    """The label source a simulated frame carries for free: its `room`. A
    frame with no `room` at all (a real camera) is NO_SOURCE -- on the car
    the cloud is the source, not wired yet (3.70 S2's car half); "unknown"
    (off the sim's rooms, or a teleop or replay frame) is NOT_HERE."""
    if "room" not in (frame or {}):
        return NO_SOURCE
    room = frame["room"]
    return NOT_HERE if room in (None, "", "unknown") else room


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


def _geodesic(free: np.ndarray, sources: list):
    """Multi-source breadth-first distance over free cells (4-connected), in
    cells, and which source reached each cell first. Unreached: -1."""
    h, w = free.shape
    d = np.full((h, w), -1, dtype=int)
    owner = np.full((h, w), -1, dtype=int)
    q = deque()
    for k, (j, i) in enumerate(sources):
        if 0 <= j < h and 0 <= i < w and d[j, i] < 0:
            d[j, i], owner[j, i] = 0, k
            q.append((j, i))
    while q:
        j, i = q.popleft()
        for nj, ni in ((j + 1, i), (j - 1, i), (j, i + 1), (j, i - 1)):
            if 0 <= nj < h and 0 <= ni < w and free[nj, ni] and d[nj, ni] < 0:
                d[nj, ni], owner[nj, ni] = d[j, i] + 1, owner[j, i]
                q.append((nj, ni))
    return d, owner


MODES = ("rooms", "spacing")
SPACING_M = 2.0


class _Cut:
    """One cut of the map, with the grid it was made on, so a later (grown,
    shifted) map's positions are looked up in it by metres, not by index."""

    def __init__(self, m: dict, seg, d, peaks, at: float):
        self.key = RoomLabeller._key(m)
        self.res, self.ox, self.oy = m["resolution_m"], m["origin_x_m"], m["origin_y_m"]
        self.seg, self.d, self.peaks, self.at = seg, d, peaks, at

    def cell(self, x: float, y: float) -> Optional[tuple]:
        j, i = int((y - self.oy) // self.res), int((x - self.ox) // self.res)
        h, w = self.seg.shape
        return (j, i) if 0 <= j < h and 0 <= i < w else None

    def room(self, x: float, y: float) -> int:
        c = self.cell(x, y)
        return int(self.seg[c]) if c else -1


class RoomLabeller:
    """Asks `label_fn(frame)` for a room's name, at most `max_calls` times,
    and `layer(m)` is the map's `room_labels` block. Two ways to decide when
    to ask, and how far an answer reaches:

    * `rooms` -- once per room of the map's shape (`segment()`), from inside
      it; the answer names that whole room.
    * `spacing` -- whenever the robot is more than `spacing_m` (walking
      distance on the map) from every earlier ask; each cell takes the
      answer of the ask nearest it by walking distance.

    Not thread-safe: the mission loop's own."""

    def __init__(self, label_fn: Callable[[dict], object],
                 max_calls: Optional[int] = None, mode: str = "rooms",
                 neck_ratio: float = NECK_RATIO, spacing_m: float = SPACING_M,
                 clock: Callable[[], float] = time.monotonic):
        if mode not in MODES:
            raise ValueError(f"Unknown room-label mode {mode!r}. Known: {', '.join(MODES)}")
        self.label_fn = label_fn
        self.max_calls = max_calls or None
        self.mode = mode
        self.neck_ratio = neck_ratio
        self.spacing_m = spacing_m
        self.clock = clock
        self.seeds: list = []          # [x_m, y_m, name]
        self.calls = 0
        self.capped = False            # the cap was reached: no more asks
        self.no_source = False         # NO_SOURCE seen: no more asks, no frames
        self._cut: Optional[_Cut] = None
        self._geo: Optional[_Cut] = None   # spacing: distances from the seeds (in .d)
        self._geo_seeds = -1
        self._retry_at: Optional[float] = None
        self._failures = 0

    @staticmethod
    def _key(m: dict) -> tuple:
        return (m.get("map_id"), m.get("map_version"), m["width"], m["height"],
                m.get("origin_x_m"), m.get("origin_y_m"))

    def _due(self, cached: Optional[_Cut], m: dict) -> bool:
        """A new cut is made for a new map at most every RESEGMENT_S -- and at
        once for another map altogether (another map_id)."""
        if cached is None or cached.key[0] != m.get("map_id"):
            return True
        return cached.key != self._key(m) and self.clock() - cached.at >= RESEGMENT_S

    def _segments(self, m: dict, fresh: bool = False) -> _Cut:
        if fresh or self._due(self._cut, m):
            seg, d = segment(m, self.neck_ratio)
            peaks = np.zeros(int(seg.max()) + 1 if seg.size and seg.max() >= 0 else 0)
            if len(peaks):
                np.maximum.at(peaks, seg[seg >= 0], d[seg >= 0])
            self._cut = _Cut(m, seg, d, peaks, self.clock())
        return self._cut

    @staticmethod
    def _free(m: dict) -> np.ndarray:
        return np.asarray(m["cells"]).reshape(m["height"], m["width"]) == CELL_FREE

    @staticmethod
    def _cell(m: dict, x: float, y: float) -> tuple:
        res = m["resolution_m"]
        return (int((y - m["origin_y_m"]) // res), int((x - m["origin_x_m"]) // res))

    def _wants(self, m: dict, x: float, y: float) -> bool:
        """Whether the robot standing at (x, y) is in space no ask covers --
        judged on the last cut; floor that cut has not seen is never asked
        about (wait for the next)."""
        if self.mode == "spacing":
            if not self.seeds:
                j, i = self._cell(m, x, y)
                free = self._free(m)
                return 0 <= j < free.shape[0] and 0 <= i < free.shape[1] and bool(free[j, i])
            if self._geo_seeds != len(self.seeds) or self._due(self._geo, m):
                free = self._free(m)
                d, _ = _geodesic(free, [self._cell(m, sx, sy) for sx, sy, _ in self.seeds])
                self._geo = _Cut(m, d, d, None, self.clock())
                self._geo_seeds = len(self.seeds)
            c = self._geo.cell(x, y)
            if c is None or self._geo.d[c] < 0:
                return False           # not reached in the last distances: wait
            return self._geo.d[c] * self._geo.res > self.spacing_m
        cut = self._segments(m)
        c = cut.cell(x, y)
        if c is None or cut.seg[c] < 0:
            return False
        room = int(cut.seg[c])
        if cut.d[c] < self.neck_ratio * cut.peaks[room]:
            return False               # a doorway or a corner: not inside yet
        return not any(cut.room(sx, sy) == room for sx, sy, _ in self.seeds)

    def observe(self, m: Optional[dict], pose: Optional[dict],
                look: Callable[[], Optional[dict]],
                alive: Callable[[], bool] = lambda: True) -> Optional[str]:
        """Called once a step with the map and the robot's pose. If the
        robot stands in space no ask covers, `look()` takes the frame to ask
        about -- now, at this pose; None means do not ask -- and the answer
        is returned; None if it did not ask. `alive()` is checked again
        after the frame: a stop that lands while it is read asks nobody.
        Errors from `look()` or the label source propagate (the caller
        skips the label, never the survey); a source error backs off."""
        if self.no_source:
            return None
        if self.max_calls and self.calls >= self.max_calls:
            self.capped = True
            return None
        if self._retry_at is not None and self.clock() < self._retry_at:
            return None
        if (not m or not m.get("usable", True) or "cells" not in m
                or not pose or not pose.get("usable")):
            return None
        x, y = pose["x_m"], pose["y_m"]
        if not self._wants(m, x, y):
            return None
        frame = look()
        if frame is None or not alive():
            return None
        self.calls += 1                # counted when SENT: a failed paid call may be billed
        try:
            name = self.label_fn(frame)
        except Exception:
            self._back_off()
            raise
        if name is NO_SOURCE:
            self.calls -= 1            # nobody was asked
            self.no_source = True
            return None
        if name is NOT_HERE:
            self.calls -= 1
            self._back_off()
            return None
        self._failures, self._retry_at = 0, None
        name = (name or UNCLEAR).strip().lower() or UNCLEAR
        self.seeds.append([round(x, 3), round(y, 3), name])
        if self.max_calls and self.calls >= self.max_calls:
            self.capped = True
        return name

    def _back_off(self) -> None:
        self._retry_at = self.clock() + min(RETRY_MAX_S, RETRY_S * 2 ** self._failures)
        self._failures += 1

    def layer(self, m: Optional[dict]) -> Optional[dict]:
        """The map's `room_labels` block, cut fresh from THIS map (so it
        lines up with the cells it is saved beside). None without a usable
        map."""
        if not m or not m.get("usable", True) or "cells" not in m:
            return None
        named = [(x, y, n) for x, y, n in self.seeds if n != UNCLEAR]
        if self.mode == "spacing":
            free = self._free(m)
            d, owner = _geodesic(free, [self._cell(m, x, y) for x, y, _ in named])
            names = sorted({n for _x, _y, n in named})
            index = {n: k for k, n in enumerate(names)}
            lut = np.array([index[n] for _x, _y, n in named] + [-1], dtype=int)
            cells = np.where(owner >= 0, lut[owner], -1)
            return {"names": names, "cells": cells.ravel().tolist()}
        cut = self._segments(m, fresh=True)
        seg = cut.seg
        votes: dict = {}
        for x, y, name in named:
            room = cut.room(x, y)
            if room >= 0:
                votes.setdefault(room, Counter())[name] += 1
        winners = {room: c.most_common(1)[0][0] for room, c in votes.items()}
        names = sorted(set(winners.values()))
        index = {name: k for k, name in enumerate(names)}
        lut = np.full(int(seg.max()) + 2 if seg.size else 1, -1, dtype=int)
        for room, name in winners.items():
            lut[room] = index[name]
        cells = np.where(seg >= 0, lut[seg], -1)
        return {"names": names, "cells": cells.ravel().tolist()}

    def status(self) -> dict:
        return {"mode": self.mode, "label_calls": self.calls,
                "label_call_limit": self.max_calls, "capped": self.capped,
                "no_source": self.no_source,
                "rooms_named": sorted({n for _x, _y, n in self.seeds if n != UNCLEAR}),
                "seeds": [list(s) for s in self.seeds]}
