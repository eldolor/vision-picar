"""
brain/room_labels.py -- 3.70 S2: rooms labelled during a survey.

A survey asks for a room's name from time to time, not once per frame: on
the car each ask is a paid cloud call. Where an answer reaches, and when to
ask again, come from the map.

**Rooms from the map's shape** (`segment()`): a watershed of clearance.
Free cells are added widest first; two pieces stay apart where the cell
joining them is a neck -- narrower than `NECK_RATIO` of the lower piece's
widest point, as a doorway is -- and merge otherwise. Unknown cells pass
distance but are not floor, so a room half seen still has its middle. Open
plans (a kitchen open to a family room) have no neck, so the cut alone
cannot separate them.

**Two ways of asking** (`mode`):

* `spacing` (the default, 3.70 S2 amendment 1) -- ask whenever the robot
  is more than `SPACING_M` from every earlier ask, by walking distance over
  the cut where every step from one of its rooms into another costs
  `DOOR_M` more. So a robot that has come through a doorway is asked again,
  an open plan is asked about every 2 m, and each cell takes the answer of
  the ask nearest it by that same distance: labels stop at doorways that
  have an ask on the other side, reach at most one doorway on (`REACH_M`)
  where none does, and between asks in an open plan meet halfway. An
  `unclear` answer holds its space and names nothing.
* `rooms` -- once per room of the cut, from inside it (wider than
  `NECK_RATIO` of the room's widest point: a robot in a doorway may be
  looking at the room it is leaving); the answer names the whole room.
  Fewer calls, worse labels: an open plan is one room.

**Labels.** A label is a *seed*: a name and the house-frame point the robot
stood on when it was asked. Seeds are kept by position, never by room id,
because the map grows and every cut numbers its rooms afresh. `layer()`
writes them as the map's `room_labels` block, `{"names": [...], "cells":
[index into names, or -1, per map cell]}` -- the shape `brain/explore.py`
reads (3.70 E3). An answer of "unclear" is kept as a seed (it labels
nothing, and the place is not asked about again). At `max_calls` asks the
rest of the house stays unlabelled -- the survey goes on (the cap ends
labelling, never the survey).

numpy only: the Jetson's install has no scipy.
"""

from __future__ import annotations

import heapq
import time
from collections import Counter
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
# BUSY -- the source is still answering an earlier ask (a paid call it gave
# up waiting for): nothing sent, nothing counted, the back-off untouched.
BUSY = object()


class SourceRefused(RuntimeError):
    """The label source refused us for good -- a wrong secret, a route not
    deployed, a reply that is not the API's: our fault, not an outage. The
    labeller stops asking (no back-off loop spending calls all survey) and
    names the reason on its status."""


# Replies that name no room. One list for every source, so a reply the
# service passes through cannot become a room called "none" (3.76 L2).
_NO_ROOM = ("", "unclear", "unknown", "none", "n/a")
_ARTICLES = ("the ", "a ", "an ")


def clean_room_name(name) -> str:
    """One spelling per room: lowercase, single spaces, no leading article
    ("The kitchen" and "kitchen" are one room, 3.76 L2); a reply naming no
    room is UNCLEAR."""
    name = " ".join(name.split()).lower() if isinstance(name, str) else ""
    marks = " *_`\"'.,:;!?"
    name = name.strip(marks)
    for article in _ARTICLES:
        if name.startswith(article):
            name = name[len(article):].strip(marks)
            break
    return UNCLEAR if name in _NO_ROOM or name + " " in _ARTICLES else name
# The map's rooms are re-cut at most this often (on the labeller's clock):
# a cut of a mapped house at 0.05 m takes 0.18-0.37 s on the Jetson at 15 W
# since 3.118 (1.45-2.77 s before it), and a SLAM
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
    them, so a room half seen still has its middle.

    A relaxation from the sources (occupied cells at 0, edge cells at 0.5)
    that revisits only the cells that changed in the last pass. Each cell
    ends at the least left-to-right float sum over every path to it -- the
    value the Dijkstra this replaced (3.118) gave, bit for bit, since adding
    a positive step is monotone in floats; `tests/test_room_cut.py` checks
    it against the frozen copy."""
    h, w = free.shape
    if h == w == 0:
        return np.zeros((0, 0))        # as the old one (0 x n raised, and still does)
    W = w + 2
    d = np.full((h + 2, W), np.inf)
    inside = np.zeros((h + 2, W), dtype=bool)
    inside[1:-1, 1:-1] = True
    core = d[1:-1, 1:-1]
    core[occ] = 0.0
    edge = np.zeros((h, w), dtype=bool)
    edge[[0, -1], :] = edge[:, [0, -1]] = True
    core[edge & (core > 0.5)] = 0.5
    d, inside = d.ravel(), inside.ravel()
    steps = [(dj * W + di, c) for dj, di, c in _N8]
    front = np.flatnonzero(np.isfinite(d))
    while front.size:
        changed = []
        for off, c in steps:
            t = front + off
            cand = d[front] + c
            better = inside[t] & (cand < d[t])
            if better.any():
                t = t[better]
                d[t] = cand[better]
                changed.append(t)
        if not changed:
            break
        hit = np.zeros(d.size, dtype=bool)
        for t in changed:
            hit[t] = True
        front = np.flatnonzero(hit)
    return np.where(free, d.reshape(h + 2, W)[1:-1, 1:-1], 0.0)


def segment(m: dict, neck_ratio: float = NECK_RATIO, min_room_m2: float = MIN_ROOM_M2):
    """Cut a map's free floor into rooms: a watershed of clearance. Free
    cells are added widest first; a cell that joins two pieces merges them
    unless it is a neck -- its clearance under `neck_ratio` x the lower of
    the two pieces' peaks -- in which case it goes to the piece with the
    higher peak and the two stay apart. Returns `(seg, peak_d)`: `seg` an
    int array (height x width) of room ids, -1 off the floor; `peak_d` each
    cell's own clearance, so a caller can tell a room's middle from its
    doorway.

    3.118 made it fast with exactly the old answers (the frozen copy is
    `tests/room_cut_reference.py`): the watershed runs over flat lists on a
    map padded by one cell, in the same order with the same rules."""
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

    # The padded map's flat index; its border stays -1, so a neighbour off
    # the map reads as unlabelled, as the bounds check did.
    W = w + 2
    label = [-1] * ((h + 2) * W)
    js, is_ = np.nonzero(free)
    order = np.argsort(-d[js, is_], kind="stable")
    flat = ((js + 1) * W + (is_ + 1))[order].tolist()
    heights = d[js, is_][order].tolist()
    nxt = 0
    for p, here in zip(flat, heights):
        roots = set()
        for q in (p + W, p - W, p + 1, p - 1):
            lq = label[q]
            if lq >= 0:
                roots.add(find(lq))
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
        label[p] = keep
        size[keep] += 1
    label = np.array(label, dtype=int).reshape(h + 2, W)[1:-1, 1:-1]
    seg = np.full((h, w), -1, dtype=int)
    if nxt:
        roots = np.array([find(a) for a in range(nxt)])
        seg[free] = roots[label[free]]
    # Tiny pieces join the neighbour they touch most; a tie goes to the one
    # touched first, cells row by row and each cell's neighbours in the order
    # below (as `Counter.most_common` broke it in the loop this replaced).
    min_cells = max(1, int(round(min_room_m2 / (res * res))))
    ids, counts = np.unique(seg[seg >= 0], return_counts=True)
    pad = np.full((h + 2, W), -1, dtype=int)
    for r, n in zip(ids, counts):
        if n >= min_cells:
            continue
        pad[1:-1, 1:-1] = seg
        rj, ri = np.nonzero(seg == r)
        rj, ri = rj + 1, ri + 1
        near = np.stack([pad[rj + 1, ri], pad[rj - 1, ri],
                         pad[rj, ri + 1], pad[rj, ri - 1]], axis=1).ravel()
        near = near[(near >= 0) & (near != r)]
        if near.size:
            names, first, hits = np.unique(near, return_index=True, return_counts=True)
            best = np.flatnonzero(hits == hits.max())
            seg[seg == r] = names[best[np.argmin(first[best])]]
    # Renumber 0..n-1.
    ids = np.unique(seg[seg >= 0])
    lut = np.full(int(seg.max()) + 2, -1, dtype=int)
    lut[ids] = np.arange(len(ids))
    seg = np.where(seg >= 0, lut[seg], -1)
    return seg, d


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


MODES = ("rooms", "spacing")
SPACING_M = 2.0
# `spacing`: walking from one of the cut's rooms into another counts this
# much extra -- more than SPACING_M, so a robot that has come through a
# doorway is asked again, and an answer reaches past a doorway only when
# nothing was asked on the other side.
DOOR_M = 3.0
# A seed whose own cell is no longer floor (a SLAM correction, a later cut)
# starts from the nearest floor cell within this distance.
SNAP_M = 0.3
# `spacing`: an answer labels floor up to this far by that same distance --
# its own room and at most one doorway on (the cut has more doorways than
# the house: stopping at the first one left 46 -> 38 of the 9 surveys' 90
# rooms under 95%; one on, as many as no limit). A room two doorways from
# any answer stays unlabelled.
REACH_M = 2 * DOOR_M


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
    to ask, and how far an answer reaches (the module's docstring):

    * `spacing` -- whenever the robot is more than `spacing_m` from every
      earlier ask, walking over the cut with `door_m` added per doorway;
      each cell takes the answer of the ask nearest it by that distance.
    * `rooms` -- once per room of the cut (`segment()`), from inside it; the
      answer names that whole room.

    Not thread-safe: the mission loop's own."""

    def __init__(self, label_fn: Callable[[dict], object],
                 max_calls: Optional[int] = None, mode: str = "spacing",
                 neck_ratio: float = NECK_RATIO, spacing_m: float = SPACING_M,
                 door_m: float = DOOR_M,
                 clock: Callable[[], float] = time.monotonic):
        if mode not in MODES:
            raise ValueError(f"Unknown room-label mode {mode!r}. Known: {', '.join(MODES)}")
        self.label_fn = label_fn
        self.max_calls = max_calls or None
        self.mode = mode
        self.neck_ratio = neck_ratio
        self.spacing_m = spacing_m
        self.door_m = door_m
        self.reach_m = REACH_M
        self.clock = clock
        self.seeds: list = []          # [x_m, y_m, name]
        self.calls = 0
        self.capped = False            # the cap was reached: no more asks
        self.no_source = False         # NO_SOURCE seen: no more asks, no frames
        self.source_error: Optional[str] = None   # why a SourceRefused stopped it
        self.skipped_for_time = 0      # asks due but skipped: `fits()` said no
        self._cut: Optional[_Cut] = None
        self._geo = None               # spacing: (cut, seeds counted, distances)
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

    def _wants(self, m: dict, x: float, y: float) -> bool:
        """Whether the robot standing at (x, y) is in space no ask covers --
        judged on the last cut; floor that cut has not seen is never asked
        about (wait for the next)."""
        cut = self._segments(m)
        if self.mode == "spacing":
            c = cut.cell(x, y)
            if c is None or cut.seg[c] < 0:
                return False           # not floor in the last cut: wait
            if not self.seeds:
                return True
            if self._geo is None or self._geo[0] is not cut or self._geo[1] != len(self.seeds):
                d, _ = _walk(cut.seg, [cut.cell(sx, sy) or (-1, -1) for sx, sy, _ in self.seeds],
                             cut.res, self.door_m)
                self._geo = (cut, len(self.seeds), d)
            return bool(self._geo[2][c] > self.spacing_m)
        c = cut.cell(x, y)
        if c is None or cut.seg[c] < 0:
            return False
        room = int(cut.seg[c])
        if cut.d[c] < self.neck_ratio * cut.peaks[room]:
            return False               # a doorway or a corner: not inside yet
        return not any(cut.room(sx, sy) == room for sx, sy, _ in self.seeds)

    def observe(self, m: Optional[dict], pose: Optional[dict],
                look: Callable[[], Optional[dict]],
                alive: Callable[[], bool] = lambda: True,
                fits: Callable[[], bool] = lambda: True) -> Optional[str]:
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
        # An ask is due. `fits()` -- the caller's: does a paid call still end
        # inside its loop's deadline? -- before the frame and again after
        # it; a skip is counted (`skipped_for_time`), a later step asks.
        if not fits():
            self.skipped_for_time += 1
            return None
        frame = look()
        if frame is None or not alive():
            return None
        if not fits():
            self.skipped_for_time += 1
            return None
        self.calls += 1                # counted when SENT: a failed paid call may be billed
        try:
            name = self.label_fn(frame)
        except SourceRefused as e:
            self.no_source = True      # the call stays counted: it was sent
            self.source_error = str(e)[:200]
            return None
        except Exception:
            self._back_off()
            raise
        if name is NO_SOURCE:
            self.calls -= 1            # nobody was asked
            self.no_source = True
            return None
        if name is BUSY:
            self.calls -= 1
            return None
        if name is NOT_HERE:
            self.calls -= 1
            self._back_off()
            return None
        self._failures, self._retry_at = 0, None
        name = clean_room_name(name)
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
        cut = self._segments(m, fresh=True)
        if self.mode == "spacing":
            # Every seed walks -- an `unclear` one holds its space and names
            # nothing -- and an answer reaches REACH_M at most: a cell it
            # reaches only through two doorways stays unlabelled.
            d, owner = _walk(cut.seg, [cut.cell(x, y) or (-1, -1) for x, y, _ in self.seeds],
                             cut.res, self.door_m)
            names = sorted({n for _x, _y, n in named})
            index = {n: k for k, n in enumerate(names)}
            lut = np.array([index.get(n, -1) for _x, _y, n in self.seeds] + [-1], dtype=int)
            cells = np.where((owner >= 0) & (d < self.reach_m), lut[owner], -1)
            return {"names": names, "cells": cells.ravel().tolist()}
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
                "source_error": self.source_error,
                "skipped_for_time": self.skipped_for_time,
                "rooms_named": sorted({n for _x, _y, n in self.seeds if n != UNCLEAR}),
                "seeds": [list(s) for s in self.seeds]}
