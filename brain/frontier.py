"""
frontier.py -- where to look next on a map the robot is still drawing
(PLAN-ros-alignment.md 3.31). Pure functions over `WorldInterface.get_map()`'s
grid, plus the retry rule the explore policy and the stuck detector share.

**A frontier is seen floor next to unknown** -- the edge of what the robot
knows: a doorway into an unseen room, the end of a hallway, the far side of
a sofa. Frontiers always lie on the SEEN side, so nav2 can always be asked
to go to one; the lidar then sweeps past it and the map grows.

**Goals are chosen on KNOWN floor with room for the chassis.** A goal is a
free cell at least `clearance_m` from anything occupied, reached from the
robot through such cells -- a cheap stand-in for nav2's own inflated
costmap, so a frontier nav2 would certainly refuse is not offered to it. It
is deliberately not nav2's planner: when the two disagree, nav2 aborts, and
the retry rule below is what handles it.

**The retry rule** (`RetryBook`) is the same for a frontier nav2 could not
reach and for a mission the safety layer keeps refusing: set it aside for a
cooldown, let the world change, try again, and give up only after `limit`
failures at different times. A person standing in a doorway makes a room
unreachable for a minute; a wall does it for ever. Only time tells them
apart, so the rule spends time, not a verdict.
"""

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Optional

import numpy as np

from world.interface import CELL_FREE, CELL_OCCUPIED, CELL_UNKNOWN

# 3.31's numbers, confirmed by the user before building.
RETRY_COOLDOWN_S = 30.0
RETRY_LIMIT = 3
# A failure at one place covers its neighbourhood: nav2 aborting on a
# doorway says nothing about a frontier 3 m away, and everything about one
# 20 cm along the same edge.
RETRY_RADIUS_M = 0.6
# A frontier shorter than the robot is wide is a crack, not a way in.
MIN_FRONTIER_M = 0.25
# Room for the chassis around a goal: its half-diagonal (0.171 m, the
# turning circle) plus a margin, so nav2's inflated costmap (0.12 m) does
# not reject the goal outright.
GOAL_CLEARANCE_M = 0.22
# Room to PASS through on the way: the chassis' half-width (0.1155 m) plus a
# margin. Lower than a goal's, because a doorway is somewhere to go through,
# not to stop in -- and nav2, not this, decides whether it really can. With
# one clearance for both, the furnished home's 0.6 m den door (narrower
# still on SLAM's thick walls) read as a wall and the den was never mapped.
PASS_CLEARANCE_M = 0.15
# Larger frontiers are worth a longer drive: a whole unseen room against a
# sliver behind a chair.
SIZE_WEIGHT = 1.0
# A frontier longer than this is cut into pieces. On a real SLAM map the
# whole edge of what the lidar reached is ONE connected frontier, tens of
# metres long (27.7 m on the scaled house's first scan), and its middle
# lands on the robot -- the first live run sent 50 goals and travelled 0 m.
MAX_FRONTIER_M = 1.5
# A goal this close is no goal: the lidar already covers it.
MIN_GOAL_M = 0.4


@dataclass
class Frontier:
    cells: list                 # (col, row) frontier cells
    goal: tuple                 # (x_m, y_m) a reachable free point near it
    distance_m: float           # path length from the robot to `goal`
    size_m: float               # the frontier's length

    @property
    def score(self) -> float:
        return self.distance_m - SIZE_WEIGHT * self.size_m


def _cell_of(m: dict, x_m: float, y_m: float) -> tuple:
    res = m["resolution_m"]
    return (int(math.floor((x_m - m["origin_x_m"]) / res)),
            int(math.floor((y_m - m["origin_y_m"]) / res)))


def _centre(m: dict, c: tuple) -> tuple:
    res = m["resolution_m"]
    return (m["origin_x_m"] + (c[0] + 0.5) * res, m["origin_y_m"] + (c[1] + 0.5) * res)


# 3.77: the grid passes run in numpy and give exactly what the pure-Python
# versions they replaced gave (`tests/frontier_reference.py` keeps those, and
# `tests/test_frontier_fast.py` compares the two on live SLAM maps). Where an
# order or a tie decides an answer, the old order is kept on purpose.

# Neighbours in the order every loop here has always visited them: dx outer,
# dy inner. A cell's BFS parent, and so its path length, depends on it.
_OFFSETS = tuple((dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if dx or dy)
_DIAG = math.sqrt(2)


def _grid(m: dict) -> np.ndarray:
    # fromiter: twice as fast as asarray from a list on the Jetson (2.5 vs 5.2 ms).
    n = m["height"] * m["width"]
    return np.fromiter(m["cells"], dtype=np.int8, count=n).reshape(m["height"], m["width"])


def _clearance_grid(cells: np.ndarray, radius_cells: int) -> np.ndarray:
    """Chebyshev distance (in cells) from every cell to the nearest occupied
    one, capped at `radius_cells`: grow the occupied set one ring at a time.
    Chebyshev never exceeds Euclidean, so "at least r" here is at least r."""
    occ = cells == CELL_OCCUPIED
    dist = np.full(cells.shape, radius_cells, dtype=np.int32)
    dist[occ] = 0
    cur = occ
    for d in range(1, radius_cells):
        grown = cur.copy()                      # one ring: rows, then columns
        grown[1:] |= cur[:-1]
        grown[:-1] |= cur[1:]
        tall = grown.copy()
        grown[:, 1:] |= tall[:, :-1]
        grown[:, :-1] |= tall[:, 1:]
        dist[grown & ~cur] = d
        cur = grown
    return dist


class _Occupied:
    """Box counts of occupied cells: `clear(k)` is `_clearance_grid(...) >= k`
    (nothing occupied within Chebyshev distance k - 1), and `at(x, y, cap)`
    one cell's value, without growing the occupied set ring by ring."""

    def __init__(self, cells: np.ndarray):
        self.occ = (cells == CELL_OCCUPIED)[None]
        self.h, self.w = cells.shape
        s = np.zeros((self.h + 1, self.w + 1), dtype=np.int64)
        s[1:, 1:] = self.occ[0].cumsum(0).cumsum(1)
        self.s = s

    def clear(self, k: int) -> np.ndarray:
        if k <= 0:
            return np.ones((self.h, self.w), dtype=bool)
        return ~_box_any_stack(self.occ, k - 1)[0]

    def at(self, x: int, y: int, cap: int) -> int:
        s, h, w = self.s, self.h, self.w
        for r in range(cap):
            y0, y1, x0, x1 = max(0, y - r), min(h, y + r + 1), max(0, x - r), min(w, x + r + 1)
            if s[y1, x1] - s[y0, x1] - s[y1, x0] + s[y0, x0]:
                return r
        return cap


def _clearance(m: dict, radius_cells: int) -> list:
    """`_clearance_grid` over a map, as a flat row-major list."""
    return _clearance_grid(_grid(m), radius_cells).ravel().tolist()


def frontier_cells(m: dict, grid: Optional[np.ndarray] = None) -> set:
    """Free cells with an unknown 4-neighbour."""
    cells = _grid(m) if grid is None else grid
    unknown = np.pad(cells == CELL_UNKNOWN, 1)
    edge = (cells == CELL_FREE) & (unknown[1:-1, 2:] | unknown[1:-1, :-2]
                                   | unknown[2:, 1:-1] | unknown[:-2, 1:-1])
    ys, xs = np.nonzero(edge)
    # Inserted in row-major order, as the loop this replaced did: the set's
    # iteration order, and so `_clusters`' order, depends on it.
    return set(zip(xs.tolist(), ys.tolist()))


def _clusters(cells: set, seed: Optional[tuple] = None) -> list:
    """8-connected groups, in exactly the order the set-popping version
    found them: a cluster is seeded by its first cell in the set's own
    iteration order (what `set.pop()` returns, since its finger only moves
    forward), and grown depth-first in `_OFFSETS` order. Flat indices on a
    grid padded by one cell stand in for tuples, so no bounds checks."""
    left = set(cells)
    if not left:
        return []
    order = list(left)
    if seed is not None:        # one group, grown from `seed`
        order.remove(seed)
        order.insert(0, seed)
    w = max(x for x, _ in order) + 3
    h = max(y for _, y in order) + 3
    lo_x = min(x for x, _ in order)
    lo_y = min(y for _, y in order)
    w, h = w - lo_x, h - lo_y
    member = bytearray(w * h)
    flat = [(y - lo_y + 1) * w + (x - lo_x + 1) for x, y in order]
    for p in flat:
        member[p] = 1
    offs = [dy * w + dx for dx in (-1, 0, 1) for dy in (-1, 0, 1)]
    groups = []
    for p in flat:
        if not member[p]:
            continue
        member[p] = 0
        group, stack = [p], [p]
        while stack:
            c = stack.pop()
            for o in offs:
                n = c + o
                if member[n]:
                    member[n] = 0
                    group.append(n)
                    stack.append(n)
        groups.append([(n % w - 1 + lo_x, n // w - 1 + lo_y) for n in group])
    return groups


class Reach:
    """Where the robot can stop, and how far each place is along the BFS
    tree from it: `order` (flat cell indices, in BFS order) and `dist` (path
    length in cells). `steps` is the same as the dict `reachable` returns,
    built on first use. Compute it once per decision and pass it to
    `find_frontiers` and `find_view_gaps` (3.77)."""

    def __init__(self, w: int, h: int, start: Optional[tuple], need: int,
                 order=None, dist=None, grid=None):
        self.w, self.h, self.start, self.need = w, h, start, need
        self.grid = grid                # the map's cells as an (h, w) array
        self.order = order if order is not None else np.zeros(0, dtype=np.int64)
        self.dist = dist if dist is not None else np.zeros(0)
        self._steps = None
        self._dense = None

    def __len__(self) -> int:
        return int(self.order.size)

    @property
    def steps(self) -> dict:
        if self._steps is None:
            w = self.w
            self._steps = {(i % w, i // w): d
                           for i, d in zip(self.order.tolist(), self.dist.tolist())}
        return self._steps

    @property
    def dense(self) -> np.ndarray:
        """Path length per cell (h x w), NaN where the robot cannot stop."""
        if self._dense is None:
            d = np.full(self.w * self.h, np.nan)
            d[self.order] = self.dist
            self._dense = d.reshape(self.h, self.w)
        return self._dense


def reach(m: dict, x_m: float, y_m: float,
          clearance_m: float = GOAL_CLEARANCE_M,
          pass_m: float = PASS_CLEARANCE_M) -> Reach:
    """`reachable`'s answer as a `Reach`. The BFS runs one level at a time:
    a level's neighbours are generated in queue order (parent, then
    `_OFFSETS`), and the first occurrence of each new cell wins -- which is
    exactly the parent the one-cell-at-a-time queue gave it, and the next
    level's queue order. Path lengths are therefore the same BFS-tree
    lengths (not shortest paths), to the bit. (Measured on the Jetson, this
    beats a pure-Python queue, 30 vs 49 ms, and a sort-free variant keeping
    each cell's least-ranked parent, 38 ms.)"""
    if not m.get("usable") or not m.get("resolution_m"):
        return Reach(0, 0, None, 1)
    res, w, h = m["resolution_m"], m["width"], m["height"]
    need = max(1, int(math.ceil(clearance_m / res)))
    cells = _grid(m)
    occupied = _Occupied(cells)
    goal_need = need
    need = max(1, min(need, int(math.ceil(pass_m / res))))
    start = _cell_of(m, x_m, y_m)
    if not (0 <= start[0] < w and 0 <= start[1] < h):
        return Reach(w, h, None, need)
    # A robot parked closer to a wall than `clearance_m` (normal: its
    # half-width is 11.5 cm) must still be able to leave, so cells as clear
    # as the one it stands in count too -- never anything nearer an
    # obstacle than where it already is.
    floor = max(1, min(need, occupied.at(start[0], start[1], goal_need + 1)))
    pw = w + 2                      # padded: the border is never safe
    safe = np.zeros((h + 2, pw), dtype=bool)
    safe[1:-1, 1:-1] = (cells == CELL_FREE) & occupied.clear(floor)
    open_ = safe.ravel()                # safe and not yet reached
    offs = np.array([dy * pw + dx for dx, dy in _OFFSETS], dtype=np.int64)
    cost = np.array([_DIAG if dx and dy else 1.0 for dx, dy in _OFFSETS])
    s = (start[1] + 1) * pw + start[0] + 1
    open_[s] = False
    level, ld = np.array([s], dtype=np.int64), np.zeros(1)
    orders, dists = [level], [ld]
    while True:
        cand = (level[:, None] + offs).ravel()
        cd = (ld[:, None] + cost).ravel()
        ok = open_[cand]
        cand, cd = cand[ok], cd[ok]
        if not cand.size:
            break
        _, first = np.unique(cand, return_index=True)
        first.sort()
        level, ld = cand[first], cd[first]
        open_[level] = False
        orders.append(level)
        dists.append(ld)
    p = np.concatenate(orders)
    d = np.concatenate(dists)
    flat = (p // pw - 1) * w + (p % pw - 1)
    stop = occupied.clear(goal_need).ravel()[flat]
    return Reach(w, h, start, goal_need, flat[stop], d[stop], grid=cells)


def _reach_for(m: dict, x_m: float, y_m: float, clearance_m: float,
               given: Optional[Reach]) -> Reach:
    """`given` if it was built for this pose and clearance, else a fresh one.
    A Reach from another pose or clearance would silently change the answer,
    so it is refused rather than used."""
    if given is None:
        return reach(m, x_m, y_m, clearance_m)
    if m.get("usable") and m.get("resolution_m"):
        need = max(1, int(math.ceil(clearance_m / m["resolution_m"])))
        start = _cell_of(m, x_m, y_m)
        if given.start is not None and (given.start != start or given.need != need
                                        or (given.w, given.h) != (m["width"], m["height"])):
            raise ValueError("reach_map was built for another pose, clearance or map")
    return given


def reachable(m: dict, x_m: float, y_m: float,
              clearance_m: float = GOAL_CLEARANCE_M,
              pass_m: float = PASS_CLEARANCE_M) -> tuple:
    """(steps, start, need): path length in cells from the robot to every
    free cell it can get to, walking through cells with `pass_m` of room;
    only cells with `clearance_m` of room -- somewhere to STOP -- are kept.
    Empty on an unusable map or a robot off it."""
    r = reach(m, x_m, y_m, clearance_m, pass_m)
    return r.steps, r.start, r.need


def _split(group: list, max_cells: int) -> list:
    """Cut a long frontier into pieces of at most `max_cells`, in the order
    a walk along it visits them (a BFS from one end), so each piece is a
    stretch of the edge rather than a scatter across it."""
    if len(group) <= max_cells:
        return [group]
    start = min(group, key=lambda c: (c[0] + c[1], c))
    # Flat integers stand in for (x, y), offset so a neighbour never wraps.
    lo_x = min(c[0] for c in group) - 1
    lo_y = min(c[1] for c in group) - 1
    w = max(c[0] for c in group) - lo_x + 2
    cells = {(x - lo_x) + (y - lo_y) * w for x, y in group}
    offs = [dx + dy * w for dx in (-1, 0, 1) for dy in (-1, 0, 1)]
    s = (start[0] - lo_x) + (start[1] - lo_y) * w
    order, seen, q = [], {s}, deque([s])
    while q:
        c = q.popleft()
        order.append(c)
        for o in offs:
            n = c + o
            if n in cells and n not in seen:
                seen.add(n)
                q.append(n)
    order = [(c % w + lo_x, c // w + lo_y) for c in order]
    return [order[i:i + max_cells] for i in range(0, len(order), max_cells)]


def _box_any_stack(mark: np.ndarray, r: int) -> np.ndarray:
    """For a stack of masks (P, H, W): True where some set cell lies within
    Chebyshev distance `r` in the same mask (box sums, by slicing)."""
    def along(a, axis):
        n = a.shape[axis]
        shape = list(a.shape)
        shape[axis] = n + 2 * r + 1
        # int16: a running sum along a long row may wrap, but every box count
        # is a difference of two of them and small, so it comes out exact.
        c = np.zeros(shape, dtype=np.int16)
        body = [slice(None)] * a.ndim
        body[axis] = slice(r + 1, r + 1 + n)
        c[tuple(body)] = a
        c = c.cumsum(axis, dtype=np.int16)
        hi, lo = [slice(None)] * a.ndim, [slice(None)] * a.ndim
        hi[axis], lo[axis] = slice(2 * r + 1, 2 * r + 1 + n), slice(0, n)
        return c[tuple(hi)] - c[tuple(lo)]
    return along(along(mark, 2), 1) > 0


# Pieces are boxed in batches of similar size, so a long piece does not
# make every box in the stack long.
_BOX_STEP = 32


def _goals_for(pieces: list, middles: list, dense: np.ndarray, eligible: np.ndarray,
               reach_cells: int) -> list:
    """For each frontier piece, the reachable cell nearest its middle (then
    nearest the robot) within `reach_cells` of one of its cells, or None --
    the first such cell in the old search's order (frontier cell, dx, dy)
    when two tie. Pieces of similar size share one stack of boxes; numpy
    shortlists, Python decides."""
    if not pieces:
        return []
    r = reach_cells
    lens = np.array([len(g) for g in pieces])
    total = int(lens.sum())
    gx = np.fromiter((c[0] for g in pieces for c in g), dtype=np.int64, count=total)
    gy = np.fromiter((c[1] for g in pieces for c in g), dtype=np.int64, count=total)
    starts = np.concatenate([[0], np.cumsum(lens)[:-1]])
    lo_x, lo_y = np.minimum.reduceat(gx, starts), np.minimum.reduceat(gy, starts)
    span_x = np.maximum.reduceat(gx, starts) - lo_x
    span_y = np.maximum.reduceat(gy, starts) - lo_y
    bucket = (span_x // _BOX_STEP) * 1000 + span_y // _BOX_STEP
    mx = np.array([p[0] for p in middles])
    my = np.array([p[1] for p in middles])
    # Eligible cells with a margin wider than any box, so a box is a slice.
    pad = int(max(span_x.max(), span_y.max())) + 2 * r + 2
    h, w = eligible.shape
    padded = np.zeros((h + 2 * pad, w + 2 * pad), dtype=bool)
    padded[pad:pad + h, pad:pad + w] = eligible
    found = {}
    for b in np.unique(bucket).tolist():
        ks = np.flatnonzero(bucket == b)
        cell = np.concatenate([np.arange(starts[k], starts[k] + lens[k]) for k in ks.tolist()])
        _shortlist(ks, np.repeat(np.arange(ks.size), lens[ks]), gx[cell], gy[cell],
                   lo_x[ks] - r, lo_y[ks] - r, int(span_x[ks].max()), int(span_y[ks].max()),
                   mx[ks], my[ks], padded, pad, r, found)
    out = []
    for k, group in enumerate(pieces):
        if k not in found:
            out.append(None)
            continue
        px, py = middles[k]
        keys = [((math.hypot(n[0] - px, n[1] - py), float(dense[n[1], n[0]])), n)
                for n in found[k]]
        best = min(key for key, _ in keys)
        tied = [n for key, n in keys if key == best]
        if len(tied) > 1:
            def first_visit(n, group=group):
                for i, (fx, fy) in enumerate(group):
                    dx, dy = n[0] - fx, n[1] - fy
                    if abs(dx) <= r and abs(dy) <= r:
                        return (i, dx, dy)
            tied.sort(key=first_visit)
        out.append(tied[0])
    return out


def _shortlist(ids, pid, gx, gy, x0, y0, span_x: int, span_y: int, mx, my,
               padded: np.ndarray, pad: int, r: int, found: dict) -> None:
    """`_goals_for`'s numpy half for one batch of pieces (`ids`, their cells
    `gx`/`gy` tagged by batch position `pid`, boxes from `x0`/`y0`; `padded`
    the eligible cells with a `pad` margin): into `found[id]`, every
    eligible cell near the piece whose distance to its middle may round to
    the least."""
    bx, by = span_x + 2 * r + 1, span_y + 2 * r + 1
    mark = np.zeros((ids.size, by, bx), dtype=bool)
    mark[pid, gy - y0[pid], gx - x0[pid]] = True
    el = np.stack([padded[y + pad:y + pad + by, x + pad:x + pad + bx]
                   for x, y in zip(x0.tolist(), y0.tolist())])
    ok = _box_any_stack(mark, r) & el
    dx2 = (x0[:, None] + np.arange(bx) - mx[:, None]) ** 2
    dy2 = (y0[:, None] + np.arange(by) - my[:, None]) ** 2
    sq = np.where(ok, dx2[:, None, :] + dy2[:, :, None], np.inf)
    least = sq.min(axis=(1, 2))
    short = ok & (sq <= least[:, None, None] * (1 + 1e-9) + 1e-12)
    for k, yy, xx in zip(*(a.tolist() for a in np.nonzero(short))):
        found.setdefault(int(ids[k]), []).append((int(x0[k]) + xx, int(y0[k]) + yy))


def find_frontiers(m: dict, x_m: float, y_m: float,
                   clearance_m: float = GOAL_CLEARANCE_M,
                   min_size_m: float = MIN_FRONTIER_M,
                   reach_map: Optional[Reach] = None) -> list:
    """Every reachable frontier on `m`, best first.

    Reachable means: some free cell with `clearance_m` of room lies within
    two clearances of the frontier and is connected to the robot through
    such cells. The goal is the reachable cell nearest the frontier's
    middle. An unusable map has no frontiers. `reach_map`, if given, is
    `reach(m, x_m, y_m, clearance_m)` already computed."""
    r = _reach_for(m, x_m, y_m, clearance_m, reach_map)
    if r.start is None:
        return []
    res, need = m["resolution_m"], r.need
    out = []
    pieces = []
    for group in _clusters(frontier_cells(m, r.grid)):
        if len(group) * res < min_size_m:
            continue
        pieces.extend(_split(group, max(1, int(MAX_FRONTIER_M / res))))
    dense = r.dense
    with np.errstate(invalid="ignore"):
        eligible = dense * res >= MIN_GOAL_M          # NaN (not a stop) -> False
    middles = [(sum(c[0] for c in g) / len(g), sum(c[1] for c in g) / len(g)) for g in pieces]
    for group, goal_cell in zip(pieces, _goals_for(pieces, middles, dense, eligible, 2 * need)):
        if goal_cell is None:
            continue
        size_m = len(group) * res
        out.append(Frontier(cells=group, goal=_centre(m, goal_cell),
                            distance_m=float(dense[goal_cell[1], goal_cell[0]]) * res,
                            size_m=size_m))
    out.sort(key=lambda f: f.score)
    return out


def known_near(m: dict, x_m: float, y_m: float, radius_m: float) -> int:
    """How many cells within `radius_m` of a point are no longer unknown."""
    if not m.get("usable") or not m.get("resolution_m"):
        return 0
    res, w, h = m["resolution_m"], m["width"], m["height"]
    cx, cy = _cell_of(m, x_m, y_m)
    r = int(math.ceil(radius_m / res))
    n = 0
    for y in range(max(0, cy - r), min(h, cy + r + 1)):
        for x in range(max(0, cx - r), min(w, cx + r + 1)):
            if m["cells"][y * w + x] != CELL_UNKNOWN:
                n += 1
    return n


@dataclass
class _Entry:
    x_m: float
    y_m: float
    failures: int
    last_t: float
    known_at_failure: int = 0


@dataclass
class RetryBook:
    """Places (or, with one entry at the origin, a whole mission) that failed,
    and when they may be tried again.

    A place is **cooling** for `cooldown_s` after a failure, **dropped**
    after `limit` failures, and comes back early when the map near it has
    grown by `early_growth` since it failed -- the world changed there, so
    the old verdict is stale."""

    cooldown_s: float = RETRY_COOLDOWN_S
    limit: int = RETRY_LIMIT
    radius_m: float = RETRY_RADIUS_M
    early_growth: float = 0.25
    entries: list = field(default_factory=list)

    def _near(self, x_m: float, y_m: float) -> Optional[_Entry]:
        for e in self.entries:
            if math.hypot(e.x_m - x_m, e.y_m - y_m) <= self.radius_m:
                return e
        return None

    def fail(self, x_m: float, y_m: float, now: float, known=0) -> int:
        """Record a failure; returns how many this place has now. `known` is
        the known-cell count around the place, or a callable giving it at a
        point -- measured at the ENTRY's own position, the one `available()`
        later compares against (3.76)."""
        e = self._near(x_m, y_m)
        if e is None:
            e = _Entry(x_m, y_m, 0, now)
            self.entries.append(e)
        e.failures += 1
        e.last_t = now
        e.known_at_failure = known(e.x_m, e.y_m) if callable(known) else known
        return e.failures

    def dropped(self, x_m: float, y_m: float) -> bool:
        e = self._near(x_m, y_m)
        return e is not None and e.failures >= self.limit

    def available(self, x_m: float, y_m: float, now: float,
                  known_now: Optional[Callable[[float, float], int]] = None) -> bool:
        """May this place be tried now?"""
        e = self._near(x_m, y_m)
        if e is None:
            return True
        if e.failures >= self.limit:
            return False
        if now - e.last_t >= self.cooldown_s:
            return True
        if known_now is not None and e.known_at_failure:
            grown = known_now(e.x_m, e.y_m) - e.known_at_failure
            return grown >= self.early_growth * e.known_at_failure
        return False

    def failures_near(self, x_m: float, y_m: float) -> int:
        """How often a goal near this place has failed (0 if never)."""
        e = self._near(x_m, y_m)
        return e.failures if e is not None else 0

    def cooling(self, now: float) -> bool:
        """Is anything still inside its cooldown? A place whose cooldown has
        passed is no longer waited on: if it is still a frontier it is
        offered again, and if the map has moved on it is simply gone. (The
        first version counted every not-yet-dropped place, so a frontier that
        vanished kept a mission "retrying in 0 s" for ever.)"""
        return any(e.failures < self.limit and now - e.last_t < self.cooldown_s
                   for e in self.entries)

    def next_ready_in(self, now: float) -> float:
        waits = [self.cooldown_s - (now - e.last_t) for e in self.entries
                 if e.failures < self.limit and now - e.last_t < self.cooldown_s]
        return max(0.0, min(waits)) if waits else 0.0


# ---------- what the CAMERA has seen (3.31) ----------
#
# The lidar sees 360 degrees to 12 m; the camera sees 60 degrees and finds
# a backpack only within a few metres. So the map runs out of frontiers
# long before the camera has looked at every floor it covers -- the first
# in-process run reached the kitchen, ran out of frontiers and ended
# `searched` without ever pointing the camera at the backpack. A search
# needs its own coverage: floor the camera has had a clear look at.

CAMERA_FOV_DEG = 60.0
# How far the camera can be trusted to find a target. The sim's detections
# reach 4.2 m; a real detector's useful range on a floor-height view is
# about the same. Kept below both.
CAMERA_RANGE_M = 3.0
# An unseen patch smaller than this is a corner, not somewhere to go.
MIN_GAP_M2 = 0.09
# Places to look from tried per patch, best-ranked first, for a line of
# sight to it (3.69 D).
LOOK_FROM_CANDIDATES = 40


def _ray_cells(m: dict, pairs: list, fov_deg: float = CAMERA_FOV_DEG,
               range_m: float = CAMERA_RANGE_M) -> tuple:
    """(col, row) arrays shaped (pairs, rays, steps): the cells the camera's
    rays walk from each (x_m, y_m, view_deg) in `pairs`, all at once -- the
    same rays, the same half-cell steps and the same float arithmetic as the
    one-ray-at-a-time loop this replaced, so the same cells."""
    res = m["resolution_m"]
    n_rays = max(2, int(fov_deg / 1.5))
    ds, d = [], 0.0
    while d <= range_m:
        ds.append(d)
        d += res * 0.5
    ds = np.array(ds)
    # view - fov/2 + fov*i/n, evaluated left to right as before.
    spread = [fov_deg * i / n_rays for i in range(n_rays + 1)]
    rad, sin, cos = math.radians, math.sin, math.cos
    angles = [rad(view_deg - fov_deg / 2 + f) for _x, _y, view_deg in pairs for f in spread]
    shape = (len(pairs), n_rays + 1, 1)
    xs = np.array([p[0] for p in pairs])[:, None, None]
    ys = np.array([p[1] for p in pairs])[:, None, None]
    px = xs + np.array([sin(a) for a in angles]).reshape(shape) * ds
    py = ys + np.array([-cos(a) for a in angles]).reshape(shape) * ds
    return (np.floor((px - m["origin_x_m"]) / res).astype(np.int64),
            np.floor((py - m["origin_y_m"]) / res).astype(np.int64))


class _Sight:
    """The map's free cells and patch labels with a border wider than the
    camera's range, so a ray's cells need no bounds checks: a border cell is
    not free, which stops a ray exactly where leaving the map did."""

    def __init__(self, m: dict, grid: np.ndarray, lab: np.ndarray):
        self.m = m
        self.pad = int(math.ceil(CAMERA_RANGE_M / m["resolution_m"])) + 2
        p, (h, w) = self.pad, grid.shape
        self.stride = w + 2 * p
        free = np.zeros((h + 2 * p, self.stride), dtype=bool)
        free[p:p + h, p:p + w] = grid == CELL_FREE
        labels = np.full((h + 2 * p, self.stride), -1, dtype=np.int64)
        labels[p:p + h, p:p + w] = lab.reshape(h, w)
        self.free, self.lab = free.ravel(), labels.ravel()

    def walk(self, pairs: list) -> tuple:
        """(padded flat cell, kept) per ray step; `kept` is False from each
        ray's first cell off the map or not free."""
        cx, cy = _ray_cells(self.m, pairs)
        flat = (cy + self.pad) * self.stride + (cx + self.pad)
        return flat, np.logical_and.accumulate(self.free[flat], axis=2)

    def sees(self, c: int, pairs: list) -> list:
        """For each (x_m, y_m, view_deg): does the camera see a cell of
        patch `c` -- `camera_seen(...)` meeting the patch, as before?"""
        if not pairs:
            return []
        flat, kept = self.walk(pairs)
        return ((self.lab[flat] == c) & kept).any(axis=(1, 2)).tolist()


def camera_seen(m: dict, x_m: float, y_m: float, view_deg: float,
                fov_deg: float = CAMERA_FOV_DEG,
                range_m: float = CAMERA_RANGE_M) -> set:
    """Free cells the camera has a clear line to from this pose: rays across
    the field of view, each stopped by anything occupied OR unknown (a
    camera cannot vouch for floor the map has not confirmed). `view_deg` is
    the compass bearing the camera points along (heading + pan)."""
    if not m.get("usable") or not m.get("resolution_m"):
        return set()
    w, h = m["width"], m["height"]
    cx, cy = _ray_cells(m, [(x_m, y_m, view_deg)], fov_deg, range_m)
    inside = (cx >= 0) & (cx < w) & (cy >= 0) & (cy < h)
    flat = np.where(inside, cy * w + cx, 0)
    free = inside & (_grid(m).ravel()[flat] == CELL_FREE)
    kept = np.logical_and.accumulate(free, axis=2)
    return {(i % w, i // w) for i in np.unique(flat[kept]).tolist()}


def _components(mask: np.ndarray) -> tuple:
    """8-connected components of `mask` as (label grid, count): runs along
    each row, joined to overlapping or diagonal-touching runs on the next
    row by union-find. Labels are arbitrary; `find_view_gaps` restores the
    order it needs."""
    h, w = mask.shape
    edges = np.diff(np.pad(mask, ((0, 0), (1, 1))).astype(np.int8), axis=1)
    sy, sx = np.nonzero(edges == 1)
    _ey, ex = np.nonzero(edges == -1)            # exclusive ends, same order
    n = sx.size
    parent = list(range(n))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    rows = sy.tolist()
    xs0, xs1 = sx.tolist(), ex.tolist()
    first = {}
    for i, y in enumerate(rows):
        first.setdefault(y, i)
    for y, i0 in first.items():
        j = first.get(y + 1)
        if j is None:
            continue
        i = i0
        while i < n and rows[i] == y and j < n and rows[j] == y + 1:
            if xs0[i] <= xs1[j] and xs0[j] <= xs1[i]:     # 8-connected
                a, b = find(i), find(j)
                if a != b:
                    parent[max(a, b)] = min(a, b)
            if xs1[i] < xs1[j]:
                i += 1
            else:
                j += 1
    roots = [find(i) for i in range(n)]
    _, run_label = np.unique(np.array(roots, dtype=np.int64), return_inverse=True)
    labels = np.full(h * w, -1, dtype=np.int64)
    lengths = ex - sx
    if n:
        starts = sy * w + sx
        offs = np.arange(lengths.sum()) - np.repeat(np.cumsum(lengths) - lengths, lengths)
        labels[np.repeat(starts, lengths) + offs] = np.repeat(run_label, lengths)
    return labels.reshape(h, w), (int(run_label.max()) + 1 if n else 0)


@dataclass
class ViewGap:
    centre: tuple              # (x_m, y_m) the unseen patch's middle
    goal: tuple                # where to stand to look at it
    distance_m: float
    area_m2: float
    face: Optional[tuple] = None   # (x_m, y_m) the part of it seen from `goal`


def find_view_gaps(m: dict, x_m: float, y_m: float, seen: set,
                   clearance_m: float = GOAL_CLEARANCE_M,
                   min_area_m2: float = MIN_GAP_M2,
                   look_from_m: float = 1.0,
                   reach_map: Optional[Reach] = None) -> list:
    """Patches of known floor the camera has not seen, nearest first, each
    with a reachable place to look at it from: the reachable cell nearest
    `look_from_m` short of the patch's middle. `reach_map`, if given, is
    `reach(m, x_m, y_m, clearance_m)` already computed."""
    r = _reach_for(m, x_m, y_m, clearance_m, reach_map)
    if r.start is None:
        return []
    res, w, h = m["resolution_m"], m["width"], m["height"]
    unseen_mask = r.grid == CELL_FREE
    for sx, sy in seen:
        if 0 <= sx < w and 0 <= sy < h:
            unseen_mask[sy, sx] = False
    labels, count = _components(unseen_mask)
    lab = labels.ravel()
    cells_at = np.flatnonzero(lab >= 0)
    owner = lab[cells_at]
    size = np.bincount(owner, minlength=count)
    sum_x = np.bincount(owner, weights=cells_at % w, minlength=count)
    sum_y = np.bincount(owner, weights=cells_at // w, minlength=count)
    by_owner = np.argsort(owner, kind="stable")
    ends = np.cumsum(size)
    stop_x = (r.order % w).astype(float)             # stops, in BFS order
    stop_y = (r.order // w).astype(float)
    # The pure-Python version found patches by popping a set; its order
    # only matters for ties, so it is rebuilt only when one happens.
    old = _OldOrder(unseen_mask, labels)
    sight = _Sight(m, r.grid, labels)
    out = []
    for c in range(count):
        n_cells = int(size[c])
        area = n_cells * res * res
        if area < min_area_m2 or not len(r):
            continue
        mx = int(sum_x[c]) / n_cells                  # exact: integer sums
        my = int(sum_y[c]) / n_cells
        want = look_from_m / res
        ranked = _ranked(r, stop_x, stop_y, mx, my, want, LOOK_FROM_CANDIDATES)
        # 3.69 D: the place to look from must SEE the patch. Nearest the
        # middle by straight line alone stood the robot behind a wall from
        # the patch (houses 27, 108), the patch was marked looked at, and
        # the target in it was never in a frame. So: the best-ranked place
        # with a clear line to some of it -- facing its middle, as before,
        # or else its nearest part; the old choice only if none has.
        cx, cy = _centre(m, (int(mx), int(my)))
        mine = cells_at[by_owner[ends[c] - n_cells:ends[c]]]
        best, face = _look_from(m, sight, c, ranked, (cx, cy), mine % w, mine // w, old)
        out.append((c, ViewGap(centre=(cx, cy), goal=_centre(m, best),
                               distance_m=float(r.dense[best[1], best[0]]) * res,
                               area_m2=area, face=face)))
    keys = [g.distance_m - g.area_m2 for _, g in out]
    if len(set(keys)) < len(keys):      # a tie: the stable sort needs the old order
        rank = old.patch_rank()
        out.sort(key=lambda cg: rank[cg[0]])
    out.sort(key=lambda cg: cg[1].distance_m - cg[1].area_m2)
    return [g for _, g in out]


# Candidates tried per batch of camera rays: the first alone (it usually
# sees the patch), then the next few, then the rest.
_LOOK_BATCHES = (1, 8, LOOK_FROM_CANDIDATES)


def _look_from(m: dict, sight: _Sight, c: int, ranked: list,
               middle: tuple, gx: np.ndarray, gy: np.ndarray, old) -> tuple:
    """(goal cell, face) for patch `c`: the first of `ranked` with a camera
    line to the patch's middle (face None), or else to the patch cell
    nearest it (face that cell) -- `ranked[0]` and None if none has. As the
    one-candidate-at-a-time loop decided it, with the rays batched."""
    cx, cy = middle
    lo = 0
    for hi in _LOOK_BATCHES:
        chunk = ranked[lo:hi]
        if not chunk:
            break
        lo = hi
        spots = [_centre(m, n) for n in chunk]
        mid_hit = sight.sees(c, [
            (nx, ny, math.degrees(math.atan2(cx - nx, -(cy - ny)))) for nx, ny in spots])
        # The patch cells nearest each candidate (integer squared distances:
        # `min(group, key=hypot)` took the first of the least, in the old
        # group order -- needed only if a tie could change the answer).
        tied, pairs = [], []
        for n, (nx, ny), hit in zip(chunk, spots, mid_hit):
            if hit:
                tied.append(())
                continue
            sq = (gx - n[0]) ** 2 + (gy - n[1]) ** 2
            least = np.flatnonzero(sq == sq.min()).tolist()
            ts = [(int(gx[i]), int(gy[i])) for i in least]
            tied.append(ts)
            for t in ts:
                qx, qy = _centre(m, t)
                pairs.append((nx, ny, math.degrees(math.atan2(qx - nx, -(qy - ny)))))
        q_hit = iter(sight.sees(c, pairs))
        for n, hit, ts in zip(chunk, mid_hit, tied):
            if hit:
                return n, None
            hits = {t: next(q_hit) for t in ts}
            if not any(hits.values()):
                continue
            q = ts[0] if len(ts) == 1 else next(t for t in old.group(c) if t in hits)
            if hits[q]:
                return n, _centre(m, q)
    return ranked[0], None


class _OldOrder:
    """The order the set-popping `_clusters` gave patches and their cells,
    rebuilt on demand from the label grid (only ties need it)."""

    def __init__(self, mask: np.ndarray, labels: np.ndarray):
        self.mask, self.labels = mask, labels
        self._order = None
        self._groups = {}

    def _pop_order(self) -> list:
        if self._order is None:
            ys, xs = np.nonzero(self.mask)
            unseen = set(zip(xs.tolist(), ys.tolist()))   # as the old code built it
            self._order = list(set(unseen))                 # `_clusters`' own copy
        return self._order

    def patch_rank(self) -> dict:
        rank = {}
        for x, y in self._pop_order():
            rank.setdefault(int(self.labels[y, x]), len(rank))
        return rank

    def group(self, c: int) -> list:
        if c not in self._groups:
            seed = next(t for t in self._pop_order() if self.labels[t[1], t[0]] == c)
            mine = np.argwhere(self.labels == c)
            self._groups[c] = _clusters({(int(x), int(y)) for y, x in mine.tolist()}
                                        | {seed}, seed=seed)[0]
        return self._groups[c]


def _ranked(r: Reach, xs: np.ndarray, ys: np.ndarray, mx: float, my: float,
            want: float, k: int) -> list:
    """The first `k` stops of `sorted(steps, key=|hypot(n - middle) - want|)`
    -- a stable sort over BFS order -- without sorting all of them: numpy
    shortlists every stop that could be among them, Python ranks those."""
    key = np.abs(np.hypot(xs - mx, ys - my) - want)
    if key.size > k:
        cut = np.partition(key, k - 1)[k - 1]
        idx = np.flatnonzero(key <= cut + 1e-9 * max(1.0, cut))
    else:
        idx = np.arange(key.size)
    w = r.w
    cand = [(int(i % w), int(i // w)) for i in r.order[idx].tolist()]
    cand.sort(key=lambda n: abs(math.hypot(n[0] - mx, n[1] - my) - want))
    return cand[:k]
