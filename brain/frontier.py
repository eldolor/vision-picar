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


def _clearance(m: dict, radius_cells: int) -> list:
    """Chebyshev distance (in cells) from every cell to the nearest occupied
    one, capped at `radius_cells` -- a multi-source BFS, so O(cells).
    Chebyshev never exceeds Euclidean, so "at least r" here is at least r."""
    w, h, cells = m["width"], m["height"], m["cells"]
    far = radius_cells
    dist = [far] * (w * h)
    q = deque()
    for idx, v in enumerate(cells):
        if v == CELL_OCCUPIED:
            dist[idx] = 0
            q.append(idx)
    while q:
        idx = q.popleft()
        d = dist[idx] + 1
        if d >= far:
            continue
        x, y = idx % w, idx // w
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < w and 0 <= ny < h:
                    n = ny * w + nx
                    if dist[n] > d:
                        dist[n] = d
                        q.append(n)
    return dist


def frontier_cells(m: dict) -> set:
    """Free cells with an unknown 4-neighbour."""
    w, h, cells = m["width"], m["height"], m["cells"]
    out = set()
    for idx, v in enumerate(cells):
        if v != CELL_FREE:
            continue
        x, y = idx % w, idx // w
        for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if 0 <= nx < w and 0 <= ny < h and cells[ny * w + nx] == CELL_UNKNOWN:
                out.add((x, y))
                break
    return out


def _clusters(cells: set) -> list:
    """8-connected groups."""
    left, groups = set(cells), []
    while left:
        seed = left.pop()
        group, stack = [seed], [seed]
        while stack:
            x, y = stack.pop()
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    n = (x + dx, y + dy)
                    if n in left:
                        left.remove(n)
                        group.append(n)
                        stack.append(n)
        groups.append(group)
    return groups


def reachable(m: dict, x_m: float, y_m: float,
              clearance_m: float = GOAL_CLEARANCE_M) -> tuple:
    """(steps, start, need): path length in cells from the robot to every
    free cell with room for the chassis, over such cells. Empty on an
    unusable map or a robot off it."""
    if not m.get("usable") or not m.get("resolution_m"):
        return {}, None, 1
    res, w, h = m["resolution_m"], m["width"], m["height"]
    need = max(1, int(math.ceil(clearance_m / res)))
    room = _clearance(m, need + 1)
    start = _cell_of(m, x_m, y_m)
    if not (0 <= start[0] < w and 0 <= start[1] < h):
        return {}, None, need
    # A robot parked closer to a wall than `clearance_m` (normal: its
    # half-width is 11.5 cm) must still be able to leave, so cells as clear
    # as the one it stands in count too -- never anything nearer an
    # obstacle than where it already is.
    floor = max(1, min(need, room[start[1] * w + start[0]]))
    safe = [m["cells"][i] == CELL_FREE and room[i] >= floor for i in range(w * h)]

    # Path length from the robot over safe cells.
    steps = {start: 0.0}
    q = deque([start])
    diag = math.sqrt(2)
    while q:
        c = q.popleft()
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == dy == 0:
                    continue
                n = (c[0] + dx, c[1] + dy)
                if not (0 <= n[0] < w and 0 <= n[1] < h) or n in steps:
                    continue
                if not safe[n[1] * w + n[0]]:
                    continue
                steps[n] = steps[c] + (diag if dx and dy else 1.0)
                q.append(n)
    return steps, start, need


def _split(group: list, max_cells: int) -> list:
    """Cut a long frontier into pieces of at most `max_cells`, in the order
    a walk along it visits them (a BFS from one end), so each piece is a
    stretch of the edge rather than a scatter across it."""
    if len(group) <= max_cells:
        return [group]
    cells = set(group)
    start = min(group, key=lambda c: (c[0] + c[1], c))
    order, seen, q = [], {start}, deque([start])
    while q:
        c = q.popleft()
        order.append(c)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                n = (c[0] + dx, c[1] + dy)
                if n in cells and n not in seen:
                    seen.add(n)
                    q.append(n)
    return [order[i:i + max_cells] for i in range(0, len(order), max_cells)]


def find_frontiers(m: dict, x_m: float, y_m: float,
                   clearance_m: float = GOAL_CLEARANCE_M,
                   min_size_m: float = MIN_FRONTIER_M) -> list:
    """Every reachable frontier on `m`, best first.

    Reachable means: some free cell with `clearance_m` of room lies within
    two clearances of the frontier and is connected to the robot through
    such cells. The goal is the reachable cell nearest the frontier's
    middle. An unusable map has no frontiers."""
    steps, start, need = reachable(m, x_m, y_m, clearance_m)
    if start is None:
        return []
    res = m["resolution_m"]
    out = []
    reach = 2 * need
    pieces = []
    for group in _clusters(frontier_cells(m)):
        if len(group) * res < min_size_m:
            continue
        pieces.extend(_split(group, max(1, int(MAX_FRONTIER_M / res))))
    for group in pieces:
        size_m = len(group) * res
        mx = sum(c[0] for c in group) / len(group)
        my = sum(c[1] for c in group) / len(group)
        best = None
        for fx, fy in group:
            for dx in range(-reach, reach + 1):
                for dy in range(-reach, reach + 1):
                    n = (fx + dx, fy + dy)
                    if n in steps and steps[n] * res >= MIN_GOAL_M:
                        key = (math.hypot(n[0] - mx, n[1] - my), steps[n])
                        if best is None or key < best[0]:
                            best = (key, n)
        if best is None:
            continue
        goal_cell = best[1]
        out.append(Frontier(cells=group, goal=_centre(m, goal_cell),
                            distance_m=steps[goal_cell] * res, size_m=size_m))
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

    def fail(self, x_m: float, y_m: float, now: float, known: int = 0) -> int:
        """Record a failure; returns how many this place has now."""
        e = self._near(x_m, y_m)
        if e is None:
            e = _Entry(x_m, y_m, 0, now)
            self.entries.append(e)
        e.failures += 1
        e.last_t = now
        e.known_at_failure = known
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

    def cooling(self, now: float) -> bool:
        """Is anything waiting out a cooldown -- i.e. not yet given up on?"""
        return any(e.failures < self.limit for e in self.entries)

    def next_ready_in(self, now: float) -> float:
        waits = [self.cooldown_s - (now - e.last_t) for e in self.entries
                 if e.failures < self.limit]
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


def camera_seen(m: dict, x_m: float, y_m: float, view_deg: float,
                fov_deg: float = CAMERA_FOV_DEG,
                range_m: float = CAMERA_RANGE_M) -> set:
    """Free cells the camera has a clear line to from this pose: rays across
    the field of view, each stopped by anything occupied OR unknown (a
    camera cannot vouch for floor the map has not confirmed). `view_deg` is
    the compass bearing the camera points along (heading + pan)."""
    if not m.get("usable") or not m.get("resolution_m"):
        return set()
    res, w, h, cells = m["resolution_m"], m["width"], m["height"], m["cells"]
    out = set()
    n_rays = max(2, int(fov_deg / 1.5))
    for i in range(n_rays + 1):
        a = math.radians(view_deg - fov_deg / 2 + fov_deg * i / n_rays)
        dx, dy = math.sin(a), -math.cos(a)
        d = 0.0
        while d <= range_m:
            c = _cell_of(m, x_m + dx * d, y_m + dy * d)
            if not (0 <= c[0] < w and 0 <= c[1] < h):
                break
            v = cells[c[1] * w + c[0]]
            if v != CELL_FREE:
                break
            out.add(c)
            d += res * 0.5
    return out


@dataclass
class ViewGap:
    centre: tuple              # (x_m, y_m) the unseen patch's middle
    goal: tuple                # where to stand to look at it
    distance_m: float
    area_m2: float


def find_view_gaps(m: dict, x_m: float, y_m: float, seen: set,
                   clearance_m: float = GOAL_CLEARANCE_M,
                   min_area_m2: float = MIN_GAP_M2,
                   look_from_m: float = 1.0) -> list:
    """Patches of known floor the camera has not seen, nearest first, each
    with a reachable place to look at it from: the reachable cell nearest
    `look_from_m` short of the patch's middle."""
    steps, start, need = reachable(m, x_m, y_m, clearance_m)
    if start is None:
        return []
    res, w = m["resolution_m"], m["width"]
    unseen = {(i % w, i // w) for i, v in enumerate(m["cells"])
              if v == CELL_FREE and (i % w, i // w) not in seen}
    out = []
    for group in _clusters(unseen):
        area = len(group) * res * res
        if area < min_area_m2:
            continue
        mx = sum(c[0] for c in group) / len(group)
        my = sum(c[1] for c in group) / len(group)
        want = look_from_m / res
        best = None
        for n, d in steps.items():
            gap_d = math.hypot(n[0] - mx, n[1] - my)
            key = abs(gap_d - want)
            if best is None or key < best[0]:
                best = (key, n)
        if best is None:
            continue
        cx, cy = _centre(m, (int(mx), int(my)))
        out.append(ViewGap(centre=(cx, cy), goal=_centre(m, best[1]),
                           distance_m=steps[best[1]] * res, area_m2=area))
    out.sort(key=lambda g: g.distance_m - g.area_m2)
    return out
