"""
evaluations/goal-learn-3101/rich.py -- 3.101: the 25 features F-rich adds to
3.96's 17 (`RICH_FEATURES`, appended in this order).

Every one comes from what the brain has at the goal choice: its map, the
camera-seen set, its own trail, the candidate pool and the clock. Nothing
reads the simulator's ground truth. Each is scaled to about 0-1 and
clipped, as 3.96's are, so the same network shape can take them.
"""
import math

import numpy as np
from scipy.ndimage import distance_transform_edt

from brain.frontier import _cell_of
from brain.room_labels import segment
from world.interface import CELL_FREE, CELL_OCCUPIED, CELL_UNKNOWN

import os as _os, sys as _sys  # noqa: E401
_sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "..", "goal-learn-396"))
from features import _seg_at, coarse  # noqa: E402  (3.96's: the room cut runs on the 0.30 m grid)

RICH_FEATURES = (
    # route
    "detour",          # path / straight line, (ratio - 1) / 4
    "path_rank",       # rank of the path in the pool, 0 nearest .. 1 farthest
    "path_gap",        # path minus the pool's shortest, / 10 m
    "nearest_other",   # distance to the nearest other candidate, / 5 m
    "cluster_2m",      # share of the pool within 2 m
    "wall_clear",      # goal's distance to the nearest occupied cell, / 1 m
    # information gain
    "lidar_gain",      # unknown cells visible from the goal within RAY_M (ray samples' share)
    "cam_gain",        # known floor the camera has not seen, visible within RAY_M
    "unknown_2m",      # unknown share of the disc within 2 m
    "unknown_4m",      # unknown share of the disc within 4 m
    "gain_per_m",      # lidar_gain / (1 + path / 5 m)
    # room graph (segment() of the brain's map)
    "room_hops",       # hops from the robot's room, / 4 (1 when unconnected)
    "room_dead_end",   # the candidate's room has exactly one neighbour
    "room_degree",     # its number of neighbours, / 4
    "room_seen",       # share of its floor the camera has seen
    # this search
    "trail_min",       # the trail's closest approach to the goal, / 5 m
    "trail_near",      # share of the trail's cells within 1.5 m
    "since_new_ground",  # goals sent since the trail last covered NEW_GROUND_M2 of new floor, / 10
    "since_growth",    # seconds since the map last grew 1%, / 120 s
    # budget
    "time_used",       # sim seconds since the mission started, / ELAPSED_S
    "pool_size",       # candidates in the pool, / 50
    "known_share",     # known share of the map's box
    # relative
    "rule_pick",       # the rule's own pick (the pool's first)
    "gain_rank",       # rank of lidar_gain in the pool, 0 best .. 1 worst
    "dominated",       # share of the pool shorter AND with more gain
)
RAY_M, RAYS = 3.0, 16
# The sweeps' missions run tens of sim seconds, not the car's 1200 s limit,
# so elapsed time is scaled to 120 s (the sim clock starts at 0 each mission).
ELAPSED_S = 120.0
NEW_GROUND_M2 = 3.0

_ANG = np.linspace(0.0, 2 * math.pi, RAYS, endpoint=False)
_DIRS = np.stack([np.cos(_ANG), np.sin(_ANG)], axis=1)


def _rank(v: np.ndarray, descending: bool = False) -> np.ndarray:
    n = len(v)
    if n < 2:
        return np.zeros(n)
    order = np.argsort(-v if descending else v, kind="stable")
    r = np.empty(n)
    r[order] = np.arange(n)
    return r / (n - 1)


def _disc_share(mask: np.ndarray, cx: np.ndarray, cy: np.ndarray, r: int,
                off: bool = False) -> np.ndarray:
    """Share of `mask` cells inside a disc of `r` cells round each centre;
    a cell off the map counts as `off` (unknown counts it as unknown, as
    3.96's crop does)."""
    h, w = mask.shape
    oy, ox = np.mgrid[-r:r + 1, -r:r + 1]
    keep = ox ** 2 + oy ** 2 <= r * r
    ox, oy = ox[keep], oy[keep]
    X = cx[:, None] + ox[None, :]
    Y = cy[:, None] + oy[None, :]
    ok = (X >= 0) & (X < w) & (Y >= 0) & (Y < h)
    v = np.where(ok, mask[np.clip(Y, 0, h - 1), np.clip(X, 0, w - 1)], off)
    return v.mean(axis=1)


def _mask_of(m: dict, pts) -> np.ndarray:
    """A boolean grid of `m`'s shape, True under each (x_m, y_m) point."""
    w, h, res = m["width"], m["height"], m["resolution_m"]
    out = np.zeros((h, w), dtype=bool)
    P = np.asarray(list(pts), dtype=float).reshape(-1, 2)
    if len(P):
        i = np.floor((P[:, 0] - m["origin_x_m"]) / res).astype(int)
        j = np.floor((P[:, 1] - m["origin_y_m"]) / res).astype(int)
        ok = (i >= 0) & (i < w) & (j >= 0) & (j < h)
        out[j[ok], i[ok]] = True
    return out


class RichState:
    """What the features remember across one mission's choices."""

    def __init__(self):
        self.ground = 0
        self.goal_at_new_ground = 0
        self.known = None
        self.t_growth = None


def rich_features(agent, m: dict, pose: dict, pool: list, feats: list, trail: list,
                  now: float) -> list:
    st = getattr(agent, "_rich3101", None)
    if st is None:
        st = agent._rich3101 = RichState()
    n = len(pool)
    w, h, res = m["width"], m["height"], m["resolution_m"]
    cells = np.asarray(m["cells"]).reshape(h, w)
    unknown = cells == CELL_UNKNOWN
    occ = cells == CELL_OCCUPIED
    free = cells == CELL_FREE
    seen = _mask_of(m, agent.seen)

    goals = np.asarray([c.goal for _, c, _ in pool], dtype=float)
    path = np.asarray([f[0] * 10.0 for f in feats])
    gc = np.asarray([_cell_of(m, x, y) for x, y in goals], dtype=int)
    gi, gj = np.clip(gc[:, 0], 0, w - 1), np.clip(gc[:, 1], 0, h - 1)

    # route
    straight = np.hypot(goals[:, 0] - pose["x_m"], goals[:, 1] - pose["y_m"])
    detour = np.clip((path / np.maximum(straight, res) - 1.0) / 4.0, 0, 1)
    dd = np.hypot(goals[:, None, 0] - goals[None, :, 0], goals[:, None, 1] - goals[None, :, 1])
    np.fill_diagonal(dd, np.inf)
    nearest = np.clip(dd.min(axis=1) / 5.0, 0, 1) if n > 1 else np.ones(n)
    cluster = ((dd < 2.0).sum(axis=1) / max(1, n - 1)) if n > 1 else np.zeros(n)
    clear = distance_transform_edt(~occ) * res
    wall = np.clip(clear[gj, gi] / 1.0, 0, 1)

    # information gain: rays from each goal; an occupied cell ends a ray
    steps = np.arange(1, int(RAY_M / res) + 1) * res
    PX = goals[:, None, None, 0] + _DIRS[None, :, None, 0] * steps[None, None, :]
    PY = goals[:, None, None, 1] + _DIRS[None, :, None, 1] * steps[None, None, :]
    I = np.floor((PX - m["origin_x_m"]) / res).astype(int)
    J = np.floor((PY - m["origin_y_m"]) / res).astype(int)
    on = (I >= 0) & (I < w) & (J >= 0) & (J < h)
    Ic, Jc = np.clip(I, 0, w - 1), np.clip(J, 0, h - 1)
    blocked = occ[Jc, Ic] & on
    visible = (np.cumsum(blocked, axis=2) - blocked == 0) & on
    total = visible.shape[1] * visible.shape[2]
    lidar_gain = (visible & unknown[Jc, Ic]).sum(axis=(1, 2)) / total
    cam_gain = (visible & free[Jc, Ic] & ~seen[Jc, Ic]).sum(axis=(1, 2)) / total
    u2 = _disc_share(unknown, gi, gj, int(round(2.0 / res)), off=True)
    u4 = _disc_share(unknown, gi, gj, int(round(4.0 / res)), off=True)
    gain_pm = lidar_gain / (1.0 + path / 5.0)

    # room graph, cut on the 0.30 m grid as 3.96's room features are (the
    # simulator's maps already are: there `rm` is `m`)
    rm = coarse(m)
    rw, rh = rm["width"], rm["height"]
    seg, _ = segment(rm)
    rseen = seen if rm is m else _mask_of(rm, agent.seen)
    a, b = seg[:, :-1], seg[:, 1:]
    c, d = seg[:-1, :], seg[1:, :]
    pairs = np.concatenate([np.stack([a[(a >= 0) & (b >= 0) & (a != b)],
                                      b[(a >= 0) & (b >= 0) & (a != b)]], 1),
                            np.stack([c[(c >= 0) & (d >= 0) & (c != d)],
                                      d[(c >= 0) & (d >= 0) & (c != d)]], 1)])
    nbr = {}
    for p, q in {tuple(x) for x in pairs.tolist()}:
        nbr.setdefault(p, set()).add(q)
        nbr.setdefault(q, set()).add(p)

    def seg_at(x, y):
        return _seg_at(seg, rm, x, y)      # 3.96's lookup: the cell, else its 8 neighbours

    here = seg_at(pose["x_m"], pose["y_m"])
    hops = {here: 0} if here >= 0 else {}
    frontier_ = [here] if here >= 0 else []
    while frontier_:
        nxt = []
        for r in frontier_:
            for q in nbr.get(r, ()):
                if q not in hops:
                    hops[q] = hops[r] + 1
                    nxt.append(q)
        frontier_ = nxt
    rooms = [seg_at(x, y) for x, y in goals]
    ids, area = np.unique(seg[seg >= 0], return_counts=True)
    seen_ids, seen_area = np.unique(seg[(seg >= 0) & rseen], return_counts=True)
    area = dict(zip(ids.tolist(), area.tolist()))
    seen_area = dict(zip(seen_ids.tolist(), seen_area.tolist()))
    room_hops = np.asarray([min(1.0, hops[r] / 4.0) if r in hops else 1.0 for r in rooms])
    room_dead = np.asarray([1.0 if r >= 0 and len(nbr.get(r, ())) == 1 else 0.0 for r in rooms])
    room_deg = np.asarray([min(1.0, len(nbr.get(r, ())) / 4.0) if r >= 0 else 0.0 for r in rooms])
    room_seen = np.asarray([seen_area.get(r, 0) / area[r] if r in area else 0.0 for r in rooms])

    # this search
    if trail:
        T = np.unique(np.floor(np.asarray(trail) / res).astype(int), axis=0)
        Tm = (T + 0.5) * res
        td = np.hypot(goals[:, None, 0] - Tm[None, :, 0], goals[:, None, 1] - Tm[None, :, 1])
        trail_min = np.clip(td.min(axis=1) / 5.0, 0, 1)
        trail_near = (td < 1.5).mean(axis=1)
        ground = len(T) * res * res
    else:
        trail_min, trail_near, ground = np.ones(n), np.zeros(n), 0.0
    if ground >= st.ground + NEW_GROUND_M2:
        st.ground, st.goal_at_new_ground = ground, agent.goals_sent
    since_ground = min(1.0, (agent.goals_sent - st.goal_at_new_ground) / 10.0)
    known = int((~unknown).sum())
    if st.known is None or known > 1.01 * st.known:
        st.known, st.t_growth = known, now
    since_growth = min(1.0, (now - st.t_growth) / 120.0)

    # budget
    time_used = min(1.0, now / ELAPSED_S)
    pool_size = min(1.0, n / 50.0)
    known_share = known / unknown.size

    # relative
    rule_pick = np.zeros(n)
    rule_pick[0] = 1.0
    gain_rank = _rank(lidar_gain, descending=True)
    dominated = (((path[None, :] < path[:, None]) & (lidar_gain[None, :] > lidar_gain[:, None]))
                 .sum(axis=1) / max(1, n - 1))

    cols = [detour, _rank(path), np.clip((path - path.min()) / 10.0, 0, 1), nearest, cluster, wall,
            lidar_gain, cam_gain, u2, u4, gain_pm,
            room_hops, room_dead, room_deg, room_seen,
            trail_min, trail_near, np.full(n, since_ground), np.full(n, since_growth),
            np.full(n, time_used), np.full(n, pool_size), np.full(n, known_share),
            rule_pick, gain_rank, dominated]
    assert len(cols) == len(RICH_FEATURES)
    return np.stack([np.asarray(x, dtype=np.float32) for x in cols], axis=1).tolist()
