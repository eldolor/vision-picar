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
from features import coarse  # noqa: E402  (3.96's: the room cut runs on the 0.30 m grid)

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
    "since_new_room",  # goals sent since the robot last stood in a new room, / 10
    "since_growth",    # seconds since the map last grew 1%, / 120 s
    # budget
    "time_used",       # share of the time limit used
    "pool_size",       # candidates in the pool, / 50
    "known_share",     # known share of the map's box
    # relative
    "rule_pick",       # the rule's own pick (the pool's first)
    "gain_rank",       # rank of lidar_gain in the pool, 0 best .. 1 worst
    "dominated",       # share of the pool shorter AND with more gain
)
RAY_M, RAYS = 3.0, 16
TIME_LIMIT_S = 1200.0      # mission_time_limit_s, the default the sweeps run under

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


def _disc_share(mask: np.ndarray, cx: np.ndarray, cy: np.ndarray, r: int) -> np.ndarray:
    """Share of `mask` cells inside a disc of `r` cells round each centre
    (off the map counts as not-mask and inside the disc)."""
    h, w = mask.shape
    oy, ox = np.mgrid[-r:r + 1, -r:r + 1]
    keep = ox ** 2 + oy ** 2 <= r * r
    ox, oy = ox[keep], oy[keep]
    X = cx[:, None] + ox[None, :]
    Y = cy[:, None] + oy[None, :]
    ok = (X >= 0) & (X < w) & (Y >= 0) & (Y < h)
    v = np.where(ok, mask[np.clip(Y, 0, h - 1), np.clip(X, 0, w - 1)], False)
    return v.mean(axis=1)


class RichState:
    """What the features remember across one mission's choices."""

    def __init__(self):
        self.t0 = None
        self.stood_rooms = 0
        self.goal_at_new_room = 0
        self.known = None
        self.t_growth = None


def rich_features(agent, m: dict, pose: dict, pool: list, feats: list, trail: list,
                  now: float) -> list:
    st = getattr(agent, "_rich3101", None)
    if st is None:
        st = agent._rich3101 = RichState()
    if st.t0 is None:
        st.t0 = now
    n = len(pool)
    w, h, res = m["width"], m["height"], m["resolution_m"]
    cells = np.asarray(m["cells"]).reshape(h, w)
    unknown = cells == CELL_UNKNOWN
    occ = cells == CELL_OCCUPIED
    free = cells == CELL_FREE
    seen = np.zeros((h, w), dtype=bool)
    for c in agent.seen:
        i, j = _cell_of(m, c[0], c[1])
        if 0 <= i < w and 0 <= j < h:
            seen[j, i] = True

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
    u2 = _disc_share(unknown, gi, gj, int(round(2.0 / res)))
    u4 = _disc_share(unknown, gi, gj, int(round(4.0 / res)))
    gain_pm = lidar_gain / (1.0 + path / 5.0)

    # room graph, cut on the 0.30 m grid as 3.96's room features are (the
    # simulator's maps already are: there `rm` is `m`)
    rm = coarse(m)
    rw, rh = rm["width"], rm["height"]
    seg, _ = segment(rm)
    rseen = seen
    if rm is not m:
        rseen = np.zeros((rh, rw), dtype=bool)
        jj, ii = np.nonzero(seen)
        for x, y in zip(m["origin_x_m"] + (ii + 0.5) * res, m["origin_y_m"] + (jj + 0.5) * res):
            i, j = _cell_of(rm, x, y)
            if 0 <= i < rw and 0 <= j < rh:
                rseen[j, i] = True
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
        i, j = _cell_of(rm, x, y)
        for di, dj in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)):
            ii, jj = i + di, j + dj
            if 0 <= ii < rw and 0 <= jj < rh and seg[jj, ii] >= 0:
                return int(seg[jj, ii])
        return -1

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
        stood = {seg_at(x, y) for x, y in Tm.tolist()} - {-1}
    else:
        trail_min, trail_near, stood = np.ones(n), np.zeros(n), set()
    if len(stood) > st.stood_rooms:
        st.stood_rooms, st.goal_at_new_room = len(stood), agent.goals_sent
    since_room = min(1.0, (agent.goals_sent - st.goal_at_new_room) / 10.0)
    known = int((~unknown).sum())
    if st.known is None or known > 1.01 * st.known:
        st.known, st.t_growth = known, now
    since_growth = min(1.0, (now - st.t_growth) / 120.0)

    # budget
    time_used = min(1.0, (now - st.t0) / TIME_LIMIT_S)
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
            trail_min, trail_near, np.full(n, since_room), np.full(n, since_growth),
            np.full(n, time_used), np.full(n, pool_size), np.full(n, known_share),
            rule_pick, gain_rank, dominated]
    assert len(cols) == len(RICH_FEATURES)
    return np.stack([np.asarray(x, dtype=np.float32) for x in cols], axis=1).tolist()
