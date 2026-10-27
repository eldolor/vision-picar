"""
goal_policy.py -- 3.65 phase 2: a learned choice of explore's next goal.

Explore's rule (`brain/explore.py` `_next_frontier`) takes the best-scored
frontier, and looks at unseen floor (view gaps) only once no frontier is
left. Under `explore_goal_policy: learned` the same candidates -- only the
ones the rule would send at all (not reached twice, not set aside, not a
view already looked at) -- are pooled and ranked by a linear score instead:
`score = w . features`, highest first. Everything else in explore stays the
rule: sightings, approaches, escapes, the retry book, when the search ends.

The weights are learned in simulation by the cross-entropy method
(`evaluations/explore-365/train.py`) and committed in
`brain/explore_goal_weights.json`; this module only reads and applies them.
Under `explore_goal_policy: network` (amendment 2) the score is that linear
term plus a small network's, trained by PPO
(`evaluations/explore-365/train_net.py`) and committed in
`brain/explore_goal_net.json`. Under `explore_goal_policy: rooms` (3.96) a
network takes the ten features plus seven about rooms (`ROOM_FEATURES`: the
candidate's room cut from the brain's own map by `segment()`, whether the
robot has stood in it, and, where the map carries room labels, how likely
the target is in a room of that type), trained by clone then PPO
(`evaluations/goal-learn-396/train.py`, arm F-large) and committed in
`brain/explore_goal_rooms.json`. It imports nothing heavier than numpy, so
the brain stays light.
"""

import json
import math
import re
from pathlib import Path
from typing import Optional

import numpy as np

from brain.frontier import _cell_of
from brain.room_labels import segment
from world.interface import CELL_FREE, CELL_OCCUPIED, CELL_UNKNOWN

WEIGHTS_PATH = Path(__file__).resolve().parent / "explore_goal_weights.json"
NET_PATH = Path(__file__).resolve().parent / "explore_goal_net.json"
ROOMS_PATH = Path(__file__).resolve().parent / "explore_goal_rooms.json"

# The order the weights are stored in. Each feature is scaled to roughly
# 0..1 over a house, so a weight's size says how much it matters.
FEATURES = (
    "path",          # path length to the goal, / 10 m
    "turn",          # |bearing from the heading to the goal|, / 180 deg
    "is_view",       # 1 for a view gap, 0 for a frontier
    "size",          # a frontier's length, / 3 m (0 for a view)
    "area",          # a view gap's area, / 3 m2 (0 for a frontier)
    "unknown_near",  # share of the cells within NEAR_M of the goal still unknown
    "unseen_near",   # share of known floor within NEAR_M the camera has not seen
    "failed",        # failures the retry book holds near the goal, / 2
    "reached",       # times a frontier goal near here was reached, / 2
    "view_late",     # is_view x goals sent so far / 20 (views matter more later)
)
NEAR_M = 1.0


def _disc(r: int):
    """The (dx, dy) offsets of a disc of radius r cells, as two int arrays."""
    d = np.arange(-r, r + 1)
    dy, dx = np.meshgrid(d, d, indexing="ij")
    keep = dx * dx + dy * dy <= r * r
    return dx[keep], dy[keep]


def goal_context(m: dict, seen_cells: set = frozenset()) -> dict:
    """What `features()` reads from the map, built once per choice (3.109):
    the grid's unknown, free and free-but-unseen cells as boolean arrays,
    and the 1 m disc's offsets. 3.103 timed the per-candidate loop at
    185 ms p90 a choice on the Jetson; this makes each candidate a few array
    lookups. The map and the seen set must not change while it is used."""
    res, w, h = m["resolution_m"], m["width"], m["height"]
    cells = np.asarray(m["cells"]).reshape(h, w)
    free = cells == CELL_FREE
    unseen = free.copy()
    if seen_cells:
        xy = np.asarray(list(seen_cells), dtype=np.int64).reshape(-1, 2)
        on = (xy[:, 0] >= 0) & (xy[:, 0] < w) & (xy[:, 1] >= 0) & (xy[:, 1] < h)
        unseen[xy[on, 1], xy[on, 0]] = False
    dx, dy = _disc(int(math.ceil(NEAR_M / res)))
    return {"w": w, "h": h, "unknown": cells == CELL_UNKNOWN, "free": free,
            "unseen": unseen, "dx": dx, "dy": dy}


def features(m: dict, pose: dict, goal: tuple, *, kind: str, path_m: float,
             size_m: float = 0.0, area_m2: float = 0.0, seen_cells: set = frozenset(),
             failures: int = 0, reached: int = 0, goals_sent: int = 0,
             ctx: Optional[dict] = None) -> list:
    """One candidate's feature vector, in `FEATURES` order. `ctx`: this
    choice's `goal_context(m, seen_cells)`, shared by its candidates (built
    here when not given). The same answers as the loop it replaced
    (`tests/goal_features_reference.py`), to the bit."""
    # The bearing as explore's FACE turn computes it (`brain/explore.py`):
    # the simulator's heading 0 points to -y_m, 90 to +x_m.
    bearing = math.degrees(math.atan2(goal[0] - pose["x_m"], -(goal[1] - pose["y_m"])))
    heading = pose.get("heading_deg")
    turn = 0.0 if heading is None else abs((bearing - heading + 180.0) % 360.0 - 180.0)
    if ctx is None:
        ctx = goal_context(m, seen_cells)
    cx, cy = _cell_of(m, goal[0], goal[1])
    xs, ys = cx + ctx["dx"], cy + ctx["dy"]
    on = (xs >= 0) & (xs < ctx["w"]) & (ys >= 0) & (ys < ctx["h"])
    xs, ys = xs[on], ys[on]
    # Python ints, so the divisions below give the loop's floats exactly.
    total = int(xs.size)
    unknown = int(ctx["unknown"][ys, xs].sum())
    free = int(ctx["free"][ys, xs].sum())
    unseen = int(ctx["unseen"][ys, xs].sum())
    is_view = 1.0 if kind == "view" else 0.0
    return [
        path_m / 10.0,
        turn / 180.0,
        is_view,
        (size_m / 3.0) if not is_view else 0.0,
        (area_m2 / 3.0) if is_view else 0.0,
        unknown / total if total else 0.0,
        unseen / free if free else 0.0,
        failures / 2.0,
        reached / 2.0,
        is_view * goals_sent / 20.0,
    ]


class LinearGoalPolicy:
    """`score = w . features`; the highest-scored candidate is chosen."""

    def __init__(self, weights):
        weights = [float(x) for x in weights]
        if len(weights) != len(FEATURES):
            raise ValueError(f"expected {len(FEATURES)} weights ({', '.join(FEATURES)}); "
                             f"got {len(weights)}")
        self.weights = weights

    def score(self, f: list) -> float:
        return sum(w * x for w, x in zip(self.weights, f))

    def best(self, candidates: list) -> Optional[int]:
        """Index of the highest-scored candidate's features, or None if none.
        Ties keep the earlier one (the rule's own order)."""
        best_i, best_s = None, -math.inf
        for i, f in enumerate(candidates):
            s = self.score(f)
            if not math.isfinite(s):
                # A NaN never beats -inf: every candidate NaN would end the
                # search `searched` with goals left. Rank it last instead.
                s = -math.inf
            if best_i is None or s > best_s:
                best_i, best_s = i, s
        return best_i

    @classmethod
    def load(cls, path: Path = WEIGHTS_PATH) -> "LinearGoalPolicy":
        data = json.loads(Path(path).read_text())
        if list(data["features"]) != list(FEATURES):
            raise ValueError(f"{path}: features {data['features']} do not match "
                             f"this code's {list(FEATURES)}")
        return cls(data["weights"])


class NetGoalPolicy(LinearGoalPolicy):
    """`score = w . features + MLP(features)`: the linear score plus a small
    tanh network's (amendment 2). `layers` is [(W, b), ...], W shaped
    (out, in); the last layer has one output."""

    def __init__(self, weights, layers):
        super().__init__(weights)
        self.layers = [(np.asarray(W, dtype=float), np.asarray(b, dtype=float))
                       for W, b in layers]
        # Every shape checked at load, so a bad file fails here and never
        # inside a live mission's first score().
        width = len(FEATURES)
        for i, (W, b) in enumerate(self.layers):
            if W.ndim != 2 or W.shape[1] != width or b.shape != (W.shape[0],):
                raise ValueError(f"network layer {i}: W {W.shape}, b {b.shape} "
                                 f"do not take {width} inputs")
            width = W.shape[0]
        if not self.layers or width != 1:
            raise ValueError(f"the network must take {len(FEATURES)} features and give one score")

    def score(self, f: list) -> float:
        x = np.asarray(f, dtype=float)
        for i, (W, b) in enumerate(self.layers):
            x = W @ x + b
            if i < len(self.layers) - 1:
                x = np.tanh(x)
        return super().score(f) + float(x[0])

    @classmethod
    def load(cls, path: Path = NET_PATH) -> "NetGoalPolicy":
        data = json.loads(Path(path).read_text())
        if list(data["features"]) != list(FEATURES):
            raise ValueError(f"{path}: features {data['features']} do not match "
                             f"this code's {list(FEATURES)}")
        return cls(data["weights"], [(l["W"], l["b"]) for l in data["layers"]])


# ---------- 3.96: rooms ----------

# Appended after FEATURES, in this order.
ROOM_FEATURES = (
    "room_new",      # the candidate's room (a `segment()` piece) never stood in
    "room_area",     # that room's known floor, / 20 m2
    "room_other",    # a different room from the robot's
    "room_share",    # share of the pool's candidates in the same room
    "type_known",    # a room label names the candidate's room type
    "prior",         # P(target | that type); 0 when the type is unknown
    "prior_new",     # prior x room_new
)
ROOM_AREA_M2 = 20.0
# Rooms are cut on the training grid's resolution (the simulator's 0.30 m
# cells): a finer map (SLAM's 0.05 m) is resampled first, so the features
# read as they did in training and `segment()` sees 36x fewer cells.
ROOM_RES_M = 0.30


def coarse(m: dict, res_m: float = ROOM_RES_M) -> dict:
    """`m` at `res_m` (an integer factor of its own resolution): a coarse
    cell is occupied if any of its cells is, free if at least half are, and
    unknown otherwise. A map already that coarse is returned as it is."""
    k = int(round(res_m / m["resolution_m"]))
    if k <= 1:
        return m
    w, h = m["width"], m["height"]
    W, H = -(-w // k), -(-h // k)
    cells = np.full((H * k, W * k), CELL_UNKNOWN, dtype=np.asarray(m["cells"]).dtype)
    cells[:h, :w] = np.asarray(m["cells"]).reshape(h, w)
    blocks = cells.reshape(H, k, W, k).swapaxes(1, 2).reshape(H, W, k * k)
    out = np.full((H, W), CELL_UNKNOWN, dtype=cells.dtype)
    out[(blocks == CELL_FREE).sum(axis=2) * 2 >= k * k] = CELL_FREE
    out[(blocks == CELL_OCCUPIED).any(axis=2)] = CELL_OCCUPIED
    return {**m, "width": W, "height": H, "resolution_m": m["resolution_m"] * k,
            "cells": out.ravel().tolist()}


def _seg_at(seg, m: dict, x_m: float, y_m: float) -> int:
    """The segment id under a point, or of the nearest floor cell within one
    cell (a goal can sit on a frontier's unknown edge); -1 if none."""
    return _seg_at_cell(seg, *_cell_of(m, x_m, y_m))


def _seg_at_cell(seg, cx: int, cy: int) -> int:
    h, w = seg.shape
    for dx, dy in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (1, -1), (-1, 1)):
        i, j = cx + dx, cy + dy
        if 0 <= i < w and 0 <= j < h and seg[j, i] >= 0:
            return int(seg[j, i])
    return -1


def room_features(m: dict, pose: dict, goals: list, *, trail, types: list,
                  prior: dict) -> list:
    """Each candidate's ROOM_FEATURES. `goals`: the pool's (x_m, y_m) in
    order; `trail`: map-frame points the robot has stood on; `types`: each
    goal's room type from the labels, or None; `prior`: type -> P(target)."""
    m = coarse(m)
    seg, _ = segment(m)
    res2 = m["resolution_m"] ** 2
    ids, counts = np.unique(seg[seg >= 0], return_counts=True)
    area = dict(zip(ids.tolist(), counts.tolist()))
    here = _seg_at(seg, m, pose["x_m"], pose["y_m"])
    # The trail once per map cell it touches, not once per point.
    pts = np.asarray(list(trail), dtype=float).reshape(-1, 2)
    cells = np.unique(np.floor((pts - (m["origin_x_m"], m["origin_y_m"]))
                               / m["resolution_m"]).astype(int), axis=0)
    stood = {_seg_at_cell(seg, int(i), int(j)) for i, j in cells} | {here}
    rooms = [_seg_at(seg, m, x, y) for x, y in goals]
    out = []
    for r, t in zip(rooms, types):
        new = 1.0 if r >= 0 and r not in stood else 0.0
        p = float(prior.get(t, 0.0)) if t is not None else 0.0
        out.append([
            new,
            (area.get(r, 0) * res2 / ROOM_AREA_M2) if r >= 0 else 0.0,
            1.0 if r >= 0 and r != here else 0.0,
            (sum(x == r for x in rooms) / len(rooms)) if r >= 0 else 0.0,
            0.0 if t is None else 1.0,
            p,
            p * new,
        ])
    return out


def room_type(name: str) -> str:
    """'bedroom 2' -> 'bedroom'."""
    return re.sub(r" \d+$", "", name)


class RoomGoalPolicy:
    """3.96's F-large: `score = w . f + MLP(f)` over the 17 features
    (FEATURES then ROOM_FEATURES), MLP 17 -> 32 -> 32 -> 1 (tanh), scored in
    float32 exactly as the test run scored it
    (`evaluations/goal-learn-396/model.py` `score_np`). `prior`: target ->
    room type -> P(target in a room of that type), from the training houses.
    Explore calls `extend()` on the pool's ten features first (`rooms`)."""

    rooms = True
    N = len(FEATURES) + len(ROOM_FEATURES)
    SHAPES = {"w": (N,), "W1": (32, N), "b1": (32,), "W2": (32, 32), "b2": (32,),
              "W3": (1, 32), "b3": (1,)}

    def __init__(self, params: dict, prior: dict):
        self.P = {k: np.asarray(params[k], dtype=np.float32) for k in self.SHAPES}
        for k, shape in self.SHAPES.items():
            if self.P[k].shape != shape:
                raise ValueError(f"{k}: shape {self.P[k].shape}, expected {shape}")
        self.prior = {t.lower(): dict(p) for t, p in prior.items()}

    def extend(self, m: dict, pose: dict, goals: list, feats: list, *, trail,
               types: list, target: Optional[str]) -> list:
        """The pool's ten features, each with its seven room features after."""
        if not goals:
            return []
        prior = self.prior.get((target or "").lower(), {})
        # A type the prior does not name reads as no type: in training every
        # labelled type had a prior, so "known, P = 0" meant a real zero.
        types = [t if t in prior else None for t in types]
        room = room_features(m, pose, goals, trail=trail, types=types, prior=prior)
        return [a + b for a, b in zip(feats, room)]

    def scores(self, candidates: list) -> np.ndarray:
        P, f = self.P, np.asarray(candidates, dtype=np.float32)
        if f.ndim != 2 or f.shape[1] != self.N:
            raise ValueError(f"expected rows of {self.N} features (extend() first); "
                             f"got shape {f.shape}")
        h = np.tanh(f @ P["W1"].T + P["b1"])
        h = np.tanh(h @ P["W2"].T + P["b2"])
        return f @ P["w"] + (h @ P["W3"].T + P["b3"]).ravel()

    def score(self, f: list) -> float:
        return float(self.scores([f])[0])

    def best(self, candidates: list) -> Optional[int]:
        """Index of the highest score, or None if none. Ties keep the earlier
        one; a non-finite score ranks last (as the test run ranked it)."""
        if not candidates:
            return None
        s = self.scores(candidates)
        return int(np.argmax(np.where(np.isfinite(s), s, -1e9)))

    @classmethod
    def load(cls, path: Path = ROOMS_PATH) -> "RoomGoalPolicy":
        data = json.loads(Path(path).read_text())
        if list(data["features"]) != list(FEATURES) + list(ROOM_FEATURES):
            raise ValueError(f"{path}: features {data['features']} do not match "
                             f"this code's {list(FEATURES) + list(ROOM_FEATURES)}")
        return cls(data["params"], data["prior"])


POLICIES = ("rule", "learned", "network", "rooms")


def goal_policy_for(name: str):
    """`explore_goal_policy` -> the policy object, or None for the rule."""
    if name == "rule":
        return None
    if name == "learned":
        return LinearGoalPolicy.load()
    if name == "network":
        return NetGoalPolicy.load()
    if name == "rooms":
        return RoomGoalPolicy.load()
    raise ValueError(f"explore_goal_policy must be one of {POLICIES}; got {name!r}")
