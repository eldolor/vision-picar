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
`brain/explore_goal_net.json`. It imports nothing heavier than numpy, so the
brain stays light.
"""

import json
import math
from pathlib import Path
from typing import Optional

import numpy as np

from brain.frontier import _cell_of
from world.interface import CELL_FREE, CELL_UNKNOWN

WEIGHTS_PATH = Path(__file__).resolve().parent / "explore_goal_weights.json"
NET_PATH = Path(__file__).resolve().parent / "explore_goal_net.json"

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


def features(m: dict, pose: dict, goal: tuple, *, kind: str, path_m: float,
             size_m: float = 0.0, area_m2: float = 0.0, seen_cells: set = frozenset(),
             failures: int = 0, reached: int = 0, goals_sent: int = 0) -> list:
    """One candidate's feature vector, in `FEATURES` order."""
    # The bearing as explore's FACE turn computes it (`brain/explore.py`):
    # the simulator's heading 0 points to -y_m, 90 to +x_m.
    bearing = math.degrees(math.atan2(goal[0] - pose["x_m"], -(goal[1] - pose["y_m"])))
    heading = pose.get("heading_deg")
    turn = 0.0 if heading is None else abs((bearing - heading + 180.0) % 360.0 - 180.0)
    res, w, h = m["resolution_m"], m["width"], m["height"]
    cx, cy = _cell_of(m, goal[0], goal[1])
    r = int(math.ceil(NEAR_M / res))
    total = unknown = free = unseen = 0
    for y in range(max(0, cy - r), min(h, cy + r + 1)):
        for x in range(max(0, cx - r), min(w, cx + r + 1)):
            if (x - cx) ** 2 + (y - cy) ** 2 > r * r:
                continue
            total += 1
            v = m["cells"][y * w + x]
            if v == CELL_UNKNOWN:
                unknown += 1
            elif v == CELL_FREE:
                free += 1
                if (x, y) not in seen_cells:
                    unseen += 1
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


POLICIES = ("rule", "learned", "network")


def goal_policy_for(name: str) -> Optional[LinearGoalPolicy]:
    """`explore_goal_policy` -> the policy object, or None for the rule."""
    if name == "rule":
        return None
    if name == "learned":
        return LinearGoalPolicy.load()
    if name == "network":
        return NetGoalPolicy.load()
    raise ValueError(f"explore_goal_policy must be one of {POLICIES}; got {name!r}")
