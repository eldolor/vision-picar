"""
tests/goal_features_reference.py -- 3.109: `brain/goal_policy.py`'s
`features()` frozen as it was before its numpy rewrite (dev at 05d1870), the
per-candidate Python loop over the 1 m disc. The rewrite must return exactly
what this returns (`tests/test_goal_features_numpy.py`). Do not edit.
"""
import math

from brain.frontier import _cell_of
from brain.goal_policy import NEAR_M
from world.interface import CELL_FREE, CELL_UNKNOWN


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
