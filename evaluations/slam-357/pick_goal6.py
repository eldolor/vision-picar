"""
python evaluations/slam-357/pick_goal6.py

PLAN-ros-alignment.md 3.57 criterion 2: the tour's new dining-room point, by
rule, not by hand. Candidates on a 5 cm grid within 1.0 m of the old point
(31.0, 30.0) ft; eligible when inside the dining room's cells, the chassis
overlaps nothing on ground truth at any heading (every 10 deg), and at every
heading forward or reverse is allowed (`goal_sweep.leavable`). Amendment 1:
also at least `leave_clearance_m(20)` (0.3265 m) from every true surface,
centre to the nearest wall or solid cell. The nearest eligible point wins;
ties go to the smaller (y, x).
"""

import importlib.util
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
os.environ.setdefault("SIM_MAP", "home_first_floor")

from brain.explore import leave_clearance_m  # noqa: E402
from sim.maps import build_world  # noqa: E402
from tests.footprint_sweep import occupied_near  # noqa: E402
from tests.demo_nav_goals import _home_m  # noqa: E402

spec = importlib.util.spec_from_file_location(
    "slam345_goal_sweep", os.path.join(ROOT, "evaluations", "slam-345", "goal_sweep.py"))
gs = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gs)

OLD_FT = (31.0, 30.0)
RADIUS_M, STEP_M, CELL = 1.0, 0.05, 0.30


def surface_m(x, y, world):
    """Centre to the nearest true wall or solid cell, metres."""
    cx, cy = x / CELL, y / CELL
    best = math.inf
    for sq in occupied_near(world, cx, cy, 2.5):
        (x0, y0), (x1, y1) = sq[0], sq[2]
        dx, dy = max(x0 - cx, 0, cx - x1), max(y0 - cy, 0, cy - y1)
        best = min(best, math.hypot(dx, dy) * CELL)
    return best


WORLD = build_world("home_first_floor")
NEED_M = leave_clearance_m(20.0)


def eligible(x, y, dining):
    if (int(x // CELL), int(y // CELL)) not in dining:
        return False
    if surface_m(x, y, WORLD) < NEED_M:
        return False
    ok, kinds = gs.leavable(x, y)
    return ok and "overlaps" not in kinds


def pick():
    ox, oy = _home_m(*OLD_FT)
    dining = set(build_world("home_first_floor").rooms["dining room"])
    n = int(RADIUS_M / STEP_M)
    cands = sorted(((math.hypot(i * STEP_M, j * STEP_M), round(oy + j * STEP_M, 3),
                     round(ox + i * STEP_M, 3))
                    for i in range(-n, n + 1) for j in range(-n, n + 1)
                    if math.hypot(i * STEP_M, j * STEP_M) <= RADIUS_M))
    for d, y, x in cands:
        if eligible(x, y, dining):
            return (x, y), round(d, 3), (ox, oy)
    return None, None, (ox, oy)


if __name__ == "__main__":
    point, d, old = pick()
    print({"old_m": old, "new_m": point, "moved_m": d,
           "old_surface_m": round(surface_m(*old, WORLD), 3),
           "new_surface_m": point and round(surface_m(*point, WORLD), 3),
           "new_ft": point and (round(point[0] / 0.3048 - (old[0] / 0.3048 - OLD_FT[0]), 2),
                                round(point[1] / 0.3048 - (old[1] / 0.3048 - OLD_FT[1]), 2))})
