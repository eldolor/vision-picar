"""
python -m tests.escape_sweep

PLAN-ros-alignment.md 3.42 criterion 1: does the explore policy's escape get
the robot out of where it got stuck on 2026-10-06?

Poses are sampled anywhere in the furnished home with the chassis within
5 cm of furniture or a wall (ground truth) -- where the 2026-10-06 runs
stopped for good (the exact live pose cannot be rebuilt from a 1 Hz
position). The poses that count are the ones that wedge -- forward AND
reverse refused -- but leave one turn direction free, until N. At each, the escape
(`ExploreAgent._queue_escape()` and its pending moves, the code a mission
runs) is driven up to `MAX_ESCAPES` times through the guarded verbs.
FREED = forward or reverse is allowed afterwards. Contacts are judged on
ground truth (`footprint_sweep.truth`'s chassis-to-cell gap), never on the
readings the safety layer used.

Headings where all four moves are refused are counted and reported, not
judged: nothing can leave them without loosening robot/safety.py (3.42
part b keeps the planner out of them instead).
"""

import json
import math
import random
import sys

from brain.explore import MAX_ESCAPES, ExploreAgent
from brain.memory import MissionMemory
from robot.safety import SafetyController
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests.footprint_sweep import truth
from tests.test_explore import WedgedNav

HOUSE = "home_first_floor"
CELL = 0.30
MIN_CM = 20.0
NEAR_CM = 5.0
N = 200


def _world(x, y, heading_deg):
    w = build_world(HOUSE)
    w.x, w.y, w.theta = x / CELL, y / CELL, math.radians(heading_deg)
    return w


def wedge_kind(x, y, heading_deg):
    """'free' (forward or reverse allowed), 'one_turn' (both refused, one
    turn free), or 'trap' (all four refused)."""
    s = SafetyController(MockRobot(_world(x, y, heading_deg), render=False), MIN_CM)
    f, _ = s.forward_clearance()
    b, _ = s.reverse_clearance()
    if not (f is not None and f <= MIN_CM and b is not None and b <= MIN_CM):
        return "free"
    left = s.pivot_blocked(+1.0) is not None
    right = s.pivot_blocked(-1.0) is not None
    return "trap" if (left and right) else "one_turn"


def freed(safety):
    f, _ = safety.forward_clearance()
    b, _ = safety.reverse_clearance()
    return (f is None or f > MIN_CM) or (b is None or b > MIN_CM)


def escape(x, y, heading_deg):
    grid = _world(x, y, heading_deg)
    robot = MockRobot(grid, render=False)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object="purple elephant"),
                         navigator=WedgedNav(robot), clock=lambda: grid.sim_time,
                         world=MockWorld(grid))
    min_gap = truth(grid)[1]
    moves = []
    for k in range(MAX_ESCAPES):
        agent._queue_escape()
        guard = 0
        while agent._pending and guard < 80:
            guard += 1
            action, done, _ = agent._do_pending()
            moves.append((action, bool(done)))
            min_gap = min(min_gap, truth(grid)[1])
        if freed(agent.safety):
            return {"freed": True, "escapes": k + 1, "min_gap_cm": round(min_gap, 2), "moves": moves}
    return {"freed": False, "escapes": MAX_ESCAPES, "min_gap_cm": round(min_gap, 2), "moves": moves}


def sweep(n=N, seed=0):
    rng = random.Random(seed)
    base = build_world(HOUSE)
    free_cells = [(cx, cy) for cy, row in enumerate(base.layout) for cx, c in enumerate(row)
                  if c != "#" and (cx, cy) not in base.solid_cells]
    rows, tries = [], 0
    while sum(r["kind"] == "one_turn" for r in rows) < n and tries < 200 * n:
        tries += 1
        cx, cy = rng.choice(free_cells)
        x, y = (cx + rng.random()) * CELL, (cy + rng.random()) * CELL
        h = rng.uniform(0, 360)
        w = _world(x, y, h)
        g = truth(w)[1]
        if not (0.0 < g <= NEAR_CM):
            continue
        kind = wedge_kind(x, y, h)
        if kind == "free":
            continue
        r = {"x_m": round(x, 3), "y_m": round(y, 3), "heading": round(h, 1), "kind": kind,
             "gap_cm": round(g, 2)}
        if kind == "one_turn":
            r.update(escape(x, y, h))
        rows.append(r)
    return rows


def summary(rows):
    one = [r for r in rows if r["kind"] == "one_turn"]
    return {"one_turn_headings": len(one),
            "freed": sum(r["freed"] for r in one),
            "freed_share": round(sum(r["freed"] for r in one) / max(len(one), 1), 3),
            "contacts": sum(r["min_gap_cm"] <= 0.0 for r in one),
            "min_gap_cm": min((r["min_gap_cm"] for r in one), default=None),
            "trap_headings": sum(r["kind"] == "trap" for r in rows)}


if __name__ == "__main__":
    rows = sweep(int(sys.argv[1]) if len(sys.argv) > 1 else N)
    print(json.dumps(summary(rows)))
    for r in rows:
        if r["kind"] == "one_turn" and not r["freed"]:
            print(json.dumps({k: r[k] for k in ("x_m", "y_m", "heading", "moves")}))
