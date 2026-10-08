"""
python evaluations/slam-345/goal_sweep.py RUN_DIR [RUN_DIR ...]

PLAN-ros-alignment.md 3.45 criterion 5: on the SLAM maps explore really had
(each live run's `maps.jsonl.gz`, a snapshot every 20 s), which goal does the
policy choose next, and could the robot leave it?

For every snapshot, a fresh `ExploreAgent` is asked for its next goal by the
code a mission runs (`_next_frontier()`), on that map, from that SLAM pose.
It is fresh, so it has no retry book and no reached or seen history: this
measures the choice a map invites, not one mission's bookkeeping. The goal
point is then judged on GROUND TRUTH with the real safety layer
(`tests/escape_sweep.wedge_kind`), every 10 degrees of heading.

* A heading the chassis cannot stand at (it would overlap furniture on
  truth) is skipped: the robot cannot arrive that way.
* A goal is **leavable** when, at every heading it can stand at, forward or
  reverse is allowed.

Bar: >= 95% of chosen goals leavable. The progress half is the share of
snapshots that offer a goal at all.
"""

import gzip
import json
import math
import os
import sys
from collections import Counter

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from brain.explore import ExploreAgent  # noqa: E402
from brain.memory import MissionMemory  # noqa: E402
from sim.maps import build_world  # noqa: E402
from sim.mock_robot import MockRobot  # noqa: E402
from tests.escape_sweep import _world, wedge_kind  # noqa: E402
from tests.footprint_sweep import truth  # noqa: E402

HOUSE = "home_first_floor"
CELL = 0.30
HEADINGS = range(0, 360, 10)
BAR = 0.95


class _Snapshot:
    """A world model that answers with one recorded snapshot."""

    def __init__(self, snap):
        self.snap = snap

    def get_map(self):
        return self.snap["map"]

    def get_pose(self):
        return self.snap["pose"]


class _Accept:
    """A navigator that takes every goal and never drives."""

    def __init__(self):
        self.goal = None

    def set_goal(self, x_m, y_m):
        self.goal = {"state": "active", "x_m": x_m, "y_m": y_m}
        return {"accepted": True}

    def get_goal(self):
        return {"goal": self.goal, "plan": []}

    def cancel_goal(self):
        return {"canceled": True}


def choose(snap):
    """(kind, (x, y)) the policy would send from this snapshot, or None."""
    t = snap["truth"]
    grid = build_world(HOUSE)
    grid.x, grid.y = t["x_m"] / CELL, t["y_m"] / CELL
    agent = ExploreAgent(MockRobot(grid, render=False),
                         MissionMemory(mission="m", target_object="purple elephant"),
                         navigator=_Accept(), clock=lambda: 0.0, world=_Snapshot(snap))
    action, _ok, _detail = agent._next_frontier(0.0)
    if action != "GOAL":
        return None
    return agent._goal["kind"], (agent._goal["x_m"], agent._goal["y_m"])


def leavable(x, y):
    """(leavable, kinds by heading) on truth at a goal point."""
    kinds = Counter()
    for h in HEADINGS:
        if truth(_world(x, y, h))[2] > 0:
            kinds["overlaps"] += 1
            continue
        kinds[wedge_kind(x, y, h)] += 1
    standable = sum(v for k, v in kinds.items() if k != "overlaps")
    return standable > 0 and kinds["free"] == standable, dict(kinds)


def snapshots(run_dir):
    with gzip.open(os.path.join(run_dir, "maps.jsonl.gz"), "rt") as f:
        for line in f:
            try:
                s = json.loads(line)
            except json.JSONDecodeError:      # a run killed mid-write
                continue
            if (s.get("map") or {}).get("usable") and (s.get("pose") or {}).get("usable") \
                    and (s.get("truth") or {}).get("x_m") is not None:
                yield s


def sweep(run_dirs, until=None):
    """`until`: per run, the wall time to stop at (the explore phase's end)."""
    rows = []
    for rd in run_dirs:
        end = (until or {}).get(rd)
        for s in snapshots(rd):
            if end and s["t"] > end:
                break
            c = choose(s)
            row = {"run": os.path.basename(rd), "t": round(s["t"]), "goal": None}
            if c:
                ok, kinds = leavable(*c[1])
                row.update(goal=[round(c[1][0], 2), round(c[1][1], 2)], kind=c[0],
                           leavable=ok, headings=kinds)
            rows.append(row)
    chosen = [r for r in rows if r["goal"]]
    ok = sum(r["leavable"] for r in chosen)
    return {"snapshots": len(rows), "with_goal": len(chosen),
            "with_goal_share": round(len(chosen) / max(len(rows), 1), 3),
            "leavable": ok, "leavable_share": round(ok / max(len(chosen), 1), 3),
            "pass": len(chosen) > 0 and ok / len(chosen) >= BAR,
            "not_leavable": [r for r in chosen if not r["leavable"]]}, rows


def explore_end(run_dir):
    """The explore phase's end from the run's JSON, if there is one."""
    for name in os.listdir(run_dir):
        if name.startswith("slam_home_") and name.endswith(".json"):
            ex = json.load(open(os.path.join(run_dir, name))).get("explore") or {}
            return ex.get("ended_at")
    return None


if __name__ == "__main__":
    dirs = sys.argv[1:]
    summary, rows = sweep(dirs, {d: explore_end(d) for d in dirs})
    print(json.dumps({k: v for k, v in summary.items() if k != "not_leavable"}))
    for r in summary["not_leavable"]:
        print(json.dumps(r))
