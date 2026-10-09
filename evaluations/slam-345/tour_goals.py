"""
python evaluations/slam-345/tour_goals.py run.json [...]

PLAN-ros-alignment.md 3.45 criterion 8: every tour goal that did not succeed,
read off the run's own nav2 log and the robot server's refusals -- so the
tour's failures are recorded as the tour's, not explore's. For each: the
goal's state and time, where the robot ended (truth), how far from the goal
point the nearest true surface is, nav2's own milestones in order, and the
refusals in the window. A goal that began with the robot already parked from
explore is marked `inherited`.
"""

import glob
import gzip
import json
import math
import os
import re
import sys
from collections import Counter

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", ".."))
from tests import demo_nav_goals as ng  # noqa: E402
from tests.escape_sweep import _world  # noqa: E402
from tests.footprint_sweep import truth  # noqa: E402

KEY = re.compile(r"Failed to make progress|failed to create plan|failed to generate|"
                 r"Goal succeeded|Goal failed|aborted|preemption|Running spin|Running backup|"
                 r"Received a goal|Begin navigating|collision_monitor", re.I)


def _open(p):
    return gzip.open(p, "rt", errors="replace") if p.endswith(".gz") else open(p, errors="replace")


def nav2_lines(run_dir, a, b):
    out = []
    for p in glob.glob(os.path.join(run_dir, "explore_ros*.log*")):
        for line in _open(p):
            m = re.search(r"\[(\d{10}\.\d+)\] \[([\w.]+)\]: (.*)", line)
            if m and a - 1 <= float(m.group(1)) <= b + 6 and KEY.search(m.group(3)):
                out.append((round(float(m.group(1)) - a, 1), m.group(2), m.group(3)[:100]))
    return sorted(out)


def refusals(run_dir, a, b):
    c = Counter()
    for p in glob.glob(os.path.join(run_dir, "explore_robot.log*")):
        for line in _open(p):
            m = re.match(r"(\d+\.\d+) WARNING server refused \((\w+)\) for (\S+): (\w+)", line)
            if m and a <= float(m.group(1)) <= b:
                c[f"{m.group(3)} {m.group(4)} {m.group(2)}"] += 1
    return dict(c)


def surface_cm(x, y):
    """Nearest true surface to the chassis at the goal point, any heading."""
    return round(min(truth(_world(x, y, h))[1] for h in range(0, 360, 30)), 1)


def report(path):
    d = json.load(open(path))
    run_dir = d.get("run_dir") or os.path.dirname(path)
    xy = {name: (x, y) for name, x, y in ng.HOME_GOALS}
    out = []
    for k, g in enumerate(d.get("goals") or []):
        if g["state"] == "succeeded":
            continue
        a, b = g["started_at"], g["ended_at"]
        s = [r for r in d["series"] if a <= r[0] <= b and r[4] is not None]
        moved = round(math.hypot(s[-1][4] - s[0][4], s[-1][5] - s[0][5]), 2) if s else None
        gx, gy = xy[g["name"]]
        out.append({"goal": k + 1, "name": g["name"], "state": g["state"],
                    "seconds": g.get("seconds"), "end_error_m": g.get("end_error_m"),
                    "reply": g.get("reply"), "moved_m": moved,
                    "inherited": k == 0 and moved is not None and moved < 0.05,
                    "goal_point_gap_cm": surface_cm(gx, gy),
                    "nav2": nav2_lines(run_dir, a, b)[:14], "refused": refusals(run_dir, a, b)})
    return out


if __name__ == "__main__":
    for p in sys.argv[1:]:
        print(os.path.basename(os.path.dirname(p)) or p)
        for r in report(p):
            print(json.dumps(r, indent=1))
