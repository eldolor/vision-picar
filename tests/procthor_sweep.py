"""
tests/procthor_sweep.py -- 3.66 criterion 5: the `explore` policy over the
first N usable ProcTHOR houses, in process, with `tests/test_explore.py`'s
`FakeNav` standing in for nav2 (a goal succeeds iff the true floor connects
it). A statement about the policy's decisions in houses it was not tuned
in, not about driving.

    python -m tests.procthor_sweep [N] [START]  # default 100 from usable house 0

3.70 M5 sweeps usable houses 200-299 (`100 200`) and records the median
goals to Found beside them. Not time: FakeNav moves the robot in one hop and
charges 2 s a goal, so the sim clock here is no driving time.
"""

import json
import statistics
import sys
import time

from control.mission_runner import FOUND, MissionRunner
from sim.maps import build_world
from sim.maps.procthor import TARGET, usable
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests.test_explore import FakeNav

MAX_STEPS = 200
TICK_CAP = 5000


def run_house(index: int) -> dict:
    grid = build_world(f"procthor:val:{index}")
    robot = MockRobot(grid, render=False)
    nav = FakeNav(robot)
    runner = MissionRunner(robot, target_object=TARGET, policy="explore", navigator=nav,
                           world=MockWorld(grid), max_steps=MAX_STEPS,
                           clock=lambda: grid.sim_time, idle=robot.pass_time)
    row = {"index": index}
    try:
        runner.start()
        ticks = 0
        while runner.tick() and ticks < TICK_CAP:
            ticks += 1
        s = runner.status()
        row.update(outcome=s["outcome"], ticks=ticks, capped=ticks >= TICK_CAP,
                   goals=s.get("explore", {}).get("goals_sent"),
                   tail=s["log_tail"][-3:])
    except Exception as e:  # criterion 5a counts these
        row.update(outcome="exception", error=f"{type(e).__name__}: {e}")
    return row


def sweep(n: int = 100, start: int = 0) -> list:
    return [run_house(i) for i in usable()[start:start + n]]


def main(argv):
    n = int(argv[0]) if argv else 100
    start = int(argv[1]) if len(argv) > 1 else 0
    t = time.time()
    rows = sweep(n, start)
    found = sum(r["outcome"] == FOUND for r in rows)
    clean = sum(r["outcome"] not in ("exception", "running") and not r.get("capped") for r in rows)
    print(json.dumps([r for r in rows if r["outcome"] != FOUND], indent=1))
    print(f"5a ended cleanly: {clean}/{len(rows)}   5b found: {found}/{len(rows)}   "
          f"({time.time() - t:.0f} s)")
    goals = [r["goals"] for r in rows if r["outcome"] == FOUND and r.get("goals") is not None]
    if goals:
        print(f"median goals to found: {statistics.median(goals)}")


if __name__ == "__main__":
    main(sys.argv[1:])
