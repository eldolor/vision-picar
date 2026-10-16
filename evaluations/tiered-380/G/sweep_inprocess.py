"""
evaluations/tiered-380/G/sweep_inprocess.py

3.80.G, a development check before the live run (not a criterion): the 21
criterion-2 houses in process, `tier_search: explore`, with FakeNav for nav2
(a goal succeeds iff the true floor connects it) and a local tier that says
`score` to every question, the cloud faked and counted. Does explore plus the
tiers' sightings and last approach reach the true target?

    .venv/bin/python evaluations/tiered-380/G/sweep_inprocess.py [score]
"""

import json
import logging
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from brain.perceive import FrameReportedPipeline  # noqa: E402
from brain.tiered import TieredVision  # noqa: E402
from control.mission_runner import MissionRunner  # noqa: E402
from sim.maps import build_world  # noqa: E402
from sim.mock_robot import MockRobot  # noqa: E402
from sim.mock_world import MockWorld  # noqa: E402
from tests.test_explore import FakeNav  # noqa: E402
from tests.test_tiered_explore import Cloud, Local  # noqa: E402

sys.path.insert(0, str(ROOT / "evaluations/tiered-380/E"))
from pilot import SWEEP_HOUSES  # noqa: E402

TARGET = "red backpack"
CELL_M = 0.3


def target_m(grid):
    cells = [c for c, n in grid.objects.items() if n == TARGET]
    return min(math.dist((grid.x, grid.y), (cx + 0.5, cy + 0.5)) for cx, cy in cells) * CELL_M


def run(house, score):
    grid = build_world(house)
    robot = MockRobot(grid, render=False)
    cloud, confirm, local = Cloud(), Cloud(), Local(score)
    tier = TieredVision(FrameReportedPipeline(TARGET), cloud, confirm_vision_fn=confirm,
                        local_identity_fn=local, local_sighting_fn=local)
    runner = MissionRunner(robot, target_object=TARGET, policy="tiered", vision_fn=tier,
                           tier_search="explore", navigator=FakeNav(robot),
                           world=MockWorld(grid), mission_time_limit_s=1200,
                           clock=lambda: grid.sim_time, idle=robot.pass_time)
    runner.start()
    ticks, arrived_at = 0, None
    while runner.tick() and ticks < 8000:
        ticks += 1
        if arrived_at is None and tier.stats.local_calls:
            arrived_at = round(target_m(grid), 2)
    s = runner.status()
    if arrived_at is None and tier.stats.local_calls:
        arrived_at = round(target_m(grid), 2)    # the check on the last tick
    return {"house": house, "outcome": s["outcome"], "sim_s": round(grid.sim_time),
            "goals": s["explore"]["goals_sent"], "arrival_checks": tier.stats.local_calls,
            "first_check_target_m": arrived_at, "end_target_m": round(target_m(grid), 2),
            "close_ins": runner.agent.close_ins, "cloud_calls": cloud.calls + confirm.calls,
            "tail": s["log_tail"][-1:]}


def main(score=0.9):
    logging.disable(logging.WARNING)
    t, rows = time.time(), []
    for h in SWEEP_HOUSES:
        rows.append(run(h, float(score)))
        print(json.dumps(rows[-1]), flush=True)
    arr = sum(1 for r in rows if r["first_check_target_m"] is not None
              and r["first_check_target_m"] <= 0.9)
    out = {"score": float(score), "missions": len(rows),
           "arrivals_at_true_target": arr,
           "found": sum(r["outcome"] == "found" for r in rows),
           "cloud_calls": sum(r["cloud_calls"] for r in rows), "rows": rows,
           "wall_s": round(time.time() - t)}
    Path(__file__).with_name(f"inprocess-{score}.json").write_text(json.dumps(out, indent=1) + "\n")
    print(json.dumps({k: v for k, v in out.items() if k != "rows"}))


if __name__ == "__main__":
    main(*sys.argv[1:])
