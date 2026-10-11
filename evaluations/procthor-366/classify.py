"""
evaluations/procthor-366/classify.py -- why explore missed the target in
3.66's sweep: every sighting (bearing, lidar range, the range to the true
target), the robot's closest approach, and whether explore counted the
target's cell as seen. Reads sweep.json beside itself.
"""
import json
import logging
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
logging.disable(logging.WARNING)

import brain.explore as E  # noqa: E402
from control.mission_runner import MissionRunner  # noqa: E402
from sim.maps import build_world  # noqa: E402
from sim.maps.procthor import TARGET  # noqa: E402
from sim.mock_robot import MockRobot  # noqa: E402
from sim.mock_world import MockWorld  # noqa: E402
from tests.test_explore import FakeNav  # noqa: E402

HERE = os.path.dirname(__file__)


def classify(index):
    grid = build_world(f"procthor:val:{index}")
    (cx, cy), = [c for c, v in grid.objects.items() if v == TARGET]
    tx, ty = (cx + 0.5) * 0.3, (cy + 0.5) * 0.3
    robot = MockRobot(grid, render=False)
    sightings, closest = [], [99.0]
    orig = E.ExploreAgent._sighting

    def spy(self, frame):
        out = orig(self, frame)
        closest[0] = min(closest[0], math.hypot(grid.x * 0.3 - tx, grid.y * 0.3 - ty))
        hits = [d for d in frame.get("detections") or () if d.get("label") == TARGET]
        if hits:
            true_r = math.hypot(grid.x * 0.3 - tx, grid.y * 0.3 - ty)
            sightings.append({"bearing": hits[0].get("bearing_deg"), "true_range": round(true_r, 2),
                              "est_err_m": None if out is None else round(math.hypot(out[0] - tx, out[1] - ty), 2)})
        return out
    E.ExploreAgent._sighting = spy
    try:
        r = MissionRunner(robot, target_object=TARGET, policy="explore", navigator=FakeNav(robot),
                          world=MockWorld(grid), max_steps=200, clock=lambda: grid.sim_time,
                          idle=robot.pass_time)
        r.start()
        while r.tick():
            pass
        agent = r.agent
        seen = any(math.hypot(x - tx, y - ty) < 0.2 for x, y in agent.seen)
    finally:
        E.ExploreAgent._sighting = orig
    return {"index": index, "outcome": r.status()["outcome"], "sightings": sightings,
            "closest_m": round(closest[0], 2), "target_cell_seen": seen}


if __name__ == "__main__":
    rows = json.load(open(os.path.join(HERE, "sweep.json")))
    out = [classify(r["index"]) for r in rows if r["outcome"] != "found"]
    json.dump(out, open(os.path.join(HERE, "misses.json"), "w"), indent=1)
    for m in out:
        s = m["sightings"]
        print(m["index"], m["outcome"], "closest", m["closest_m"], "seen-cell", m["target_cell_seen"],
              "sightings", len(s), [(x["bearing"], x["true_range"], x["est_err_m"]) for x in s[:4]])
