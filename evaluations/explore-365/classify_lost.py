"""
evaluations/explore-365/classify_lost.py -- 3.65 phase 2 criterion 2: why
each test house the rule finds is missed under the network. Runs the house
under the rule and under `brain/explore_goal_net.json` with the harness's
settings and reports, for each: outcome, goals, sightings of the target
(bearing, true range), the closest the robot came to it, and whether explore
counted the target's cell as seen by the camera.

    python evaluations/explore-365/classify_lost.py HOUSE [HOUSE ...]
"""
import logging
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, ".")
logging.disable(logging.WARNING)

import brain.explore as E                                   # noqa: E402
from brain.goal_policy import NetGoalPolicy                 # noqa: E402
from control.mission_runner import MissionRunner            # noqa: E402
from harness import MeteredNav                              # noqa: E402
from sim.maps import build_world                            # noqa: E402
from sim.maps.procthor import TARGET                        # noqa: E402
from sim.mock_robot import MockRobot                        # noqa: E402
from sim.mock_world import MockWorld                        # noqa: E402
from tests.procthor_sweep import MAX_STEPS, TICK_CAP        # noqa: E402


def run(index, policy):
    grid = build_world(f"procthor:val:{index}")
    (cx, cy), = [c for c, v in grid.objects.items() if v == TARGET]
    tx, ty = (cx + 0.5) * 0.3, (cy + 0.5) * 0.3
    robot = MockRobot(grid, render=False)
    sightings, closest = [], [99.0]
    # Every tick's frame, through _choose (which every step calls), not
    # _sighting (skipped while queued looks run -- where glimpses happen).
    orig = E.ExploreAgent._choose

    def spy(self, frame, scene):
        d = math.hypot(grid.x * 0.3 - tx, grid.y * 0.3 - ty)
        closest[0] = min(closest[0], d)
        if any(x.get("label") == TARGET for x in frame.get("detections") or ()):
            sightings.append((self.goals_sent, round(d, 2)))
        return orig(self, frame, scene)
    E.ExploreAgent._choose = spy
    try:
        r = MissionRunner(robot, target_object=TARGET, policy="explore", navigator=MeteredNav(robot),
                          world=MockWorld(grid), max_steps=MAX_STEPS, clock=lambda: grid.sim_time,
                          idle=robot.pass_time)
        if policy is not None:
            r.agent.goal_policy = policy
        r.start()
        ticks = 0
        while r.tick() and ticks < TICK_CAP:
            ticks += 1
        a = r.agent
        seen = any(math.hypot(x - tx, y - ty) < 0.2 for x, y in a.seen)
        s = r.status()
    finally:
        E.ExploreAgent._choose = orig
    return {"outcome": s["outcome"], "goals": a.goals_sent, "closest_m": round(closest[0], 2),
            "target_cell_seen": seen, "sightings (goal, range m)": sightings[:6],
            "n_sightings": len(sightings), "explore": {k: v for k, v in s.get("explore", {}).items()
                                                       if k in ("last_event", "goals_failed", "camera_seen_m2")}}


if __name__ == "__main__":
    net = NetGoalPolicy.load()
    for h in map(int, sys.argv[1:]):
        print(h, "rule   ", run(h, None))
        print(h, "network", run(h, net))
