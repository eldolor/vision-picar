"""
evaluations/explore-365/harness.py -- 3.65 phase 2: one `explore` mission in
a usable ProcTHOR house, as tests/procthor_sweep.py runs it (FakeNav for
nav2), with the goal choice given as weights (None = the rule) and the
metres driven measured: each succeeded nav goal's shortest path on the true
floor, plus what the verbs drove (`MockRobot`'s odometer).
"""
import math
import sys
from collections import deque

sys.path.insert(0, ".")
from brain.goal_policy import LinearGoalPolicy            # noqa: E402
from control.mission_runner import MissionRunner          # noqa: E402
from sim.maps import build_world                          # noqa: E402
from sim.maps.procthor import TARGET                      # noqa: E402
from sim.mock_robot import MockRobot                      # noqa: E402
from sim.mock_world import MockWorld                      # noqa: E402
from tests.procthor_sweep import MAX_STEPS, TICK_CAP      # noqa: E402
from sim.grid_world import CELL_WALL                       # noqa: E402
from tests.test_explore import CELL, FakeNav              # noqa: E402

GOAL_COST, METRE_COST = 0.02, 0.01   # 3.65 phase 2's reward, fixed before training
# A mission that raises scores far below any miss (200 goals and 2 km driven
# is -25), so the trainer can never learn to crash.
CRASH_COST = 100.0


class MeteredNav(FakeNav):
    """FakeNav that adds each succeeded goal's shortest true-floor path."""

    def __init__(self, robot):
        super().__init__(robot)
        self.metres = 0.0

    def _path_cells(self, a, b):
        g = self.grid
        dist, q = {a: 0}, deque([a])
        while q:
            c = q.popleft()
            if c == b:
                return dist[c]
            for n in ((c[0] + 1, c[1]), (c[0] - 1, c[1]), (c[0], c[1] + 1), (c[0], c[1] - 1)):
                if n not in dist and g._cell(*n) != CELL_WALL and n not in g.objects:
                    dist[n] = dist[c] + 1
                    q.append(n)
        return 0

    def get_goal(self):
        g = self.grid
        start = (g.robot_x, g.robot_y)
        active = bool(self.goal and self.goal["state"] == "active")
        out = super().get_goal()
        if active and self.goal["state"] == "succeeded":
            to = (math.floor(self.goal["x_m"] / CELL), math.floor(self.goal["y_m"] / CELL))
            self.metres += self._path_cells(start, to) * CELL
        return out


def run_house(index: int, weights=None, policy=None) -> dict:
    """`weights` -> a LinearGoalPolicy; `policy` -> any object with
    `best`/`score` (the network, or train_net.py's sampler). Neither: the rule."""
    grid = build_world(f"procthor:val:{index}")
    robot = MockRobot(grid, render=False)
    nav = MeteredNav(robot)
    runner = MissionRunner(robot, target_object=TARGET, policy="explore", navigator=nav,
                           world=MockWorld(grid), max_steps=MAX_STEPS,
                           clock=lambda: grid.sim_time, idle=robot.pass_time)
    if weights is not None:
        runner.agent.goal_policy = LinearGoalPolicy(weights)
    if policy is not None:
        runner.agent.goal_policy = policy
    row = {"index": index}
    try:
        runner.start()
        ticks = 0
        while runner.tick() and ticks < TICK_CAP:
            ticks += 1
        s = runner.status()
        row.update(outcome=s["outcome"], ticks=ticks, capped=ticks >= TICK_CAP,
                   goals=s.get("explore", {}).get("goals_sent") or 0,
                   metres=round(nav.metres + robot._path_m, 2))
    except Exception as e:  # noqa: BLE001 -- counted, never hidden
        row.update(outcome="exception", error=f"{type(e).__name__}: {e}", goals=0, metres=0.0)
    if row["outcome"] == "exception":
        # Never better than a real miss, or the trainer would learn to crash.
        row["reward"] = -CRASH_COST
    else:
        row["reward"] = round((row["outcome"] == "found") - GOAL_COST * row["goals"]
                              - METRE_COST * row["metres"], 4)
    return row
