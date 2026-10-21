"""
evaluations/goal-tap-388/oracle.py -- 3.88 B: a privileged oracle for
explore's goal choice, and one mission run under a chooser.

The oracle sees ground truth: of the candidates explore's pool offers (the
same ones 3.65's learned policy ranks, `ExploreAgent._pool`), it picks the
one whose goal cell has the shortest true floor path to the target. Ties go
to the pool's order, whose first is the rule's own pick.

A `Chooser` replaces `ExploreAgent._learned_goal` for one mission. Every
choice of two or more candidates is recorded with all candidates' features,
their oracle costs, the oracle's pick and the pick made; who drives is the
oracle (`weights=None`) or a linear policy (DAgger's learner). Explore is
otherwise the rule's, as in 3.65's harness.
"""
import fcntl
import math
import os
import subprocess
import sys
import time
import types
from collections import deque

HERE = os.path.dirname(__file__)
ROOT = os.path.join(HERE, "..", "..")
sys.path.insert(0, os.path.join(ROOT, "evaluations", "explore-365"))
sys.path.insert(0, ROOT)
from harness import CRASH_COST, GOAL_COST, METRE_COST, MeteredNav  # noqa: E402
from brain.goal_policy import LinearGoalPolicy                     # noqa: E402
from control.mission_runner import MissionRunner                   # noqa: E402
from sim.grid_world import CELL_WALL                               # noqa: E402
from sim.maps import build_world                                   # noqa: E402
from sim.maps.procthor import TARGET                               # noqa: E402
from sim.mock_robot import MockRobot                               # noqa: E402
from sim.mock_world import MockWorld                               # noqa: E402
from tests.procthor_sweep import MAX_STEPS, TICK_CAP               # noqa: E402
from tests.test_explore import CELL                                # noqa: E402

INF = math.inf


def yield_to_live_runs() -> None:
    """Wait while another session's live run holds the heavy-run lock
    (`tests/demo_explore.heavy_lock`), so a sweep never shares the laptop
    with one. A shared lock is taken and dropped at once: it never makes the
    live run wait more than an instant."""
    common = subprocess.run(["git", "rev-parse", "--git-common-dir"], cwd=ROOT,
                            capture_output=True, text=True).stdout.strip()
    lock = os.path.join(os.path.abspath(os.path.join(ROOT, common)), "heavy-run.lock")
    if not os.path.exists(lock):
        return
    with open(lock) as f:
        while True:
            try:
                fcntl.flock(f, fcntl.LOCK_SH | fcntl.LOCK_NB)
                fcntl.flock(f, fcntl.LOCK_UN)
                return
            except BlockingIOError:
                time.sleep(30)


def distance_field(grid) -> dict:
    """Cells -> 4-connected steps to the target, over the true grid: walls
    and every object other than the target block."""
    targets = [c for c, name in grid.objects.items() if name == TARGET]
    dist = {c: 0 for c in targets}
    q = deque(targets)
    while q:
        c = q.popleft()
        for n in ((c[0] + 1, c[1]), (c[0] - 1, c[1]), (c[0], c[1] + 1), (c[0], c[1] - 1)):
            if n not in dist and grid._cell(*n) != CELL_WALL and n not in grid.objects:
                dist[n] = dist[c] + 1
                q.append(n)
    return dist


def cost_at(dist: dict, x_m: float, y_m: float) -> float:
    """The oracle cost of a goal at (x_m, y_m): its cell's distance, or the
    nearest of its 8 neighbours' when the cell itself is blocked."""
    cx, cy = math.floor(x_m / CELL), math.floor(y_m / CELL)
    if (cx, cy) in dist:
        return float(dist[(cx, cy)])
    near = [dist[(cx + dx, cy + dy)] + 1 for dx in (-1, 0, 1) for dy in (-1, 0, 1)
            if (dx or dy) and (cx + dx, cy + dy) in dist]
    return float(min(near)) if near else INF


def oracle_pick(costs: list) -> int:
    """Lowest cost; ties (and an all-unreachable pool) keep the pool's order."""
    return min(range(len(costs)), key=lambda i: (costs[i], i))


class Chooser:
    """Replaces explore's pooled choice for one mission. `weights=None`: the
    oracle drives; otherwise a linear policy drives and the oracle only
    labels."""

    def __init__(self, dist: dict, weights=None):
        self.dist = dist
        self.learner = None if weights is None else LinearGoalPolicy(weights)
        self.records = []
        self.singles = 0

    def install(self, agent) -> None:
        chooser = self
        # A non-None goal_policy routes explore through its pooled choice.
        agent.goal_policy = self.learner or LinearGoalPolicy([0.0] * 10)

        def _learned_goal(self, m, pose, frontiers, leave, known, now, skipped, rch=None):
            pool, feats = self._pool(m, pose, frontiers, leave, known, now, skipped, rch)
            if not pool:
                return None
            costs = [cost_at(chooser.dist, c.goal[0], c.goal[1]) for _, c, _ in pool]
            o = oracle_pick(costs)
            i = o if chooser.learner is None else chooser.learner.best(feats)
            if len(pool) > 1:
                chooser.records.append({"feats": [[round(x, 5) for x in f] for f in feats],
                                        "costs": [None if c == INF else c for c in costs],
                                        "oracle": o, "pick": i})
            else:
                chooser.singles += 1
            kind, c, rank = pool[i]
            return self._send_pooled(kind, c, rank, now, len(pool), skipped,
                                     learned=round(-costs[i], 3) if costs[i] != INF else None)

        agent._learned_goal = types.MethodType(_learned_goal, agent)


def run(index: int, split: str = "val", weights=None, policy=None, label: bool = True) -> dict:
    """One explore mission in a usable ProcTHOR house, 3.65's harness and
    reward. `label`: a Chooser (oracle drives if `weights` is None, else the
    linear learner) records every choice. `label=False` with `weights` or
    `policy`: 3.65's plain policy path; neither: the rule."""
    grid = build_world(f"procthor:{split}:{index}")
    robot = MockRobot(grid, render=False)
    nav = MeteredNav(robot)
    runner = MissionRunner(robot, target_object=TARGET, policy="explore", navigator=nav,
                           world=MockWorld(grid), max_steps=MAX_STEPS,
                           clock=lambda: grid.sim_time, idle=robot.pass_time)
    chooser = None
    if label:
        chooser = Chooser(distance_field(grid), weights)
        chooser.install(runner.agent)
    elif policy is not None:
        runner.agent.goal_policy = policy
    elif weights is not None:
        runner.agent.goal_policy = LinearGoalPolicy(weights)
    row = {"index": index, "split": split}
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
        row["reward"] = -CRASH_COST
    else:
        row["reward"] = round((row["outcome"] == "found") - GOAL_COST * row["goals"]
                              - METRE_COST * row["metres"], 4)
    if chooser is not None:
        row["choices"] = chooser.records
        row["singles"] = chooser.singles
    return row
