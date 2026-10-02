"""
tests/explore_sweep.py

The instrument for `PLAN-ros-alignment.md` 3.31 -- a search that uses the
map. Shared by `tests/test_explore.py` (a pinned sample) and
`tests/demo_explore_sweep.py` (the full sweep, printed).

Missions run end to end through the real path -- `MissionRunner` -> the
policy -> `robot/safety.py` -> `MockRobot` -- and are judged on GROUND TRUTH
read here, by the harness, never by the policy: whether the target was
reached, how much of the reachable floor the map saw, and the chassis'
clearance (`tests/footprint_sweep.py`'s geometry).
"""

import logging
import math
import random

from brain.perceive import FrameReportedPipeline
from brain.tiered import TieredVision
from control.mission_runner import MissionRunner
from sim import renderer
from sim.maps import build_world
from sim.mock_robot import MockRobot
from tests import footprint_sweep as fs
from tests.conftest import mock_world_for
from tests.test_bearing_turns import TARGET, _quiet_cloud

HOME = "home_first_floor"
STARTS_SEED = 331
ARRIVED_CELLS = 1.05          # R1's ground-truth arrival bar


def starts(house=HOME, n=20, seed=STARTS_SEED):
    """`n` seeded starts on free floor, each with a seeded heading."""
    rng = random.Random(f"explore-{house}-{seed}")
    return [(x, y, rng.uniform(0, 360)) for x, y in fs.starts(house, n, seed)]


def _target_cell(world):
    for cell, name in world.objects.items():
        if name == TARGET:
            return cell
    return None


def reachable_free(world, x, y):
    """Ground truth: the free cells 4-connected to the start cell, through
    free floor (walls and solid objects block). The denominator of coverage."""
    blocked = set(world.solid_cells)
    start = (int(x), int(y))
    seen, todo = {start}, [start]
    while todo:
        cx, cy = todo.pop()
        for nx, ny in ((cx + 1, cy), (cx - 1, cy), (cx, cy + 1), (cx, cy - 1)):
            if (nx, ny) in seen or (nx, ny) in blocked:
                continue
            if renderer._cell_at(world.layout, nx, ny) == "#":
                continue
            seen.add((nx, ny))
            todo.append((nx, ny))
    return seen


def run_mission(policy, start, house=HOME, target_present=True, max_steps=200, **runner_kw):
    """One mission, end to end. Returns the outcome and ground-truth measures."""
    x, y, heading_deg = start
    world = build_world(house)
    if not target_present:
        cell = _target_cell(world)
        if cell is not None:
            del world.objects[cell]
    world.x, world.y, world.theta = x, y, math.radians(heading_deg)
    robot = MockRobot(world, render=False)
    mworld = mock_world_for(robot)
    kw = dict(runner_kw)
    if policy == "tiered":
        kw["vision_fn"] = TieredVision(FrameReportedPipeline(TARGET), _quiet_cloud,
                                       steer_on_sight=True, hold_goal=True)
    runner = MissionRunner(robot, target_object=TARGET, max_steps=max_steps,
                           policy=policy, world=mworld, **kw)
    reach = reachable_free(world, x, y)
    min_gap_cm, contact = math.inf, False
    logging.disable(logging.WARNING)
    try:
        runner.start()
        while runner.tick():
            _, gap, pen = fs.truth(world)
            min_gap_cm = min(min_gap_cm, gap)
            contact = contact or pen > 0
    finally:
        logging.disable(logging.NOTSET)
    status = runner.status()
    m = mworld.get_map()
    seen_free = set()
    for k, v in enumerate(m["cells"]):
        if v == 0:
            seen_free.add((k % m["width"], k // m["width"]))
    goal = _target_cell(world)
    cells_to_goal = (math.dist((world.x, world.y), (goal[0] + 0.5, goal[1] + 0.5))
                     if goal else None)
    return {
        "policy": policy, "start": (round(x, 2), round(y, 2), round(heading_deg)),
        "outcome": status["outcome"], "steps": status.get("step"),
        "driven_m": robot.get_odometry().get("distance_m"),
        "arrived": cells_to_goal is not None and cells_to_goal <= ARRIVED_CELLS,
        "coverage": len(seen_free & reach) / max(1, len(reach)),
        "min_gap_cm": min_gap_cm, "contact": contact,
    }
