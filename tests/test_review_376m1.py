"""
tests/test_review_376m1.py

`PLAN-ros-alignment.md` 3.76, step M1: the bugs the review of the
simulator's core (`sim/grid_world.py`, `mock_robot.py`, `mock_world.py`,
`renderer.py`, `sensors.py`, `movers.py`, whole files) confirmed and fixed
here -- the ones that move no measured number. Each test was written before
its fix and seen to fail on `dev` for the reason it names.
"""

import math
import random

from sim.maps import build_movers, build_world
from sim.mock_robot import MockRobot
from sim.movers import point_to_cell_cells


# ---------- M1.1: the sim's event log grew for the life of the server ----------


def test_the_event_log_is_bounded():
    """Every depth grid, distance read and frame appended to a plain list
    that only tests read -- 20 Hz under the wheel loop's vet, for the life
    of a robot server that runs sim mode around the clock on the Jetson."""
    robot = MockRobot(build_world("scaled_house"), render=False)
    for _ in range(6000):
        robot.get_distance()
    assert len(robot.world.log) <= 2000, len(robot.world.log)


# ---------- M1.2: a mover hopped under the chassis ----------


def _covers(world, cell) -> bool:
    """Would the chassis, where it stands, overlap `cell` if it were solid?"""
    objects = world.objects
    world.objects = {**objects, cell: "probe"}
    try:
        return world.footprint_overlaps(world.x, world.y)
    finally:
        world.objects = objects


def test_a_mover_never_hops_under_the_chassis():
    """`_hop` refused only a cell NEARER the robot's centre than the mover's;
    a neighbour at the same distance can lie under a corner of the
    rectangle (Thermos: 37 in 200,000), and an overlapping pose switches the
    footprint's collision checks off."""
    rng = random.Random(3760)
    found = 0
    for _ in range(20000):
        world = build_world("scaled_house")
        mover = build_movers("scaled_house", "hallway_crossing")[0]
        world.add_mover(mover)
        src, dst = mover.cell, mover.path[1]
        world.x = src[0] + 0.5 + rng.uniform(-0.6, 1.6)
        world.y = src[1] + 0.5 + rng.uniform(0.3, 1.2)
        world.theta = rng.uniform(-math.pi, math.pi)
        if world.footprint_overlaps(world.x, world.y) or not _covers(world, dst):
            continue
        if point_to_cell_cells(world.x, world.y, dst) < point_to_cell_cells(world.x, world.y, src):
            continue                                       # refused already
        found += 1
        world.advance_time(mover.hop_s + 1e-6)
        assert mover.cell == src, (world.x, world.y, world.theta, src, dst)
        assert not world.footprint_overlaps(world.x, world.y)
    assert found, "no pose put the next cell under the chassis"


# ---------- M1.3: /wheels took any command ----------


def test_a_wheel_command_is_capped_as_the_board_caps_it_and_never_non_finite():
    """The car's board scales commands into 2.0 m/s a wheel; the sim
    integrated whatever came, so 1e5 rad/s ran hundreds of thousands of
    substeps on the event loop, and NaN raised on every tick."""
    robot = MockRobot(build_world("scaled_house"), render=False)
    def cmd(state):
        return state["left"]["velocity_rad_s"], state["right"]["velocity_rad_s"]

    left, right = cmd(robot.set_wheel_velocity(1e5, 5e4))
    assert abs(left) * 0.040 <= 2.0 + 1e-9, left
    assert right / left == 0.5
    assert cmd(robot.set_wheel_velocity(float("nan"), float("inf"))) == (0.0, 0.0)
    robot.step(0.05)                                      # no raise
