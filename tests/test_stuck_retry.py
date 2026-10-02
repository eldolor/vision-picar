"""
tests/test_stuck_retry.py

PLAN-ros-alignment.md 3.31, criterion 4's mechanism: the stuck detector
shares the explore policy's retry rule. Five refused FORWARDs no longer end
a mission at once -- it backs off, waits out a cooldown, and tries again,
ending `blocked` only after RETRY_LIMIT such episodes. A wall stays a wall
(the jamb starts in tests/test_bearing_turns.py still end `blocked`); a
person in a doorway moves on.
"""

import math

from brain.frontier import RETRY_COOLDOWN_S, RETRY_LIMIT
from brain.perceive import FrameReportedPipeline
from brain.tiered import TieredVision
from control.mission_runner import BLOCKED, FOUND, MissionRunner
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.movers import Mover
from tests.conftest import mock_world_for
from tests.test_bearing_turns import _quiet_cloud

TARGET = "red backpack"


def _facing_the_kitchen_door(mover=None, **runner_kw):
    grid = build_world("scaled_house")
    # The backpack moved onto the door's middle row, so the straight line to
    # it runs through the middle of the door -- where the person stands --
    # and not past a jamb, which would be a wall and correctly `blocked`.
    grid.move_object((23, 5), (23, 4))
    grid.x, grid.y, grid.theta = 14.5, 4.5, 0.0       # hallway, facing east
    if mover:
        grid.add_mover(mover)
    robot = MockRobot(grid, render=False)
    tier = TieredVision(FrameReportedPipeline(TARGET), _quiet_cloud,
                        steer_on_sight=True, hold_goal=True)
    runner = MissionRunner(robot, target_object=TARGET, max_steps=200, policy="tiered",
                           vision_fn=tier, world=mock_world_for(robot), **runner_kw)
    runner.start()
    while runner.tick():
        robot.pass_time(0.25)       # the brain server's tick interval
    return runner.status(), grid


def test_a_person_who_moves_on_is_waited_for_not_called_blocked():
    """Someone in the middle of the kitchen door for 60 s, then gone into
    the kitchen: the mission backs off, waits, and arrives."""
    person = Mover("person", [(16, 4), (17, 4), (18, 4), (18, 3)],
                   hop_s=1.0, start_s=60.0, loop=False)
    status, grid = _facing_the_kitchen_door(person)
    assert status["outcome"] == FOUND, status["log_tail"][-6:]
    assert 1 <= status["stuck_episodes"] < RETRY_LIMIT


def test_someone_who_never_moves_is_blocked_after_the_limit():
    person = Mover("person", [(16, 4), (16, 5)], hop_s=1e9, start_s=1e9)
    status, grid = _facing_the_kitchen_door(person)
    assert status["outcome"] == BLOCKED
    assert status["stuck_episodes"] == RETRY_LIMIT
    # it waited out RETRY_LIMIT - 1 cooldowns before believing it
    assert grid.sim_time >= (RETRY_LIMIT - 1) * RETRY_COOLDOWN_S


def test_a_cooldown_is_reported_while_it_runs():
    person = Mover("person", [(16, 4), (16, 5)], hop_s=1e9, start_s=1e9)
    grid = build_world("scaled_house")
    grid.move_object((23, 5), (23, 4))
    grid.x, grid.y, grid.theta = 14.5, 4.5, 0.0
    grid.add_mover(person)
    robot = MockRobot(grid, render=False)
    tier = TieredVision(FrameReportedPipeline(TARGET), _quiet_cloud,
                        steer_on_sight=True, hold_goal=True)
    runner = MissionRunner(robot, target_object=TARGET, max_steps=200, policy="tiered",
                           vision_fn=tier, world=mock_world_for(robot))
    runner.start()
    while runner.tick() and runner.status()["cooling_down_s"] is None:
        robot.pass_time(0.25)
    s = runner.status()
    assert s["running"] and 0 < s["cooling_down_s"] <= RETRY_COOLDOWN_S
    assert s["stuck_episodes"] == 1
