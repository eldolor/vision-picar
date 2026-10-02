"""
tests/test_halt_gate_verbs.py

Found during PLAN-ros-alignment.md 3.31: a mission's verbs were UNGUARDED.
`MissionRunner` hands its agent a `_HaltGate` around the robot, and the gate
inherited `RobotInterface.verb_plan()`'s "no plan" -- so `SafetyController`
vetted a FORWARD once and drove the whole cell, the pre-3.22 behaviour 3.22
decided against ("the runner lives in SafetyController, so in-process
agents and the server behave alike"). Over HTTP the robot server re-vets
every verb with the guard, so the deployed path was never affected; every
in-process mission since 3.22 was.

Judged on ground truth, the way 3.18 judges: the chassis' gap to what it
drove at, from tests/footprint_sweep.py's own geometry.
"""

import math

from control.mission_runner import _HaltGate
from robot.safety import SafetyController
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.movers import Mover
from tests.footprint_sweep import CELL_CM, chassis, gap, square


def _forward_at_a_person(wrap):
    g = build_world("scaled_house")
    g.x, g.y, g.theta = 14.5, 4.5, 0.0
    g.add_mover(Mover("person", [(16, 4), (16, 5)], hop_s=1e9))
    robot = MockRobot(g, render=False)
    driven = _HaltGate(robot, lambda: True) if wrap else robot
    SafetyController(driven, 20.0).check_and_execute("FORWARD")
    return gap(chassis(g.x, g.y, g.theta), square(16, 4)) * CELL_CM


def test_a_verb_through_the_mission_gate_is_guarded_like_one_without_it():
    raw, gated = _forward_at_a_person(False), _forward_at_a_person(True)
    assert gated >= 18.0, f"through the gate the robot ended {gated:.1f} cm from the person"
    assert math.isclose(raw, gated, abs_tol=0.5)


def test_the_gate_still_refuses_a_verb_once_the_mission_is_over():
    g = build_world("scaled_house")
    robot = MockRobot(g, render=False)
    gate = _HaltGate(robot, lambda: False)
    import pytest
    from control.mission_runner import MissionHalted
    with pytest.raises(MissionHalted):
        SafetyController(gate, 20.0).check_and_execute("FORWARD")
