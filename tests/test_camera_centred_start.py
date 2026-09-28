"""
tests/test_camera_centred_start.py

`PLAN-ros-alignment.md` 3.20 -- a mission starts with the camera centred.
A mission that ends mid-peek leaves the camera panned; the next policy's
first frame, depth grid and scene are then cast 90 degrees off its heading.
One test per criterion, written before the fix and confirmed red first.
"""

import json
from pathlib import Path

import pytest

from control.mission_runner import MissionRunner
from robot.interface import Preempted
from sim.maps import build_world
from sim.mock_robot import MockRobot
from tests.conftest import mock_world_for

TRACE = Path(__file__).parent / "data" / "frontier_trace_centred.json"
PANS = (-1.0, -0.5, 0.5, 1.0)


class FirstReadRobot(MockRobot):
    """Records where the camera pointed at the policy's first sensor read."""

    first_read_pan = None

    def _saw(self):
        if self.first_read_pan is None:
            self.first_read_pan = self.world.pan

    def get_camera_frame(self):
        self._saw()
        return super().get_camera_frame()

    def get_depth_grid(self):
        self._saw()
        return super().get_depth_grid()

    def get_distance(self):
        self._saw()
        return super().get_distance()


def _scene(frame):
    return {"obstacles_ahead": [], "free_space": "clear", "doorway_visible": False,
            "important_objects": [], "safest_direction": "FORWARD"}


def _runner(pan, policy="frontier", vision_fn=None, robot_cls=FirstReadRobot, max_steps=3):
    robot = robot_cls(build_world("starter_house"), render=False)
    robot.world.pan = pan
    return MissionRunner(robot, target_object="red backpack", max_steps=max_steps,
                         policy=policy, vision_fn=vision_fn, world=mock_world_for(robot)), robot


@pytest.mark.parametrize("pan", PANS)
@pytest.mark.parametrize("policy", ["frontier", "vision"])
def test_criterion_1_the_first_decision_sees_a_centred_camera(pan, policy):
    runner, robot = _runner(pan, policy, _scene if policy == "vision" else None)
    runner.start()
    runner.tick()
    assert robot.first_read_pan == 0, (
        f"{policy}: the first decision read its sensors with the camera at pan "
        f"{robot.first_read_pan} (left behind at {pan})")


def test_criterion_2_centring_costs_no_step_and_no_vision_call():
    calls = []

    def counting(frame):
        calls.append(frame)
        return _scene(frame)

    runner, _ = _runner(-1.0, "vision", counting, max_steps=5)
    runner.start()
    runner.tick()
    assert runner.status()["step"] == 1
    assert len(calls) == 1, "one tick, one vision call -- centring is not a decision"


def test_criterion_3_a_centred_start_is_unchanged():
    pinned = json.loads(TRACE.read_text())
    runner, _ = _runner(0.0, max_steps=150, robot_cls=MockRobot)
    runner.start()
    while runner.tick():
        pass
    assert runner.status()["outcome"] == pinned["outcome"]
    assert [h.action for h in runner.agent.history] == pinned["actions"]


def test_criterion_4_a_refused_centring_ends_the_mission_like_any_refused_move():
    class Outranked(MockRobot):
        def look_center(self):
            raise Preempted("twin-dpad holds the robot")

    runner, _ = _runner(-1.0, robot_cls=Outranked)
    runner.start()
    assert runner.tick() is False
    assert runner.status()["outcome"] == "preempted"
