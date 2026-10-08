"""
tests/test_offline_arrival.py

3.47 (`docs/plans/ros-alignment/3.47-offline-arrival.md`): **an arrival the
cloud could not check ends `arrived_unconfirmed`, never `found` and no longer
`failed`.**

Since handoff 1a only a cloud yes on the arrival frame makes `found`. When
the cloud is unreachable the confirmation raises, B3.2 counts it, and before
3.47 the mission ended `failed` with the robot parked at the target (12 / 12
clear starts, sync and async, measured 2026-10-07). A cloud that answers
"no" is a refusal and is untouched.

Through the whole mission path (`MissionRunner` -> agent -> `robot/safety.py`
-> `MockRobot`, scaled house), with the cloud faked. Ground truth (distance
to the target) is read by the test only.
"""

import logging
import math

import pytest

from brain.perceive import FrameReportedPipeline
from brain.tiered import TRIGGER_ARRIVAL, TieredVision
from control.mission_runner import (
    ARRIVED_UNCONFIRMED, FAILED, FOUND, MissionRunner)
from sim.mock_robot import MockRobot
from tests.conftest import mock_world_for
from tests.test_arrival_confirmation import _disagreeing_cloud
from tests.test_bearing_turns import (
    ARRIVED_CELLS, CLEAR_STARTS, GOAL, TARGET, _build, _quiet_cloud)


@pytest.fixture(autouse=True)
def _quiet_logs():
    logging.disable(logging.WARNING)
    yield
    logging.disable(logging.NOTSET)


def _dead_cloud(frame):
    raise ConnectionError("no network")


def _run(cloud_for, start, async_cloud=False, max_calls=None, policy="tiered"):
    """`cloud_for(tier)` builds the cloud, so a cloud can read the tier's
    own counters (criterion 4's blip)."""
    x, y, off = start
    grid = _build()
    grid.x, grid.y = x, y
    grid.theta = math.atan2(GOAL[1] - y, GOAL[0] - x) + math.radians(off)
    robot = MockRobot(grid, render=False)
    holder = {}
    tier = TieredVision(FrameReportedPipeline(TARGET), lambda f: holder["cloud"](f),
                        steer_on_sight=True, hold_goal=True,
                        async_cloud=async_cloud, max_calls=max_calls)
    holder["cloud"] = cloud_for(tier)
    runner = MissionRunner(robot, target_object=TARGET, max_steps=60, policy=policy,
                           vision_fn=tier if policy == "tiered" else holder["cloud"],
                           world=mock_world_for(robot))
    runner.start()
    while runner.tick():
        pass
    return runner, math.dist((grid.x, grid.y), GOAL)


@pytest.mark.parametrize("async_cloud", [False, True], ids=["sync", "async"])
def test_1_2_an_unreachable_cloud_at_arrival_ends_arrived_unconfirmed(async_cloud):
    """Criteria 1 and 2: every start that reaches the target ends
    `arrived_unconfirmed` (baseline: 0 of 12, all `failed`), at the target by
    ground truth, and none ends `found`."""
    outcomes = []
    for start in CLEAR_STARTS:
        runner, cells = _run(lambda tier: _dead_cloud, start, async_cloud=async_cloud)
        status = runner.status()
        outcomes.append(status["outcome"])
        assert cells <= ARRIVED_CELLS, (start, cells, "never reached -- proves nothing")
        assert status["outcome"] == ARRIVED_UNCONFIRMED, (start, status["log_tail"][-2:])
        assert not runner.memory.found
        assert status["arrival"]["state"] == "unconfirmed", status["arrival"]
        assert status["error"] is None, "not a failure: the robot did its part"
        assert "identity unconfirmed" in status["log_tail"][-1]
    assert outcomes.count(ARRIVED_UNCONFIRMED) == len(CLEAR_STARTS) == 12
    assert FOUND not in outcomes


def test_3_a_cloud_that_says_no_is_a_refusal_not_an_outage():
    """Criterion 3: a disagreeing cloud never ends `arrived_unconfirmed`."""
    for start in CLEAR_STARTS:
        runner, cells = _run(lambda tier: _disagreeing_cloud, start)
        status = runner.status()
        assert cells <= ARRIVED_CELLS, (start, cells)
        assert status["outcome"] not in (FOUND, ARRIVED_UNCONFIRMED), (start, status["outcome"])
        assert status["arrival"]["state"] == "refused"


def test_3_a_spent_call_cap_is_not_reported_as_an_outage():
    runner, cells = _run(lambda tier: _quiet_cloud, CLEAR_STARTS[0], max_calls=0)
    status = runner.status()
    assert cells <= ARRIVED_CELLS, cells
    assert status["outcome"] not in (FOUND, ARRIVED_UNCONFIRMED), status["outcome"]
    assert "cap" in (status["arrival"].get("identity") or {}).get("reason", ""), status["arrival"]


def _blip(fail_first):
    """A cloud that fails the first `fail_first` arrival confirmations, then
    answers yes. The tier counts the trigger before it calls the cloud."""
    def cloud_for(tier):
        def cloud(frame):
            if 0 < tier.stats.triggers.get(TRIGGER_ARRIVAL, 0) <= fail_first:
                raise ConnectionError("dropped")
            return _quiet_cloud(frame)
        return cloud
    return cloud_for


@pytest.mark.parametrize("fail_first", [1, 2])
def test_4_a_blip_shorter_than_the_budget_still_ends_found(fail_first):
    """Criterion 4: B3.2's budget is 3; fewer failures than that retry."""
    for start in CLEAR_STARTS:
        runner, _ = _run(_blip(fail_first), start)
        status = runner.status()
        assert status["outcome"] == FOUND, (start, status["log_tail"][-3:])
        assert status["tier"]["stats"]["triggers"][TRIGGER_ARRIVAL] == fail_first + 1


def test_5_a_dead_cloud_with_no_arrival_still_fails():
    """Criterion 5: the `vision` policy has no local arrival to report."""
    runner, _ = _run(lambda tier: _dead_cloud, CLEAR_STARTS[0], policy="vision")
    status = runner.status()
    assert status["outcome"] == FAILED, status["outcome"]
    assert status["arrival"] is None or status["arrival"].get("state") != "unconfirmed"
