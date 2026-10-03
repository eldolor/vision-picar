"""
tests/test_arrival_confirmation.py

Handoff 2026-10-02 1a (decided by the user): **the cloud confirms identity at
arrival; the lidar decides distance.**

Before this, a tiered mission could end `found` on a wrong object: one local
DETECTED frame steers, the two-frame hysteresis gates only the cloud
triggers, and `brain/arrival.py` declared `found` on local detections plus a
lidar range alone -- the cloud's identity was never asked. Now one paid
cloud call on the arrival frame must say the target is visible, or there is
no `found`. Steering is unchanged.

Through the whole mission path (`MissionRunner` -> `VisionAgent` ->
`robot/safety.py` -> `MockRobot`, scaled house), with the cloud faked. The
"wrong object" is whatever the local tier is locked onto while the cloud says
the target is not in the picture -- which is exactly the disagreement the
rule exists for, whatever the object really is.
"""

import logging
import math

import pytest

from brain.perceive import ABSENT, FrameReportedPipeline, Perception
from brain.tiered import TRIGGER_ARRIVAL, TieredVision
from control.mission_runner import FOUND, MissionRunner
from sim.mock_robot import MockRobot
from tests.conftest import mock_world_for
from tests.test_bearing_turns import (
    ARRIVED_CELLS, CLEAR_STARTS, GOAL, TARGET, _build, _quiet_cloud)


@pytest.fixture(autouse=True)
def _quiet_logs():
    logging.disable(logging.WARNING)
    yield
    logging.disable(logging.NOTSET)


def _disagreeing_cloud(frame):
    """The cloud looks at the arrival frame and says: not the target."""
    scene = _quiet_cloud(frame)
    scene["_navigate"] = {**scene.get("_navigate", {}), "target_visible": False}
    return scene


class _ScriptedRun:
    """absent, absent, then whatever the frame reports -- a run of local
    detections that begins mid-mission, as a false positive does."""

    def __init__(self, absent_first=2):
        self.inner = FrameReportedPipeline(TARGET)
        self.left = absent_first
        self.detector = self.inner.detector

    def perceive(self, frame):
        if self.left > 0:
            self.left -= 1
            return Perception(status=ABSENT, synthesised=True)
        return self.inner.perceive(frame)


def _run(cloud, pipeline=None, start=CLEAR_STARTS[0], max_steps=60):
    x, y, off = start
    grid = _build()
    grid.x, grid.y = x, y
    grid.theta = math.atan2(GOAL[1] - y, GOAL[0] - x) + math.radians(off)
    robot = MockRobot(grid, render=False)
    tier = TieredVision(pipeline or FrameReportedPipeline(TARGET), cloud,
                        steer_on_sight=True, hold_goal=True)
    runner = MissionRunner(robot, target_object=TARGET, max_steps=max_steps,
                           policy="tiered", vision_fn=tier,
                           world=mock_world_for(robot))
    runner.start()
    while runner.tick():
        pass
    return runner.status(), math.dist((grid.x, grid.y), GOAL), tier


def test_the_right_object_still_ends_found():
    status, cells, tier = _run(_quiet_cloud)
    assert status["outcome"] == FOUND, status
    assert cells <= ARRIVED_CELLS
    assert tier.stats.triggers.get(TRIGGER_ARRIVAL) == 1, tier.stats.triggers


def test_a_wrong_object_arrival_is_refused_when_the_cloud_disagrees():
    status, cells, tier = _run(_disagreeing_cloud)
    assert cells <= ARRIVED_CELLS, "the robot never reached the object -- the test proves nothing"
    assert status["outcome"] != FOUND, status
    asked = tier.stats.triggers.get(TRIGGER_ARRIVAL, 0)
    assert asked >= 1, "arrival was never put to the cloud"
    # One paid call per arrival, not one per frame parked in front of it.
    assert asked <= 2, f"{asked} confirmation calls for one arrival"


def test_a_single_false_positive_run_cannot_end_found():
    status, cells, _ = _run(_disagreeing_cloud, pipeline=_ScriptedRun())
    assert cells <= ARRIVED_CELLS, "the robot never reached the object -- the test proves nothing"
    assert status["outcome"] != FOUND, status
    # ... and the same run, with a cloud that agrees, does end found.
    status, _, _ = _run(_quiet_cloud, pipeline=_ScriptedRun())
    assert status["outcome"] == FOUND, status


def test_the_confirmation_call_is_a_counted_cloud_call():
    tier = TieredVision(FrameReportedPipeline(TARGET), _quiet_cloud)
    before = tier.stats.cloud_calls
    verdict = tier.confirm_arrival({"detections": [{"label": TARGET, "bearing_deg": 0.0}]})
    assert verdict["confirmed"] is True
    assert tier.stats.cloud_calls == before + 1
    verdict = tier.confirm_arrival({"detections": []})
    assert verdict["confirmed"] is False


def test_the_call_cap_refuses_rather_than_confirms():
    tier = TieredVision(FrameReportedPipeline(TARGET), _quiet_cloud, max_calls=0)
    verdict = tier.confirm_arrival({"detections": [{"label": TARGET, "bearing_deg": 0.0}]})
    assert verdict["confirmed"] is False and "cap" in verdict["reason"]
