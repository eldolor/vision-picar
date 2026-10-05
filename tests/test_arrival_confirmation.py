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


# ---------------------------------------------------------------------------
# Spec review 3, fixes 8-10.

import threading  # noqa: E402
import time  # noqa: E402


class _CountingCloud:
    """A correct cloud that takes `delay_s` and records peak concurrency."""

    def __init__(self, base, delay_s=0.2):
        self.base, self.delay_s = base, delay_s
        self.live = self.peak = 0
        self.lock = threading.Lock()

    def __call__(self, frame):
        with self.lock:
            self.live += 1
            self.peak = max(self.peak, self.live)
        try:
            time.sleep(self.delay_s)
            return self.base(frame)
        finally:
            with self.lock:
                self.live -= 1


def _run_async(cloud):
    x, y, off = CLEAR_STARTS[0]
    grid = _build()
    grid.x, grid.y = x, y
    grid.theta = math.atan2(GOAL[1] - y, GOAL[0] - x) + math.radians(off)
    robot = MockRobot(grid, render=False)
    tier = TieredVision(FrameReportedPipeline(TARGET), cloud, steer_on_sight=True,
                        hold_goal=True, async_cloud=True, stale_after=1)
    runner = MissionRunner(robot, target_object=TARGET, max_steps=60, policy="tiered",
                           vision_fn=tier, world=mock_world_for(robot))
    runner.start()
    while runner.tick():
        pass
    return runner, tier


def test_8_the_confirmation_never_overlaps_an_async_call():
    """policy ARCH: at most one cloud call in flight. stale_after=1 keeps a
    trigger call in flight on nearly every frame, so arrival lands on one."""
    cloud = _CountingCloud(_quiet_cloud)
    runner, tier = _run_async(cloud)
    assert tier.stats.triggers.get(TRIGGER_ARRIVAL), tier.stats.triggers
    assert cloud.peak == 1, f"{cloud.peak} cloud calls were in flight at once"


def test_10_a_found_missions_status_counts_the_confirmation():
    status, _, tier = _run(_quiet_cloud)
    assert status["outcome"] == FOUND
    stats = status["tier"]["stats"]
    assert stats["triggers"].get(TRIGGER_ARRIVAL) == 1, stats["triggers"]
    assert stats["cloud_calls"] == tier.stats.cloud_calls


def test_9_the_paid_arrival_step_is_labelled_in_the_log():
    status, _, _ = _run(_quiet_cloud)
    assert any("[cloud: arrival_confirmation]" in line for line in status["log_tail"]), \
        status["log_tail"][-3:]


def test_9_a_refused_arrival_says_so_to_the_end():
    status, _, _ = _run(_disagreeing_cloud)
    assert status["outcome"] != FOUND
    assert status["arrival"]["state"] == "refused"
    assert status["arrival"].get("identity", {}).get("confirmed") is False, status["arrival"]
    ended = status["log_tail"][-1]
    assert "identity" in ended and "obstructed" not in ended, ended
