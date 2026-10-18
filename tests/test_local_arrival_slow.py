"""
tests/test_local_arrival_slow.py

3.80.O amendment 3 (`docs/plans/ros-alignment/3.80-tiered-identity.md`): one
whole in-process mission (procthor val 3, ~9 s), so on `SLOW_FILES`.
"""

import logging

import pytest

from brain.perceive import FrameReportedPipeline
from brain.tiered import TieredVision
from control.mission_runner import MissionRunner
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests.test_explore import WalkingNav
from tests.test_local_arrival import HFOV, TARGET, TruthCloud, TruthLocal


@pytest.fixture(autouse=True)
def _quiet_logs():
    logging.disable(logging.WARNING)
    yield
    logging.disable(logging.NOTSET)


def test_amendment3_a_refused_turn_to_a_local_yes_is_not_restarted_in_place():
    """In-process val 3: wedged in a corner, every local yes re-queued a turn
    the pivot guard refused, and the clock never moved. The mission must end."""
    grid = build_world("procthor:val:3")
    robot = MockRobot(grid, render=False)
    pipeline = FrameReportedPipeline(TARGET, blind=True)
    pipeline.hfov_deg = HFOV
    tier = TieredVision(pipeline, TruthCloud(answer=False), confirm_vision_fn=TruthCloud(),
                        local_identity_fn=TruthLocal(), local_sighting_fn=TruthLocal(),
                        async_cloud=True)
    runner = MissionRunner(robot, target_object=TARGET, policy="tiered", vision_fn=tier,
                           tier_search="explore", navigator=WalkingNav(robot),
                           world=MockWorld(grid), mission_time_limit_s=1200,
                           clock=lambda: grid.sim_time, idle=robot.pass_time)
    runner.start()
    ticks = 0
    while runner.tick() and ticks < 6000:
        ticks += 1
    # It ends, and on its own terms -- not parked until the time limit.
    assert runner.status()["outcome"] not in ("running", "out_of_budget"), \
        runner.status()["log_tail"][-1:]
    assert tier.stats.local_triggers.get("local_track", 0) < 2000
