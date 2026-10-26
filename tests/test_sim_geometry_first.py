"""3.91.D2: in the simulator the geometry judges an arrival a held cloud no
would refuse, or hold waiting, for free (3.98's check ran only after them).
"""

import math

from brain.agent import MissionAgent
from brain.memory import MissionMemory
from control.brain_config import load_brain_config
from control.brain_server import _tiered_vision_fn
from control.mission_runner import MissionRunner
from sim.mock_robot import MockRobot
from tests.conftest import mock_world_for
from tests.test_bearing_turns import CLEAR_STARTS, GOAL, TARGET, _build
from tests.test_sim_arrival_geometry import NoArrivalCall, det, sim_frame
from tests.test_tier_remember_no import D
from tests.test_tiered_frames import Cloud


def _robot():
    x, y, off = CLEAR_STARTS[0]
    grid = _build()
    grid.x, grid.y = x, y
    grid.theta = math.atan2(GOAL[1] - y, GOAL[0] - x) + math.radians(off)
    return MockRobot(grid, render=True)


def _tier(cloud, confirm, **settings):
    config = {**load_brain_config(), **settings}
    return _tiered_vision_fn(TARGET, cloud, config, simulated=True, confirm_vision_fn=confirm)


def _held_no(tier):
    tier._lost, tier._episode = 0, 1
    tier._perception_this_frame = D()
    tier._reject("cloud", None, 1)


def test_d2_in_the_sim_the_geometry_judges_an_arrival_a_held_cloud_no_would_refuse(monkeypatch):
    monkeypatch.delenv("LOCAL_VLM_URL", raising=False)
    robot = _robot()
    confirm = NoArrivalCall()
    tier = _tier(Cloud(), confirm)               # geometry, the default
    _held_no(tier)
    assert tier.cloud_spoke() == "no"
    runner = MissionRunner(robot, target_object=TARGET, max_steps=80, policy="tiered",
                           vision_fn=tier, world=mock_world_for(robot), vision_timeout_s=5.0)
    v = runner._guarded_confirm(sim_frame([det(TARGET, 0.0, 0.3)]))
    assert v["tier"] == "sim" and v["confirmed"] is True and confirm.calls == 0


def test_d2_and_one_the_capped_wait_would_hold(monkeypatch):
    monkeypatch.delenv("LOCAL_VLM_URL", raising=False)
    robot = _robot()
    tier = _tier(Cloud(), NoArrivalCall(), tier_max_calls=1)
    _held_no(tier)
    tier._episode = 2                            # the no outlives its run
    tier.stats.cloud_calls = 1
    assert tier.cloud_spoke() == "wait"
    runner = MissionRunner(robot, target_object=TARGET, max_steps=80, policy="tiered",
                           vision_fn=tier, world=mock_world_for(robot), vision_timeout_s=5.0)
    v = runner._guarded_confirm(sim_frame([det(TARGET, 0.0, 0.3)]))
    assert v["tier"] == "sim" and v["confirmed"] is True


def test_d2_outside_a_runner_the_agent_asks_the_geometry_too(monkeypatch):
    monkeypatch.delenv("LOCAL_VLM_URL", raising=False)
    tier = _tier(Cloud(), NoArrivalCall())
    _held_no(tier)
    agent = MissionAgent(_robot(), MissionMemory(mission="m", target_object=TARGET),
                         vision_fn=tier)
    out = agent._confirm_identity({"state": "arrived", "source": "perception"},
                                  sim_frame([det(TARGET, 0.0, 0.3)]))
    assert out["identity"]["tier"] == "sim" and out["identity"]["confirmed"] is True
