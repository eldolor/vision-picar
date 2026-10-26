"""3.91.D2: in the simulator a held cloud no neither refuses an arrival free
nor holds it waiting: the arrival goes the cloud-only way, where the sim's
geometry judges it, free (3.98's check ran only after the held no). Real
frames are untouched."""

from brain.agent import MissionAgent
from brain.memory import MissionMemory
from control.brain_config import load_brain_config
from control.brain_server import _tiered_vision_fn
from control.mission_runner import MissionRunner
from tests.conftest import mock_world_for
from tests.test_bearing_turns import TARGET
from tests.test_sim_arrival_geometry import NoArrivalCall, det, sim_frame
from tests.test_tier_remember_no import D
from tests.test_tiered_frames import Cloud


def _robot():
    # 3.98's own start (tests/test_sim_arrival_geometry.py).
    import math
    from sim.mock_robot import MockRobot
    from tests.test_bearing_turns import CLEAR_STARTS, GOAL, _build
    x, y, off = CLEAR_STARTS[0]
    grid = _build()
    grid.x, grid.y = x, y
    grid.theta = math.atan2(GOAL[1] - y, GOAL[0] - x) + math.radians(off)
    return MockRobot(grid, render=True)


def _tier(cloud, confirm, monkeypatch, **settings):
    monkeypatch.delenv("LOCAL_VLM_URL", raising=False)
    return _tiered_vision_fn(TARGET, cloud, {**load_brain_config(), **settings},
                             simulated=True, confirm_vision_fn=confirm)


def _runner(tier, robot=None):
    """A started mission: the arrival's guards (a Stop ends it undecided)
    see it running."""
    robot = robot or _robot()
    runner = MissionRunner(robot, target_object=TARGET, max_steps=80, policy="tiered",
                           vision_fn=tier, world=mock_world_for(robot), vision_timeout_s=5.0)
    runner.start()
    return runner


def _held_no(tier, episode=1):
    tier._lost, tier._episode = 0, episode
    tier._perception_this_frame = D()
    tier._reject("cloud", None, episode)


def test_d2_the_geometry_confirms_the_target_under_a_held_no(monkeypatch):
    confirm = NoArrivalCall()
    tier = _tier(Cloud(), confirm, monkeypatch)
    _held_no(tier)
    v = _runner(tier)._guarded_confirm(sim_frame([det(TARGET, 0.0, 0.3)]))
    assert v["tier"] == "sim" and v["confirmed"] is True and confirm.calls == 0


def test_d2_the_geometry_also_refuses_under_a_held_no(monkeypatch):
    tier = _tier(Cloud(), NoArrivalCall(), monkeypatch)
    _held_no(tier)
    v = _runner(tier)._guarded_confirm(sim_frame([det("chair", 0.0, 0.3)]))
    assert v["tier"] == "sim" and v["confirmed"] is False


def test_d2_the_capped_wait_neither_holds_nor_pays_nor_starts(monkeypatch):
    confirm = NoArrivalCall()
    tier = _tier(Cloud(), confirm, monkeypatch, tier_max_calls=1)
    _held_no(tier)
    tier._episode = 2                            # the no outlives its run
    tier.stats.cloud_calls = 1
    v = _runner(tier)._guarded_confirm(sim_frame([det(TARGET, 0.0, 0.3)]))
    assert v["tier"] == "sim" and v["confirmed"] is True
    assert confirm.calls == 0 and tier.stats.cloud_calls == 1
    assert "wait_mono" not in tier._rejection    # the wait's clock not started


def test_d2_a_real_frame_keeps_the_free_refusal(monkeypatch):
    confirm = NoArrivalCall()
    tier = _tier(Cloud(), confirm, monkeypatch)
    _held_no(tier)
    real = {"image_base64": "x", "image_width": 640, "pan_deg": 0.0,
            "metadata": {"source": "teleop"}}
    assert tier.arrival_route(real) == "no"
    v = _runner(tier)._guarded_confirm(real)
    assert v["tier"] == "cloud" and v["confirmed"] is False and confirm.calls == 0


def test_d2_outside_a_runner_the_agent_routes_the_same_way(monkeypatch):
    tier = _tier(Cloud(), NoArrivalCall(), monkeypatch)
    _held_no(tier)
    agent = MissionAgent(_robot(), MissionMemory(mission="m", target_object=TARGET),
                         vision_fn=tier)
    out = agent._confirm_identity({"state": "arrived", "source": "perception"},
                                  sim_frame([det(TARGET, 0.0, 0.3)]))
    assert out["identity"]["tier"] == "sim" and out["identity"]["confirmed"] is True
