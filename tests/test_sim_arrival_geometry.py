"""3.98 (the user, 2026-10-24): in the simulator an arrival is confirmed from
the sim's own geometry -- the nearest object near the frame's centre must be
the target -- and the paid cloud call is opt-in (`sim_arrival_confirm:
cloud`). The car is unchanged. docs/plans/ros-alignment/3.98-sim-arrival-geometry.md"""

import math

import pytest

from control.brain_config import load_brain_config
from control.brain_server import _tiered_vision_fn
from tests.test_tiered_frames import Cloud

SIM = {"source": "sim"}


def sim_frame(detections, metadata=SIM):
    f = {"image_base64": "x", "image_width": 640, "pan_deg": 0.0, "detections": detections}
    if metadata is not None:
        f["metadata"] = metadata
    return f


def det(label, bearing, dist):
    return {"label": label, "bearing_deg": bearing, "distance_m": dist}


class NoArrivalCall:
    """The arrival's cloud client: any call is a failure of the test."""
    calls = 0

    def __call__(self, frame):
        self.calls += 1
        return {"_navigate": {"target_visible": True}}


def _tier(config=None, simulated=True, confirm=None, monkeypatch=None):
    if monkeypatch is not None:
        monkeypatch.delenv("LOCAL_VLM_URL", raising=False)
    return _tiered_vision_fn("backpack", Cloud(), config or load_brain_config(),
                             simulated=simulated, confirm_vision_fn=confirm)


def test_the_default_is_geometry(monkeypatch):
    monkeypatch.delenv("SIM_ARRIVAL_CONFIRM", raising=False)
    assert load_brain_config()["sim_arrival_confirm"] == "geometry"


@pytest.mark.parametrize("dets,expected", [
    ([det("red backpack", 1.0, 0.35)], True),
    # The target in view but behind the chair the robot stopped at.
    ([det("chair", 0.5, 0.35), det("red backpack", 2.0, 1.4)], False),
    # The target near but off to the side; the centred thing is a box.
    ([det("box", 0.0, 0.40), det("red backpack", 25.0, 0.30)], False),
    ([], False),
])
def test_the_nearest_centred_object_decides(monkeypatch, dets, expected):
    confirm = NoArrivalCall()
    tier = _tier(confirm=confirm, monkeypatch=monkeypatch)
    try:
        v = tier.confirm_arrival(sim_frame(dets))
        assert v["confirmed"] is expected, v
        assert v["cloud_called"] is False and v["tier"] == "sim", v
        assert confirm.calls == 0 and tier.stats.cloud_calls == 0
    finally:
        tier.close()


def test_a_frame_with_no_detections_is_refused(monkeypatch):
    tier = _tier(confirm=NoArrivalCall(), monkeypatch=monkeypatch)
    try:
        f = sim_frame([])
        del f["detections"]
        v = tier.confirm_arrival(f)
        assert v["confirmed"] is False and v["cloud_called"] is False, v
    finally:
        tier.close()


def test_cloud_keeps_todays_paid_call(monkeypatch, tmp_path):
    cfg = tmp_path / "robot.yaml"
    cfg.write_text("brain:\n  sim_arrival_confirm: cloud\n")
    confirm = NoArrivalCall()
    tier = _tier(config=load_brain_config(str(cfg)), confirm=confirm, monkeypatch=monkeypatch)
    try:
        v = tier.confirm_arrival(sim_frame([det("chair", 0.0, 0.3)]))
        assert v["cloud_called"] is True and confirm.calls == 1, v
    finally:
        tier.close()


def test_real_frames_never_use_geometry(monkeypatch):
    """Not the simulator's frames: the cloud is asked, whatever the setting
    -- a sim's detections would be a claim about a real room."""
    confirm = NoArrivalCall()
    tier = _tier(confirm=confirm, monkeypatch=monkeypatch)   # a real tier needs the models
    try:
        v = tier.confirm_arrival(sim_frame([det("red backpack", 0.0, 0.3)], metadata=None))
        assert v["cloud_called"] is True and confirm.calls == 1, v
    finally:
        tier.close()


def test_a_bad_setting_is_refused(tmp_path):
    cfg = tmp_path / "robot.yaml"
    cfg.write_text("brain:\n  sim_arrival_confirm: maybe\n")
    with pytest.raises(ValueError, match="sim_arrival_confirm"):
        load_brain_config(str(cfg))


def test_a_sim_mission_ends_found_with_no_arrival_call(monkeypatch):
    """The whole path: MissionRunner -> agent -> safety -> MockRobot, the
    tier built as the brain server builds it, the arrival's cloud client
    counting calls. Before 3.98 the arrival paid one."""
    from control.mission_runner import FOUND, MissionRunner
    from sim.mock_robot import MockRobot
    from tests.conftest import mock_world_for
    from tests.test_bearing_turns import CLEAR_STARTS, GOAL, TARGET, _build

    monkeypatch.delenv("LOCAL_VLM_URL", raising=False)
    x, y, off = CLEAR_STARTS[0]
    grid = _build()
    grid.x, grid.y = x, y
    grid.theta = math.atan2(GOAL[1] - y, GOAL[0] - x) + math.radians(off)
    robot = MockRobot(grid, render=True)
    confirm = NoArrivalCall()
    tier = _tiered_vision_fn(TARGET, Cloud(), load_brain_config(), simulated=True,
                             confirm_vision_fn=confirm)
    runner = MissionRunner(robot, target_object=TARGET, max_steps=80, policy="tiered",
                           vision_fn=tier, world=mock_world_for(robot), vision_timeout_s=5.0)
    runner.start()
    while runner.tick():
        pass
    s = runner.status()
    assert s["outcome"] == FOUND, s["log_tail"]
    assert confirm.calls == 0, "the arrival paid a cloud call"


def test_the_step_a_geometry_verdict_ends_is_labelled(monkeypatch):
    """Review round 1: a geometry verdict made no call, so the agent did not
    label its step (`identity_tier`, the refused arrival's reasoning)."""
    from brain.agent import MissionAgent
    seen = []
    real = MissionAgent._label_confirmation

    def spy(scene, readout, verdict):
        seen.append(verdict.get("tier"))
        return real(scene, readout, verdict)

    monkeypatch.setattr(MissionAgent, "_label_confirmation", staticmethod(spy))
    test_a_sim_mission_ends_found_with_no_arrival_call(monkeypatch)
    assert "sim" in seen, seen
