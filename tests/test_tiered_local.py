"""
tests/test_tiered_local.py

3.80.C (`docs/plans/ros-alignment/3.80-tiered-identity.md`): the arrival's
identity question goes to the local VLM first and to the cloud only when the
local tier is absent or has been unable `tier_local_max_unable` times in one
arrival. Acceptance C1-C7, through the whole mission path (`MissionRunner`
-> agent -> `robot/safety.py` -> `MockRobot`, scaled house), with the local
tier and the cloud faked; the client against the real service app.
"""

import base64
import io
import logging
import math
import sys

import httpx
import pytest
from fastapi.testclient import TestClient

from brain.arrival import CHECKING, REFUSED
from brain.local_vlm import LocalVLMAbsent, LocalVLMClient
from brain.navigate import CloudUnavailable
from brain.perceive import FrameReportedPipeline
from brain.tiered import TRIGGER_ARRIVAL, TieredVision
from control.mission_runner import ARRIVED_UNCONFIRMED, FOUND, MissionRunner
from sim.mock_robot import MockRobot
from tests.conftest import mock_world_for
from tests.test_bearing_turns import (
    ARRIVED_CELLS, CLEAR_STARTS, GOAL, TARGET, _build, _quiet_cloud)


@pytest.fixture(autouse=True)
def _quiet_logs():
    logging.disable(logging.WARNING)
    yield
    logging.disable(logging.NOTSET)


class Local:
    """A scripted local tier: each ask returns the next answer (the last one
    repeats). An answer is a dict, or an exception to raise."""

    def __init__(self, *answers):
        self.answers = list(answers)
        self.asked = 0

    def __call__(self, frame):
        a = self.answers[min(self.asked, len(self.answers) - 1)]
        self.asked += 1
        if isinstance(a, BaseException):
            raise a
        return a


def yes(score=0.9):
    return {"status": "answered", "score": score}


def no(score=0.02):
    return {"status": "answered", "score": score}


def unable(status="unparsed", score=None):
    return {"status": status, "score": score}


class Cloud:
    """The cloud, counting arrival calls; `see` decides its answer, `down`
    makes every call raise CloudUnavailable."""

    def __init__(self, see=True, down=False):
        self.see, self.down = see, down
        self.calls = 0

    def __call__(self, frame):
        self.calls += 1
        if self.down:
            raise CloudUnavailable("connection refused")
        scene = _quiet_cloud(frame)
        scene["_navigate"] = {**scene.get("_navigate", {}), "target_visible": self.see}
        return scene


def _run(local, cloud, max_steps=60, start=CLEAR_STARTS[0], **tier_kw):
    x, y, off = start
    grid = _build()
    grid.x, grid.y = x, y
    grid.theta = math.atan2(GOAL[1] - y, GOAL[0] - x) + math.radians(off)
    robot = MockRobot(grid, render=False)
    # Triggers get a cloud that always answers, so only the ARRIVAL
    # question is what each test varies.
    tier = TieredVision(FrameReportedPipeline(TARGET), _quiet_cloud, confirm_vision_fn=cloud,
                        steer_on_sight=True, hold_goal=True, local_identity_fn=local,
                        **tier_kw)
    runner = MissionRunner(robot, target_object=TARGET, max_steps=max_steps,
                           policy="tiered", vision_fn=tier, world=mock_world_for(robot),
                           vision_timeout_s=5.0)
    runner.start()
    while runner.tick():
        pass
    return runner.status(), math.dist((grid.x, grid.y), GOAL), tier


def test_c1_a_local_yes_ends_found_with_no_cloud_call():
    local, cloud = Local(yes()), Cloud()
    status, cells, tier = _run(local, cloud)
    assert status["outcome"] == FOUND, status
    assert cells <= ARRIVED_CELLS
    assert local.asked == 1 and cloud.calls == 0
    assert tier.stats.triggers.get(TRIGGER_ARRIVAL) is None
    assert tier.stats.local == {"yes": 1} and tier.stats.local_calls == 1
    assert status["arrival"]["identity"]["tier"] == "local"


def test_c2_a_local_no_is_refused_once_with_no_cloud_call():
    local, cloud = Local(no()), Cloud()
    status, cells, tier = _run(local, cloud)
    assert cells <= ARRIVED_CELLS, "the robot never reached the object -- proves nothing"
    assert status["outcome"] != FOUND, status
    assert cloud.calls == 0
    # One question per arrival, not one per frame parked in front of it.
    assert 1 <= local.asked <= 2, local.asked


@pytest.mark.parametrize("answer", [
    unable("answered", 0.3),       # in the band between no_below and 0.5
    unable("unparsed"), unable("timeout"), unable("busy"), unable("error"),
], ids=["band", "unparsed", "timeout", "busy", "error"])
def test_c3_unable_twice_then_the_cloud_decides(answer):
    local, cloud = Local(answer), Cloud(see=True)
    status, cells, tier = _run(local, cloud)
    assert status["outcome"] == FOUND, status
    assert local.asked == 2, "the local tier gets max_unable (2) tries per arrival"
    assert cloud.calls == 1
    assert tier.stats.escalations == 1
    assert status["arrival"]["identity"]["tier"] == "cloud"


def test_c3_an_unable_frame_is_checking_not_refused():
    tier = TieredVision(FrameReportedPipeline(TARGET), _quiet_cloud,
                        local_identity_fn=Local(unable()))
    first = tier.confirm_arrival({})
    assert first["undecided"] and not first["confirmed"] and not first["cloud_called"]
    assert "asking again" in first["reason"]
    second = tier.confirm_arrival({})
    assert second["undecided"] and "asking the cloud next" in second["reason"]
    third = tier.confirm_arrival({"image_base64": "x"})
    assert third["tier"] == "cloud" and third["cloud_called"]


def test_c3_the_agent_reads_undecided_as_checking():
    from brain.agent import MissionAgent
    tier = TieredVision(FrameReportedPipeline(TARGET), _quiet_cloud,
                        local_identity_fn=Local(unable()))
    agent = MissionAgent.__new__(MissionAgent)
    agent.arrival_confirm_fn = None
    agent.vision_fn = tier
    agent._identity_refused = False
    agent._refusal = None
    out = agent._confirm_identity({"state": "arrived"}, {})
    assert out["state"] == CHECKING and not agent._identity_refused
    assert agent._confirmed_this_frame  # the step is labelled as a local ask


def test_c3_the_client_timing_out_is_unable_not_absent():
    """The real client's own timeout (a wedged service) is an unable answer."""
    def hang(request):
        raise httpx.ReadTimeout("no reply", request=request)
    client = LocalVLMClient("http://local", TARGET,
                            client=httpx.Client(transport=httpx.MockTransport(hang)))
    got = client.ask({"image_base64": base64.b64encode(b"\xff\xd8jpeg").decode()})
    assert got["status"] == "timeout"


@pytest.mark.parametrize("why", ["refused", "503"])
def test_c4_an_absent_local_tier_hands_the_same_frame_to_the_cloud(why):
    local, cloud = Local(LocalVLMAbsent(why)), Cloud(see=True)
    status, _, tier = _run(local, cloud)
    assert status["outcome"] == FOUND, status
    assert local.asked == 1 and cloud.calls == 1, (local.asked, cloud.calls)
    assert tier.stats.local.get("absent") == 1
    assert status["arrival"]["identity"]["tier"] == "cloud"


def test_c4_a_fault_in_the_client_is_treated_as_absent():
    local, cloud = Local(ValueError("bug in the client")), Cloud(see=True)
    status, _, _ = _run(local, cloud)
    assert status["outcome"] == FOUND and cloud.calls == 1


def test_c5_offline_a_local_yes_still_ends_found():
    local, cloud = Local(yes()), Cloud(down=True)
    status, _, _ = _run(local, cloud)
    assert status["outcome"] == FOUND, status
    assert cloud.calls == 0


@pytest.mark.parametrize("local", [Local(unable()), Local(LocalVLMAbsent("gone"))],
                         ids=["unable", "absent"])
def test_c5_offline_with_no_local_answer_is_arrived_unconfirmed(local):
    status, cells, _ = _run(local, Cloud(down=True))
    assert cells <= ARRIVED_CELLS
    assert status["outcome"] == ARRIVED_UNCONFIRMED, status


def test_c6_a_new_arrival_gives_the_local_tier_its_tries_again():
    tier = TieredVision(FrameReportedPipeline(TARGET), _quiet_cloud,
                        local_identity_fn=Local(unable()))
    tier.confirm_arrival({})
    tier.confirm_arrival({})
    assert not tier.next_confirm_is_local()
    for _ in range(3):  # LOCAL_RESET_FRAMES off the arrival rule, in a row
        tier.arrival_ended()
    assert tier.next_confirm_is_local()
    assert tier.confirm_arrival({})["tier"] == "local"


def test_c6_the_agent_ends_the_arrival_when_the_rule_stops_holding():
    from brain.agent import MissionAgent
    ended = []
    agent = MissionAgent.__new__(MissionAgent)
    agent.memory = type("M", (), {"target_object": TARGET, "is_complete": lambda s: False})()
    agent.arrival = type("A", (), {"observe": lambda s, scene, robot: {"state": "approaching"}})()
    agent.robot = None
    agent.vision_fn = type("V", (), {"arrival_ended": lambda s: ended.append(1)})()
    agent._identity_refused = True
    agent._refusal = {}
    agent._review_scene({"_perception": {"status": "detected"}}, {})
    assert ended == [1] and not agent._identity_refused


def test_c7_with_no_local_tier_the_cloud_alone_confirms():
    cloud = Cloud(see=True)
    status, _, tier = _run(None, cloud)
    assert status["outcome"] == FOUND
    assert cloud.calls == 1 and tier.stats.local_calls == 0 and tier.stats.local == {}
    assert status["arrival"]["identity"]["tier"] == "cloud"


def test_c7_config_defaults_leave_the_local_tier_off(monkeypatch):
    from control.brain_config import load_brain_config
    from control.brain_server import _local_identity_fn
    monkeypatch.delenv("LOCAL_VLM_URL", raising=False)
    config = load_brain_config()
    assert config["tier_local_vlm_url"] == ""
    assert _local_identity_fn(TARGET, config, None) is None
    monkeypatch.setenv("LOCAL_VLM_URL", "http://127.0.0.1:8005")
    client = _local_identity_fn(TARGET, load_brain_config(), "s")
    assert isinstance(client, LocalVLMClient) and client.timeout_s == 5.0


@pytest.mark.parametrize("key,value", [("tier_local_no_below", 0.5),
                                       ("tier_local_max_unable", 0),
                                       ("tier_local_timeout_s", 0)])
def test_config_refuses_bad_local_thresholds(tmp_path, key, value):
    from control.brain_config import load_brain_config
    cfg = tmp_path / "robot.yaml"
    cfg.write_text(f"brain:\n  {key}: {value}\n")
    with pytest.raises(ValueError, match=key):
        load_brain_config(str(cfg))


def test_closing_the_policy_closes_the_local_client_and_the_late_check_is_cloud_only():
    closed = []
    local = Local(yes())
    local.close = lambda: closed.append(1)
    tier = TieredVision(FrameReportedPipeline(TARGET), _quiet_cloud, local_identity_fn=local)
    tier.close()
    assert closed == [1]
    assert tier.confirm_arrival({"image_base64": "x"})["tier"] == "cloud"
    assert local.asked == 0


# --- the client against the real service app (service/local_vlm) ----------

def _service(tmp_path, monkeypatch):
    from service.local_vlm.server import create_app
    from tests.test_local_vlm import make_driver, wait_ready
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    app = create_app(lambda: make_driver(tmp_path), on_death=lambda: None,
                     ready_timeout_s=10, watch_interval_s=0.05, death_delay_s=0.0)
    tc = TestClient(app)
    tc.__enter__()
    wait_ready(tc)
    return tc


def _jpeg_frame():
    from PIL import Image
    out = io.BytesIO()
    Image.new("RGB", (8, 8)).save(out, format="JPEG")
    return {"image_base64": base64.b64encode(out.getvalue()).decode()}


def test_client_against_the_service_yes_and_no(tmp_path, monkeypatch):
    tc = _service(tmp_path, monkeypatch)
    try:
        # The fake driver answers by the target's words (tests/fake_edgellm_driver.py).
        got = LocalVLMClient("http://testserver", "yes thing", client=tc).ask(_jpeg_frame())
        assert got["status"] == "answered" and got["score"] == pytest.approx(0.9)
        got = LocalVLMClient("http://testserver", "plain thing", client=tc).ask(_jpeg_frame())
        assert got["score"] == pytest.approx(0.05)
    finally:
        tc.__exit__(None, None, None)


def test_client_png_frames_are_sent_as_jpeg(tmp_path, monkeypatch):
    from PIL import Image
    out = io.BytesIO()
    Image.new("RGB", (8, 8)).save(out, format="PNG")
    tc = _service(tmp_path, monkeypatch)
    try:
        got = LocalVLMClient("http://testserver", "yes thing", client=tc).ask(
            {"image_base64": base64.b64encode(out.getvalue()).decode(),
             "media_type": "image/png"})
        assert got["status"] == "answered"
    finally:
        tc.__exit__(None, None, None)


def test_client_a_dead_driver_is_absent(tmp_path, monkeypatch):
    tc = _service(tmp_path, monkeypatch)
    try:
        client = LocalVLMClient("http://testserver", "die thing", client=tc)
        with pytest.raises(LocalVLMAbsent):
            client.ask(_jpeg_frame())
    finally:
        tc.__exit__(None, None, None)


def test_client_refused_connection_is_absent():
    with pytest.raises(LocalVLMAbsent):
        LocalVLMClient("http://127.0.0.1:9", TARGET).ask(_jpeg_frame())


def test_client_a_wrong_secret_or_no_image_is_unable_not_absent():
    def reject(request):
        return httpx.Response(401, text="bad secret")
    client = LocalVLMClient("http://local", TARGET,
                            client=httpx.Client(transport=httpx.MockTransport(reject)))
    assert client.ask(_jpeg_frame())["status"] == "error"
    assert client.ask({})["status"] == "error"


# --- review fixes (2026-10-14) ----------------------------------------------

def test_review_a_slow_local_tier_under_a_short_deadline_is_unable_not_a_cloud_outage():
    """/code-review: with B3.2's deadline under the client's 5 s, a slow local
    VLM used the whole step and the cloud path was charged an outage it never
    had. Now the local call gets part of the step and counts unable."""
    import threading
    import time as _t
    release = threading.Event()

    def slow(frame):
        release.wait(5)
        return yes()
    cloud = Cloud(see=True)
    tier = TieredVision(FrameReportedPipeline(TARGET), _quiet_cloud, confirm_vision_fn=cloud,
                        local_identity_fn=slow)
    runner = MissionRunner(MockRobot(_build(), render=False), target_object=TARGET,
                           policy="tiered", vision_fn=tier, vision_timeout_s=2.0)
    t0 = _t.monotonic()
    verdict = runner._guarded_confirm({"image_base64": "x"})
    assert _t.monotonic() - t0 < 2.0
    assert verdict["undecided"] and verdict["local_status"] == "timeout"
    assert cloud.calls == 0
    assert tier.stats.local.get("unable_runner_timeout") == 1
    # The abandoned call's late yes changes nothing.
    release.set()
    _t.sleep(0.3)
    assert tier.stats.local.get("yes") is None and tier._local_unable == 1


def test_review_a_local_refusal_is_not_credited_to_the_cloud_on_later_frames():
    from brain.agent import MissionAgent
    tier = TieredVision(FrameReportedPipeline(TARGET), _quiet_cloud,
                        local_identity_fn=Local(no()))
    agent = MissionAgent.__new__(MissionAgent)
    agent.arrival_confirm_fn = None
    agent.vision_fn = tier
    agent._identity_refused = False
    agent._refusal = None
    first = agent._confirm_identity({"state": "arrived"}, {})
    assert first["state"] == REFUSED
    later = agent._confirm_identity({"state": "arrived"}, {})
    assert "the local model did not confirm" in later["reason"], later["reason"]
    assert "cloud" not in later["reason"]


def test_review_a_flickering_arrival_still_reaches_the_cloud():
    """Bug pass: one or two frames off the arrival rule (a missed detection)
    must not give the local tier fresh tries every time, or an undecidable
    target is never put to the cloud."""
    tier = TieredVision(FrameReportedPipeline(TARGET), _quiet_cloud,
                        local_identity_fn=Local(unable()))
    tier.confirm_arrival({})
    tier.arrival_ended()
    tier.arrival_ended()
    tier.confirm_arrival({})
    assert not tier.next_confirm_is_local()
    assert tier.confirm_arrival({"image_base64": "x"})["tier"] == "cloud"
    assert tier.stats.escalations == 1


def test_review_escalations_count_cloud_asks_not_spent_tries():
    tier = TieredVision(FrameReportedPipeline(TARGET), _quiet_cloud,
                        local_identity_fn=Local(unable()))
    tier.confirm_arrival({})
    tier.confirm_arrival({})
    assert tier.stats.escalations == 0  # not yet handed to the cloud
    for _ in range(3):
        tier.arrival_ended()
    assert tier.stats.escalations == 0


def test_review_a_stop_during_the_local_check_buys_no_cloud_call():
    cloud = Cloud(see=True)
    holder = {}

    def stopped_mid_call(frame):
        holder["runner"].stop()
        raise LocalVLMAbsent("client closed by the stop")
    tier = TieredVision(FrameReportedPipeline(TARGET), cloud, confirm_vision_fn=cloud,
                        local_identity_fn=stopped_mid_call)
    runner = MissionRunner(MockRobot(_build(), render=False), target_object=TARGET,
                           policy="tiered", vision_fn=tier, vision_timeout_s=5.0)
    holder["runner"] = runner
    runner.start()
    verdict = runner._guarded_confirm({"image_base64": "x"})
    assert not verdict["confirmed"] and not verdict["cloud_called"]
    assert cloud.calls == 0 and tier.stats.cloud_calls == 0


def test_review_the_shared_secret_goes_only_to_a_local_service(monkeypatch):
    from control.brain_server import _local_vlm_secret
    monkeypatch.delenv("LOCAL_VLM_SECRET", raising=False)
    assert _local_vlm_secret({"tier_local_vlm_url": "http://127.0.0.1:8005"}, "s") == "s"
    assert _local_vlm_secret({"tier_local_vlm_url": "http://vlm.example.com"}, "s") is None
    monkeypatch.setenv("LOCAL_VLM_SECRET", "own")
    assert _local_vlm_secret({"tier_local_vlm_url": "http://vlm.example.com"}, "s") == "own"
