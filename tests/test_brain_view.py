"""
tests/test_brain_view.py

What ROS tools are shown of the brain (2026-09-27): picar_bridge polls the
brain's /mission/status and republishes it as /brain/status, /diagnostics
and /brain/markers. The mapping lives in plain Python
(service/slam/src/picar_bridge/picar_bridge/brain_view.py) and is pinned
here without ROS; tests/test_brain_view_live.py checks the topics.
"""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "service/slam/src/picar_bridge"))
from picar_bridge import brain_view  # noqa: E402

from tests.conftest import fresh_mock_runner  # noqa: E402


def _status(**kw):
    base = {"running": False, "outcome": "idle", "error": None, "policy": "tiered",
            "mission": "Find the red backpack.", "step": 0, "max_steps": 120,
            "last_action": None, "tier": None, "perception": None, "arrival": None}
    return {**base, **kw}


# ---------- /diagnostics ----------

def test_an_unreachable_brain_is_stale_not_ok():
    d = brain_view.diagnostic(None, "connection refused")
    assert d["level"] == brain_view.STALE and "connection refused" in d["message"]


@pytest.mark.parametrize("outcome,level", [
    ("running", brain_view.OK), ("found", brain_view.OK), ("stopped", brain_view.OK),
    ("blocked", brain_view.WARN), ("preempted", brain_view.WARN),
    ("max_steps", brain_view.WARN), ("failed", brain_view.ERROR)])
def test_each_outcome_has_a_level(outcome, level):
    assert brain_view.diagnostic(_status(outcome=outcome))["level"] == level


def test_every_real_outcome_is_mapped():
    """A new outcome added to the runner must be given a level on purpose,
    not fall through to WARN unnoticed."""
    import control.mission_runner as mr
    outcomes = {v for k, v in vars(mr).items()
                if k.isupper() and isinstance(v, str) and k in (
                    "RUNNING", "IDLE", "FOUND", "ROOM_REACHED", "STOPPED", "MAX_STEPS",
                    "FAILED", "PREEMPTED", "BLOCKED")}
    assert outcomes <= set(brain_view._OUTCOME_LEVEL), outcomes - set(brain_view._OUTCOME_LEVEL)


def test_a_running_mission_says_its_step_and_carries_the_numbers():
    d = brain_view.diagnostic(_status(running=True, outcome="running", step=12,
                                      last_action="FORWARD",
                                      tier={"stats": {"cloud_calls": 3, "frames": 14}},
                                      arrival={"state": "approaching", "range_m": 0.62}))
    assert d["message"] == "running, step 12/120"
    values = dict(d["values"])
    assert values["cloud_calls"] == "3" and values["frames"] == "14"
    assert values["arrival_range_m"] == "0.62" and values["last_action"] == "FORWARD"
    assert all(isinstance(v, str) for v in values.values()), "KeyValue.value is a string"


def test_a_failure_names_its_error():
    d = brain_view.diagnostic(_status(outcome="failed", error="vision failed 3 times"))
    assert d["message"] == "failed: vision failed 3 times"


def test_a_real_runner_status_maps_without_error():
    runner = fresh_mock_runner(target_object="red backpack", max_steps=5)
    runner.start()
    for _ in range(5):
        runner.tick()
    status = runner.status()
    d = brain_view.diagnostic(status)
    assert dict(d["values"])["step"] == str(status["step"])
    assert brain_view.markers(status)[0]["type"] == "text"


# ---------- /brain/markers ----------

def test_no_sighting_deletes_the_arrow():
    m = brain_view.markers(_status())
    assert [x["type"] for x in m] == ["text", "delete"] and m[1]["id"] == 1


def test_a_target_to_the_right_points_the_arrow_right_in_ros():
    """Project bearings are CLOCKWISE (+30 = right); ROS yaw is CCW, so the
    arrow's yaw is -30 degrees -- and its tip has NEGATIVE y (ROS +y is left)."""
    m = brain_view.markers(_status(perception={"bearing_deg": 30.0}))[1]
    assert m["type"] == "arrow" and m["yaw_rad"] == pytest.approx(math.radians(-30))
    assert m["length_m"] * math.sin(m["yaw_rad"]) < 0


def test_the_lidar_range_sets_the_arrow_length_and_arrival_turns_it_green():
    m = brain_view.markers(_status(arrival={"state": "arrived", "bearing_deg": 0.0,
                                            "range_m": 0.35}))[1]
    assert m["length_m"] == pytest.approx(0.35)
    assert m["colour"][1] > m["colour"][0], "arrived is green"


def test_a_panned_camera_rotates_the_arrow_with_it():
    m = brain_view.markers(_status(perception={"bearing_deg": 0.0}), pan_rad=math.radians(45))[1]
    assert m["yaw_rad"] == pytest.approx(math.radians(45))


def test_the_caption_names_policy_step_and_action():
    text = brain_view.markers(_status(step=7, last_action="LEFT", outcome="running"))[0]["text"]
    assert "tiered" in text and "7/120" in text and "LEFT" in text


def test_an_idle_brain_says_idle_not_step_none():
    assert brain_view.markers({"running": False, "outcome": "idle"})[0]["text"] == "brain idle"


def test_unreachable_brain_still_draws_a_caption():
    m = brain_view.markers(None)
    assert m[0]["text"] == "brain unreachable" and m[1]["type"] == "delete"
