"""
tests/test_tiered_steer.py

3.80.F (`docs/plans/ros-alignment/3.80-tiered-identity.md`): the local tiers
steer. F-a: a search trigger's local question that is not a yes is followed
by the direction question, whose answer the policy holds as it holds the
cloud's; an unable direction counts toward the escalation. F-b: YOLOE + CLIP
steer toward a best crop in the uncertain band. Perception, the local tier
and the cloud are faked (`tests/test_tiered_frames.py`'s fakes).
"""

import logging

import pytest

from brain.local_vlm import LocalVLMAbsent
from brain.perceive import ABSENT, DETECTED
from brain.tiered import (TRIGGER_BAND, TRIGGER_COLD_SEARCH, TRIGGER_START,
                          TieredVision)
from tests.test_tiered_frames import (Cloud, Local, P, Pipeline, frame, no, play,
                                      unable, yes)


@pytest.fixture(autouse=True)
def _quiet_logs():
    logging.disable(logging.WARNING)
    yield
    logging.disable(logging.NOTSET)


def went(word, p=0.9):
    return {"status": "answered", "direction": word, "p": p, "mass": 0.95}


class Direction:
    """A scripted direction question (the last answer repeats)."""

    def __init__(self, *answers):
        self.answers = list(answers)
        self.asked = 0

    def __call__(self, frame):
        a = self.answers[min(self.asked, len(self.answers) - 1)]
        self.asked += 1
        if isinstance(a, BaseException):
            raise a
        return dict(a)


def tier_for(frames, local, direction=None, cloud=None, **kw):
    kw.setdefault("cold_search_after", 99)
    kw.setdefault("stale_after", 0)
    kw.setdefault("spin_guard_after", 0)
    cloud = cloud or Cloud(direction="RIGHT")
    tier = TieredVision(Pipeline(frames), cloud, local_sighting_fn=local,
                        local_direction_fn=direction, **kw)
    return tier, cloud


# --- F-a: the local VLM's direction -----------------------------------------

def test_fa_a_no_on_a_search_question_asks_the_direction_and_holds_it():
    d = Direction(went("left"))
    tier, cloud = tier_for([P(ABSENT)], Local(no()), d)
    scenes = play(tier, 4)
    assert d.asked == 1 and cloud.calls == 0
    # Held on the frame it landed and every free frame after it.
    assert [s["safest_direction"] for s in scenes] == ["LEFT"] * 4
    assert "local VLM's direction LEFT" in scenes[1]["_navigate"]["reasoning"]
    assert tier.stats.local_direction == {"left": 1}
    assert scenes[0]["_tier"]["local_landed"]["direction"] == "LEFT"


def test_fa_forward_is_held_as_a_cloud_forward_is():
    tier, _ = tier_for([P(ABSENT)], Local(no()), Direction(went("forward")))
    assert [s["safest_direction"] for s in play(tier, 3)] == ["FORWARD"] * 3


def test_fa_a_newer_answer_replaces_the_held_one():
    d = Direction(went("left"), went("right"))
    tier, _ = tier_for([P(ABSENT)], Local(no()), d, cold_search_after=3)
    got = [s["safest_direction"] for s in play(tier, 6)]
    # start on 1 (left); cold search on 4 (right).
    assert got[:3] == ["LEFT"] * 3 and got[3:] == ["RIGHT"] * 3
    assert d.asked == 2


def test_fa_a_yes_asks_no_direction():
    d = Direction(went("left"))
    tier, _ = tier_for([P(ABSENT)], Local(yes()), d)
    play(tier, 2)
    assert d.asked == 0 and tier.stats.local_direction == {}


def test_fa_only_search_triggers_ask_for_a_direction():
    # A band question (frame 3) answered no asks nothing more.
    d = Direction(went("left"))
    frames = [P(ABSENT), P(ABSENT, 0.6), P(ABSENT, 0.6), P(ABSENT, 0.6)]
    tier, _ = tier_for(frames, Local(no()), d)
    play(tier, 4)
    assert dict(tier.stats.local_triggers) == {TRIGGER_START: 1, TRIGGER_BAND: 1}
    assert d.asked == 1


def test_fa_an_unable_sighting_answer_still_asks_the_direction():
    d = Direction(went("right"))
    tier, cloud = tier_for([P(ABSENT)], Local(unable()), d)
    scenes = play(tier, 1)
    assert d.asked == 1 and scenes[0]["safest_direction"] == "RIGHT"
    # The direction answered: the count of unable answers starts again.
    assert tier._frame_unable == 0 and cloud.calls == 0


@pytest.mark.parametrize("reply", [unable("unparsed"), unable("timeout"),
                                   {"status": "answered", "direction": "up"}],
                         ids=["unparsed", "timeout", "not-a-word"])
def test_fa_an_unable_direction_counts_toward_the_cloud(reply):
    d = Direction(reply)
    tier, cloud = tier_for([P(ABSENT)] * 20, Local(no()), d, cold_search_after=2)
    scenes = play(tier, 5)
    # start (1) and cold search (3): no + an unable direction, twice -> the
    # next search question (5) is the cloud's, and its answer is applied.
    assert d.asked == 2 and cloud.calls == 1
    assert tier.stats.frame_escalations == {"unable": 1}
    assert scenes[4]["_tier"]["cloud_called"] and scenes[4]["safest_direction"] == "RIGHT"
    assert sum(tier.stats.local_direction.values()) == 2


def test_fa_a_cloud_answer_replaces_a_held_local_direction():
    d = Direction(went("left"), unable(), unable())
    tier, cloud = tier_for([P(ABSENT)] * 20, Local(no()), d, cold_search_after=2)
    got = [s["safest_direction"] for s in play(tier, 8)]
    # left held from 1; unable on 3 and 5 keep it; the cloud on 7 replaces it.
    assert got[:6] == ["LEFT"] * 6 and cloud.calls == 1 and got[6:] == ["RIGHT"] * 2


def test_fa_without_a_direction_fn_today_unchanged():
    tier, cloud = tier_for([P(ABSENT)], Local(no()), None)
    scenes = play(tier, 3)
    # 3.80.D: no direction, the scan.
    assert [s["safest_direction"] for s in scenes] == ["RIGHT"] * 3
    assert tier.stats.local_direction == {}
    assert "local VLM's direction" not in scenes[1]["_navigate"]["reasoning"]


def test_fa_a_held_turn_is_a_search_step_and_the_spin_guard_still_counts_it():
    tier, _ = tier_for([P(ABSENT)], Local(no()), Direction(went("left")),
                       spin_guard_after=2)
    scenes = play(tier, 6)
    assert scenes[0]["turn_deg"] == 45
    assert "FORWARD" in [s["safest_direction"] for s in scenes]


def test_fa_async_the_direction_lands_with_the_answer():
    d = Direction(went("left"))
    tier, cloud = tier_for([P(ABSENT)], Local(no()), d, async_cloud=True)
    tier(frame())
    tier.wait_inflight()
    scene = tier(frame())
    assert scene["safest_direction"] == "LEFT" and d.asked == 1
    tier.close()


def test_fa_a_stop_before_the_direction_asks_nothing():
    d = Direction(went("left"))
    tier, _ = tier_for([P(ABSENT)], Local(no()), d)
    tier.close()
    from brain.tiered import _Question
    q = _Question(trigger=TRIGGER_START, epoch=tier._epoch, pose=None, pan_deg=0.0,
                  hfov_deg=60.0)
    assert tier._ask_direction(frame(), q, no()) is None and d.asked == 0


def test_fa_the_brain_wires_the_direction_to_the_same_client(monkeypatch):
    from control.brain_config import load_brain_config
    from control.brain_server import _tiered_vision_fn
    monkeypatch.setenv("LOCAL_VLM_URL", "http://127.0.0.1:9")
    monkeypatch.delenv("TIER_LOCAL_DIRECTION", raising=False)
    tier = _tiered_vision_fn("backpack", Cloud(), load_brain_config(), simulated=True)
    assert tier.local_direction_fn is None    # off by default: F4 failed
    tier.close()
    monkeypatch.setenv("TIER_LOCAL_DIRECTION", "true")
    tier = _tiered_vision_fn("backpack", Cloud(), load_brain_config(), simulated=True)
    assert tier.local_direction_fn.__self__ is tier.local_identity_fn
    assert tier.steer_on_candidate is True    # F5 met: on by default
    tier.close()
    monkeypatch.setenv("TIER_LOCAL_DIRECTION", "false")
    monkeypatch.setenv("TIER_STEER_ON_CANDIDATE", "false")
    config = load_brain_config()
    tier = _tiered_vision_fn("backpack", Cloud(), config, simulated=True)
    assert tier.local_direction_fn is None and tier.steer_on_candidate is False
    tier.close()
    monkeypatch.delenv("LOCAL_VLM_URL")
    monkeypatch.setenv("TIER_LOCAL_DIRECTION", "true")
    monkeypatch.setenv("TIER_STEER_ON_CANDIDATE", "true")
    tier = _tiered_vision_fn("backpack", Cloud(), load_brain_config(), simulated=True)
    # No local tier: no direction and no candidate steer (F6).
    assert tier.local_direction_fn is None and tier.steer_on_candidate is False
    tier.close()


def test_fa_config_refuses_a_bad_boolean(monkeypatch):
    from control.brain_config import load_brain_config
    monkeypatch.setenv("TIER_LOCAL_DIRECTION", "maybe")
    with pytest.raises(ValueError, match="TIER_LOCAL_DIRECTION"):
        load_brain_config()


def test_p_env_sets_the_question_pace(monkeypatch):
    from control.brain_config import load_brain_config
    monkeypatch.delenv("TIER_COLD_SEARCH_AFTER", raising=False)
    assert load_brain_config()["tier_cold_search_after"] == 6
    monkeypatch.setenv("TIER_COLD_SEARCH_AFTER", "1")
    assert load_brain_config()["tier_cold_search_after"] == 1
    for bad in ("0", "-1", "1.5", "often", "\u00b2"):
        monkeypatch.setenv("TIER_COLD_SEARCH_AFTER", bad)
        with pytest.raises(ValueError, match="TIER_COLD_SEARCH_AFTER"):
            load_brain_config()


def test_p_a_local_tier_asks_on_every_free_frame_and_the_cloud_keeps_six(monkeypatch):
    """3.80.P, the user's option (a): pace 1 only with a local VLM tier."""
    from control.brain_config import load_brain_config
    from control.brain_server import _tiered_vision_fn
    for env in ("TIER_COLD_SEARCH_AFTER", "TIER_LOCAL_COLD_SEARCH_AFTER"):
        monkeypatch.delenv(env, raising=False)
    config = load_brain_config()
    assert config["tier_local_cold_search_after"] == 1
    assert config["tier_cold_search_after"] == 6
    monkeypatch.delenv("LOCAL_VLM_URL", raising=False)
    tier = _tiered_vision_fn("backpack", Cloud(), load_brain_config(), simulated=True)
    assert tier.local_identity_fn is None and tier.local_cold_search_after is None
    assert tier.cold_search_after == 6
    tier.close()
    monkeypatch.setenv("LOCAL_VLM_URL", "http://127.0.0.1:9")
    tier = _tiered_vision_fn("backpack", Cloud(), load_brain_config(), simulated=True)
    assert tier.local_identity_fn is not None
    assert tier.local_cold_search_after == 1 and tier.cold_search_after == 6
    tier.close()
    monkeypatch.setenv("TIER_LOCAL_COLD_SEARCH_AFTER", "6")
    tier = _tiered_vision_fn("backpack", Cloud(), load_brain_config(), simulated=True)
    assert tier.local_cold_search_after == 6
    tier.close()
    for bad in ("0", "x"):
        monkeypatch.setenv("TIER_LOCAL_COLD_SEARCH_AFTER", bad)
        with pytest.raises(ValueError, match="TIER_LOCAL_COLD_SEARCH_AFTER"):
            load_brain_config()


def _cold_counts(local, **kw):
    tier, cloud = tier_for([P(ABSENT)], local, async_cloud=False, cold_search_after=6, **kw)
    play(tier, 36)
    tier.close()
    return tier.stats.local_triggers.get(TRIGGER_COLD_SEARCH, 0), cloud.calls


def test_p_the_local_pace_asks_the_local_tier_every_frame():
    local_asked, cloud_calls = _cold_counts(Local(no()), local_cold_search_after=1)
    paced_asked, _ = _cold_counts(Local(no()))
    assert local_asked >= 30 and paced_asked <= 6
    assert cloud_calls <= 1           # the start's question at most: no is decided


def test_p_a_dead_local_vlm_never_brings_the_local_pace_to_the_cloud():
    """3.80.P review: an absent local tier hands its question to the cloud;
    at the local pace that must still come only at the cloud's bar."""
    _, fast = _cold_counts(Local(LocalVLMAbsent("down")), local_cold_search_after=1)
    _, paced = _cold_counts(Local(LocalVLMAbsent("down")))
    assert fast <= paced <= 8, (fast, paced)


def test_p_unable_answers_reach_the_cloud_only_at_its_bar():
    _, fast = _cold_counts(Local(unable("timeout")), local_cold_search_after=1)
    _, paced = _cold_counts(Local(unable("timeout")))
    assert fast <= paced + 1 and fast <= 8, (fast, paced)


# --- F-b: YOLOE + CLIP steer on an uncertain candidate ----------------------

def test_fb_a_band_candidate_off_centre_turns_toward_it():
    tier, _ = tier_for([P(ABSENT), P(ABSENT, 0.6, bearing=-20.0)], Local(no()),
                       steer_on_candidate=True, local_band_low=0.5)
    scenes = play(tier, 2)
    assert scenes[1]["safest_direction"] == "LEFT" and scenes[1]["turn_deg"] == 20
    assert "uncertain candidate" in scenes[1]["_navigate"]["reasoning"]
    assert tier.stats.candidate_steers == 1


def test_fb_a_centred_band_candidate_drives_forward():
    tier, _ = tier_for([P(ABSENT, 0.7, bearing=1.0)], Local(unable()),
                       steer_on_candidate=True)
    assert play(tier, 1)[0]["safest_direction"] == "FORWARD"


@pytest.mark.parametrize("p", [0.3, 0.49, 0.8, 0.95])
def test_fb_outside_the_band_no_candidate_steer(p):
    tier, _ = tier_for([P(ABSENT, p, bearing=-20.0)], Local(unable()),
                       steer_on_candidate=True)
    play(tier, 1)
    assert tier.stats.candidate_steers == 0


def test_fb_off_when_not_asked_for():
    tier, _ = tier_for([P(ABSENT, 0.6, bearing=-20.0)], Local(unable()))
    play(tier, 2)
    assert tier.stats.candidate_steers == 0


def test_fb_a_local_no_rejects_the_crop_until_the_band_run_breaks():
    # Band frames 2-3 (asked on 3, no); still in view on 4-12: no steer.
    # Frame 13 breaks the run; the crop back on 14 is steered on again.
    band = P(ABSENT, 0.6, bearing=-20.0)
    frames = [P(ABSENT)] + [band] * 11 + [P(ABSENT)] + [band] * 2
    tier, _ = tier_for(frames, Local(no()), steer_on_candidate=True, stale_after=5)
    scenes = play(tier, 15)
    steered = ["uncertain candidate" in s["_navigate"]["reasoning"] for s in scenes]
    assert steered[1] is True             # frame 2: steer while not yet asked
    assert not any(steered[2:13])         # rejected while the run lasts
    assert steered[13] is True            # a new run: steered again


def test_fb_a_no_on_a_frame_with_no_crop_rejects_nothing():
    frames = [P(ABSENT), P(ABSENT, 0.6, bearing=-20.0)]
    tier, _ = tier_for(frames, Local(no()), steer_on_candidate=True)
    scenes = play(tier, 2)
    assert "uncertain candidate" in scenes[1]["_navigate"]["reasoning"]


def test_fb_a_detection_still_steers_first():
    tier, _ = tier_for([P(DETECTED, bearing=10.0)], Local(yes()), steer_on_candidate=True)
    scene = play(tier, 1)[0]
    assert scene["safest_direction"] == "RIGHT" and tier.stats.candidate_steers == 0


def test_fb_a_panned_camera_gives_no_candidate_steer():
    p = P(ABSENT, 0.6, bearing=-20.0)
    p.pan_deg = 30.0
    tier, _ = tier_for([p], Local(unable()), steer_on_candidate=True)
    play(tier, 1)
    assert tier.stats.candidate_steers == 0


def test_fb_a_candidate_beats_a_held_direction():
    frames = [P(ABSENT), P(ABSENT, 0.6, bearing=20.0)]
    tier, _ = tier_for(frames, Local(no()), Direction(went("left")),
                       steer_on_candidate=True)
    scenes = play(tier, 2)
    assert scenes[0]["safest_direction"] == "LEFT"     # the held direction
    assert scenes[1]["safest_direction"] == "RIGHT"    # the candidate, measured now


def test_fb_the_cold_search_still_runs_while_steering():
    frames = [P(ABSENT, 0.55, bearing=-20.0)] * 6
    tier, _ = tier_for(frames, Local(no()), steer_on_candidate=True,
                       cold_search_after=3)
    play(tier, 6)
    assert tier.stats.local_triggers.get(TRIGGER_COLD_SEARCH, 0) >= 1


# --- the bug pass, 2026-10-16 -----------------------------------------------

def test_review_no_direction_is_asked_about_a_frame_that_sees_the_target():
    # Staleness on a detected frame, answered no: the target is in view, a
    # search direction is not wanted (and would hold the VLM from an arrival).
    d = Direction(went("left"))
    frames = [P(DETECTED, bearing=0.0)] * 6
    tier, _ = tier_for(frames, Local(no()), d, stale_after=2)
    play(tier, 6)
    assert d.asked == 0


def test_review_the_vlm_is_free_only_after_the_direction():
    import threading
    order = []

    class Slow(Direction):
        def __call__(self, frame):
            order.append(("direction", q.local_done.is_set()))
            return super().__call__(frame)

    from brain.tiered import _Question
    tier, _ = tier_for([P(ABSENT)], Local(no()), Slow(went("left")))
    q = _Question(trigger=TRIGGER_START, epoch=tier._epoch, pose=None, pan_deg=0.0,
                  hfov_deg=60.0)
    ans = tier._ask_tiers(frame(), q, Local(no()))
    assert order == [("direction", False)] and q.local_done.is_set()
    assert ans.direction["direction"] == "left"
    assert isinstance(threading.Event(), type(q.local_done))


def test_review_an_answered_direction_closes_the_search_question():
    # The sighting answer is unable every time; the direction answers. The
    # opening question must not be asked again on the next frame.
    d = Direction(went("left"))
    tier, _ = tier_for([P(ABSENT)] * 10, Local(unable()), d, cold_search_after=4)
    play(tier, 4)
    assert dict(tier.stats.local_triggers) == {TRIGGER_START: 1}
    assert d.asked == 1


def test_review_unable_sighting_and_direction_still_escalate():
    d = Direction(unable())
    tier, cloud = tier_for([P(ABSENT)] * 10, Local(unable()), d)
    play(tier, 3)
    assert cloud.calls == 1


def test_review_a_yes_drops_a_held_direction():
    d = Direction(went("left"))
    tier, _ = tier_for([P(ABSENT)] * 10, Local(no(), yes()), d, cold_search_after=2)
    scenes = play(tier, 5)
    assert scenes[0]["safest_direction"] == "LEFT"
    assert scenes[-1]["safest_direction"] == "RIGHT"     # the scan, not the old LEFT
    assert "local VLM's direction" not in scenes[-1]["_navigate"]["reasoning"]


@pytest.mark.parametrize("reply", [LocalVLMAbsent("gone"),
                                   {"status": "unsupported", "score": None}],
                         ids=["absent", "unsupported"])
def test_review_no_direction_to_be_had_changes_nothing(reply):
    # 3.80.D's behaviour: the no stands, nothing held, no escalation earned.
    d = Direction(reply)
    tier, cloud = tier_for([P(ABSENT)] * 10, Local(no()), d, cold_search_after=2)
    scenes = play(tier, 8)
    assert cloud.calls == 0 and tier._frame_unable == 0
    assert all(s["safest_direction"] == "RIGHT" for s in scenes)
    assert sum(tier.stats.local_direction.values()) == d.asked >= 2


@pytest.mark.parametrize("status", ["timeout", "busy"])
def test_review_no_direction_behind_a_sighting_still_draining(status):
    d = Direction(went("left"))
    tier, _ = tier_for([P(ABSENT)], Local(unable(status)), d)
    play(tier, 2)
    assert d.asked == 0


def test_review_close_drops_the_direction_client():
    tier, _ = tier_for([P(ABSENT)], Local(no()), Direction(went("left")))
    tier.close()
    assert tier.local_direction_fn is None


def test_review_a_service_without_the_route_is_unsupported_not_unable():
    import httpx
    from brain.local_vlm import LocalVLMClient
    transport = httpx.MockTransport(lambda req: httpx.Response(404, text="Not Found"))
    client = LocalVLMClient("http://vlm", "backpack", client=httpx.Client(transport=transport))
    import base64
    jpeg = base64.b64encode(b"\xff\xd8\xff\xe0" + b"\0" * 16).decode()
    assert client.direction({"image_base64": jpeg})["status"] == "unsupported"
