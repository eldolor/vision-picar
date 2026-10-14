"""
tests/test_tiered_frames.py

3.80.D (`docs/plans/ros-alignment/3.80-tiered-identity.md`): every question a
frame raises goes to the local VLM first -- today's four triggers, the
uncertain CLIP band and an out-of-vocabulary target -- and to the cloud only
after `tier_local_max_unable` unable answers in a row, or when the local VLM
is absent. Acceptance D1-D8, with perception, the local tier and the cloud
faked; D11 is `tests/demo_local_tier.py --frames`.
"""

import logging
import math
import threading
import time
from types import SimpleNamespace

import pytest

from brain.local_vlm import LocalVLMAbsent
from brain.navigate import CloudUnavailable
from brain.perceive import (ABSENT, DETECTED, UNAVAILABLE, Box, Candidate, Detection,
                            Perception)
from brain.tiered import (
    TRIGGER_BAND, TRIGGER_CANDIDATE, TRIGGER_COLD_SEARCH, TRIGGER_OOV,
    TRIGGER_STALE, TRIGGER_START, TieredVision)


@pytest.fixture(autouse=True)
def _quiet_logs():
    logging.disable(logging.WARNING)
    yield
    logging.disable(logging.NOTSET)


def P(status=ABSENT, p=None, bearing=None):
    """A frame's perception: `p` is the best candidate's CLIP P."""
    cands = [SimpleNamespace(probability=p, bearing_deg=bearing)] if p is not None else []
    best = cands[0] if (status == DETECTED and cands) else None
    if status == DETECTED and best is None:
        best = Candidate(detection=Detection(Box(0, 0, 1, 1), "thing", 1.0),
                         similarity=1.0, bearing_deg=bearing)
        cands = [best]
    return Perception(status=status, candidates=cands, best=best, reason="scripted")


class Pipeline:
    """Plays perceptions back, one per frame (the last one repeats)."""

    def __init__(self, frames, vocabulary=None, hfov_deg=60.0):
        self.frames = list(frames)
        self.i = 0
        self.vocabulary = vocabulary
        self.hfov_deg = hfov_deg

    def perceive(self, frame):
        p = self.frames[min(self.i, len(self.frames) - 1)]
        self.i += 1
        return p


def vocab(in_vocabulary):
    return SimpleNamespace(in_vocabulary=in_vocabulary,
                           as_dict=lambda: {"in_vocabulary": in_vocabulary})


class Local:
    """A scripted local tier for sightings: each ask returns the next answer
    (the last repeats); an exception is raised. Records the box flag."""

    def __init__(self, *answers, gate=None):
        self.answers = list(answers)
        self.asked = 0
        self.boxes = []
        self.gate = gate

    def __call__(self, frame, box=False):
        self.boxes.append(box)
        if self.gate is not None:
            self.gate.wait(5)
        a = self.answers[min(self.asked, len(self.answers) - 1)]
        self.asked += 1
        if isinstance(a, BaseException):
            raise a
        return dict(a)


def yes(score=0.9, frac=None):
    out = {"status": "answered", "score": score}
    if frac is not None:
        out["box"] = [0, 0, 1, 1]
        out["box_frac"] = frac
    return out


def no(score=0.02):
    return {"status": "answered", "score": score}


def unable(status="unparsed", score=None):
    return {"status": status, "score": score}


class Cloud:
    def __init__(self, direction="LEFT", fail=None):
        self.calls = 0
        self.direction = direction
        self.fail = fail

    def __call__(self, frame):
        self.calls += 1
        if self.fail is not None:
            raise self.fail
        return {"obstacles_ahead": [], "free_space": "clear", "doorway_visible": False,
                "important_objects": [], "safest_direction": self.direction,
                "_navigate": {"reasoning": "cloud", "target_visible": False}}


def frame(heading=None, pan=0.0):
    f = {"image_base64": "x", "image_width": 640, "pan_deg": pan}
    if heading is not None:
        f["odometry"] = {"usable": True, "distance_m": 0.0, "heading_deg": heading}
    return f


def tier_for(frames, local, cloud=None, **kw):
    kw.setdefault("cold_search_after", 99)
    kw.setdefault("stale_after", 0)
    cloud = cloud or Cloud()
    pipe = Pipeline(frames, vocabulary=kw.pop("vocabulary", None))
    return TieredVision(pipe, cloud, local_sighting_fn=local, **kw), cloud


def play(tier, n, make=frame):
    return [tier(make()) for _ in range(n)]


def asked(tier):
    return dict(tier.stats.local_triggers)


# --- D1: the uncertain band --------------------------------------------------

def test_d1_two_band_frames_ask_the_local_vlm_never_the_cloud_first():
    local = Local(no())
    tier, cloud = tier_for([P(ABSENT, 0.2), P(ABSENT, 0.6), P(ABSENT, 0.6)], local)
    play(tier, 3)
    assert asked(tier) == {TRIGGER_START: 1, TRIGGER_BAND: 1}
    assert cloud.calls == 0 and tier.stats.cloud_calls == 0
    assert local.boxes == [True, True]


@pytest.mark.parametrize("p", [0.49, 0.2])
def test_d1_below_the_band_does_not_ask(p):
    tier, cloud = tier_for([P(ABSENT, p)], Local(no()))
    play(tier, 6)
    assert asked(tier) == {TRIGGER_START: 1}


def test_d1_a_candidate_at_the_gate_is_not_the_band():
    # Gated by margin, an ABSENT frame can carry P 0.8: that is above the band.
    tier, cloud = tier_for([P(ABSENT, 0.8)], Local(no()))
    play(tier, 6)
    assert TRIGGER_BAND not in asked(tier)


def test_d1_asked_once_per_run_and_a_broken_run_arms_it_again():
    frames = [P(ABSENT, 0.2)] + [P(ABSENT, 0.6)] * 5 + [P(ABSENT, 0.2)] + [P(ABSENT, 0.6)] * 2
    tier, cloud = tier_for(frames, Local(no()))
    play(tier, len(frames))
    assert asked(tier)[TRIGGER_BAND] == 2
    assert cloud.calls == 0


def test_d8_no_band_or_oov_question_without_a_local_tier():
    frames = [P(ABSENT, 0.6)] * 8
    tier = TieredVision(Pipeline(frames, vocabulary=vocab(False)), Cloud(),
                        cold_search_after=99, stale_after=0)
    play(tier, 8)
    assert tier.stats.triggers == {TRIGGER_START: 1}
    assert tier.stats.local_triggers == {}


# --- D2: out of vocabulary ---------------------------------------------------

def test_d2_out_of_vocabulary_asks_the_local_vlm_after_two_absent_frames():
    local = Local(no())
    tier, cloud = tier_for([P(ABSENT)], local, vocabulary=vocab(False))
    play(tier, 7)
    # start, then every 2 absent frames after an answer: frames 3, 5, 7.
    assert asked(tier) == {TRIGGER_START: 1, TRIGGER_OOV: 3}
    assert cloud.calls == 0


def test_d2_in_vocabulary_keeps_the_cold_search_bar():
    tier, cloud = tier_for([P(ABSENT)], Local(no()), vocabulary=vocab(True),
                           cold_search_after=6)
    play(tier, 7)
    assert asked(tier) == {TRIGGER_START: 1, TRIGGER_COLD_SEARCH: 1}
    assert cloud.calls == 0


# --- D3: today's four triggers go local first --------------------------------

@pytest.mark.parametrize("answer", [yes(), no()])
def test_d3_every_trigger_goes_to_the_local_vlm_first(answer):
    # start on 1, cold search on 4, candidate on 6, staleness on 11.
    frames = ([P(ABSENT)] * 4 + [P(DETECTED, bearing=0.0)] * 8)
    tier, cloud = tier_for(frames, Local(answer), cold_search_after=3, stale_after=4)
    play(tier, len(frames))
    got = asked(tier)
    for t in (TRIGGER_START, TRIGGER_COLD_SEARCH, TRIGGER_CANDIDATE, TRIGGER_STALE):
        assert got.get(t, 0) >= 1, (t, got)
    assert cloud.calls == 0 and tier.stats.cloud_calls == 0
    assert tier.stats.triggers == {}


# --- D4: unable --------------------------------------------------------------

UNABLE = [unable("unparsed"), unable("timeout"), unable("busy"), unable("error"),
          {"status": "answered", "score": 0.3}, {"status": "answered", "score": None}]


@pytest.mark.parametrize("u", UNABLE, ids=["unparsed", "timeout", "busy", "error",
                                           "band", "noscore"])
def test_d4_two_unable_answers_hand_the_next_question_to_the_cloud(u):
    local = Local(u)
    tier, cloud = tier_for([P(ABSENT)], local)
    scenes = play(tier, 3)
    # Frame 1 asks (unable), frame 2 asks the same question again (unable),
    # frame 3 goes to the cloud -- and its answer is applied as today's.
    assert local.asked == 2 and cloud.calls == 1
    assert tier.stats.triggers == {TRIGGER_START: 1}
    assert tier.stats.frame_escalations == {"unable": 1}
    assert scenes[2]["_tier"]["cloud_called"] and scenes[2]["safest_direction"] == "LEFT"


def test_d4_a_decided_answer_resets_the_count():
    local = Local(unable(), no(), unable(), no())
    tier, cloud = tier_for([P(ABSENT)], local, vocabulary=vocab(False))
    play(tier, 12)
    assert cloud.calls == 0


def test_d4_unable_keeps_the_pacing_so_the_question_is_asked_again_next_frame():
    local = Local(no(), unable(), no())
    frames = [P(ABSENT)] * 10
    tier, cloud = tier_for(frames, local, cold_search_after=3)
    play(tier, 6)
    # start (no) on 1; cold search on 4 (unable) and again on 5 (no).
    assert asked(tier) == {TRIGGER_START: 1, TRIGGER_COLD_SEARCH: 2}


def test_d4_unable_on_a_candidate_arms_its_edge_again():
    local = Local(no(), unable(), yes())
    frames = [P(ABSENT)] + [P(DETECTED, bearing=0.0)] * 4
    tier, cloud = tier_for(frames, local)
    play(tier, 5)
    assert asked(tier) == {TRIGGER_START: 1, TRIGGER_CANDIDATE: 2}


def test_d4_unable_on_the_band_asks_it_again():
    local = Local(no(), unable(), no())
    frames = [P(ABSENT, 0.2)] + [P(ABSENT, 0.6)] * 5
    tier, cloud = tier_for(frames, local)
    play(tier, 6)
    assert asked(tier) == {TRIGGER_START: 1, TRIGGER_BAND: 2}


# --- D5: absent ---------------------------------------------------------------

@pytest.mark.parametrize("err", [LocalVLMAbsent("refused"), RuntimeError("client fault")])
def test_d5_absent_asks_the_cloud_about_the_same_frame(err):
    local = Local(err)
    tier, cloud = tier_for([P(ABSENT)], local)
    scenes = play(tier, 1)
    assert cloud.calls == 1 and tier.stats.cloud_calls == 1
    assert tier.stats.triggers == {TRIGGER_START: 1}
    assert tier.stats.frame_escalations == {"absent": 1}
    assert scenes[0]["_tier"]["cloud_called"] and scenes[0]["safest_direction"] == "LEFT"


def test_d5_the_call_cap_still_holds_for_an_absent_local_tier():
    tier, cloud = tier_for([P(ABSENT)], Local(LocalVLMAbsent("gone")),
                           vocabulary=vocab(False), max_calls=1)
    play(tier, 8)
    assert cloud.calls == 1 and tier.stats.cloud_calls == 1


# --- D6: where a yes points ---------------------------------------------------

def test_d6_a_yes_with_a_box_turns_toward_it_then_holds_forward():
    # Box centre at 0.75 of a 60-degree view: 15 degrees right.
    local = Local(yes(frac=[0.7, 0.4, 0.8, 0.6]))
    tier, cloud = tier_for([P(ABSENT)], local, hold_goal=True)
    s1, s2, s3 = play(tier, 3)
    assert s1["safest_direction"] == "RIGHT" and s1["turn_deg"] == 15
    assert s2["safest_direction"] == "FORWARD" and s3["safest_direction"] == "FORWARD"


def test_d6_the_bearing_is_corrected_for_a_turn_made_while_the_question_was_out():
    gate = threading.Event()
    local = Local(yes(frac=[0.7, 0.4, 0.8, 0.6]), gate=gate)
    tier, cloud = tier_for([P(ABSENT)], local, async_cloud=True, hold_goal=True)
    headings = iter([90.0, 100.0, 100.0])
    tier(frame(heading=next(headings)))      # dispatched while facing 90
    gate.set()
    tier._inflight.result(timeout=5)
    # The robot turned 10 degrees right meanwhile: 15 - 10 = 5 degrees right.
    s = tier(frame(heading=next(headings)))
    assert s["safest_direction"] == "RIGHT" and s["turn_deg"] == 5
    tier.close()


def test_d6_a_yes_without_a_box_and_a_no_change_no_direction():
    for answer in (yes(), no()):
        tier, cloud = tier_for([P(ABSENT)], Local(answer), hold_goal=True)
        s = play(tier, 2)
        assert all(x["safest_direction"] == "RIGHT" and x["turn_deg"] == 45 for x in s), s


def test_d6_a_no_drops_a_forward_held_from_a_local_yes():
    local = Local(yes(frac=[0.45, 0.4, 0.55, 0.6]), no())
    tier, cloud = tier_for([P(ABSENT)], local, vocabulary=vocab(False), hold_goal=True)
    s = play(tier, 4)
    assert [x["safest_direction"] for x in s[:2]] == ["FORWARD", "FORWARD"]
    # Frame 3 asked again (out of vocabulary) and heard no: the FORWARD is gone.
    assert s[2]["safest_direction"] == "RIGHT"


def test_d6_perception_still_steers_first():
    local = Local(yes(frac=[0.7, 0.4, 0.8, 0.6]))
    tier, cloud = tier_for([P(DETECTED, bearing=-20.0)], local)
    s = play(tier, 1)[0]
    assert s["safest_direction"] == "LEFT" and s["turn_deg"] == 20


# --- D7: one question at a time; Stop; B3.2 ------------------------------------

def test_d7_one_question_in_flight_across_both_tiers():
    gate = threading.Event()
    local = Local(no(), gate=gate)
    tier, cloud = tier_for([P(ABSENT)], local, vocabulary=vocab(False), async_cloud=True)
    for _ in range(6):
        s = tier(frame())
    assert local.asked == 0 and len(local.boxes) == 1 and cloud.calls == 0
    assert s["_tier"]["in_flight"] == TRIGGER_START and s["_tier"]["in_flight_tier"] == "local"
    gate.set()
    tier._inflight.result(timeout=5)
    tier.close()


def test_d7_a_stop_during_a_local_question_sends_no_cloud_call():
    gate = threading.Event()
    local = Local(LocalVLMAbsent("gone"), gate=gate)
    tier, cloud = tier_for([P(ABSENT)], local, async_cloud=True)
    tier(frame())
    fut = tier._inflight
    tier.close()
    gate.set()
    fut.result(timeout=5)
    assert cloud.calls == 0 and tier.stats.cloud_calls == 0


def test_d7_a_late_local_answer_after_a_stop_is_not_applied():
    gate = threading.Event()
    local = Local(yes(frac=[0.9, 0.4, 1.0, 0.6]), gate=gate)
    tier, cloud = tier_for([P(ABSENT)], local, async_cloud=True)
    tier(frame())
    fut = tier._inflight
    tier.reset_epoch()
    gate.set()
    fut.result(timeout=5)
    s = tier(frame())
    assert s["safest_direction"] != "RIGHT" or s["turn_deg"] == 45
    assert tier.stats.local_frame == {}


def test_d7_a_cloud_failure_after_an_escalation_reaches_b32():
    local = Local(LocalVLMAbsent("gone"))
    tier, cloud = tier_for([P(ABSENT)], local, cloud=Cloud(fail=CloudUnavailable("down")),
                           async_cloud=True)
    tier(frame())
    tier._inflight.exception(timeout=5)
    with pytest.raises(CloudUnavailable):
        tier(frame())
    tier.close()


def test_d7_a_local_failure_is_never_a_mission_failure():
    local = Local(unable("error"))
    tier, cloud = tier_for([P(ABSENT)], local, vocabulary=vocab(False), async_cloud=True)
    for _ in range(4):
        tier(frame())
        if tier._inflight is not None:
            tier._inflight.result(timeout=5)
    tier(frame())   # no raise
    tier.close()


# --- the mission path: found with no cloud at all -------------------------------

def test_mission_found_through_the_local_tier_with_the_cloud_down():
    """D3 + 3.80.C on the whole path: MissionRunner -> agent -> safety ->
    MockRobot, every trigger and the arrival answered locally, the cloud
    refusing every call."""
    from brain.perceive import FrameReportedPipeline
    from control.mission_runner import FOUND, MissionRunner
    from sim.mock_robot import MockRobot
    from tests.conftest import mock_world_for
    from tests.test_bearing_turns import ARRIVED_CELLS, CLEAR_STARTS, GOAL, TARGET, _build

    x, y, off = CLEAR_STARTS[0]
    grid = _build()
    grid.x, grid.y = x, y
    grid.theta = math.atan2(GOAL[1] - y, GOAL[0] - x) + math.radians(off)
    robot = MockRobot(grid, render=False)
    cloud = Cloud(fail=CloudUnavailable("refused"))
    local = Local(yes(frac=[0.45, 0.4, 0.55, 0.6]))
    tier = TieredVision(FrameReportedPipeline(TARGET), cloud, confirm_vision_fn=cloud,
                        steer_on_sight=True, hold_goal=True, async_cloud=True,
                        local_identity_fn=lambda f: yes(), local_sighting_fn=local)
    runner = MissionRunner(robot, target_object=TARGET, max_steps=60, policy="tiered",
                           vision_fn=tier, world=mock_world_for(robot), vision_timeout_s=5.0)
    runner.start()
    while runner.tick():
        time.sleep(0.01)
    status = runner.status()
    assert status["outcome"] == FOUND, status
    assert math.dist((grid.x, grid.y), GOAL) <= ARRIVED_CELLS
    assert cloud.calls == 0 and tier.stats.cloud_calls == 0
    assert sum(tier.stats.local_triggers.values()) >= 1


# --- D8: the brain's wiring -----------------------------------------------------

def test_d8_the_brain_wires_sightings_to_the_same_client_with_the_box(monkeypatch):
    from control.brain_config import load_brain_config
    from control.brain_server import _tiered_vision_fn
    monkeypatch.delenv("LOCAL_VLM_URL", raising=False)
    config = load_brain_config()
    assert config["tier_local_band_low"] == 0.5
    tier = _tiered_vision_fn("backpack", Cloud(), config, simulated=True)
    assert tier.local_sighting_fn is None and tier.local_identity_fn is None
    tier.close()
    monkeypatch.setenv("LOCAL_VLM_URL", "http://127.0.0.1:9")
    tier = _tiered_vision_fn("backpack", Cloud(), load_brain_config(), simulated=True)
    assert tier.local_sighting_fn.keywords == {"box": True}
    assert tier.local_sighting_fn.func.__self__ is tier.local_identity_fn
    assert tier.local_band_low == 0.5
    tier.close()


@pytest.mark.parametrize("value", [0.0, 0.8, 1.2])
def test_d8_config_refuses_a_band_outside_zero_to_the_gate(tmp_path, value):
    from control.brain_config import load_brain_config
    cfg = tmp_path / "robot.yaml"
    cfg.write_text(f"brain:\n  tier_local_band_low: {value}\n")
    with pytest.raises(ValueError, match="tier_local_band_low"):
        load_brain_config(str(cfg))
