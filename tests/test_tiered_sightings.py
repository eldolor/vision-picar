"""
tests/test_tiered_sightings.py

3.80.D (`docs/plans/ros-alignment/3.80-tiered-identity.md`): a frame's
question -- today's four cloud triggers, and a candidate in the uncertain
band below perception's gate -- goes to the local VLM first, and to the cloud
only after `tier_local_max_unable` unable answers in a row or when the local
tier is absent. Acceptance D1-D7 against the policy with scripted perception,
a scripted local tier and a counting cloud; D8 (cloud calls per mission)
through the whole mission path in `tests/test_tiered_local_sweep.py`.
"""

import base64
import logging
import threading
import time
from types import SimpleNamespace

import httpx
import pytest

from brain.local_vlm import LocalVLMAbsent, LocalVLMClient
from brain.perceive import ABSENT, DETECTED, UNAVAILABLE, Perception
from brain.tiered import (
    TRIGGER_CANDIDATE, TRIGGER_COLD_SEARCH, TRIGGER_STALE, TRIGGER_START,
    TRIGGER_UNCERTAIN, TieredVision)


@pytest.fixture(autouse=True)
def _quiet_logs():
    logging.disable(logging.WARNING)
    yield
    logging.disable(logging.NOTSET)


def cand(p, label="bag", bearing=20.0):
    """A scored crop, as far as the tier reads one."""
    return SimpleNamespace(probability=p, detection=SimpleNamespace(label=label),
                           bearing_deg=bearing, margin=0.0, similarity=0.3)


def absent(*cands):
    return Perception(status=ABSENT, candidates=list(cands))


def detected(bearing=0.0, label="bag"):
    c = cand(0.95, label, bearing)
    return Perception(status=DETECTED, candidates=[c], best=c)


class Script:
    """Perception that reads the next scripted frame (the last one repeats)."""

    def __init__(self, *frames):
        self.frames = list(frames)
        self.i = 0

    def perceive(self, frame):
        p = self.frames[min(self.i, len(self.frames) - 1)]
        self.i += 1
        return p


def yes(score=0.9):
    return {"status": "answered", "score": score}


def no(score=0.02):
    return {"status": "answered", "score": score}


class Local:
    """The local tier: each ask returns the next answer (the last repeats);
    an exception is raised. Records the triggers it was asked about via the
    frame, and the most requests it ever had out at once."""

    def __init__(self, *answers, gate=None):
        self.answers = list(answers)
        self.asked = 0
        self.gate = gate                 # a threading.Event to hold each ask on
        self.out = 0
        self.most_out = 0
        self.lock = threading.Lock()

    def __call__(self, frame):
        with self.lock:
            a = self.answers[min(self.asked, len(self.answers) - 1)]
            self.asked += 1
            self.out += 1
            self.most_out = max(self.most_out, self.out)
        try:
            if self.gate is not None:
                assert self.gate.wait(5), "the test never released the local call"
            if isinstance(a, BaseException):
                raise a
            return a
        finally:
            with self.lock:
                self.out -= 1


class Cloud:
    def __init__(self, direction="LEFT"):
        self.calls = 0
        self.direction = direction

    def __call__(self, frame):
        self.calls += 1
        return {"obstacles_ahead": [], "free_space": "unknown", "doorway_visible": False,
                "important_objects": [], "safest_direction": self.direction,
                "_navigate": {"target_visible": False, "reasoning": "cloud"}}


def tier_for(script, local, cloud, **kw):
    kw.setdefault("async_cloud", True)
    return TieredVision(script, cloud, local_identity_fn=local, **kw)


def settle(tier):
    """Let the local and cloud calls out finish, so the next frame collects
    them: the frame-to-frame timing of a real mission, made deterministic."""
    call = tier._sight_call
    if call is not None:
        try:
            call[3].result(5)
        except BaseException:  # noqa: BLE001
            pass
    fut = tier._inflight
    if fut is not None:
        try:
            fut.result(5)
        except BaseException:  # noqa: BLE001
            pass


def run(tier, n, frame=None):
    scenes = []
    for _ in range(n):
        scenes.append(tier(dict(frame or {})))
        settle(tier)
    return scenes


# A frame the real client can encode (it sends JPEG bytes as they are).
JPEG_FRAME = {"image_base64": base64.b64encode(b"\xff\xd8not-really-a-jpeg").decode()}


# ---------- D1: local first, on all four triggers ----------

def test_d1_every_trigger_asks_the_local_tier_and_an_answer_costs_no_cloud_call():
    # start (absent) -> 6 absent: cold_search -> detected x2: candidate ->
    # detected on: staleness 8 frames after the last answer.
    script = Script(*([absent()] * 8 + [detected()] * 14))
    local, cloud = Local(no()), Cloud()
    tier = tier_for(script, local, cloud)
    run(tier, 22)
    assert set(tier.stats.local_triggers) == {TRIGGER_START, TRIGGER_COLD_SEARCH,
                                              TRIGGER_CANDIDATE, TRIGGER_STALE}, \
        tier.stats.local_triggers
    assert cloud.calls == 0 and tier.stats.cloud_calls == 0
    assert tier.stats.local.get("no") == local.asked


def test_d1_a_yes_is_an_answer_too():
    tier = tier_for(Script(absent()), Local(yes()), cloud := Cloud())
    run(tier, 20)
    assert cloud.calls == 0 and tier.stats.local_triggers.get(TRIGGER_START) == 1


def test_d1_pacing_counts_from_the_local_answer():
    """An answered cold search is not asked again on the next frame."""
    tier = tier_for(Script(absent()), local := Local(no()), Cloud())
    run(tier, 7)                       # the start, then cold_search on frame 7
    asked = local.asked
    run(tier, 3)
    assert local.asked == asked


def test_d1_the_frame_says_what_went_to_the_local_tier():
    tier = tier_for(Script(absent()), Local(no()), Cloud())
    first = tier({})
    assert first["_tier"]["local_dispatched"] == TRIGGER_START
    assert first["_tier"]["cloud_called"] is False
    assert "local VLM" in first["_navigate"]["reasoning"]
    settle(tier)
    second = tier({})
    assert second["_tier"]["local_landed"]["answer"] == "no"
    assert second["_tier"]["local_landed"]["trigger"] == TRIGGER_START
    third = tier({})
    assert third["_tier"]["local_landed"] is None, "a landed answer is shown once"


# ---------- D2: the uncertain band ----------

def _band_tier(*answers, frames=None, **kw):
    frames = frames or ([absent()] + [absent(cand(0.6, bearing=-30.0))] * 10)
    local = Local(no(), *answers)       # the start's answer, then the band's
    tier = tier_for(Script(*frames), local, Cloud(), **kw)
    return tier, local


def test_d2_a_band_candidate_asks_once_and_a_yes_steers_on_it():
    tier, local = _band_tier(yes())
    scenes = run(tier, 11)
    assert tier.stats.local_triggers.get(TRIGGER_UNCERTAIN) == 1
    # Frame 1 the start; frames 2-3 the band, asked on 3; landed by 4.
    assert scenes[2]["_tier"]["local_dispatched"] == TRIGGER_UNCERTAIN
    for s in scenes[3:]:
        assert s["_perception"]["status"] == DETECTED
        assert s["_navigate"]["target_visible"] is True
        assert s["safest_direction"] == "LEFT" and s["turn_deg"] == 30
    assert "local VLM said yes" in scenes[4]["_perception"]["reason"]
    # The promoted detection is then a candidate sighting like any other,
    # and asks the local tier once more (free).
    assert tier.stats.local_triggers.get(TRIGGER_CANDIDATE) == 1


def test_d2_a_no_leaves_it_absent_and_asks_no_more_while_it_stays():
    tier, local = _band_tier(no())
    scenes = run(tier, 11)
    assert tier.stats.local_triggers.get(TRIGGER_UNCERTAIN) == 1
    assert all(s["_perception"]["status"] == ABSENT for s in scenes)


def test_d2_a_frame_without_one_arms_it_again():
    b = absent(cand(0.6))
    tier, _ = _band_tier(no(), frames=[absent(), b, b, b, absent(), b, b, b])
    run(tier, 8)
    assert tier.stats.local_triggers.get(TRIGGER_UNCERTAIN) == 2


def test_d2_one_band_frame_is_not_enough():
    b = absent(cand(0.6))
    tier, _ = _band_tier(no(), frames=[absent(), b, absent(), b, absent(), b, absent()])
    run(tier, 7)
    assert TRIGGER_UNCERTAIN not in tier.stats.local_triggers


def test_d2_below_the_band_asks_nothing():
    tier, _ = _band_tier(yes(), frames=[absent()] + [absent(cand(0.49))] * 5)
    run(tier, 6)
    assert TRIGGER_UNCERTAIN not in tier.stats.local_triggers


def test_d2_a_yes_promotes_only_that_label():
    frames = ([absent()] + [absent(cand(0.6, "bag"))] * 3
              + [absent(cand(0.6, "box"))] * 2)
    tier, _ = _band_tier(yes(), no(), frames=frames)
    scenes = run(tier, 6)
    assert scenes[3]["_perception"]["status"] == DETECTED
    assert scenes[4]["_perception"]["status"] == ABSENT
    assert scenes[4]["_perception"]["label"] is None


def test_d2_out_of_view_ends_the_promotion():
    b = absent(cand(0.6))
    tier, _ = _band_tier(yes(), no(), frames=[absent(), b, b, b, absent(), b])
    scenes = run(tier, 6)
    assert scenes[3]["_perception"]["status"] == DETECTED
    assert scenes[5]["_perception"]["status"] == ABSENT


def test_d2_an_unavailable_frame_keeps_the_promotion():
    b = absent(cand(0.6))
    tier, _ = _band_tier(yes(), frames=[absent(), b, b, b,
                                        Perception(status=UNAVAILABLE), b])
    scenes = run(tier, 6)
    assert scenes[5]["_perception"]["status"] == DETECTED


def test_d2_with_the_local_tier_off_a_band_candidate_asks_nothing():
    b = absent(cand(0.6))
    plain, banded = Cloud(), Cloud()
    a = TieredVision(Script(*([absent()] * 20)), plain)
    c = TieredVision(Script(absent(), *([b] * 19)), banded)
    sa, sc = run(a, 20), run(c, 20)
    assert banded.calls == plain.calls
    assert [s["_tier"]["trigger"] for s in sa] == [s["_tier"]["trigger"] for s in sc]
    assert all("local_dispatched" not in s["_tier"] for s in sc)
    assert c.stats.local_triggers == {}


# ---------- D3: unable twice, then the cloud ----------

@pytest.mark.parametrize("answer", [
    {"status": "answered", "score": 0.3}, {"status": "unparsed", "score": None},
    {"status": "timeout", "score": None}, {"status": "busy", "score": None},
    {"status": "error", "score": None}],
    ids=["band", "unparsed", "timeout", "busy", "error"])
def test_d3_two_unable_answers_then_the_cloud_once(answer):
    local, cloud = Local(answer, answer, no()), Cloud(direction="LEFT")
    tier = tier_for(Script(absent()), local, cloud)
    scenes = run(tier, 4)
    # Frame 1 asks (start), 2 asks again, 3 hands it to the cloud.
    assert [s["_tier"].get("local_dispatched") for s in scenes[:2]] == [TRIGGER_START] * 2
    assert scenes[2]["_tier"]["cloud_called"] is True
    assert scenes[2]["_tier"]["trigger"] == TRIGGER_START
    assert cloud.calls == 1 and local.asked == 2
    assert tier.stats.escalations == 1
    assert scenes[3]["safest_direction"] == "LEFT", "the cloud's direction is held"


def test_d3_the_count_starts_again_after_the_cloud():
    u = {"status": "unparsed", "score": None}
    local, cloud = Local(u, u, u, no()), Cloud()
    tier = tier_for(Script(absent()), local, cloud)
    run(tier, 10)   # start: local, local, cloud; cold_search at 6 absent: local again
    assert cloud.calls == 1
    assert local.asked >= 3
    assert tier.stats.local_triggers.get(TRIGGER_COLD_SEARCH, 0) >= 1


def test_d3_a_local_answer_starts_the_count_again():
    u = {"status": "unparsed", "score": None}
    local, cloud = Local(u, no(), u, no()), Cloud()
    tier = tier_for(Script(absent()), local, cloud)
    run(tier, 20)
    assert cloud.calls == 0


def test_d3_synchronous_policy_answers_on_the_same_frame():
    u = {"status": "unparsed", "score": None}
    local, cloud = Local(u, u), Cloud()
    tier = tier_for(Script(absent()), local, cloud, async_cloud=False)
    scenes = [tier({}) for _ in range(3)]
    assert scenes[0]["_tier"]["local_landed"]["answer"] == "unable_unparsed"
    assert scenes[2]["_tier"]["cloud_called"] is True and cloud.calls == 1


# ---------- D4: absent ----------

def _client_503():
    return LocalVLMClient("http://local", "bag", client=httpx.Client(
        transport=httpx.MockTransport(lambda r: httpx.Response(503, text="loading"))))


@pytest.mark.parametrize("make", [
    lambda: Local(LocalVLMAbsent("refused")), lambda: Local(ValueError("client bug")),
    _client_503], ids=["refused", "client-fault", "503"])
def test_d4_an_absent_local_tier_hands_the_question_to_the_cloud_next_frame(make):
    cloud = Cloud()
    tier = tier_for(Script(absent()), make(), cloud)
    scenes = run(tier, 3, JPEG_FRAME)
    assert scenes[0]["_tier"]["local_dispatched"] == TRIGGER_START
    assert scenes[1]["_tier"]["cloud_called"] is True and cloud.calls == 1
    assert tier.stats.local.get("absent") == 1
    assert tier.stats.escalations == 1


def test_d4_each_later_question_tries_the_local_tier_first():
    local, cloud = Local(LocalVLMAbsent("gone"), no()), Cloud()
    tier = tier_for(Script(absent()), local, cloud)
    run(tier, 10)
    assert cloud.calls == 1
    assert tier.stats.local_triggers.get(TRIGGER_COLD_SEARCH) == 1


# ---------- D5: one local call at a time ----------

def test_d5_never_two_local_requests_and_the_question_waits():
    gate = threading.Event()
    local, cloud = Local(no(), gate=gate), Cloud()
    tier = tier_for(Script(absent()), local, cloud)
    tier({})                                   # the start's question, held
    for _ in range(8):                         # cold_search fires meanwhile
        s = tier({})
    assert local.asked == 1
    assert s["_tier"]["local_in_flight"] == TRIGGER_START
    gate.set()
    settle(tier)
    tier({})                                   # collects the start's answer
    assert local.most_out == 1 and cloud.calls == 0


def test_d5_a_question_held_back_is_asked_once_the_slot_frees():
    gate = threading.Event()
    local = Local(no(), gate=gate)
    tier = tier_for(Script(absent(), *([detected()] * 4)), local, Cloud())
    tier({})                                   # start, held
    tier({}), tier({})                         # candidate_sighting fires, held back
    gate.set()
    settle(tier)
    tier({})
    settle(tier)
    assert tier.stats.local_triggers.get(TRIGGER_CANDIDATE) == 1


def test_d5_an_arrival_waits_for_a_frames_question_and_decides_on_its_own():
    gate = threading.Event()
    local = Local(no(), yes(), gate=gate)      # the frame's no, the arrival's yes
    tier = tier_for(Script(absent()), local, Cloud())
    tier({})                                   # the start's question, held
    got = {}
    t = threading.Thread(target=lambda: got.update(v=tier.confirm_arrival({})))
    t.start()
    time.sleep(0.2)
    assert local.asked == 1, "the arrival did not send beside the frame's question"
    gate.set()
    t.join(5)
    assert got["v"]["confirmed"] and got["v"]["tier"] == "local"
    assert local.most_out == 1 and tier._local_unable == 0


def test_d5_an_arrival_check_still_out_holds_the_frames_questions():
    gate = threading.Event()
    local = Local(yes(), gate=gate)
    tier = tier_for(Script(absent()), local, Cloud())
    cid = tier.begin_local()
    t = threading.Thread(target=tier.confirm_local, args=({}, cid))
    t.start()
    time.sleep(0.2)
    tier.abandon_local(cid)                    # the runner gave up; it is still out
    s = tier({})
    assert local.asked == 1 and "still out" in s["_navigate"]["reasoning"]
    gate.set()
    t.join(5)
    tier({})
    settle(tier)
    assert tier.stats.local_triggers.get(TRIGGER_START) == 1 and local.most_out == 1


def test_d5_an_answer_from_an_ended_mission_changes_nothing():
    gate = threading.Event()
    local = Local({"status": "unparsed", "score": None}, gate=gate)
    tier = tier_for(Script(absent()), local, cloud := Cloud())
    tier({})
    tier.reset_epoch()
    gate.set()
    settle(tier)
    time.sleep(0.05)
    tier._collect_local()
    assert tier.stats.local == {} and tier._pending_question is None
    assert cloud.calls == 0


# ---------- D6: Stop ----------

@pytest.mark.parametrize("answer", [{"status": "unparsed", "score": None},
                                    LocalVLMAbsent("gone")], ids=["unable", "absent"])
def test_d6_after_a_stop_nothing_more_is_asked(answer):
    gate = threading.Event()
    local, cloud = Local(answer, gate=gate), Cloud()
    # One unable answer is enough to hand the question up, if anything could.
    tier = tier_for(Script(absent()), local, cloud, local_max_unable=1)
    tier({})
    tier.close()
    gate.set()
    time.sleep(0.1)
    for _ in range(10):
        tier({})
    assert cloud.calls == 0 and local.asked == 1


def test_d6_a_stop_between_the_check_and_the_send_sends_nothing():
    local, cloud = Local(no()), Cloud()
    tier = tier_for(Script(absent()), local, cloud)
    tier._closed = True                        # close() landed mid-frame
    s = tier({})
    assert local.asked == 0 and s["_navigate"]["reasoning"].endswith("mission ended")
    assert tier._local_inflight is None


# ---------- D7: off by default ----------

def test_d7_without_a_local_tier_the_frames_are_what_they_were():
    script = [absent()] * 8 + [absent(cand(0.7))] * 4 + [detected()] * 10
    cloud = Cloud()
    tier = TieredVision(Script(*script), cloud, async_cloud=True)
    scenes = run(tier, 22)
    assert tier.stats.local_triggers == {} and tier.stats.local_calls == 0
    for s in scenes:
        assert not any(k.startswith("local_") for k in s["_tier"]), s["_tier"].keys()


# ---------- review fixes (/code-review, 2026-10-15) ----------

def test_review_an_absent_tier_hands_the_band_up_and_the_clouds_yes_promotes():
    class Sees(Cloud):
        def __call__(self, frame):
            scene = super().__call__(frame)
            scene["_navigate"]["target_visible"] = True
            return scene
    b = absent(cand(0.6, bearing=-30.0))
    local, cloud = Local(no(), LocalVLMAbsent("gone")), Sees()
    tier = tier_for(Script(absent(), *([b] * 8)), local, cloud)
    scenes = run(tier, 8)
    assert tier.stats.triggers.get(TRIGGER_UNCERTAIN) == 1
    # Promoted, it is a candidate sighting, asked (absent) and handed up too.
    assert scenes[-1]["_perception"]["status"] == DETECTED


def test_review_a_band_question_whose_candidate_left_is_dropped():
    u = {"status": "unparsed", "score": None}
    frames = [absent(), absent(cand(0.6, "bag")), absent(cand(0.6, "bag"))] \
        + [absent(cand(0.2, "chair"))] * 3 + [absent(cand(0.6, "chair"))] * 2
    local = Local(no(), u, yes())
    tier = tier_for(Script(*frames), local, Cloud())
    scenes = run(tier, 8)
    assert tier.stats.local_triggers.get(TRIGGER_UNCERTAIN) == 2, tier.stats.local_triggers
    assert tier._band_label in (None, "chair")
    # The chair was promoted only after the local tier was asked about it.
    assert scenes[6]["_perception"]["status"] == ABSENT


def test_review_a_yes_on_a_frame_without_a_band_candidate_stores_no_label():
    tier = tier_for(Script(absent()), Local(yes()), Cloud())
    tier._land_local(TRIGGER_UNCERTAIN, absent(cand(0.2, "chair")), yes(), None, 1.0)
    assert tier._band_label is None


def test_review_an_arrival_given_up_while_waiting_for_a_frames_question_is_not_unable():
    gate = threading.Event()
    local = Local(no(), gate=gate)
    tier = tier_for(Script(absent()), local, Cloud())
    tier({})                                   # the frame's question, held
    cid = tier.begin_local()
    t = threading.Thread(target=tier.confirm_local, args=({}, cid))
    t.start()
    time.sleep(0.2)
    v = tier.abandon_local(cid)                # the runner's budget ran out
    assert v["undecided"] and not v["local_called"]
    assert tier._local_unable == 0 and tier.next_confirm_is_local()
    assert tier.stats.local.get("arrival_waited") == 1
    gate.set()
    t.join(5)
    assert local.asked == 1, "the abandoned arrival never sent its question"


def test_review_the_escalated_question_goes_up_not_a_fresh_one():
    local, cloud = Local(LocalVLMAbsent("gone"), no()), Cloud()
    tier = tier_for(Script(absent(), *([detected()] * 5)), local, cloud,
                    consecutive_frames=1)
    run(tier, 3)
    assert tier.stats.triggers == {TRIGGER_START: 1}, tier.stats.triggers
    assert tier.stats.local_triggers.get(TRIGGER_CANDIDATE) == 1


def test_review_an_escalation_waits_for_the_cloud_call_out_and_is_answered_by_it():
    u = {"status": "unparsed", "score": None}
    gate = threading.Event()

    class Held(Cloud):
        entered = 0

        def __call__(self, frame):
            self.entered += 1
            if self.entered > 1:               # the second call waits on the test
                assert gate.wait(5)
            return super().__call__(frame)

    local, cloud = Local(u), Held()
    tier = tier_for(Script(absent()), local, cloud, local_max_unable=1)
    for _ in range(9):                         # start: local, cloud; cold_search: local, cloud
        tier({})
        call = tier._sight_call
        if call is not None:
            call[3].result(5)
        if tier.stats.cloud_calls < 2 and tier._inflight is not None:
            tier._inflight.result(5)
    assert tier.stats.cloud_calls == 2 and tier._inflight is not None
    asked = local.asked
    for _ in range(20):                        # the cloud call is out: a third escalation waits
        tier({})
        call = tier._sight_call
        if call is not None:
            call[3].result(5)
    assert tier.stats.cloud_calls == 2
    gate.set()
    tier._inflight.result(5)
    for _ in range(3):
        tier({})
    assert tier.stats.cloud_calls == 2, "no paid call straight behind the one that landed"
    # One new cold search went to the local tier (unable) and waits for the
    # cloud call out; it never bounces back to be asked again.
    assert local.asked == asked + 1, (local.asked, asked)


def test_review_an_arrival_handed_up_by_an_absent_tier_is_an_escalation():
    tier = TieredVision(Script(absent()), Cloud(), local_identity_fn=Local(LocalVLMAbsent("x")))
    assert tier.confirm_arrival({})["tier"] == "cloud"
    assert tier.stats.escalations == 1


# ---------- bug pass fixes (2026-10-15) ----------

def test_bugpass_an_older_questions_failure_is_not_counted_against_a_newer_one():
    gate = threading.Event()
    local, cloud = Local(LocalVLMAbsent("blip"), no(), gate=gate), Cloud()
    tier = tier_for(Script(absent(), *([detected()] * 6)), local, cloud)
    tier({})                                   # start, held
    tier({}), tier({})                         # candidate_sighting waits behind it
    gate.set()
    settle(tier)
    for _ in range(3):
        tier({})
        settle(tier)
    assert cloud.calls == 0, "the candidate got its own local try"
    assert tier.stats.local_triggers.get(TRIGGER_CANDIDATE) == 1


def test_bugpass_a_cloud_landing_answers_only_its_own_escalated_question():
    gate = threading.Event()

    class Held(Cloud):
        def __call__(self, frame):
            assert gate.wait(5)
            return super().__call__(frame)

    b = absent(cand(0.6))
    # start -> absent -> cloud (held); then the band question -> absent, held
    # behind it; the start's landing must not count as the band's answer.
    local, cloud = Local(LocalVLMAbsent("gone")), Held()
    tier = tier_for(Script(absent(), absent(), b, b, b, b, b, b), local, cloud)
    for _ in range(5):
        tier({})
        call = tier._sight_call
        if call is not None:
            call[3].result(5)
    assert tier._pending_question == TRIGGER_UNCERTAIN
    gate.set()
    tier._inflight.result(5)
    tier({})                       # collects the start's answer; the band's goes up
    assert tier.stats.triggers.get(TRIGGER_UNCERTAIN) == 1


def test_bugpass_a_candidate_edge_held_behind_an_escalation_fires_later():
    gate = threading.Event()

    class Held(Cloud):
        def __call__(self, frame):
            assert gate.wait(5)
            return super().__call__(frame)

    u = {"status": "unparsed", "score": None}
    local, cloud = Local(u), Held()
    # F1 start -> unable; F2 the start to the cloud (held); F3-F8 absent ->
    # cold_search to the local tier, unable; F9 it waits behind the held
    # call; F10 the candidate edge fires on that waiting frame.
    tier = tier_for(Script(*([absent()] * 8), *([detected()] * 12)), local, cloud,
                    local_max_unable=1)
    for _ in range(10):
        tier({})
        call = tier._sight_call
        if call is not None:
            call[3].result(5)
    assert tier._pending_question == TRIGGER_COLD_SEARCH and tier._inflight is not None
    gate.set()
    tier._inflight.result(5)
    for _ in range(6):
        tier({})
        settle(tier)
    # Asked of the local tier first (never straight to the cloud), and the
    # waiting cold_search was answered by the landing, not sent behind it.
    assert tier.stats.local_triggers.get(TRIGGER_CANDIDATE) == 1, tier.stats.local_triggers
    assert TRIGGER_COLD_SEARCH not in tier.stats.triggers, tier.stats.triggers
    assert tier.stats.cloud_calls == tier.stats.escalations



# ---------- second review round (2026-10-15) ----------

def test_review2_no_paid_call_straight_behind_a_landing_for_a_navigation_question():
    gate = threading.Event()

    class Held(Cloud):
        def __call__(self, frame):
            assert gate.wait(5)
            return super().__call__(frame)

    local, cloud = Local(LocalVLMAbsent("gone")), Held()
    tier = tier_for(Script(absent()), local, cloud, cold_search_after=3)
    for _ in range(7):                       # start up (held); cold_search escalates behind it
        tier({})
        call = tier._sight_call
        if call is not None:
            call[3].result(5)
    assert tier._pending_question == TRIGGER_COLD_SEARCH
    gate.set()
    tier._inflight.result(5)
    tier({})
    assert tier.stats.cloud_calls == 1, "the landing answered the waiting cold_search"


def test_review2_a_new_question_does_not_inherit_the_old_ones_count():
    u = {"status": "unparsed", "score": None}
    local, cloud = Local(u), Cloud()
    tier = tier_for(Script(absent(), *([detected()] * 4)), local, cloud,
                    consecutive_frames=1)
    scenes = run(tier, 4)
    # Frame 1 start (unable); frame 2 the candidate (unable, its first);
    # frame 3 asks the candidate again locally, not the cloud; after its own
    # second unable answer, frame 4 hands it up.
    assert scenes[2]["_tier"].get("local_dispatched") == TRIGGER_CANDIDATE
    assert scenes[3]["_tier"]["cloud_called"] and scenes[3]["_tier"]["trigger"] == TRIGGER_CANDIDATE
    assert cloud.calls == 1


def test_review2_a_superseded_candidate_question_fires_again():
    gate = threading.Event()
    u = {"status": "unparsed", "score": None}
    gate.set()
    local = Local(no(), u, no(), gate=gate)
    tier = tier_for(Script(absent(), *([detected()] * 30)), local, Cloud())
    run(tier, 1)                             # start: no
    tier({})                                 # collects it
    gate.clear()
    tier({})                                 # candidate out, held
    assert tier._sight_call is not None and tier._sight_call[1] == TRIGGER_CANDIDATE
    tier._pending_question = TRIGGER_STALE   # a newer question waits behind it
    gate.set()
    settle(tier)
    for _ in range(4):
        tier({})
        settle(tier)
    assert tier.stats.local_triggers.get(TRIGGER_CANDIDATE) == 2, tier.stats.local_triggers


def test_review2_at_the_cap_the_fresh_question_is_kept_for_the_local_tier():
    local, cloud = Local(LocalVLMAbsent("gone"), no()), Cloud()
    tier = tier_for(Script(absent(), *([detected()] * 5)), local, cloud,
                    consecutive_frames=1, max_calls=0)
    run(tier, 3)
    assert cloud.calls == 0
    assert tier.stats.local_triggers.get(TRIGGER_CANDIDATE) == 1, tier.stats.local_triggers
