"""
tests/test_tiered.py

Phase P2 (`PLAN-onboard-perception.md` 4.10) -- the trigger discipline.

Section 2.4: *"Tiering pays off only if deliberation is event-driven,
never periodic."* This is where that claim becomes code, so this is where
it gets pinned. The properties worth protecting are not "does it call the
cloud" but **when it refuses to**, and section 6.1 is the reason the
refusals need testing at all: firing on every raw edge gives 2.8x, and
requiring two consecutive frames first gives 4.1x, so roughly 40% of a
naive trigger count is field noise rather than events.

No models and no network here -- a fake pipeline states each frame's
perception outcome directly, which is the only way to test a policy about
*sequences* of outcomes without also testing a detector.

Run with: pytest tests/test_tiered.py -v
"""

import pytest

from brain.perceive import (ABSENT, DETECTED, UNAVAILABLE, Box, Candidate,
                            Detection, Perception)
from brain.tiered import (
    DEFAULT_CONSECUTIVE,
    SCAN_ACTION,
    TRIGGER_CANDIDATE,
    TRIGGER_COLD_SEARCH,
    TRIGGER_STALE,
    TRIGGER_START,
    UNAVAILABLE_TRIGGERS,
    TieredVision,
    tiered_vision_fn_for,
)


class ScriptedPipeline:
    """Plays a list of statuses back, one per frame."""

    def __init__(self, statuses):
        self.statuses = list(statuses)
        self.i = 0

    def perceive(self, frame):
        status = self.statuses[min(self.i, len(self.statuses) - 1)]
        self.i += 1
        return Perception(status=status, reason="scripted")


class FakeCloud:
    """Counts calls and returns a recognisable scene."""

    def __init__(self):
        self.calls = 0
        self.searched = None

    def __call__(self, frame):
        self.calls += 1
        return {"obstacles_ahead": [], "free_space": "clear",
                "doorway_visible": False, "important_objects": [],
                "safest_direction": "FORWARD",
                "_navigate": {"reasoning": "cloud said so"}}

    def set_searched_rooms(self, rooms):
        self.searched = list(rooms)


def run(statuses, **kwargs):
    cloud = FakeCloud()
    tier = TieredVision(ScriptedPipeline(statuses), cloud, **kwargs)
    scenes = [tier({"image_base64": "x", "image_width": 640})
              for _ in range(len(statuses))]
    return tier, cloud, scenes


# ---------- the saving, which is the whole point ----------


def test_the_cloud_is_not_called_every_frame():
    """6.3: *"a deliberation-call counter that visibly does not climb
    every step. That single number makes the whole architecture
    watchable."* This is that number, asserted."""
    tier, cloud, _ = run([ABSENT] * 20)
    assert cloud.calls < 20
    assert tier.stats.as_dict()["frames"] == 20


def test_a_quiet_walk_costs_one_call_per_cold_search_interval():
    """Nothing found for a stretch, so the only cost is 2.4's cold search
    -- the trigger that stops a robot driving past an out-of-vocabulary
    target indefinitely."""
    tier, cloud, _ = run([ABSENT] * 19, cold_search_after=6)
    # frame 1 is mission_start; then a cold search every 6 absent frames.
    assert cloud.calls == 4
    assert tier.stats.triggers[TRIGGER_START] == 1
    assert tier.stats.triggers[TRIGGER_COLD_SEARCH] == 3


def test_the_first_frame_always_calls_out():
    """2.5: mission start is the one genuinely blocking call."""
    tier, cloud, scenes = run([ABSENT])
    assert cloud.calls == 1
    assert scenes[0]["_tier"]["trigger"] == TRIGGER_START


# ---------- hysteresis: section 6.1's finding ----------


def test_a_single_frame_sighting_does_not_fire_a_candidate_trigger():
    """The 24.1% visibility flip rate, defended against. One frame of
    `detected` between absences is noise, and paying Opus for it is how a
    trigger policy fires on the corpus's own instability."""
    tier, cloud, _ = run([ABSENT, ABSENT, DETECTED, ABSENT, ABSENT],
                         cold_search_after=99)
    assert cloud.calls == 1  # mission_start only
    assert TRIGGER_CANDIDATE not in tier.stats.triggers


def test_two_consecutive_sightings_do_fire_it():
    """The other half: a real sighting must still get through, and with
    only one frame of lag."""
    tier, cloud, _ = run([ABSENT, DETECTED, DETECTED, ABSENT],
                         cold_search_after=99)
    assert tier.stats.triggers[TRIGGER_CANDIDATE] == 1
    assert cloud.calls == 2  # start + candidate


def test_consecutive_frames_of_one_reproduces_the_naive_count():
    """6.1 measured the naive policy at 2.8x against 4.1x. Being able to
    reproduce it is what makes the comparison checkable on a real walk
    rather than only in the plan."""
    flapping = [ABSENT, DETECTED, ABSENT, DETECTED, ABSENT, DETECTED]
    naive, naive_cloud, _ = run(flapping, consecutive_frames=1,
                                cold_search_after=99)
    steady, steady_cloud, _ = run(flapping, consecutive_frames=2,
                                  cold_search_after=99)
    assert naive_cloud.calls > steady_cloud.calls
    assert naive.stats.triggers.get(TRIGGER_CANDIDATE, 0) == 3
    assert steady.stats.triggers.get(TRIGGER_CANDIDATE, 0) == 0


def test_a_sustained_sighting_fires_once_not_every_frame():
    """A candidate is an *edge*, not a state. Ten frames of the same
    backpack is one event and one confirmation."""
    tier, cloud, _ = run([ABSENT, ABSENT] + [DETECTED] * 10,
                         cold_search_after=99)
    assert tier.stats.triggers[TRIGGER_CANDIDATE] == 1
    assert cloud.calls == 2  # start + one confirmation, for twelve frames


def test_losing_and_refinding_the_target_fires_a_second_confirmation():
    """It has to re-confirm: 1.11 says a VLM "not here" triggers
    re-confirmation and never a silent override, and the same logic
    applies to the target coming back."""
    tier, _, _ = run(
        [ABSENT, ABSENT, DETECTED, DETECTED] + [ABSENT] * 3
        + [DETECTED, DETECTED],
        cold_search_after=99)
    assert tier.stats.triggers[TRIGGER_CANDIDATE] == 2


def test_a_mission_that_starts_looking_at_the_target_pays_once_not_twice():
    """The opening call goes out while looking at the same view a
    candidate trigger would ask about, so seeding the edge detector from
    the start frame is what keeps it to one answer per question."""
    tier, cloud, _ = run([DETECTED] * 6, cold_search_after=99)
    assert cloud.calls == 1
    assert tier.stats.triggers[TRIGGER_START] == 1
    assert TRIGGER_CANDIDATE not in tier.stats.triggers


# ---------- unavailable is never an event ----------


def test_a_wedged_camera_never_fires_a_trigger():
    """`unavailable` is the absence of information (1.12). Firing on it
    would buy a cloud call every few frames from a broken camera and log
    it as a search."""
    tier, cloud, _ = run([UNAVAILABLE] * 15, cold_search_after=3)
    assert cloud.calls == 1  # mission_start, before anything was known
    assert TRIGGER_COLD_SEARCH not in tier.stats.triggers


def test_a_wedged_camera_does_not_advance_the_cold_search_streak():
    """The streak counts frames that *looked and found nothing*. A camera
    that could not look has not searched anywhere."""
    tier, cloud, _ = run([ABSENT] * 3 + [UNAVAILABLE] * 10 + [ABSENT] * 2,
                         cold_search_after=6)
    assert TRIGGER_COLD_SEARCH not in tier.stats.triggers


def test_unavailable_frames_are_counted_and_visible():
    """Not firing is right; hiding it is not. A run that saw nothing but a
    broken camera must be tellable from a run that searched an empty
    house."""
    tier, _, _ = run([UNAVAILABLE] * 5)
    assert tier.stats.as_dict()["perception"][UNAVAILABLE] == 5


# ---------- the stand-in, and what it must not pretend ----------


def test_a_frame_with_no_trigger_returns_a_scan_and_says_no_model_was_asked():
    """The log line matters as much as the action. A reasoning string that
    reads like a model's when nothing was called is exactly the confusion
    this project has already paid for twice."""
    _, _, scenes = run([ABSENT, ABSENT], cold_search_after=99)
    local = scenes[1]
    assert local["safest_direction"] == SCAN_ACTION
    assert local["_tier"]["cloud_called"] is False
    assert "no cloud call" in local["_navigate"]["reasoning"]


def test_the_local_scene_claims_no_knowledge_of_depth():
    """M1's argument. Nothing local measures distance, so `free_space` is
    "unknown" -- and `brain/agent.py`'s proximity veto must never act on a
    number nobody produced."""
    _, _, scenes = run([ABSENT, ABSENT], cold_search_after=99)
    assert scenes[1]["free_space"] == "unknown"
    assert scenes[1]["_navigate"]["obstacle_ahead"] is None


def test_the_local_scene_never_reports_the_target_as_found():
    """`important_objects` ends a mission (`MissionMemory` treats a match
    there as found). The reactive tier raises candidates; it does not get
    to declare success -- 1.11 gives identity to the VLM."""
    _, _, scenes = run([DETECTED] * 6, cold_search_after=99)
    for scene in scenes[1:]:
        if not scene["_tier"]["cloud_called"]:
            assert scene["important_objects"] == []


# ---------- arbitration: local evidence beside the cloud's, never over it ----------


def test_a_cloud_scene_is_returned_intact_with_the_local_evidence_beside_it():
    """1.11 splits arbitration by question. A high CLIP margin raises a
    candidate; it never confirms one, and it must not rewrite the VLM's
    answer on the way past."""
    _, _, scenes = run([ABSENT, DETECTED, DETECTED], cold_search_after=99)
    confirmed = scenes[2]
    assert confirmed["safest_direction"] == "FORWARD"
    assert confirmed["_navigate"]["reasoning"] == "cloud said so"
    assert confirmed["_perception"]["status"] == DETECTED
    assert confirmed["_tier"]["trigger"] == TRIGGER_CANDIDATE


# ---------- budget ----------


def test_the_call_cap_is_a_hard_stop():
    """Every cloud call is real money. Same shape as Robot view's 120-call
    cap and for the same reason."""
    tier, cloud, _ = run([DETECTED, DETECTED, ABSENT] * 10, max_calls=2,
                         cold_search_after=2)
    assert cloud.calls == 2


def test_the_cap_still_returns_a_usable_scene():
    """Hitting the budget must degrade to the stand-in, not raise -- the
    mission ends on its own terms rather than through the failure budget."""
    _, _, scenes = run([ABSENT, DETECTED, DETECTED, DETECTED], max_calls=1,
                       cold_search_after=99)
    capped = scenes[2]
    assert capped["safest_direction"] == SCAN_ACTION
    assert "cap" in capped["_navigate"]["reasoning"]


def test_frames_per_call_is_reported():
    """The number to compare against 6.1's measured 4-6x."""
    tier, _, _ = run([ABSENT] * 12, cold_search_after=6)
    stats = tier.stats.as_dict()
    assert stats["frames_per_call"] == pytest.approx(stats["frames"] / stats["cloud_calls"])


# ---------- the seams it must not break ----------


def test_searched_rooms_passes_through_to_the_cloud_call():
    """S2b's room-level step memory is the cloud call's business.
    `MissionRunner._guarded_vision()` calls this when present, and a tier
    that swallowed it would silently disable the memory."""
    cloud = FakeCloud()
    tier = TieredVision(ScriptedPipeline([ABSENT]), cloud)
    tier.set_searched_rooms(["kitchen"])
    assert cloud.searched == ["kitchen"]


def test_it_is_callable_as_a_plain_vision_fn():
    """The whole point of the seam: nothing in `control/` learns that
    perception grew a tier (2.6's first invariant)."""
    tier = TieredVision(ScriptedPipeline([ABSENT]), FakeCloud())
    scene = tier({"image_base64": "x", "image_width": 640})
    assert set(scene) >= {"obstacles_ahead", "free_space", "doorway_visible",
                          "important_objects", "safest_direction"}


def test_the_factory_does_not_need_the_heavy_dependencies_when_given_a_pipeline():
    """So a test, a replay and a live walk all construct it the same way,
    and only a real run pays for torch."""
    tier = tiered_vision_fn_for("red backpack", FakeCloud(),
                                pipeline=ScriptedPipeline([ABSENT]))
    assert isinstance(tier, TieredVision)


def test_the_triggers_this_cannot_fire_are_named_rather_than_missing():
    """A trigger that cannot fire is worse than one that is absent,
    because it looks like coverage. 2.4 lists seven; FOUR are reachable
    now (staleness joined them 2026-09-07), and the other three say why
    not -- each needs a tier that holds a goal, or the lidar."""
    assert set(UNAVAILABLE_TRIGGERS) == {
        "goal_achieved", "goal_impossible", "room_change"}
    assert all(v for v in UNAVAILABLE_TRIGGERS.values())
    assert DEFAULT_CONSECUTIVE == 2


# ---------- the staleness floor (found by a real run, 2026-09-07) ----------
#
# Once perception got good, both rig walks stopped finishing: the mission
# went from `found` to `max_steps` because nothing triggered after the
# first frame. `candidate_sighting` fires on the EDGE into detected and
# `cold_search` needs a run of `absent`, so a robot that can see its
# target continuously asks the cloud once and never again -- and arrival
# is the cloud's call. Better perception starved the deliberation tier.


def test_a_continuously_visible_target_still_gets_a_call():
    """The regression, pinned. Twenty frames of uninterrupted `detected`
    used to buy exactly one call after the opening one."""
    tier, cloud, _ = run([DETECTED] * 20)
    assert cloud.calls > 2, (
        "a robot that can see its target continuously stopped deliberating "
        "-- and arrival is the VLM's call, so the mission never ends")
    assert tier.stats.triggers.get(TRIGGER_STALE, 0) >= 1


def test_the_floor_does_not_fire_while_events_are_firing():
    """It is a floor, not a metronome: anything that says something more
    specific about why we are calling resets it."""
    tier, _, _ = run([ABSENT, DETECTED] * 12, cold_search_after=2)
    stale = tier.stats.triggers.get(TRIGGER_STALE, 0)
    assert stale <= 1, tier.stats.triggers


def test_the_floor_is_disablable_for_measuring_the_difference():
    """0 reproduces the pre-2026-09-07 edge-only behaviour, which is worth
    being able to measure against and not worth defaulting to."""
    tier, cloud, _ = run([DETECTED] * 20, stale_after=0)
    assert TRIGGER_STALE not in tier.stats.triggers
    # ONE call in twenty frames, and worse than it looks: the opening call
    # seeds the edge detector (see _trigger_for), so a mission that starts
    # already looking at its target never fires `candidate_sighting`
    # either. That is the whole regression in one number.
    assert cloud.calls == 1, "mission_start, then silence for the whole walk"


def test_the_floor_is_bounded_by_the_call_cap():
    """It must not become a way around the hard stop on spend."""
    tier, cloud, _ = run([DETECTED] * 40, stale_after=2, max_calls=3)
    assert cloud.calls == 3


# ---------- what 6.3 draws (phase P2's twin surface) ----------


def test_the_scene_names_the_models_that_produced_it():
    """6.3: the detector's own name on screen, because *"swap the HEF and
    the name on screen changes; that is the experiment loop made
    watchable."* A readout that showed only the output could not tell one
    model's answer from another's."""

    class NamedDetector:
        weights = "yolo11s.pt"

    class NamedScorer:
        model_name = "RN50"

    class Pipeline:
        detector = NamedDetector()
        scorer = NamedScorer()
        target = "red backpack"
        crop_source = "label_gate"

        def perceive(self, frame):
            return Perception(status=ABSENT)

    tier = TieredVision(Pipeline(), FakeCloud())
    scene = tier({"image_base64": "x"})
    assert scene["_tier"]["models"]["detector"] == "yolo11s.pt"
    assert scene["_tier"]["models"]["scorer"] == "RN50"
    assert scene["_tier"]["models"]["target"] == "red backpack"


def test_a_pipeline_that_is_only_a_duck_still_reports_something():
    """The stand-ins in this file are a `perceive()` method and nothing
    else. A readout is not worth crashing a mission loop for, and a blank
    detector name would read as "no detector" rather than "a fake"."""
    tier, _, scenes = run([ABSENT, ABSENT])
    assert scenes[0]["_tier"]["models"]["detector"] is None
    assert tier.models["scorer"] is None


def test_the_name_falls_back_to_the_class_rather_than_going_blank():
    """A backend with no declared name is still a backend, and saying so is
    the difference between "a fake is loaded" and "nothing is loaded"."""

    class Anonymous:
        pass

    class Pipeline:
        detector = Anonymous()
        scorer = Anonymous()

        def perceive(self, frame):
            return Perception(status=ABSENT)

    tier = TieredVision(Pipeline(), FakeCloud())
    assert tier.models["detector"] == "Anonymous"


# ---------- the direction the stand-in reports (found in a real run) ----------
#
# The local scene set `target_direction: "not_visible"` unconditionally
# while setting `target_visible` from the tri-state -- so a frame where
# perception had found the target at a +0.068 margin logged as "target
# not_visible". The two fields contradicted each other on exactly the
# frames perception did its job on, which is the worst place for a readout
# to be wrong.


def _perception(status, bearing=None):
    class P:
        detector = None
        scorer = None

        def perceive(self, frame):
            return Perception(status=status,
                              best=(Candidate(
                                  detection=Detection(Box(0, 0, 1, 1), "backpack", 0.9),
                                  similarity=0.3, best_distractor=0.2,
                                  bearing_deg=bearing) if bearing is not None else None))
    return P()


def _local(status, bearing=None):
    tier = TieredVision(_perception(status, bearing), FakeCloud())
    tier({"image_base64": "x"})            # the mission_start call
    return tier({"image_base64": "x"})     # a free frame


def test_a_detected_target_is_never_logged_as_not_visible():
    """The contradiction, pinned. `not_visible` beside `target_visible: True`
    is not a cosmetic problem -- the two fields mean opposite things to
    anything reading them."""
    scene = _local(DETECTED, bearing=0.0)
    nav = scene["_navigate"]
    assert nav["target_visible"] is True
    assert nav["target_direction"] != "not_visible"


def test_the_direction_comes_from_the_bearing_perception_measured():
    """1.11: the bearing is perception's own output, from the box. Reporting
    it is not the stand-in growing into a policy -- inventing one would be."""
    assert _local(DETECTED, bearing=-40.0)["_navigate"]["target_direction"] == "left"
    assert _local(DETECTED, bearing=0.0)["_navigate"]["target_direction"] == "center"
    assert _local(DETECTED, bearing=40.0)["_navigate"]["target_direction"] == "right"


def test_a_detected_target_with_no_bearing_is_unknown_not_not_visible():
    """The state a recorded walk actually produces when the frame carries no
    width. "I see it but cannot say where" and "I do not see it" must not
    collapse into one word."""
    assert _local(DETECTED)["_navigate"]["target_direction"] == "unknown"


def test_an_absent_target_is_still_not_visible():
    """The one case where `not_visible` is the honest answer."""
    assert _local(ABSENT)["_navigate"]["target_direction"] == "not_visible"
    assert _local(ABSENT)["_navigate"]["target_visible"] is False


def test_the_readout_names_the_floor_mask_only_when_it_is_running():
    """Which crop sources were on is the first thing you need reading a walk
    back: the detector alone measured 86% recall on the corpus, the detector
    plus the mask 94%. A walk that cannot say which it used cannot be
    compared with one that can."""

    class Pipeline:
        detector = type("D", (), {"weights": "yolo11s.pt"})()
        scorer = type("S", (), {"model_name": "RN50"})()
        proposer = None

        def perceive(self, frame):
            return Perception(status=ABSENT)

    off = TieredVision(Pipeline(), FakeCloud())
    assert off.models["proposer"] is None

    class WithMask(Pipeline):
        proposer = type("P", (), {"model_name": "nvidia/segformer-b0"})()

    on = TieredVision(WithMask(), FakeCloud())
    assert on.models["proposer"] == "nvidia/segformer-b0"


# ---------- 1.11a: corroborated identity, REPORTED and not enforced ----------
#
# The amendment is proposed and undecided. What is under test here is
# therefore two things at once, and the second matters as much as the first:
# that the verdict is computed correctly, **and that it changes nothing**.
# 1.11a's own status line is explicit -- *"it should not be implemented until
# the two extra searches exist"* -- and a reporting-only variant that quietly
# started gating would be the worst of both, because the walks meant to
# decide it would then be measuring the decision.

from brain.perceive import CLIP_LOGIT_SCALE  # noqa: E402
from brain.tiered import (  # noqa: E402
    CORROBORATED,
    CORROBORATION_UNAVAILABLE,
    DEFAULT_CORROBORATION_P,
    NO_CLAIM,
    UNCLEAR,
    corroboration_for,
)


def _perception_at(probability, status=DETECTED):
    """A Perception whose best candidate scores exactly `probability`.

    Built by inverting the softmax over two texts rather than by writing a
    similarity and hoping: the gate is P, and a test that set a similarity
    would be pinning the wrong number.
    """
    if probability is None:
        return Perception(status=status, reason="scripted")
    import math

    # P = e^(a*s) / (e^(a*s) + e^(a*d)) => a*(s-d) = logit(P)
    delta = math.log(probability / (1 - probability)) / CLIP_LOGIT_SCALE
    candidate = Candidate(
        detection=Detection(box=Box(0, 0, 10, 10), label="bottle",
                            confidence=0.5),
        similarity=delta, best_distractor=0.0, scores=(delta, 0.0))
    return Perception(status=status, candidates=[candidate], best=candidate,
                      reason="scripted")


def _scene(target_visible):
    return {"safest_direction": "FORWARD",
            "_navigate": {"target_visible": target_visible,
                          "reasoning": "cloud said so"}}


def test_a_claim_the_local_tier_supports_is_corroborated():
    out = corroboration_for(_scene(True), _perception_at(0.69))
    assert out["verdict"] == CORROBORATED
    assert out["local_probability"] == pytest.approx(0.69, abs=1e-3)


def test_a_claim_the_local_tier_does_not_support_is_unclear_not_absent():
    """M3's tri-state argument one tier up. Read as absent it discards a
    real sighting; read as present it keeps the confabulation. The storage
    bin frames scored 0.00-0.44 locally -- this is that frame."""
    assert corroboration_for(_scene(True), _perception_at(0.14))["verdict"] == UNCLEAR


def test_the_bar_is_lower_than_the_detection_gate_and_that_is_the_finding():
    """At the shipped 0.8 gate, corroboration keeps 2 of 10 true sightings
    and is unusable; at 0.5 it keeps 8 and still rejects 34 of 34 bin
    claims. `P >= 0.8` asks *is this a sighting on its own evidence*;
    corroboration asks a different question and deserves a lower bar."""
    from brain.perceive import DEFAULT_MATCH_PROBABILITY

    assert DEFAULT_CORROBORATION_P < DEFAULT_MATCH_PROBABILITY
    borderline = _perception_at(0.62)   # the frame that saved arrival on walk 4
    assert corroboration_for(_scene(True), borderline)["verdict"] == CORROBORATED
    assert corroboration_for(_scene(True), borderline,
                             bar=DEFAULT_MATCH_PROBABILITY)["verdict"] == UNCLEAR


def test_the_clouds_negative_is_trusted_and_not_second_guessed():
    """1.11a part 1: the VLM's recall is 10/10 across the corpus and it has
    never missed a sighting, so only its positive needs support. A local
    detection under a cloud `not visible` is not a disagreement worth
    naming -- naming it would cost recall and buy nothing."""
    out = corroboration_for(_scene(False), _perception_at(0.99))
    assert out["verdict"] == NO_CLAIM
    assert out["claimed"] is False


def test_a_wedged_camera_does_not_read_as_a_failure_to_corroborate():
    """1.12 again: `unavailable` is the absence of information. Treating it
    as "the local tier disagrees" would make a dead camera look like the
    cloud confabulating."""
    out = corroboration_for(_scene(True), _perception_at(None, status=UNAVAILABLE))
    assert out["verdict"] == CORROBORATION_UNAVAILABLE


def test_a_claim_with_no_candidate_at_all_is_unclear():
    """No crop survived, so there is no local evidence either way -- which
    under 1.11a is exactly the state that may steer and may not commit."""
    out = corroboration_for(_scene(True), Perception(status=ABSENT))
    assert out["verdict"] == UNCLEAR
    assert out["local_probability"] is None


def test_the_verdict_rides_on_the_scene_and_the_counters_tally_it():
    """1.11a's own list of what it still needs: the counter has to show
    corroborated-versus-claimed, or the twin cannot show this working."""
    cloud = FakeCloud()

    def claiming(frame):
        cloud.calls += 1
        return _scene(True)

    pipeline = ScriptedPipeline([DETECTED])
    pipeline.perceive = lambda frame: _perception_at(0.9)
    tier = TieredVision(pipeline, claiming)
    scene = tier({"image_base64": "x", "image_width": 640})
    corroboration = scene["_tier"]["corroboration"]
    assert corroboration["verdict"] == CORROBORATED
    assert corroboration["bar"] == DEFAULT_CORROBORATION_P
    stats = scene["_tier"]["stats"]
    assert stats["claims"] == 1 and stats["corroborated"] == 1
    assert stats["verdicts"][CORROBORATED] == 1


def test_a_free_frame_carries_no_verdict_rather_than_the_last_one():
    """A stale verdict held over from the previous paid call would read as
    this frame's, on a panel that updates every tick."""
    tier, cloud, scenes = run([ABSENT, ABSENT])
    assert scenes[1]["_tier"]["cloud_called"] is False
    assert scenes[1]["_tier"]["corroboration"] is None


def test_nothing_is_enforced_and_the_payload_says_so():
    """**The behaviour change is NOT implemented.** 1.11a asks for two more
    searches on out-of-vocabulary targets first, because its falsifier is
    real: a target the local tier cannot see makes every sighting `unclear`
    and the robot steers forever without committing. This test is what
    stops the reporting-only variant drifting into the enforced one."""
    cloud = FakeCloud()

    def claiming(frame):
        cloud.calls += 1
        return dict(_scene(True), _navigate={"target_visible": True,
                                             "target_reached": True,
                                             "reasoning": "arrived"})

    pipeline = ScriptedPipeline([DETECTED])
    pipeline.perceive = lambda frame: _perception_at(0.02)   # a bin frame
    tier = TieredVision(pipeline, claiming)
    scene = tier({"image_base64": "x", "image_width": 640})

    assert scene["_tier"]["corroboration"]["verdict"] == UNCLEAR
    assert scene["_tier"]["corroboration"]["enforced"] is False
    # The cloud's answer is passed through untouched, in both fields the
    # amendment would eventually gate: identity and arrival.
    assert scene["_navigate"]["target_visible"] is True
    assert scene["_navigate"]["target_reached"] is True


def test_the_bar_is_settable_rather_than_compiled_in():
    """1.11a asks for the same treatment DEFAULT_MATCH_PROBABILITY got --
    measured, named, and settable in config."""
    cloud = FakeCloud()
    pipeline = ScriptedPipeline([DETECTED])
    pipeline.perceive = lambda frame: _perception_at(0.4)
    tier = TieredVision(pipeline, lambda f: _scene(True), corroboration_bar=0.3)
    scene = tier({"image_base64": "x", "image_width": 640})
    assert scene["_tier"]["corroboration"]["verdict"] == CORROBORATED
    assert scene["_tier"]["corroboration"]["bar"] == 0.3


# ---------------------------------------------------------------------------
# The vocabulary verdict on the tier readout (2026-09-12)
# ---------------------------------------------------------------------------

from brain.perceive import Vocabulary  # noqa: E402


class VocabPipeline:
    """A `perceive()` and a `vocabulary`, which is all the tier reads."""

    def __init__(self, statuses, in_vocabulary=True):
        self.statuses = list(statuses)
        self.vocabulary = Vocabulary(
            target="a woven laundry basket",
            coco_class="suitcase" if in_vocabulary else None,
            affinity=("suitcase", "handbag"),
            crop_source="soft_gate", affinity_k=2)

    def perceive(self, frame):
        status = self.statuses.pop(0) if self.statuses else ABSENT
        return Perception(status=status, reason="test")


def _tiered(pipeline, **kwargs):
    return TieredVision(pipeline, FakeCloud(), **kwargs)


def test_the_verdict_rides_on_every_tier_readout_free_frames_included():
    """A panel that only shows it on a paid step shows it one frame in six,
    which is how a mission-start fact becomes invisible."""
    tier = _tiered(VocabPipeline([ABSENT] * 3, in_vocabulary=False))
    out = tier({"image_base64": "x"})          # mission_start -- paid
    assert out["_tier"]["vocabulary"]["in_vocabulary"] is False
    free = tier({"image_base64": "x"})         # no trigger -- free
    assert free["_tier"]["cloud_called"] is False
    assert free["_tier"]["vocabulary"]["in_vocabulary"] is False


def test_the_verdict_says_it_is_not_enforced_until_it_is():
    off = _tiered(VocabPipeline([ABSENT], in_vocabulary=False))
    assert off({"image_base64": "x"})["_tier"]["vocabulary"]["enforced"] is False

    on = _tiered(VocabPipeline([ABSENT], in_vocabulary=False),
                 oov_cold_search_after=2)
    assert on({"image_base64": "x"})["_tier"]["vocabulary"]["enforced"] is True


def test_the_trigger_policy_is_bit_for_bit_unchanged_by_default():
    """The experiment is opt-in. Left off, an out-of-vocabulary target
    waits exactly as long for `cold_search` as any other."""
    tier = _tiered(VocabPipeline([ABSENT] * 20, in_vocabulary=False),
                   cold_search_after=6)
    assert tier._cold_search_bar() == 6


def test_an_out_of_vocabulary_target_can_be_given_the_shorter_wait():
    """Because its `absent` carries less information -- not because the
    target is more likely to be there."""
    oov = _tiered(VocabPipeline([ABSENT] * 20, in_vocabulary=False),
                  cold_search_after=6, oov_cold_search_after=2)
    assert oov._cold_search_bar() == 2

    known = _tiered(VocabPipeline([ABSENT] * 20, in_vocabulary=True),
                    cold_search_after=6, oov_cold_search_after=2)
    assert known._cold_search_bar() == 6


def test_a_pipeline_with_no_verdict_still_works():
    """Every fake in this file is a `perceive()` and nothing else, and a
    readout that crashed the mission loop over a missing attribute would be
    a poor trade for a label."""
    tier = TieredVision(ScriptedPipeline([ABSENT]), FakeCloud())
    assert tier({"image_base64": "x"})["_tier"]["vocabulary"] is None
