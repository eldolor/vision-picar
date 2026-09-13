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
    # Phase F: a free frame holds the last cloud goal rather than
    # scanning. What this test is about is unchanged -- the scene must say
    # plainly that no model was asked.
    assert local["safest_direction"] in (SCAN_ACTION, "FORWARD")
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
    assert capped["safest_direction"] in (SCAN_ACTION, "FORWARD")
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


# ---------------------------------------------------------------------------
# Phase C -- pacing the cloud by ground covered, not by frame count
# ---------------------------------------------------------------------------


def odo_frame(distance_m=None, usable=True, **extra):
    f = {"image_base64": "x", "image_width": 640}
    if distance_m is not None or not usable:
        f["odometry"] = {"usable": usable,
                         "distance_m": distance_m,
                         "heading_deg": 0.0}
    f.update(extra)
    return f


def test_distance_is_off_by_default_and_the_frame_rule_is_untouched():
    """Phase C ships inert. The project's own lesson, recorded twice: do
    not move a default on one walk."""
    cloud = FakeCloud()
    tier = TieredVision(ScriptedPipeline([ABSENT] * 20), cloud,
                        cold_search_after=6)
    assert tier.cold_search_after_cm is None
    for m in range(20):
        tier(odo_frame(distance_m=m * 10.0))
    # start + one cold_search every 6 absent frames, and nothing extra
    assert cloud.calls == 1 + 19 // 6


def test_a_moving_robot_triggers_on_ground_covered():
    cloud = FakeCloud()
    tier = TieredVision(ScriptedPipeline([ABSENT] * 12), cloud,
                        cold_search_after=100,      # frame rule out of the way
                        cold_search_after_cm=50.0)
    for m in range(12):
        tier(odo_frame(distance_m=m * 0.25))        # 25cm per frame
    # mission_start, then one call per 50cm of the ~2.75m covered
    assert cloud.calls > 1
    assert tier.stats.triggers.get("cold_search")


def test_a_STOPPED_robot_burns_no_calls_on_the_distance_rule():
    """The half of the frame-count proxy that wastes money: a robot parked
    while the operator reads the log covers no new view, and there is
    nothing new for the cloud to say about the same pixels."""
    cloud = FakeCloud()
    tier = TieredVision(ScriptedPipeline([ABSENT] * 30), cloud,
                        cold_search_after=100, cold_search_after_cm=50.0,
                        # The staleness floor is a third rule and fires on
                        # its own clock; disabled here so this test
                        # measures the distance rule and nothing else.
                        stale_after=0)
    for _ in range(30):
        tier(odo_frame(distance_m=1.0))             # never moves
    assert cloud.calls == 1, "only mission_start should have fired"


def test_the_frame_floor_still_fires_for_a_robot_scanning_in_place():
    """Turning is zero path by design (a pivot on a differential chassis
    covers no ground), so distance alone would never re-trigger during a
    scan -- which is exactly when the view IS changing. The two rules are
    an OR for this case."""
    cloud = FakeCloud()
    tier = TieredVision(ScriptedPipeline([ABSENT] * 20), cloud,
                        cold_search_after=6, cold_search_after_cm=500.0)
    for _ in range(20):
        tier(odo_frame(distance_m=1.0))             # scanning, not driving
    assert cloud.calls == 1 + 19 // 6


def test_no_encoders_falls_back_to_frames_and_SAYS_so():
    """The teleop rig -- the only real-pixels backend -- has no encoders,
    so this is the path every validation walk takes. A silent fallback
    would make those walks unattributable."""
    cloud = FakeCloud()
    tier = TieredVision(ScriptedPipeline([ABSENT] * 14), cloud,
                        cold_search_after=6, cold_search_after_cm=50.0)
    out = [tier(odo_frame(usable=False)) for _ in range(14)]
    assert cloud.calls == 1 + 13 // 6
    pacing = out[-1]["_tier"]["pacing"]
    assert pacing["rule"] == "frames"
    assert pacing["odometry_usable"] is False
    assert "no odometry" in pacing["reason"]
    # Unknown, NOT zero. A readout saying "0.0cm since the last call" on a
    # backend with no encoders is indistinguishable from a robot that has
    # genuinely not moved, and that is the reading a distance rule would
    # act on.
    assert pacing["cm_since_call"] is None


def test_a_frame_with_no_odometry_key_is_treated_as_no_encoders():
    """A replay harness, a test, or any caller predating Phase C. Reading
    a missing key as "travelled 0cm" would freeze the distance rule
    permanently and invisibly."""
    cloud = FakeCloud()
    tier = TieredVision(ScriptedPipeline([ABSENT] * 8), cloud,
                        cold_search_after=4, cold_search_after_cm=10.0)
    out = [tier({"image_base64": "x"}) for _ in range(8)]
    assert out[-1]["_tier"]["pacing"]["rule"] == "frames"
    assert cloud.calls == 1 + 7 // 4


def test_the_pacing_readout_names_the_rule_in_force():
    cloud = FakeCloud()
    tier = TieredVision(ScriptedPipeline([ABSENT] * 6), cloud,
                        cold_search_after=100, cold_search_after_cm=40.0)
    out = [tier(odo_frame(distance_m=i * 0.1)) for i in range(6)]
    p = out[-1]["_tier"]["pacing"]
    assert p["rule"] == "distance"
    assert p["cm_bar"] == 40.0
    assert p["odometry_usable"] is True
    assert p["reason"] is None


def test_any_cloud_call_resets_the_distance_baseline():
    """Not just cold_search. A candidate_sighting call looked at this view
    too, so counting the same ground again would fire a redundant
    cold_search moments later."""
    cloud = FakeCloud()
    tier = TieredVision(ScriptedPipeline([DETECTED, DETECTED] + [ABSENT] * 6),
                        cloud, cold_search_after=100, cold_search_after_cm=30.0,
                        stale_after=0)
    out = [tier(odo_frame(distance_m=i * 0.05)) for i in range(8)]
    # 35cm covered, a 30cm bar: exactly one cold_search after the opening
    # call. Without the reset the already-looked-at ground is counted
    # again and a second fires on the very next frame -- measured at 3.
    assert cloud.calls == 2
    # And the readout must show the baseline moving, not just the count.
    assert out[6]["_tier"]["trigger"] == "cold_search"
    assert out[6]["_tier"]["pacing"]["cm_since_call"] == 0.0


# ---------------------------------------------------------------------------
# Phase A -- the cloud call stops blocking the tick
# ---------------------------------------------------------------------------

import threading as _threading  # noqa: E402
import time as _time  # noqa: E402


class SlowCloud(FakeCloud):
    """A cloud that takes as long as you tell it to, and can be released."""

    def __init__(self, delay=None):
        super().__init__()
        self.delay = delay
        self.release = _threading.Event()
        self.entered = _threading.Event()

    def __call__(self, frame):
        self.entered.set()
        if self.delay is not None:
            _time.sleep(self.delay)
        else:
            self.release.wait(timeout=5)
        return super().__call__(frame)


class AngryCloud:
    def __init__(self):
        self.calls = 0

    def __call__(self, frame):
        self.calls += 1
        raise RuntimeError("bedrock said no")


def test_a_dispatched_call_does_not_block_the_frame():
    """The whole point. Synchronously this frame would take the round trip;
    dispatched it returns now and the answer lands later."""
    cloud = SlowCloud()
    tier = TieredVision(ScriptedPipeline([ABSENT] * 5), cloud,
                        async_cloud=True)
    started = _time.perf_counter()
    out = tier({"image_base64": "x"})          # mission_start, dispatched
    elapsed = _time.perf_counter() - started
    assert elapsed < 1.0, "the frame waited on the cloud"
    assert out["_tier"]["cloud_called"] is False
    assert out["_tier"]["in_flight"] == "mission_start"
    cloud.release.set()
    tier.close()


def test_only_one_call_is_outstanding_at_a_time():
    """A second dispatch while the first is out spends twice for one
    answer, and stacking calls on one vision task is the load that used to
    make replays lose frames."""
    cloud = SlowCloud()
    tier = TieredVision(ScriptedPipeline([ABSENT] * 30), cloud,
                        cold_search_after=1, async_cloud=True)
    for _ in range(10):
        tier({"image_base64": "x"})
    assert cloud.entered.wait(timeout=5)
    assert tier.stats.cloud_calls == 1
    cloud.release.set()
    tier.close()


def test_the_robot_holds_the_last_cloud_goal_while_waiting():
    """Without this the stand-in scans through every frame it is waiting
    on, and a 3.6s round trip becomes a visible spin."""
    cloud = SlowCloud(delay=0.05)
    tier = TieredVision(ScriptedPipeline([ABSENT] * 20), cloud,
                        cold_search_after=3, async_cloud=True)
    tier({"image_base64": "x"})                       # dispatch mission_start
    for _ in range(30):                               # let it land
        if tier._last_cloud_scene: break
        tier({"image_base64": "x"})
        _time.sleep(0.02)
    assert tier._last_cloud_scene, "the first answer never landed"

    # Drive to the next trigger, then check the waiting frames.
    seen = []
    for _ in range(8):
        seen.append(tier({"image_base64": "x"}))
    holding = [s for s in seen if s["_tier"]["holding"]]
    assert holding, "no frame held a goal while a call was outstanding"
    assert holding[0]["safest_direction"] == cloud({"image_base64": "x"})["safest_direction"]
    assert holding[0]["safest_direction"] != SCAN_ACTION
    tier.close()


def test_a_free_frame_HOLDS_the_goal_rather_than_scanning(browser=None):
    """Phase F, from five rig walks. The stand-in used to scan on every
    free frame, and with cold_search at 6 that is five frames in six -- so
    it outvoted the cloud 5:1 and the robot spun. Now it keeps doing what
    the cloud last said."""
    cloud = FakeCloud()
    tier = TieredVision(ScriptedPipeline([ABSENT] * 10), cloud,
                        cold_search_after=100, stale_after=0, async_cloud=False)
    first = tier({"image_base64": "x"})          # mission_start, a real call
    assert first["safest_direction"] == "FORWARD"
    free = tier({"image_base64": "x"})
    assert free["_tier"]["cloud_called"] is False
    assert free["safest_direction"] == "FORWARD", "the free frame scanned"
    assert free["_tier"]["holding"] == "FORWARD"


def test_scanning_is_still_available_for_a_run_that_wants_it(browser=None):
    """`hold_goal=False` restores the pre-2026-09-12 stand-in exactly --
    every recorded walk before then was produced by it."""
    cloud = FakeCloud()
    tier = TieredVision(ScriptedPipeline([ABSENT] * 6), cloud,
                        cold_search_after=100, stale_after=0, hold_goal=False)
    tier({"image_base64": "x"})
    free = tier({"image_base64": "x"})
    assert free["safest_direction"] == SCAN_ACTION
    assert free["_tier"]["holding"] is None


def test_the_spin_guard_breaks_a_run_of_turns(browser=None):
    """A held goal of RIGHT repeated forever is the same spin by another
    route. Every one of the five walks would have tripped this."""
    class Turner(FakeCloud):
        def __call__(self, frame):
            s = super().__call__(frame)
            s["safest_direction"] = "RIGHT"
            return s

    tier = TieredVision(ScriptedPipeline([ABSENT] * 30), Turner(),
                        cold_search_after=100, stale_after=0,
                        spin_guard_after=4)
    acts = [tier({"image_base64": "x"})["safest_direction"] for _ in range(12)]
    assert "FORWARD" in acts, f"spun forever: {acts}"
    # And it must not fire every frame -- that would be a different
    # degenerate mode, not a fix for this one.
    assert acts.count("FORWARD") < len(acts) / 2, acts


def test_the_spin_guard_does_not_fire_while_the_target_is_DETECTED(browser=None):
    """Turning toward a target you can see is not a spin. Forcing FORWARD
    there would drive past the thing the mission is for."""
    class Turner(FakeCloud):
        def __call__(self, frame):
            s = super().__call__(frame)
            s["safest_direction"] = "RIGHT"
            return s

    tier = TieredVision(ScriptedPipeline([DETECTED] * 20), Turner(),
                        cold_search_after=100, stale_after=0,
                        spin_guard_after=3)
    acts = [tier({"image_base64": "x"})["safest_direction"] for _ in range(10)]
    assert "FORWARD" not in acts[1:], acts


def test_the_spin_guard_can_be_switched_off(browser=None):
    class Turner(FakeCloud):
        def __call__(self, frame):
            s = super().__call__(frame)
            s["safest_direction"] = "RIGHT"
            return s

    tier = TieredVision(ScriptedPipeline([ABSENT] * 20), Turner(),
                        cold_search_after=100, stale_after=0,
                        spin_guard_after=0)
    acts = [tier({"image_base64": "x"})["safest_direction"] for _ in range(10)]
    assert set(acts) == {"RIGHT"}, acts


def test_an_answer_that_lands_after_the_mission_ends_is_DROPPED():
    """The orphaned-in-flight-call class, one layer down. It put a dead
    session's decision over a live camera view in Robot view, and filed one
    walk's frame into the next walk's directory. Both were epoch bugs."""
    cloud = SlowCloud()
    tier = TieredVision(ScriptedPipeline([ABSENT] * 5), cloud,
                        async_cloud=True)
    tier({"image_base64": "x"})
    assert cloud.entered.wait(timeout=5)
    tier.reset_epoch()                                # the mission ended
    cloud.release.set()
    _time.sleep(0.2)
    tier({"image_base64": "x"})
    assert tier._last_cloud_scene is None, (
        "a superseded answer was applied to a mission that had ended")
    tier.close()


def test_an_async_failure_still_reaches_B3_2s_budget():
    """The failsafe must not be quietly disarmed by moving the call off
    the tick. It is raised on the frame after it lands, where
    MissionRunner._guarded_vision() turns it into VisionUnavailable
    exactly as it does a synchronous failure."""
    cloud = AngryCloud()
    tier = TieredVision(ScriptedPipeline([ABSENT] * 10), cloud,
                        async_cloud=True)
    tier({"image_base64": "x"})                       # dispatch
    raised = None
    for _ in range(50):
        try:
            tier({"image_base64": "x"})
        except RuntimeError as e:
            raised = e
            break
        _time.sleep(0.02)
    assert raised is not None, "an async cloud failure was swallowed"
    assert "bedrock said no" in str(raised)
    tier.close()


def test_synchronous_is_still_the_default_and_is_unchanged():
    """Phase A ships off. It changes what the robot does on a free frame
    and nothing has measured that yet."""
    cloud = FakeCloud()
    tier = TieredVision(ScriptedPipeline([ABSENT] * 6), cloud)
    assert tier.async_cloud is False
    out = tier({"image_base64": "x"})
    assert out["_tier"]["cloud_called"] is True
    assert out["_tier"]["trigger"] == "mission_start"
    assert tier._executor is None, "a synchronous mission started a thread"


def test_close_is_idempotent_and_releases_the_worker():
    cloud = FakeCloud()
    tier = TieredVision(ScriptedPipeline([ABSENT] * 3), cloud, async_cloud=True)
    tier({"image_base64": "x"})
    tier.close()
    tier.close()
    assert tier._executor is None


def test_a_QUEUED_call_never_reaches_the_cloud_after_the_mission_ends():
    """The worker-side epoch check, which is about money rather than
    correctness -- and so needs its own test, because dropping the answer
    happens anyway.

    There is one worker. A call that is still QUEUED when the mission ends
    has not been paid for yet, and must not be. The answer-side guard
    (`_inflight = None`) cannot help: by then the call has been made."""
    cloud = SlowCloud()
    tier = TieredVision(ScriptedPipeline([ABSENT] * 5), cloud, async_cloud=True)

    tier({"image_base64": "x"})                  # first call occupies the worker
    assert cloud.entered.wait(timeout=5)
    first = tier._inflight
    tier._dispatch({"image_base64": "y"}, "cold_search")   # queued behind it
    queued = tier._inflight

    tier.reset_epoch()                           # mission ends
    cloud.release.set()
    for f in (first, queued):
        try:
            f.result(timeout=5)
        except Exception:
            pass
    assert cloud.calls == 1, (
        "the queued call was paid for after the mission had already ended")
    tier.close()


def test_an_async_verdict_is_shown_once_on_the_frame_it_LANDS():
    """1.11a's per-frame measurement must survive Phase A.

    Async computes corroboration against the perception the call was made
    on, but that happens at collection time -- on a later frame. Without
    carrying it, every frame of an async mission reports "no claim this
    step" while the tally climbs behind it, and walk.jsonl records
    `corroboration: null` on every line. The per-frame measurement, which
    is the entire reason the reporting-only variant exists, would be lost
    silently."""
    import time as _t

    class ClaimingCloud(FakeCloud):
        def __call__(self, frame):
            s = super().__call__(frame)
            s["_navigate"]["target_visible"] = True
            return s

    tier = TieredVision(ScriptedPipeline([DETECTED] * 8), ClaimingCloud(),
                        async_cloud=True, stale_after=0)
    seen = []
    for _ in range(6):
        out = tier({"image_base64": "x", "image_width": 640})
        seen.append(out["_tier"]["corroboration"])
        _t.sleep(0.05)

    landed = [c for c in seen if c]
    assert len(landed) == 1, (
        f"expected exactly one verdict, saw {len(landed)} -- either it was "
        "lost entirely or it is being repeated on later frames")
    # Marked, so a reader cannot take it for a verdict about the frame in
    # front of the camera now.
    assert landed[0]["landed_late"] is True
    assert landed[0]["for_trigger"] == "mission_start"
    tier.close()


def test_a_landed_verdict_is_CLEARED_and_never_goes_stale():
    """The rule _local_scene already enforces: a verdict held over from an
    earlier call reads as this frame's, and that is how a stale readout
    becomes a believed one."""
    import time as _t

    class ClaimingCloud(FakeCloud):
        def __call__(self, frame):
            s = super().__call__(frame)
            s["_navigate"]["target_visible"] = True
            return s

    tier = TieredVision(ScriptedPipeline([DETECTED] * 12), ClaimingCloud(),
                        async_cloud=True, stale_after=0)
    for _ in range(2):
        tier({"image_base64": "x", "image_width": 640})
        _t.sleep(0.05)
    after = [tier({"image_base64": "x", "image_width": 640})["_tier"]["corroboration"]
             for _ in range(4)]
    assert all(c is None for c in after), "a landed verdict went stale"
    tier.close()
