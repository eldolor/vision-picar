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
    because it looks like coverage. 2.4 lists seven; three are reachable
    without C6 and C8, and the other four say why not."""
    assert set(UNAVAILABLE_TRIGGERS) == {
        "goal_achieved", "goal_impossible", "room_change", "staleness"}
    assert all(v for v in UNAVAILABLE_TRIGGERS.values())
    assert DEFAULT_CONSECUTIVE == 2


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
