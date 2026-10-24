"""
tests/test_tier_remember_no.py

3.91.C (`docs/plans/ros-alignment/3.91-tier-fusion.md`, the user's choice of
2026-10-23: the proposal plus 3.92's disagreement trigger). A "no" about a
candidate perception called DETECTED -- the local VLM's, where its box is
15-95% of the frame, or the cloud's "not visible" -- is remembered: the
candidate is neither steered at nor asked about again until it changes
(moved 0.5 m or turned 30 degrees with odometry; without, lost for 3 frames
and found again, or 20 s), or a yes. A local no it cannot tell (the box out
of range) leaves the steer alone, and perception passing the same candidate
at P >= 0.95 asks the cloud once.

Each behaviour test runs the same frames with `remember_no=False` too, and
asserts the old behaviour there: the red half.
"""

import logging
import math

import pytest

from brain.perceive import (ABSENT, CLIP_LOGIT_SCALE, DETECTED, Box, Candidate, Detection,
                            Perception)
from brain.tiered import (TRIGGER_CANDIDATE, TRIGGER_DISAGREE, TRIGGER_STALE,
                          TieredVision)
from tests.test_tiered_frames import Cloud, Local, Pipeline, no, yes


@pytest.fixture(autouse=True)
def _quiet_logs():
    logging.disable(logging.WARNING)
    yield
    logging.disable(logging.NOTSET)


def D(p=0.9, bearing=0.0, share=0.5):
    """A DETECTED frame at CLIP P `p`, its best box `share` of a 100 x 100
    frame. P is a softmax over the scores, so two scores whose logits are
    log(p) and log(1 - p) give exactly p."""
    side = 100 * share ** 0.5
    scores = (math.log(p) / CLIP_LOGIT_SCALE, math.log(1 - p) / CLIP_LOGIT_SCALE)
    best = Candidate(detection=Detection(Box(0, 0, side, side), "bag", 1.0),
                     similarity=scores[0], bearing_deg=bearing, scores=scores)
    return Perception(status=DETECTED, candidates=[best], best=best, reason="scripted",
                      image_size=(100, 100))


A = Perception(status=ABSENT, reason="scripted")


class Clock:
    """Frames stamped by hand (a phone's `captured_at`), with or without
    odometry (x is the distance driven straight ahead, heading in degrees)."""

    def __init__(self, odometry=False):
        self.t, self.dist, self.heading, self.odometry = 1000.0, 0.0, 0.0, odometry

    def frame(self):
        f = {"image_base64": "x", "image_width": 100, "captured_at": self.t}
        if self.odometry:
            f["odometry"] = {"usable": True, "distance_m": self.dist,
                             "heading_deg": self.heading}
        self.t += 0.5
        return f


def run(frames, local=None, cloud=None, clock=None, step=None, **kw):
    kw.setdefault("cold_search_after", 99)
    kw.setdefault("stale_after", 4)
    cloud = cloud or Cloud(direction="REVERSE")
    clock = clock or Clock()
    tier = TieredVision(Pipeline(frames), cloud, local_sighting_fn=local, **kw)
    scenes = []
    for i in range(len(frames)):
        if step:
            step(i, clock)
        scenes.append(tier(clock.frame()))
    return tier, cloud, scenes


def actions(scenes):
    return [s["safest_direction"] for s in scenes]


def reversals(acts):
    last, n = None, 0
    for a in acts:
        if a in ("FORWARD", "REVERSE"):
            n += last == "FORWARD" and a == "REVERSE"
            last = a
    return n


# --- rule 1: a local no about the candidate is remembered ---------------------

def test_a_local_no_stops_the_steer_and_the_questions_about_that_candidate():
    frames = [A] + [D()] * 12
    on, _, s_on = run(frames, Local(no()))
    off, _, s_off = run(frames, Local(no()), remember_no=False)
    # Off: perception steers at the candidate the VLM refused, and staleness
    # asks about it again and again.
    assert actions(s_off)[4:].count("FORWARD") == 9
    assert off.stats.local_triggers.get(TRIGGER_STALE, 0) >= 2
    # On: asked once (the candidate edge), then left alone.
    assert "FORWARD" not in actions(s_on)[4:]
    assert on.stats.local_triggers == {"mission_start": 1, TRIGGER_CANDIDATE: 1}
    assert on.stats.rejections == {"local": 1}
    assert on.stats.masked_frames == 11     # 2 (the no lands on it: review), then 3-12
    assert s_on[-1]["_tier"]["rejected"]["by"] == "local"
    assert "not steering at a candidate the local said" in \
        s_on[-1]["_navigate"]["reasoning"]


def test_on_a_phone_a_candidate_lost_for_three_frames_is_asked_about_again():
    frames = [A] + [D()] * 4 + [A] * 3 + [D()] * 4
    tier, _, scenes = run(frames, Local(no(), no(), yes()), stale_after=0)
    assert tier.stats.releases == {"lost": 1}
    assert tier.stats.local_triggers[TRIGGER_CANDIDATE] == 2
    # The second answer was a yes: the steer is back.
    assert actions(scenes)[-1] == "FORWARD"


def test_two_lost_frames_are_the_same_candidate():
    frames = [A] + [D()] * 4 + [A] * 2 + [D()] * 4
    tier, _, scenes = run(frames, Local(no(), no(), yes()), stale_after=0)
    assert tier.stats.releases == {}
    assert tier.stats.local_triggers[TRIGGER_CANDIDATE] == 1
    assert "FORWARD" not in actions(scenes)[-4:]


def test_on_a_phone_a_no_is_forgotten_after_twenty_seconds():
    frames = [A] + [D()] * 50          # 0.5 s a frame
    tier, _, scenes = run(frames, Local(no(), no(), yes()), stale_after=0)
    assert tier.stats.releases == {"timeout": 1}
    first_free = next(i for i, s in enumerate(scenes[4:], 4) if s["_tier"]["rejected"] is None)
    assert 40 <= first_free <= 44          # 20 s after the no landed (frame 2)
    assert actions(scenes)[-1] == "FORWARD"


@pytest.mark.parametrize("move,why", [((0.6, 0.0), "moved"), ((0.0, 35.0), "turned")])
def test_with_odometry_a_no_is_forgotten_once_the_body_moved_or_turned(move, why):
    frames = [A] + [D()] * 12

    def step(i, clock):
        if i == 8:
            clock.dist, clock.heading = move

    tier, _, scenes = run(frames, Local(no(), no(), yes()), clock=Clock(odometry=True),
                          step=step, stale_after=0)
    assert tier.stats.releases == {why: 1}
    assert actions(scenes)[-1] == "FORWARD"


def test_with_odometry_time_and_loss_do_not_forget_it():
    frames = [A] + [D()] * 4 + [A] * 4 + [D()] * 50
    tier, _, scenes = run(frames, Local(no(), no(), yes()), clock=Clock(odometry=True),
                          stale_after=0)
    assert tier.stats.releases == {}
    # Never steered at (odometry here never turns, so the spin guard's
    # FORWARD does come: that is the guard, not the candidate).
    assert not any("steering on local sighting" in s["_navigate"]["reasoning"]
                   for s in scenes[-50:])


def _local_answer(got, episode=1, detected=True, share=0.5):
    q = type("Q", (), {"epoch": 0, "trigger": "x", "detected": detected, "band": False,
                       "episode": episode, "box_share": share, "pose": None,
                       "pan_deg": 0.0, "hfov_deg": 60.0})()
    return type("A", (), {"q": q, "got": got, "absent": False, "direction": None})()


def _held(by, episode=1, rank=None):
    return {"by": by, "rank": rank or by, "episode": episode, "pose": None,
            "at_stamp": None, "at_mono": 0.0}


def test_a_local_yes_forgets_a_local_no():
    tier = TieredVision(Pipeline([D()]), Cloud(), local_sighting_fn=Local(yes()))
    tier._episode = 1                       # the run the answer is about
    tier._rejection = _held("local")
    tier._apply_local(_local_answer(yes()))
    assert tier._rejection is None and tier.stats.releases == {"yes": 1}


# --- amendment 3: probation after a release by loss or time --------------------

def test_a_candidate_released_by_loss_is_not_steered_at_until_a_yes():
    # Asked again at frame 10 (the run confirms on 9-10), the yes lands there.
    frames = [A] + [D()] * 4 + [A] * 3 + [D()] * 4
    tier, _, scenes = run(frames, Local(no(), no(), yes()), stale_after=0)
    found = scenes[8]                    # the first frame the bag is back
    assert found["_tier"]["probation"] is True
    assert found["safest_direction"] != "FORWARD"
    assert "until a yes (probation)" in found["_navigate"]["reasoning"]
    assert actions(scenes)[-1] == "FORWARD" and scenes[-1]["_tier"]["probation"] is False


def test_probation_ends_only_with_a_yes():
    frames = [A] + [D()] * 4 + [A] * 3 + [D()] * 4
    tier, _, scenes = run(frames, Local(no(), no(), no()), stale_after=0)
    # The second no is remembered again; nothing steered after the first.
    assert tier.stats.rejections == {"local": 2}
    assert not any(s["safest_direction"] == "FORWARD" for s in scenes[3:])


def test_a_release_by_moving_sets_no_probation():
    frames = [A] + [D()] * 12

    def step(i, clock):
        if i == 8:
            clock.dist = 0.6

    tier, _, scenes = run(frames, Local(no()), clock=Clock(odometry=True), step=step,
                          stale_after=0)
    assert tier.stats.releases == {"moved": 1}
    assert scenes[8]["_tier"]["probation"] is False


def test_with_the_cap_spent_and_no_local_tier_probation_ends():
    frames = [A] + [D()] * 4 + [A] * 3 + [D()] * 6
    tier, cloud, scenes = run(frames, None, cloud=Cloud(direction="REVERSE"),
                              stale_after=0, max_calls=2)
    assert cloud.calls == 2 and tier.stats.releases == {"lost": 1}
    assert actions(scenes)[-1] == "FORWARD"


# --- rule 2: a no counts only where Qwen can tell ------------------------------

def cloud_yes(direction="FORWARD"):
    base = Cloud(direction=direction)

    def ask(f):
        return {**base(f), "_navigate": {"reasoning": "cloud", "target_visible": True}}
    ask.base = base
    return ask


@pytest.mark.parametrize("share", [0.05, 0.99])
def test_a_no_about_a_box_out_of_range_cannot_tell_and_goes_to_the_cloud(share):
    # Option (a) (the user, 2026-10-24): at any P. The cloud says yes, so the
    # steer stays; nothing is remembered.
    frames = [A] + [D(p=0.9, share=share)] * 8
    cloud = cloud_yes()
    tier, _, scenes = run(frames, Local(no()), cloud=cloud, stale_after=0)
    assert tier.stats.rejections == {} and tier.stats.cannot_tell == 1
    assert tier.stats.triggers == {TRIGGER_DISAGREE: 1} and cloud.base.calls == 1
    assert actions(scenes)[-1] == "FORWARD"


@pytest.mark.parametrize("share", [0.05, 0.99])
def test_the_clouds_no_after_a_cannot_tell_is_remembered(share):
    frames = [A] + [D(p=0.9, share=share)] * 8
    tier, cloud, scenes = run(frames, Local(no()), cloud=Cloud(direction="REVERSE"),
                              stale_after=0)
    assert cloud.calls == 1 and tier.stats.rejections == {"cloud": 1}
    assert "FORWARD" not in actions(scenes)[-4:]


@pytest.mark.parametrize("share", [0.15, 0.95])
def test_the_range_is_inclusive(share):
    tier, _, _ = run([A] + [D(share=share)] * 4, Local(no()), stale_after=0)
    assert tier.stats.rejections == {"local": 1}


def test_an_unknown_box_counts():
    # The simulator's geometry has no frame size: the no is remembered.
    p = D()
    p.image_size = None
    tier, _, _ = run([A] + [p] * 4, Local(no()), stale_after=0)
    assert tier.stats.rejections == {"local": 1}


# --- rule 3: the tiers disagree -> the cloud, once per candidate ---------------

def test_a_cannot_tell_no_against_p_095_asks_the_cloud_once():
    frames = [A] + [D(p=0.97, share=0.05)] * 10
    cloud = Cloud(direction="REVERSE")
    tier, cloud, scenes = run(frames, Local(no()), cloud=cloud, stale_after=0)
    assert tier.stats.triggers == {TRIGGER_DISAGREE: 1}
    assert tier.stats.frame_escalations == {"disagreement": 1}
    assert cloud.calls == 1
    # The cloud said not visible: remembered, the candidate left alone.
    assert tier.stats.rejections == {"cloud": 1}
    assert "FORWARD" not in actions(scenes)[-5:]


def test_the_disagreement_is_asked_again_only_about_a_new_candidate():
    frames = ([A] + [D(p=0.97, share=0.05)] * 5 + [A] * 3 + [D(p=0.97, share=0.05)] * 5)
    cloud = Cloud(direction="REVERSE")
    cloud_yes = lambda f: {**cloud(f), "_navigate": {"target_visible": True}}  # noqa: E731
    tier, _, _ = run(frames, Local(no()), cloud=cloud_yes, stale_after=0)
    # A claim, so nothing remembered; one disagreement per candidate run.
    assert tier.stats.triggers == {TRIGGER_DISAGREE: 2}
    assert tier.stats.rejections == {}


def test_no_disagreement_without_a_local_tier():
    frames = [A] + [D(p=0.97, share=0.05)] * 6
    tier, _, _ = run(frames, None, stale_after=0)
    assert TRIGGER_DISAGREE not in tier.stats.triggers


# --- the cloud's no, without a local tier --------------------------------------

def test_a_cloud_not_visible_about_a_candidate_is_remembered():
    frames = [A] + [D()] * 14
    on, c_on, s_on = run(frames, None, cloud=Cloud(direction="REVERSE"))
    off, c_off, s_off = run(frames, None, cloud=Cloud(direction="REVERSE"), remember_no=False)
    assert c_off.calls >= 4 and c_on.calls == 2      # start, then the candidate once
    assert on.stats.rejections == {"cloud": 1}
    assert "FORWARD" not in actions(s_on)[4:]


def test_async_a_landed_cloud_no_is_remembered():
    import threading
    gate = threading.Event()

    class Slow(Cloud):
        def __call__(self, frame):
            gate.wait(5)
            return super().__call__(frame)

    frames = [A] + [D()] * 8
    cloud = Slow(direction="REVERSE")
    tier = TieredVision(Pipeline(frames), cloud, async_cloud=True, cold_search_after=99,
                        stale_after=0)
    clock = Clock()
    try:
        tier(clock.frame())                 # mission_start dispatched
        gate.set()
        tier.wait_inflight()
        for _ in range(3):                  # the candidate edge, dispatched
            tier(clock.frame())
        tier.wait_inflight()
        scenes = [tier(clock.frame()) for _ in range(4)]
    finally:
        tier.close()
    assert tier.stats.rejections == {"cloud": 1}
    assert "FORWARD" not in actions(scenes)


# --- criterion 6's shape: the taupe loop ----------------------------------------

def test_the_taupe_loop_reverses_at_most_once_an_approach():
    """The 2026-10-21 walk's shape, cloud only: perception passes a lookalike
    in bursts, the cloud says not visible and REVERSE. Off, every burst
    steers FORWARD into a held REVERSE; on, the first no holds."""
    burst = [D()] * 4 + [A] * 2
    frames = [A] + burst * 6           # 18.5 s: inside one 20 s hold
    on, c_on, s_on = run(frames, None, cloud=Cloud(direction="REVERSE"))
    off, c_off, s_off = run(frames, None, cloud=Cloud(direction="REVERSE"),
                            remember_no=False)
    assert reversals(actions(s_off)) >= 5
    assert reversals(actions(s_on)) <= 1
    assert c_on.calls <= 4 < c_off.calls


# --- config -----------------------------------------------------------------------

def test_the_brain_passes_the_settings_through(monkeypatch):
    from control import brain_server
    from control.brain_config import load_brain_config
    cfg = load_brain_config()
    for key in ("tier_remember_no", "tier_reject_hold_s", "tier_reject_move_m",
                "tier_reject_turn_deg", "tier_no_box_range"):
        assert key in cfg
    assert "tier_disagree_p" not in cfg          # option (a): no P bar
    cfg = {**cfg, "tier_remember_no": False, "tier_reject_hold_s": 7.0,
           "tier_no_box_range": [0.2, 0.9]}
    tier = brain_server._tiered_vision_fn("bag", Cloud(), cfg, simulated=True)
    assert (tier.remember_no, tier.reject_hold_s, tier.no_box_range) == (False, 7.0, (0.2, 0.9))


@pytest.mark.parametrize("bad", [{"tier_no_box_range": [0.9, 0.2]},
                                 {"tier_no_box_range": [0.1]},
                                 {"tier_disagree_p": 0.95},     # removed: unknown
                                 {"tier_reject_hold_s": -1}])
def test_bad_settings_are_refused(bad, tmp_path):
    import yaml
    from control.brain_config import load_brain_config
    path = tmp_path / "robot.yaml"
    path.write_text(yaml.safe_dump({"brain": bad}))
    with pytest.raises(ValueError):
        load_brain_config(str(path))


# --- review fixes (/code-review + the Thermos pass, 2026-10-23) ----------------

def _band(p=0.7):
    """An absent frame whose best crop is under the gate, in the band."""
    scores = (math.log(p) / CLIP_LOGIT_SCALE, math.log(1 - p) / CLIP_LOGIT_SCALE)
    c = Candidate(detection=Detection(Box(0, 0, 50, 50), "bag", 1.0), similarity=scores[0],
                  bearing_deg=0.0, scores=scores)
    return Perception(status=ABSENT, candidates=[c], reason="scripted")


def test_a_yes_does_not_arm_the_same_question_again():
    # A yes landing on a held no (an async answer) releases it without
    # arming the candidate edge: the yes has answered it, and an armed edge
    # bought a second (paid, without a local tier) call on the next run.
    tier = TieredVision(Pipeline([D()]), Cloud())
    tier._last_confirmed = DETECTED
    tier._rejection = _held("cloud", episode=0)
    tier._on_yes("cloud", 0)
    assert tier._rejection is None and tier._last_confirmed == DETECTED
    assert tier.stats.releases == {"yes": 1}


def test_an_in_range_no_settles_an_earlier_disagreement():
    # A far no left a disagreement open on this run; a later countable no
    # about the same run settles it, so no paid call follows its release.
    tier = TieredVision(Pipeline([D()]), Cloud(), local_sighting_fn=Local(no()))
    tier._escalate_episode = tier._episode
    tier._lost = 0                              # the run is in view
    tier._reject("local", None, tier._episode)
    assert tier._escalate_episode is None


def test_on_a_phone_a_late_no_about_an_ended_run_is_not_remembered():
    tier = TieredVision(Pipeline([D()]), Cloud(), local_sighting_fn=Local(no()))
    tier._episode, tier._lost = 5, 0
    tier._reject("local", None, 4)
    assert tier._rejection is None and tier._probation is False
    tier._reject("local", None, 5)
    assert tier._rejection is not None


def test_band_frames_are_not_lost_frames_and_do_not_steer_under_a_no():
    frames = [A] + [D()] * 4 + [_band()] * 4 + [D()] * 3
    tier, _, scenes = run(frames, Local(no()), stale_after=0, steer_on_candidate=True)
    assert tier.stats.releases == {}           # the bag never left view
    assert tier.stats.candidate_steers == 0
    assert "FORWARD" not in actions(scenes)[3:]


def test_with_remember_no_off_there_is_no_disagreement_question():
    frames = [A] + [D(p=0.97, share=0.05)] * 6
    tier, cloud, _ = run(frames, Local(no()), stale_after=0, remember_no=False)
    assert TRIGGER_DISAGREE not in tier.stats.triggers and cloud.calls == 0


def test_a_far_no_ends_a_local_probation():
    # Amendment 4: released by loss, then the re-asked no is a far one --
    # not remembered, so the steer comes back; the cloud is asked (it says
    # yes here).
    frames = [A] + [D(share=0.5)] * 4 + [A] * 3 + [D(p=0.9, share=0.05)] * 4
    tier, _, scenes = run(frames, Local(no()), cloud=cloud_yes(), stale_after=0)
    assert tier.stats.releases == {"lost": 1} and tier.stats.cannot_tell == 1
    assert tier.stats.triggers == {TRIGGER_DISAGREE: 1}
    assert scenes[-1]["_tier"]["probation"] is False
    assert actions(scenes)[-1] == "FORWARD"


def test_a_mission_reset_forgets_the_disagreement():
    tier = TieredVision(Pipeline([D()]), Cloud(), local_sighting_fn=Local(no()))
    tier._escalate_episode = tier._episode
    tier._disagree_key = (tier._episode, 0)
    tier.reset_epoch()
    assert tier._escalate_episode is None and tier._disagree_key is None


def test_a_hold_is_never_timed_across_two_clocks():
    tier = TieredVision(Pipeline([D()]), Cloud(), local_sighting_fn=Local(no()))
    tier._frame_stamp = 1.79e9                  # a phone frame's wall stamp
    tier._lost = 0                              # the run is in view
    tier._reject("local", None, tier._episode)
    tier._frame_stamp = None                    # then an unstamped frame
    assert tier._held_s(tier._rejection) < 1.0


def test_a_fabric_only_no_keeps_probation():
    # Amendment 4, the user's 2026-10-24 reading: a no about a box filling
    # the frame cannot tell either, but the robot is at that object -- it
    # stays unsteered (the taupe replay reversed twice without this).
    # Option (a): the cloud is asked too; its no is remembered.
    frames = [A] + [D(share=0.5)] * 4 + [A] * 3 + [D(p=0.9, share=0.99)] * 4
    tier, cloud, scenes = run(frames, Local(no()), stale_after=0)
    assert tier.stats.releases == {"lost": 1} and tier.stats.cannot_tell == 1
    assert scenes[9]["_tier"]["probation"] is True
    assert cloud.calls == 1 and tier.stats.rejections == {"local": 1, "cloud": 1}
    assert "FORWARD" not in actions(scenes)[8:]


# --- review round 2 -------------------------------------------------------------

def test_a_late_no_while_its_run_is_being_lost_is_not_remembered():
    # The run has 3 absent frames behind it but no new detection yet: the
    # answer is about a candidate no longer in view.
    tier = TieredVision(Pipeline([D()]), Cloud(), local_sighting_fn=Local(no()))
    tier._episode, tier._lost = 5, 3
    tier._reject("local", None, 5)
    assert tier._rejection is None


def test_with_odometry_a_late_no_about_an_ended_run_is_not_remembered():
    from brain.goal_pose import Pose
    tier = TieredVision(Pipeline([D()]), Cloud(), local_sighting_fn=Local(no()))
    tier._episode, tier._lost = 5, 0
    tier._reject("local", Pose(0.0, 0.0, 0.0), 4)
    assert tier._rejection is None


def test_probation_ends_when_its_run_is_lost_again_unanswered():
    # The user (2026-10-24): probation covers the re-found run only. Frame 8
    # re-finds the bag (released by loss: probation), it is lost again before
    # a question confirms, and frame 12 is a new run, steered at.
    frames = [A] + [D()] * 4 + [A] * 3 + [D()] + [A] * 3 + [D()] * 2
    tier, _, scenes = run(frames, Local(no(), no(), yes()), stale_after=0)
    assert tier.stats.releases == {"lost": 1}
    assert scenes[8]["_tier"]["probation"] is True
    assert scenes[12]["_tier"]["probation"] is False
    assert scenes[12]["safest_direction"] == "FORWARD"


def test_with_a_dead_local_tier_and_the_cap_spent_probation_ends():
    frames = [A] + [D()] * 4 + [A] * 3 + [D()] * 6
    tier, cloud, scenes = run(frames, Local(RuntimeError("local VLM down")),
                              cloud=Cloud(direction="REVERSE"), stale_after=0,
                              max_calls=2)
    assert cloud.calls == 2 and tier.stats.releases == {"lost": 1}
    assert scenes[-1]["_tier"]["probation"] is False
    assert actions(scenes)[-1] == "FORWARD"


def test_a_cloud_reply_it_could_not_parse_is_not_a_no():
    from brain.navigate import to_scene
    garbled = {"action": "STOP", "target_visible": False, "target_direction": "not_visible",
               "reasoning": "Unable to analyze image.", "_raw": "I think the bag is"}
    assert to_scene(garbled, "bag")["_navigate"]["unparsed"] is True
    clean = {k: v for k, v in garbled.items() if k != "_raw"}
    assert "unparsed" not in to_scene(clean, "bag")["_navigate"]
    frames = [A] + [D()] * 8
    tier, cloud, _ = run(frames, None, cloud=lambda f: to_scene(garbled, "bag"))
    assert tier.stats.rejections == {}


def test_offline_the_disagreement_question_does_not_fail_the_mission():
    # Sync: the cloud raises; the frame goes on (cloud_errors_fatal is the
    # default), counted as a cloud error, asked once a run.
    frames = [A] + [D(p=0.97, share=0.05)] * 8
    cloud = Cloud(direction="REVERSE", fail=RuntimeError("offline"))
    tier, cloud, scenes = run(frames, Local(no()), cloud=cloud, stale_after=0)
    assert tier.stats.triggers == {TRIGGER_DISAGREE: 1}
    assert cloud.calls == 1 and tier.stats.cloud_errors == 1
    assert len(scenes) == len(frames)


def test_async_offline_the_disagreement_question_does_not_fail_the_mission():
    frames = [A] + [D(p=0.97, share=0.05)] * 10
    cloud = Cloud(direction="REVERSE", fail=RuntimeError("offline"))
    tier = TieredVision(Pipeline(frames), cloud, local_sighting_fn=Local(no()),
                        async_cloud=True, cold_search_after=99, stale_after=0)
    clock = Clock()
    try:
        for _ in frames:
            tier(clock.frame())              # raises on dev's code
            if tier._inflight is not None:
                tier._inflight.exception()   # let it land before the next frame
    finally:
        tier.close()
    assert tier.stats.triggers == {TRIGGER_DISAGREE: 1}
    assert tier.stats.cloud_errors == 1


def test_a_no_is_measured_from_where_the_body_was_when_it_landed():
    # An approach while the call was out (an async answer): 0.6 m driven
    # since the asked frame must not release the no it lands with.
    from brain.goal_pose import Pose
    clock = Clock(odometry=True)
    tier = TieredVision(Pipeline([A] + [D()] * 6), Cloud(), local_sighting_fn=Local(no()),
                        cold_search_after=99, stale_after=0)
    tier(clock.frame())
    tier(clock.frame())
    asked = tier._pose_now()
    clock.dist = 0.6
    tier(clock.frame())
    tier._reject("local", Pose(asked.x, asked.y, asked.heading_deg), tier._episode)
    tier(clock.frame())
    assert tier._rejection is not None and tier.stats.releases == {}


def test_an_async_no_is_held_from_the_frame_it_lands_on():
    frames = [A] + [D()] * 8
    tier = TieredVision(Pipeline(frames), Cloud(), local_sighting_fn=Local(no()),
                        async_cloud=True, cold_search_after=99, stale_after=0)
    clock = Clock()
    landed_on = held_from = None
    try:
        for _ in frames:
            f = clock.frame()
            tier(f)
            if landed_on is None and tier._rejection is not None:
                landed_on, held_from = f["captured_at"], tier._rejection["at_stamp"]
            if tier._inflight is not None:
                tier._inflight.exception()
    finally:
        tier.close()
    assert landed_on is not None and held_from == landed_on


def test_masked_frames_do_not_vote_for_the_release_frame():
    # 2 of 3, stamped 0.5 s apart: the window holds this frame and the one
    # before. The frame the 20 s hold releases on is one fresh vote, and it
    # must not fire the candidate question on the masked frames' votes.
    frames = [A] + [D()] * 50
    tier, _, scenes = run(frames, Local(no(), no(), yes()), stale_after=0,
                          frame_vote=(2, 3))
    assert tier.stats.releases == {"timeout": 1}
    i = next(i for i, s in enumerate(scenes[4:], 4) if s["_tier"]["rejected"] is None)
    assert scenes[i]["_tier"]["local_called"] != TRIGGER_CANDIDATE
    assert TRIGGER_CANDIDATE in [s["_tier"]["local_called"] for s in scenes[i + 1:i + 3]]


def test_with_remember_no_off_a_far_no_counts_nothing():
    tier, _, _ = run([A] + [D(share=0.05)] * 4, Local(no()), stale_after=0,
                     remember_no=False)
    assert tier.stats.cannot_tell == 0


def test_the_brain_reports_the_settings_on_health():
    from fastapi.testclient import TestClient
    from control.brain_server import create_app
    from control.brain_config import load_brain_config
    cfg = load_brain_config()
    with TestClient(create_app()) as client:
        body = client.get("/health").json()
    assert body["tier_remember_no"] is cfg["tier_remember_no"]


def test_a_timeout_while_the_bag_is_out_of_view_keeps_probation_for_its_return():
    # Review round 3: no odometry, the refused bag leaves view for longer
    # than the 20 s hold. The timeout must put the bag's NEXT run on
    # probation, not end it the moment the bag comes back.
    frames = [A] + [D()] * 4 + [A] * 45 + [D()] * 3
    tier, _, scenes = run(frames, Local(no(), no(), no()), stale_after=0)
    assert tier.stats.releases == {"timeout": 1}
    back = scenes[50]                       # the first frame the bag is back
    assert back["_tier"]["probation"] is True
    # Never steered at (50 turning frames earn the spin guard's FORWARD:
    # that is the guard, not the bag).
    assert not any("steering on local sighting" in s["_navigate"]["reasoning"]
                   for s in scenes[50:])


def test_probation_reads_false_once_its_run_is_lost():
    # Thermos round 3: on the third absent frame, not at the next detection.
    frames = [A] + [D()] * 4 + [A] * 3 + [D()] + [A] * 3
    tier, _, scenes = run(frames, Local(no(), no(), yes()), stale_after=0)
    assert scenes[8]["_tier"]["probation"] is True
    assert scenes[10]["_tier"]["probation"] is True
    assert scenes[11]["_tier"]["probation"] is False


# --- the tiers' order (the user, 2026-10-24): perception < local < cloud ------

def test_a_local_yes_does_not_lift_the_clouds_no():
    tier = TieredVision(Pipeline([D()]), Cloud(), local_sighting_fn=Local(yes()))
    tier._episode = 1                       # the run both answers are about
    tier._rejection = _held("cloud")
    tier._apply_local(_local_answer(yes(frac=[0.4, 0.4, 0.6, 0.6])))
    assert tier._rejection is not None and tier.stats.releases == {}
    # Its box steers nothing either.
    assert tier._local_turn is None and tier._local_held is None


def test_a_cloud_yes_stands_against_a_local_no_about_that_run():
    tier = TieredVision(Pipeline([D()]), Cloud(), local_sighting_fn=Local(no()))
    tier._lost, tier._episode = 0, 1
    tier._on_yes("cloud", 1)
    tier._apply_local(_local_answer(no(), episode=1))
    assert tier._rejection is None and tier.stats.rejections == {}
    # ...but not about a later run.
    tier._episode = 2
    tier._apply_local(_local_answer(no(), episode=2))
    assert tier._rejection is not None


def test_a_local_yes_on_a_clouds_probation_asks_the_cloud():
    # Cloud only at first (no local answer is ever a no here): the cloud's no,
    # released by loss -- the re-found bag is on the cloud's probation. Qwen
    # says yes: outranked, so the cloud is asked, and its yes lifts it.
    frames = [A] + [D()] * 4 + [A] * 3 + [D()] * 6
    answers = iter([False, False] + [True] * 10)
    base = Cloud(direction="REVERSE")

    def cloud(f):
        visible = next(answers)
        return {**base(f), "_navigate": {"reasoning": "cloud", "target_visible": visible},
                "safest_direction": "FORWARD" if visible else "REVERSE"}

    local = Local(RuntimeError("down"), RuntimeError("down"), yes())
    tier, _, scenes = run(frames, local, cloud=cloud, stale_after=0)
    assert tier.stats.rejections == {"cloud": 1} and tier.stats.releases.get("lost") == 1
    assert tier.stats.triggers.get(TRIGGER_DISAGREE) == 1
    assert scenes[-1]["_tier"]["probation"] is False
    assert actions(scenes)[-1] == "FORWARD"


def test_a_local_no_replacing_a_clouds_probation_keeps_the_clouds_rank():
    tier = TieredVision(Pipeline([D()]), Cloud(), local_sighting_fn=Local(no()))
    tier._lost, tier._episode = 0, 2
    tier._probation, tier._probation_episode, tier._probation_by = True, 2, "cloud"
    tier._reject("local", None, 2)
    assert tier._rejection["by"] == "local" and tier._rejection["rank"] == "cloud"
    tier._apply_local(_local_answer(yes(), episode=2))
    assert tier._rejection is not None          # a local yes cannot lift it


def test_with_the_cap_spent_a_clouds_probation_ends():
    tier = TieredVision(Pipeline([D()]), Cloud(), local_sighting_fn=Local(no()),
                        max_calls=1)
    tier.stats.cloud_calls = 1
    tier._probation, tier._probation_episode, tier._probation_by = True, 0, "cloud"
    from brain.tiered import _RouteDecision
    bb = _RouteDecision(frame={}, perception=D(), trigger=TRIGGER_DISAGREE, epoch=0)
    bb.local_fn = None
    tier._who_call_cap(bb)
    assert tier._probation is False


def test_the_tiers_order_is_named_among_the_hard_rules():
    tier = TieredVision(Pipeline([D()]), Cloud())
    text = tier.describe_hard_rules()
    assert "perception < local VLM < cloud" in text and "only it overrides" in text


# --- review round 4 -------------------------------------------------------------

def test_a_local_track_yes_steers_nothing_while_the_cloud_holds_a_no():
    # Round 4 let it steer and the red replay reversed 3 times: with
    # perception silent, nothing says the box is not the refused object.
    tier = TieredVision(Pipeline([D()]), Cloud(), local_sighting_fn=Local(yes()))
    tier._episode = 1                       # the run both answers are about
    tier._rejection = _held("cloud")
    tier._apply_local(_local_answer(yes(frac=[0.6, 0.4, 0.8, 0.6]), detected=False))
    assert tier._rejection is not None and tier.stats.releases == {}
    assert tier._local_held is None and tier._local_turn is None


def test_a_cannot_tell_no_on_the_clouds_probation_renews_the_clouds_no_free():
    # Round 4's data: the cloud's no must be HELD again (a held no outlives
    # the next loss, where a probation does not); round 5: without a call.
    tier = TieredVision(Pipeline([D()]), Cloud(), local_sighting_fn=Local(no()))
    tier._lost, tier._episode = 0, 2
    tier._probation, tier._probation_episode, tier._probation_by = True, 2, "cloud"
    tier._apply_local(_local_answer(no(), episode=2, share=0.05))
    assert tier._escalate_episode is None and tier.stats.cannot_tell == 1
    assert tier._rejection["rank"] == "cloud"


def test_a_later_cloud_no_ends_its_earlier_yes_about_the_run():
    tier = TieredVision(Pipeline([D()]), Cloud(), local_sighting_fn=Local(no()))
    tier._lost, tier._episode = 0, 1
    tier._on_yes("cloud", 1)
    tier._reject("cloud", None, 1)
    tier._release("moved")
    tier._apply_local(_local_answer(no(), episode=1))
    assert tier._rejection is not None and tier._rejection["by"] == "local"


def test_a_late_local_yes_escalates_only_the_run_it_was_about():
    tier = TieredVision(Pipeline([D()]), Cloud(), local_sighting_fn=Local(yes()))
    tier._lost, tier._episode = 0, 4
    tier._probation, tier._probation_episode, tier._probation_by = True, 4, "cloud"
    tier._apply_local(_local_answer(yes(), episode=3))     # asked in run 3
    assert tier._escalate_episode is None
    tier._apply_local(_local_answer(yes(), episode=4))
    assert tier._escalate_episode == 4


def test_every_scene_says_whether_its_candidate_is_refused():
    from brain.tiered import candidate_refused
    frames = [A] + [D()] * 6
    tier, _, scenes = run(frames, Local(no()), stale_after=0)
    assert not candidate_refused(scenes[1])
    assert all(candidate_refused(s) for s in scenes[3:])


def test_explore_does_not_sight_or_close_in_on_a_refused_candidate():
    from brain.explore import ExploreAgent
    scene = {"_perception": {"status": DETECTED, "bearing_deg": 3.0, "pan_deg": 0.0},
             "_tier": {"candidate_refused": True}}
    agent = ExploreAgent.__new__(ExploreAgent)
    agent.sight_from = "perception"
    assert agent._close_in(scene, (0.0, 0.0)) is None


def test_the_arrival_goes_to_the_cloud_once_the_cloud_spoke():
    tier = TieredVision(Pipeline([D()]), Cloud(), local_sighting_fn=Local(no()))
    tier._lost, tier._episode = 0, 1
    assert tier.cloud_spoke() is None
    tier._on_yes("cloud", 1)
    assert tier.cloud_spoke() == "ask"
    tier._episode = 2
    assert tier.cloud_spoke() is None
    tier._reject("cloud", None, 2)
    tier._remask(D())                            # the frame shows that run
    tier._perception_this_frame = D()
    assert tier.cloud_spoke() == "no"            # refused without a call
    tier._release("lost")
    assert tier._probation_by == "cloud" and tier.cloud_spoke() == "ask"


def test_cloud_spoke_is_off_with_remember_no_off_and_at_a_spent_cap():
    off = TieredVision(Pipeline([D()]), Cloud(), local_sighting_fn=Local(no()),
                       remember_no=False)
    off._on_yes("cloud", off._episode)
    assert off.cloud_spoke() is None
    capped = TieredVision(Pipeline([D()]), Cloud(), local_sighting_fn=Local(no()), max_calls=1)
    capped._lost, capped._episode = 0, 1
    capped._on_yes("cloud", 1)
    capped.stats.cloud_calls = 1
    assert capped.cloud_spoke() is None          # no answer can come: the local tier
    capped._reject("cloud", None, 1)
    capped._remask(D())
    capped._perception_this_frame = D()
    assert capped.cloud_spoke() == "no"          # a held no still refuses free


def test_the_arrivals_cloud_no_is_remembered():
    tier = TieredVision(Pipeline([D()]), Cloud(direction="REVERSE"),
                        local_sighting_fn=Local(no()))
    tier._lost, tier._episode = 0, 1
    tier._perception_this_frame = D()
    verdict = tier.confirm_cloud({"image_base64": "x"})
    assert verdict["confirmed"] is False
    assert tier._rejection is not None and tier._rejection["by"] == "cloud"
    tier._remask(D())
    tier._perception_this_frame = D()
    assert tier.cloud_spoke() == "no"


def test_on_a_phone_a_far_bag_both_tiers_refuse_costs_one_call():
    # Round 5: the local far no on the cloud's probation renews the cloud's
    # no for free -- it paid every 20 s (400 frames: 10 calls).
    frames = [A] + [D(share=0.10)] * 400
    tier, cloud, _ = run(frames, Local(no()), cloud=Cloud(direction="REVERSE"),
                         stale_after=0)
    assert cloud.calls == 1
    assert tier.stats.releases.get("timeout", 0) >= 5


def test_a_sync_cloud_no_marks_its_own_frame_refused():
    from brain.tiered import candidate_refused
    frames = [A] + [D()] * 6
    tier, cloud, scenes = run(frames, None, cloud=Cloud(direction="REVERSE"))
    landed = next(i for i, s in enumerate(scenes) if s["_tier"].get("cloud_called")
                  and i > 0)
    assert candidate_refused(scenes[landed])


def test_a_local_yes_on_the_clouds_probation_asks_again_after_a_new_release():
    # Thermos round 4: the run's question spent, the cloud's no released by
    # time into its probation; a local yes asks the cloud once more.
    tier = TieredVision(Pipeline([D()]), Cloud(), local_sighting_fn=Local(yes()))
    tier._lost, tier._episode = 0, 1
    tier._disagree_key = (1, tier._release_gen)        # spent this run
    tier._reject("cloud", None, 1)
    tier._release("timeout")
    tier._apply_local(_local_answer(yes(), episode=1))
    assert tier._escalate_episode == 1
    assert tier._disagree_key != (1, tier._release_gen)    # free to fire



# --- review round 6 -------------------------------------------------------------

def test_a_far_no_ends_a_local_probation_with_the_cloud_offline():
    # Round 6: amendment 4 had moved where it could never run; with the
    # cloud failing the far target was never closed on.
    frames = [A] + [D(share=0.5)] * 4 + [A] * 3 + [D(p=0.9, share=0.05)] * 4
    cloud = Cloud(direction="REVERSE", fail=RuntimeError("offline"))
    tier, _, scenes = run(frames, Local(no()), cloud=cloud, stale_after=0)
    assert tier.stats.releases == {"lost": 1}
    assert scenes[-1]["_tier"]["probation"] is False
    assert actions(scenes)[-1] == "FORWARD"


def test_only_the_clouds_own_no_about_the_run_in_view_refuses_free():
    from brain.goal_pose import Pose
    tier = TieredVision(Pipeline([D()]), Cloud(), local_sighting_fn=Local(no()))
    tier._lost, tier._episode = 0, 1
    tier._reject("cloud", Pose(0.0, 0.0, 0.0), 1)
    tier._remask(D())
    tier._perception_this_frame = D()
    assert tier.cloud_spoke() == "no"
    tier._episode = 2                     # a new run, the no still held (odometry)
    assert tier.cloud_spoke() == "ask"    # asked, not refused: maybe the target
    # A local no renewing the cloud's probation has the cloud's rank but is
    # not the cloud's word: asked.
    renewed = TieredVision(Pipeline([D()]), Cloud(), local_sighting_fn=Local(no()))
    renewed._lost, renewed._episode = 0, 3
    renewed._probation, renewed._probation_episode, renewed._probation_by = True, 3, "cloud"
    renewed._reject("local", None, 3)
    renewed._remask(D())
    renewed._perception_this_frame = D()
    assert renewed._rejection["rank"] == "cloud" and renewed.cloud_spoke() == "ask"


def test_an_arrival_answer_is_about_what_was_in_view_when_asked():
    # Round 6: an abandoned arrival call landing later must not refuse the
    # run then in view.
    tier = TieredVision(Pipeline([D()]), Cloud(), local_sighting_fn=Local(no()))
    tier._lost, tier._episode = 0, 1
    tier._perception_this_frame = D()

    def late(frame):
        tier._episode = 2                   # ticks went on meanwhile
        return {"_navigate": {"target_visible": False}}

    tier.confirm_vision_fn = late
    tier.confirm_cloud({"image_base64": "x"})
    assert tier._rejection is None


# --- review round 7 -------------------------------------------------------------

def test_a_late_yes_about_an_ended_run_lifts_nothing():
    tier = TieredVision(Pipeline([D()]), Cloud(), local_sighting_fn=Local(no()))
    tier._lost, tier._episode = 0, 2
    tier._reject("cloud", None, 2)
    tier._on_yes("cloud", 1)                          # about run 1, landing in run 2
    assert tier._rejection is not None and tier.stats.releases == {}
    assert tier._cloud_yes_episode is None


def test_a_far_no_ends_only_the_probation_of_its_own_run():
    tier = TieredVision(Pipeline([D()]), Cloud(), local_sighting_fn=Local(no()))
    tier._lost, tier._episode = 0, 5
    tier._probation, tier._probation_episode, tier._probation_by = True, 5, "local"
    tier._apply_local(_local_answer(no(), episode=4, share=0.05))   # about run 4
    assert tier._probation is True


def test_the_arrival_refusal_reads_this_frames_perception_not_a_stale_mask():
    tier = TieredVision(Pipeline([D()]), Cloud(), local_sighting_fn=Local(no()))
    tier._lost, tier._episode = 0, 1
    tier._perception_this_frame = D()
    tier._reject("cloud", None, 1)          # landed on another thread: no remask
    assert tier._masked is False and tier.cloud_spoke() == "no"


def test_with_the_cap_spent_a_cloud_ranked_no_from_another_run_waits():
    # Round 7: not "ask" (the spent cloud would refuse, dropping the place);
    # round 8: not the local tier either (it may not overrule the cloud's
    # no) -- undecided until the no is released.
    tier = TieredVision(Pipeline([D()]), Cloud(), local_sighting_fn=Local(no()), max_calls=1)
    tier._lost, tier._episode = 0, 1
    tier._reject("cloud", None, 1)
    tier._episode = 2                       # a new run, the no held (odometry)
    tier._perception_this_frame = D()
    tier.stats.cloud_calls = 1
    assert tier.cloud_spoke() == "wait"
    tier._release("moved")
    assert tier.cloud_spoke() is None       # released: the local tier decides



# --- review round 8 -------------------------------------------------------------

def test_with_remember_no_off_a_late_located_yes_still_steers():
    # Round 8: "off is dev's behaviour" -- a yes landing in a later run still
    # steers by its box (it lifts nothing, there being nothing to lift).
    tier = TieredVision(Pipeline([D()]), Cloud(), local_sighting_fn=Local(yes()),
                        remember_no=False)
    tier._episode = 2
    tier._apply_local(_local_answer(yes(frac=[0.6, 0.4, 0.8, 0.6]), episode=1))
    assert tier._local_held == "FORWARD" and tier._local_turn is not None


def test_a_late_yes_under_a_higher_tiers_no_steers_nothing():
    tier = TieredVision(Pipeline([D()]), Cloud(), local_sighting_fn=Local(yes()))
    tier._lost, tier._episode = 0, 2
    tier._reject("cloud", None, 2)
    tier._apply_local(_local_answer(yes(frac=[0.6, 0.4, 0.8, 0.6]), episode=1))
    assert tier._local_held is None and tier._rejection is not None
