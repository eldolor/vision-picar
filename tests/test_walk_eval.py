"""
Tests for control/walk_eval.py -- the recorded-walk scorecard.

The anchor for the whole suite is that the scorer must reproduce, from the
log alone, the two diagnoses that were made by hand on 2026-08-29 by
reading frames and JSON:

  * walk red-backpack-20260829-195904 (Claude Sonnet 4.5, despite the docs
    saying Nova Lite) -- FORWARD on 1 of 22 frames, then 21 frames of
    LEFT/RIGHT/STOP with the target visible and centred the whole time.
    That is "stalled", and it is what the user experienced as "it would
    not let me walk closer to the ottoman".
  * the same 22 frames replayed through Nova Lite -- FORWARD on 22 of 22,
    obstacle_ahead never once true. A clean-looking run that means the
    model is not reading the scene, which is the failure
    tests/manual_replay_navigate.py's docstring warns about. That is
    "degenerate", and it must NOT score well just because it moved.

No AWS: judge_walk() is driven through a fake client, so the sampling,
the concurrency and the failure handling are all exercised without a
Bedrock call.
"""

import pytest

from control.walk_eval import (
    check_collisions,
    compute_metrics,
    judge_frame,
    judge_walk,
    metric_flags,
    score_walk,
    _sample_indices,
)


def entry(seq, action, visible=True, obstacle=False, reached=False, model=None):
    nav = {
        "action": action,
        "target_visible": visible,
        "target_direction": "center" if visible else "not_visible",
        "target_reached": reached,
        "obstacle_ahead": obstacle,
        "reasoning": "because",
    }
    if model:
        nav["model_id"] = model
    return {"seq": seq, "file": f"frame-{seq:04d}.jpg", "navigate": nav}


def walk(actions, **kw):
    return [entry(i, a, **kw) for i, a in enumerate(actions)]


# ---------- the two real walks, reproduced from their action sequences ----------

# red-backpack-20260829-195904, as the deployed service actually answered it.
STALLED_WALK = [
    "FORWARD", "RIGHT", "RIGHT", "RIGHT", "RIGHT", "LEFT", "LEFT", "LEFT",
    "STOP", "RIGHT", "RIGHT", "RIGHT", "STOP", "LEFT", "STOP", "RIGHT",
    "STOP", "STOP", "LEFT", "RIGHT", "RIGHT", "RIGHT",
]
# the same frames through Nova Lite
DEGENERATE_WALK = ["FORWARD"] * 22


def test_the_real_stalled_walk_is_flagged_stalled():
    m = compute_metrics(walk(STALLED_WALK, obstacle=True))
    assert m["frames"] == 22
    assert m["forward_rate"] == pytest.approx(1 / 22, abs=0.01)
    # 21 consecutive frames after the opening FORWARD, none of them FORWARD
    assert m["longest_no_forward_run"] == 21
    assert "stalled" in metric_flags(m)


def test_the_real_degenerate_walk_is_flagged_degenerate():
    m = compute_metrics(walk(DEGENERATE_WALK))
    assert m["dominant_action_share"] == 1.0
    assert "degenerate" in metric_flags(m)


def test_an_always_forward_walk_does_not_outscore_a_discriminating_one():
    """The point of the degeneracy term: moving every frame is not the same
    as navigating, and a scorer that rewarded raw FORWARD count would rank
    the blind model first -- which is exactly the wrong answer, since on a
    genuine close obstacle Nova drove straight into the couch."""
    blind = score_walk(compute_metrics(walk(DEGENERATE_WALK)), None)
    # A walk that reads the scene: mostly forward, a deliberate turn taken
    # in one direction rather than flip-flopping, then forward again.
    discriminating = score_walk(
        compute_metrics(walk(["FORWARD"] * 8 + ["RIGHT", "RIGHT", "FORWARD", "FORWARD", "STOP"])),
        None,
    )
    assert blind["score"] < discriminating["score"]
    assert "degenerate" in blind["flags"]


# ---------- individual metrics ----------


def test_empty_walk_is_handled_not_crashed():
    m = compute_metrics([])
    assert m["frames"] == 0 and m["empty"] is True
    assert metric_flags(m) == ["empty"]
    assert score_walk(m, None)["score"] == 0


def test_oscillation_counts_direction_reversals_not_turns():
    """Four turns the same way is a deliberate route around something; four
    turns alternating is a policy with no memory of its last move."""
    deliberate = compute_metrics(walk(["RIGHT"] * 8))
    flapping = compute_metrics(walk(["LEFT", "RIGHT"] * 4))
    assert deliberate["oscillation_rate"] == 0.0
    assert flapping["oscillation_rate"] == 1.0
    assert "oscillating" in metric_flags(flapping)
    assert "oscillating" not in metric_flags(deliberate)


def test_visibility_flips_detect_the_blanket_vs_backpack_confusion():
    entries = [
        entry(0, "FORWARD", visible=True),
        entry(1, "FORWARD", visible=False),
        entry(2, "FORWARD", visible=True),
        entry(3, "FORWARD", visible=False),
        entry(4, "FORWARD", visible=True),
    ]
    m = compute_metrics(entries)
    assert m["visibility_flips"] == 4
    assert "unstable-identity" in metric_flags(m)


def test_model_ids_are_collected_from_the_entries():
    entries = walk(["FORWARD", "LEFT"], model="amazon.nova-lite-v1:0")
    assert compute_metrics(entries)["model_ids"] == ["amazon.nova-lite-v1:0"]


def test_missing_navigate_block_does_not_crash():
    """Frames recorded before a /navigate reply existed (or a failed call)
    carry navigate: null -- admin must still be able to score the walk."""
    entries = [{"seq": 0, "file": "frame-0000.jpg", "navigate": None},
               entry(1, "FORWARD")]
    m = compute_metrics(entries)
    assert m["frames"] == 2
    assert score_walk(m, None)["score"] >= 0


# ---------- scoring ----------


def test_score_is_higher_with_a_good_judge_than_a_bad_one():
    m = compute_metrics(walk(["FORWARD", "RIGHT", "FORWARD", "LEFT"]))
    good = score_walk(m, {"sensible_rate": 1.0})
    bad = score_walk(m, {"sensible_rate": 0.0})
    assert good["score"] > bad["score"]
    assert good["basis"] == "judge+metrics"


def test_metrics_only_still_uses_the_full_range():
    """A walk with no judge must not be capped below "good" purely for
    lacking a judge -- otherwise the cheap tier is unreadable on its own."""
    entries = walk(["FORWARD"] * 6 + ["RIGHT", "FORWARD", "FORWARD"])
    entries[-1]["navigate"]["target_reached"] = True
    s = score_walk(compute_metrics(entries), None)
    assert s["basis"] == "metrics-only"
    assert s["verdict"] == "good"


def test_reaching_the_target_is_worth_more_than_not():
    """The task is to reach the object. Nothing in the score noticed whether
    that happened until schema 4: a walk that turned in place for 80 frames
    and never arrived rated the same as one that arrived in 11."""
    actions = ["FORWARD"] * 6 + ["RIGHT", "FORWARD"]
    missed = walk(actions)
    found = walk(actions)
    found[-1]["navigate"]["target_reached"] = True

    judge = {"sensible_rate": 1.0}
    assert score_walk(compute_metrics(found), judge)["score"] > \
        score_walk(compute_metrics(missed), judge)["score"]


def test_oscillation_and_identity_flips_actually_move_the_score():
    """Both were computed, displayed as flags, and then ignored by the
    number -- four walks oscillating at 40-60% paid nothing for it."""
    judge = {"sensible_rate": 1.0}
    # Identical action COUNTS (3 forward, 3 right, 3 left) -- only the
    # ordering differs, so nothing but the oscillation term can separate
    # them. Grouped turns are a deliberate route around something; alternating
    # ones are a policy with no memory of its last move.
    steady = compute_metrics(walk(
        ["FORWARD", "RIGHT", "RIGHT", "RIGHT", "FORWARD", "LEFT", "LEFT", "LEFT", "FORWARD"]))
    flapping = compute_metrics(walk(
        ["FORWARD", "RIGHT", "LEFT", "RIGHT", "LEFT", "FORWARD", "RIGHT", "LEFT", "FORWARD"]))
    assert steady["action_spread"] == flapping["action_spread"]
    assert score_walk(steady, judge)["score"] > score_walk(flapping, judge)["score"]

    stable = walk(["FORWARD"] * 6)
    flipping = [entry(i, "FORWARD", visible=(i % 2 == 0)) for i in range(6)]
    assert score_walk(compute_metrics(stable), judge)["score"] > \
        score_walk(compute_metrics(flipping), judge)["score"]


def test_the_score_reports_its_own_components():
    """So a number can be argued with rather than just believed."""
    c = score_walk(compute_metrics(walk(["FORWARD", "RIGHT"])), {"sensible_rate": 0.5})["components"]
    assert set(c) == {"judge", "behaviour", "completion", "non_degeneracy",
                      "progress", "smoothness", "identity", "collisions"}
    assert c["judge"] == 0.5


def test_verdict_bands():
    assert score_walk({"dominant_action_share": 0.5, "longest_no_forward_run": 0,
                       "frames": 10}, {"sensible_rate": 1.0})["verdict"] == "good"
    assert score_walk({"dominant_action_share": 1.0, "longest_no_forward_run": 40,
                       "frames": 40}, {"sensible_rate": 0.0})["verdict"] == "poor"


# ---------- judge ----------


class FakeClient:
    """Stands in for a bedrock-runtime client."""

    def __init__(self, reply='{"sensible": true, "better_action": null, "why": "ok"}',
                 raise_on=None):
        self.reply = reply
        self.raise_on = raise_on or set()
        self.calls = []

    def converse(self, modelId, messages, inferenceConfig):
        self.calls.append(modelId)
        if len(self.calls) in self.raise_on:
            raise RuntimeError("bedrock: throttled")
        return {"output": {"message": {"content": [{"text": self.reply}]}}}


def test_judge_frame_parses_a_fenced_json_reply():
    client = FakeClient('```json\n{"sensible": false, "better_action": "FORWARD", "why": "open floor"}\n```')
    out = judge_frame(client, "m", b"jpeg", {"action": "STOP"}, "red backpack")
    assert out["sensible"] is False
    assert out["better_action"] == "FORWARD"


def test_judge_walk_samples_rather_than_judging_every_frame():
    client = FakeClient()
    entries = walk(["FORWARD"] * 40)
    out = judge_walk(client, "m", entries, lambda e: b"jpeg", "red backpack", sample=8)
    assert out["judged"] == 8
    assert len(client.calls) == 8
    assert out["sensible_rate"] == 1.0


def test_judge_walk_survives_a_bedrock_failure_on_one_frame():
    """One throttled call must cost that frame, not the walk."""
    client = FakeClient(raise_on={2})
    entries = walk(["FORWARD"] * 4)
    out = judge_walk(client, "m", entries, lambda e: b"jpeg", "red backpack", sample=4,
                     max_workers=1)
    assert out["judged"] == 3
    assert out["attempted"] == 4
    assert out["sensible_rate"] == 1.0


def test_judge_walk_skips_frames_whose_bytes_are_missing():
    client = FakeClient()
    entries = walk(["FORWARD"] * 4)
    out = judge_walk(client, "m", entries, lambda e: None, "red backpack", sample=4)
    assert out["judged"] == 0
    assert out["sensible_rate"] is None


def test_sample_indices_always_include_first_and_last():
    idx = _sample_indices(22, 8)
    assert idx[0] == 0 and idx[-1] == 21
    assert len(idx) == 8
    assert _sample_indices(3, 8) == [0, 1, 2]


# ---------- the judge can see the walk, not just the frame ----------


def test_the_judge_is_told_what_the_robot_just_did():
    """The structural flaw in per-frame judging: one RIGHT is defensible on
    its own, and so is the sixtieth. A walk that turned in place for 80
    consecutive frames was rated 6-of-8 sensible because every sampled frame
    really did look fine by itself."""
    from control.walk_eval import _history_note

    note = _history_note(["RIGHT", "LEFT", "RIGHT", "LEFT"])
    assert "RIGHT, LEFT, RIGHT, LEFT" in note
    assert "NOT moved forward" in note

    assert "first move" in _history_note([])
    # A window containing progress must not claim the robot is stuck.
    assert "NOT moved forward" not in _history_note(["RIGHT", "FORWARD", "LEFT"])


def test_history_is_capped_to_a_window():
    from control.walk_eval import _history_note, HISTORY_WINDOW

    note = _history_note(["RIGHT"] * 40)
    assert note.count("RIGHT") == HISTORY_WINDOW


def test_the_history_given_to_a_frame_comes_from_the_whole_walk():
    """Sampling judges 8 frames of a 120-frame walk. The prior actions must
    come from every frame in between -- that gap is exactly where a stall
    hides -- not from the sampled subset."""
    seen = []

    class Recorder:
        def converse(self, modelId, messages, inferenceConfig):
            seen.append(messages[0]["content"][1]["text"])
            return {"output": {"message": {"content": [
                {"text": '{"sensible": true, "better_action": null, "why": "ok"}'}]}}}

    entries = walk(["RIGHT"] * 30 + ["FORWARD"] * 2)
    judge_walk(Recorder(), "m", entries, lambda e: b"jpeg", "red backpack", sample=4,
               max_workers=1)

    # The last sampled frame is near the end of a walk that was almost all
    # turns; its prompt must show that history rather than a clean slate.
    assert any("RIGHT, RIGHT" in prompt for prompt in seen)
    assert any("NOT moved forward" in prompt for prompt in seen)


def test_a_handful_of_turns_is_not_called_oscillation():
    """Two turns that happen to differ read as 100% reversal. Every short
    successful walk was being flagged `oscillating` on 3-5 turns, which said
    nothing about them -- the rate is noise until there are enough turns for
    a pattern to exist."""
    few = compute_metrics(walk(["FORWARD", "LEFT", "RIGHT", "FORWARD"]))
    assert few["oscillation_rate"] == 0.0
    assert "oscillating" not in metric_flags(few)

    many = compute_metrics(walk(["LEFT", "RIGHT"] * 4))
    assert many["oscillation_rate"] == 1.0
    assert "oscillating" in metric_flags(many)


def test_a_couple_of_opening_turns_is_not_called_stuck():
    """Orientation at the start of a walk is not a stall. Claiming it was
    made the judge reject the opening moves of walks that went on to reach
    the target in 13 frames."""
    from control.walk_eval import _history_note

    assert "NOT moved forward" not in _history_note(["RIGHT"])
    assert "NOT moved forward" not in _history_note(["RIGHT", "LEFT"])
    assert "NOT moved forward" in _history_note(["RIGHT", "LEFT", "RIGHT"])


def test_a_judge_that_recommends_the_same_action_has_not_found_a_fault():
    """It did this steadily over obstacle_ahead: "not sensible ... the robot
    incorrectly reported obstacle_ahead=True", better_action FORWARD, on a
    frame whose action was FORWARD. That is an objection to the scene
    description, not to the move -- and since one model reports
    obstacle_ahead on nearly every frame, it was a systematic penalty
    against that model rather than a finding about its navigation."""
    client = FakeClient(
        '{"sensible": false, "better_action": "FORWARD", "why": "obstacle_ahead was wrong"}')
    out = judge_frame(client, "m", b"jpeg", {"action": "FORWARD"}, "red backpack")
    assert out["sensible"] is True

    # A genuine disagreement about the move is still a fault.
    client = FakeClient(
        '{"sensible": false, "better_action": "STOP", "why": "wall within one step"}')
    out = judge_frame(client, "m", b"jpeg", {"action": "FORWARD"}, "red backpack")
    assert out["sensible"] is False


def test_the_judge_is_told_a_collision_outranks_a_wasted_step():
    """A real walk ended with three consecutive commands to drive FORWARD
    into a wall that filled the frame, and the judge called the last one
    sensible -- agreeing there was "no immediate obstacle". Three passes of
    telling it not to penalise FORWARD had biased it into approving a
    collision.

    The same model, asked the /navigate question on that frame, answers
    RIGHT with obstacle_ahead true. So the prompt has to rank the two error
    kinds explicitly rather than leaning on the model's judgement."""
    from control.walk_eval import JUDGE_PROMPT

    prompt = JUDGE_PROMPT.lower()
    serious = prompt.index("cannot undo")
    wasteful = prompt.index("costs a step but breaks nothing")
    assert serious < wasteful, "collision must be presented as the graver error"
    # The specific excuse the judge accepted must be refused by name.
    assert "continue the search" in prompt
    assert "fills most of the frame" in prompt


# ---------- collisions: the one failure with physical consequences ----------


def test_only_forward_frames_are_candidates_for_a_collision():
    """A turn or a stop cannot drive into anything."""
    from control.walk_eval import collision_candidates

    entries = walk(["FORWARD", "LEFT", "FORWARD", "STOP", "RIGHT"])
    assert collision_candidates(entries) == [0, 2]


def test_candidates_are_not_filtered_by_the_walks_own_obstacle_claim():
    """The log holds the model's CLAIM about obstacles, and the failure being
    hunted is exactly a model that says "clear" while facing a wall. Trusting
    obstacle_ahead here would skip precisely the frames that matter -- the
    real one said "a plain wall with no visible target or obstacle" and drove
    at it."""
    from control.walk_eval import collision_candidates

    entries = [entry(0, "FORWARD", obstacle=False), entry(1, "FORWARD", obstacle=False)]
    assert collision_candidates(entries) == [0, 1]


def test_a_collision_caps_the_score_however_good_the_rest_was():
    """A walk that hit something is not a good walk. Before this, a walk that
    ended with three commands into a wall scored 56 -- entirely for missing
    the target -- and the wall was never mentioned."""
    m = compute_metrics(walk(["FORWARD"] * 4 + ["RIGHT", "FORWARD"]))
    m["target_reached"] = True
    clean = score_walk(m, {"sensible_rate": 1.0})
    hit = score_walk(m, {"sensible_rate": 1.0}, {"collisions": [{"seq": 5}]})

    assert clean["score"] > 70
    assert hit["score"] <= 40
    assert "collision" in hit["flags"]
    assert hit["verdict"] == "poor"


def test_no_collisions_found_changes_nothing():
    m = compute_metrics(walk(["FORWARD", "RIGHT"]))
    with_check = score_walk(m, {"sensible_rate": 1.0}, {"checked": 6, "collisions": []})
    without = score_walk(m, {"sensible_rate": 1.0})
    assert with_check["score"] == without["score"]
    assert "collision" not in with_check["flags"]


def test_the_collision_check_looks_at_the_end_of_a_walk():
    """A walk that drives into something tends to do it once it is lost,
    which is late -- and a recording stops where the operator saw it happen."""
    client = FakeClient('{"would_collide": true, "why": "wall fills the frame"}')
    entries = walk(["FORWARD"] * 30)
    out = check_collisions(client, "m", entries, lambda e: b"jpeg", max_checks=4,
                           max_workers=1)
    assert out["checked"] == 4
    assert out["forward_frames"] == 30
    assert [c["seq"] for c in out["collisions"]] == [26, 27, 28, 29]


def test_a_failed_collision_check_costs_that_frame_not_the_walk():
    client = FakeClient(raise_on={2})
    entries = walk(["FORWARD"] * 3)
    out = check_collisions(client, "m", entries, lambda e: b"jpeg", max_workers=1)
    assert out["checked"] == 2


def test_arriving_at_the_target_is_not_a_collision():
    """Found by a real walk: hunting a blue bottle, the check flagged "a blue
    water bottle is directly in front of the camera at close range, blocking"
    -- it called ARRIVAL a crash. The judge prompt had been told the target
    is the goal rather than an obstacle; the collision prompt had not, and was
    not even given the target's name."""
    from control.walk_eval import COLLISION_PROMPT

    filled = COLLISION_PROMPT.format(target_object="blue bottle")
    assert "blue bottle" in filled
    assert "arrival, not a collision" in filled
    assert "Answer true only if something ELSE is in the way." in filled


def test_the_target_name_reaches_the_collision_prompt():
    seen = []

    class Recorder:
        def converse(self, modelId, messages, inferenceConfig):
            seen.append(messages[0]["content"][1]["text"])
            return {"output": {"message": {"content": [
                {"text": '{"would_collide": false, "why": "clear"}'}]}}}

    check_collisions(Recorder(), "m", walk(["FORWARD"]), lambda e: b"jpeg",
                     target_object="blue bottle", max_workers=1)
    assert "blue bottle" in seen[0]


def test_a_capped_score_still_reports_what_it_would_have_been():
    """The cap is right -- a walk that hit something is not a good walk -- but
    on its own it flattens the top of the range: three real walks that failed
    for quite different reasons all scored exactly 40, which makes the number
    useless for the comparison it exists to support."""
    m = compute_metrics(walk(["FORWARD"] * 8 + ["RIGHT"]))
    m["target_reached"] = True
    hit = {"collisions": [{"seq": 1}]}

    good = score_walk(m, {"sensible_rate": 1.0}, hit)
    mediocre = score_walk(m, {"sensible_rate": 0.4}, hit)

    # Both capped to the same headline number, which is the point of the cap
    # and also why it cannot be the only number.
    assert good["score"] == mediocre["score"] == 40
    assert good["score_uncapped"] > mediocre["score_uncapped"]

    # A walk already below the cap is left alone -- the cap is a ceiling, not
    # a floor.
    weak = score_walk(compute_metrics(walk(["STOP"] * 20)), {"sensible_rate": 0.1}, hit)
    assert weak["score"] == weak["score_uncapped"] < 40


def test_replaying_a_walk_with_no_entries_returns_an_empty_result():
    """A walk directory with frames but an unreadable walk.jsonl."""
    from control.walk_replay import replay_walk

    out = replay_walk([], lambda e: b"jpeg", "red backpack", lambda *a: {},
                      model_id="m")
    assert out["frames"] == 0 and out["entries"] == [] and out["agreement"] is None


def test_a_frame_whose_image_is_missing_is_an_error_not_a_silent_skip():
    """A deleted frame leaves its walk.jsonl entry behind. The replay has to
    report that rather than quietly scoring a shorter walk."""
    from control.walk_replay import replay_walk

    entries = walk(["FORWARD", "LEFT"])
    out = replay_walk(entries, lambda e: None, "red backpack", lambda *a: {},
                      model_id="m")
    assert out["errors"] == 2
    assert all(d["error"] == "no image bytes" for d in out["diff"])


# ---------- P25: command stability ----------

def test_a_command_that_changes_every_frame_reads_as_a_median_run_of_one():
    """The measurement behind P25, and the reason it is separate from
    `oscillation_rate`.

    That metric looks only at TURNS and only at adjacent pairs, so the
    sequence below -- FORWARD, LEFT, FORWARD, LEFT -- reports 0.00
    oscillation while the command changes on every single frame. An
    operator watching the phone described exactly this as confusing, and
    three of six rig walks measured a median run of one. A walk whose
    command never survives a frame is not turning; it is re-deciding.
    """
    entries = [{"navigate": {"action": a}}
               for a in ["FORWARD", "LEFT", "FORWARD", "LEFT", "FORWARD"]]
    m = compute_metrics(entries)

    assert m["median_command_run"] == 1
    assert m["command_changes"] == 4
    # Every frame restores the command from two frames back.
    assert m["command_restored"] == 3
    # The point of the test: the OLD metric sees nothing wrong here.
    assert m["oscillation_rate"] == 0.0


def test_a_steady_walk_reads_a_long_run_and_no_restorations():
    entries = [{"navigate": {"action": a}}
               for a in ["FORWARD"] * 8 + ["LEFT"] * 4]
    m = compute_metrics(entries)

    assert m["median_command_run"] == 8
    assert m["longest_command_run"] == 8
    assert m["command_changes"] == 1
    assert m["command_restored"] == 0


def test_stability_metrics_ignore_frames_that_decided_nothing():
    """A frame with no action is a frame the policy never answered for --
    a vision failure or a dropped call. Counting it as a change would make
    an unreliable LINK look like an unstable POLICY, which are different
    faults with different repairs."""
    entries = [{"navigate": {"action": "FORWARD"}},
               {"navigate": {}},
               {"navigate": {"action": "FORWARD"}}]
    m = compute_metrics(entries)

    assert m["median_command_run"] == 2
    assert m["command_changes"] == 0
