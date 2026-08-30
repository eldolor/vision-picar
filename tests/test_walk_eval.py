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
    mixed = score_walk(
        compute_metrics(walk(["FORWARD"] * 14 + ["RIGHT", "LEFT", "FORWARD", "STOP"] * 2)),
        None,
    )
    assert blind["score"] < mixed["score"]
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
    deliberate = compute_metrics(walk(["RIGHT", "RIGHT", "RIGHT", "RIGHT"]))
    flapping = compute_metrics(walk(["LEFT", "RIGHT", "LEFT", "RIGHT"]))
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
    s = score_walk(compute_metrics(walk(["FORWARD", "RIGHT", "FORWARD", "LEFT"])), None)
    assert s["basis"] == "metrics-only"
    assert s["verdict"] == "good"


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
