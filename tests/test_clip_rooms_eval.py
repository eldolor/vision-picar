"""
tests/test_clip_rooms_eval.py

3.51 (`docs/plans/ros-alignment/3.51-clip-rooms.md`): the arithmetic of the
CLIP room read, and its recorded numbers pinned against the committed
per-frame results (`docs/evaluations/clip-rooms-3.51-cpu.json`), so the
section's table cannot drift from the data. No model is loaded.
"""

import json
from pathlib import Path

import pytest

from tools.clip_rooms_eval import (
    ENSEMBLE, ROOMS, TEMPLATES, UNSURE, accuracy, load_walk, macro_accuracy, predictions,
    normalise_room, report, smooth, switches_per_100)

RESULTS = Path(__file__).resolve().parent.parent / "docs" / "evaluations" / "clip-rooms-3.51-cpu.json"
RESULTS_MPS = RESULTS.with_name("clip-rooms-3.51-mps.json")
LABELS = Path(__file__).resolve().parent.parent / "docs" / "evaluations" / "room-labels-3.51.json"


def test_cloud_answers_map_onto_the_seven_rooms():
    assert normalise_room("Foyer") == "hallway"
    assert normalise_room(" home office ") == "home office"
    assert normalise_room("Family Room") == "living room"
    assert normalise_room("unclear") is None
    assert normalise_room(None) is None
    assert set(v for v in map(normalise_room, ROOMS)) == set(ROOMS)


def test_unsure_frames_never_count():
    pairs = [("living room", "living room"), (UNSURE, "kitchen"), ("hallway", "kitchen")]
    assert accuracy(pairs) == 0.5
    assert accuracy([(UNSURE, "x")]) is None


def test_macro_is_the_guard_against_always_answering_the_majority():
    pairs = [("living room", "living room")] * 90 + [("hallway", "living room")] * 10
    assert accuracy(pairs) == 0.9
    assert macro_accuracy(pairs) == 0.5
    # A room with fewer frames than the floor is left out of the mean.
    assert macro_accuracy(pairs + [("bedroom", "x")] * 3) == 0.5


def test_smoothing_is_causal_and_ties_go_to_the_latest():
    preds = ["a", "b", "a", "b", "b", "c"]
    assert smooth(preds, 1) == preds
    # Frame i sees only frames <= i.
    assert smooth(preds, 3)[:2] == ["a", "b"]
    assert smooth(preds, 3) == ["a", "b", "a", "b", "b", "b"]
    assert switches_per_100(["a", "a", "b", "b", "a"]) == 50.0
    assert switches_per_100(["a"]) == 0.0


def test_load_walk_reads_labels_and_the_cloud_guess(tmp_path):
    w = tmp_path / "walk-1"
    w.mkdir()
    (w / "room_labels.json").write_text(json.dumps(
        {"labels": {"frame-0001.jpg": "hallway", "frame-0000.jpg": "living room"}}))
    (w / "walk.jsonl").write_text(
        json.dumps({"file": "frame-0000.jpg", "navigate": {"room_guess": "living room"}}) + "\n"
        + json.dumps({"file": "frame-0001.jpg", "navigate": {"room_guess": "Foyer"}}) + "\n")
    rows = load_walk(tmp_path, "walk-1")
    assert [r["frame"] for r in rows] == ["frame-0000.jpg", "frame-0001.jpg"]
    assert [r["cloud"] for r in rows] == ["living room", "hallway"]


def test_the_committed_labels_cover_the_pinned_frame_set():
    walks = json.loads(LABELS.read_text())["walks"]
    counts = {w: len(v) for w, v in walks.items()}
    assert counts == {"blue-bottle-20260907-142454": 33, "red-backpack-20260907-144856": 38,
                      "blue-shoes-20260907-152528": 19, "blue-bottle-20260907-185007": 209}


def test_the_results_were_scored_against_the_committed_labels():
    """Per frame, so settling a disputed frame in the labels without
    re-scoring fails here rather than leaving the table stale."""
    walks = json.loads(LABELS.read_text())["walks"]
    for path in (RESULTS, RESULTS_MPS):
        rows = json.loads(path.read_text())["rows"]
        assert {(r["walk"], r["frame"]): r["label"] for r in rows} == \
            {(w, f): lab for w, frames in walks.items() for f, lab in frames.items()}


def test_cpu_and_mps_predict_the_same_room_on_every_frame():
    cpu, mps = (json.loads(p.read_text()) for p in (RESULTS, RESULTS_MPS))
    for prompt in (ENSEMBLE,) + TEMPLATES:
        assert predictions(cpu["rows"], cpu["rooms"], prompt) == \
            predictions(mps["rows"], mps["rooms"], prompt), prompt


def test_the_recorded_numbers():
    """3.51's measured table. Criteria 1-4 failed, 5 met; a change here
    must be a new measurement, recorded in the section first."""
    r = report(RESULTS)
    assert (r["frames"], r["decided"]) == (299, 224)
    # 1. frame accuracy, bar 80%: 67.0%
    assert r[ENSEMBLE]["accuracy"] == pytest.approx(150 / 224)
    # 2. macro, bar 70%: 39.7%
    assert r[ENSEMBLE]["macro"] == pytest.approx(0.3965, abs=1e-4)
    assert r[ENSEMBLE]["per_room"] == {"living room": (137, 154), "hallway": (12, 60),
                                       "home office": (1, 10)}
    # Each template alone.
    assert [round(r[t]["accuracy"] * 224) for t in TEMPLATES] == [145, 150, 158]
    # 3. against the cloud, gap bar 5 points: cloud 77.2%, gap 10.3
    assert r["cloud"]["accuracy"] == pytest.approx(173 / 224)
    assert r["cloud"]["per_room"] == {"living room": (153, 154), "hallway": (16, 60),
                                      "home office": (4, 10)}
    assert r["clip_cloud_agreement"] == pytest.approx(171 / 224)
    # 4. laptop-CPU p90, bar 62 ms: failed. Timings vary run to run, so
    # the band, not the exact value; the exact one is in the section.
    assert 100 < r["encode_ms"]["p90"] < 250
    assert 30 < report(RESULTS_MPS)["encode_ms"]["p90"] < 62
    # 5. smoothing on the held-out walk: met.
    s = r["smoothing"]
    assert (s["n"], s["chosen_by_rule"]) == (5, True)
    assert s["held_out_raw"]["accuracy"] == pytest.approx(90 / 160)
    assert s["held_out_raw"]["switches_per_100"] == pytest.approx(87 / 2.08)
    assert s["held_out_smoothed"]["accuracy"] == pytest.approx(95 / 160)
    assert s["held_out_smoothed"]["switches_per_100"] == pytest.approx(29 / 2.08)
    assert r["held_out"]["accuracy"] == pytest.approx(0.5625)
    assert r["held_out"]["macro"] == pytest.approx(0.3852, abs=1e-4)


def test_nothing_decided_reports_none_and_never_pretends_to_pick_n(tmp_path):
    """Review finding: an all-unsure tuning set used to crash, and a failed
    selection fell back to N = 3 as if the rule had chosen it."""
    rows = [{"walk": w, "frame": f"frame-{i:04d}.jpg", "label": UNSURE, "cloud": None,
             "probs": {ENSEMBLE: [1.0] + [0.0] * (len(ROOMS) - 1)}}
            for w in ("short-walk", "blue-bottle-20260907-185007") for i in range(3)]
    path = tmp_path / "r.json"
    path.write_text(json.dumps({"device": "cpu", "rooms": list(ROOMS), "templates": [],
                                "encode_ms": [1.0, 2.0], "rows": rows}))
    r = report(path)
    assert r[ENSEMBLE]["accuracy"] is None and r["clip_cloud_agreement"] is None
    assert r["smoothing"]["n"] is None and r["smoothing"]["chosen_by_rule"] is False
    assert r["smoothing"]["held_out_smoothed"] is None
