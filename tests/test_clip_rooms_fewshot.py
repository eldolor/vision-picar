"""
tests/test_clip_rooms_fewshot.py

3.54: few-shot room recognition. The protocol's arithmetic, and the
measured numbers pinned against the committed per-frame predictions
(`docs/evaluations/clip-rooms-3.54-cpu.json`), so a change to the scorer
that moves them fails here. No CLIP needed.
"""

from pathlib import Path

import numpy as np
import pytest

from tools.clip_rooms_fewshot import (
    LABELS_354, ROOMS, cloud_room, leave_one_walk_out, load_labels, pick_evenly, report)

RESULTS = Path(__file__).resolve().parent.parent / "docs" / "evaluations" / "clip-rooms-3.54-cpu.json"


def test_pick_evenly_takes_the_ends_and_spaces_the_rest():
    assert pick_evenly(list(range(10)), 3) == [0, 4, 9]   # round(4.5) == 4
    assert pick_evenly(list(range(4)), 10) == [0, 1, 2, 3]
    assert pick_evenly(["a", "b"], 1) == ["a"]


def _unit(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


def test_the_walk_under_test_never_supplies_a_reference():
    """Walk A is the only walk with a 'basement'. Tested on its own frames,
    which sit exactly on the basement direction, it cannot be called a
    basement: there is no basement reference outside it."""
    rows = [{"walk": "A", "frame": "f0", "label": "basement"},
            {"walk": "B", "frame": "f0", "label": "home gym"},
            {"walk": "C", "frame": "f0", "label": "living room"}]
    emb = np.stack([_unit([1, 0, 0]), _unit([0, 1, 0]), _unit([0, 0, 1])])
    preds = leave_one_walk_out(rows, emb, k=10)
    assert preds[0] != "basement"


def test_nearest_centroid_picks_the_closest_room():
    rows = [{"walk": w, "frame": "f", "label": room}
            for w, room in [("A", "home gym"), ("B", "home gym"),
                            ("C", "basement"), ("D", "basement"), ("E", "home gym")]]
    gym, base = _unit([1, 0.1]), _unit([0.1, 1])
    emb = np.stack([gym, gym, base, base, _unit([0.9, 0.2])])
    preds = leave_one_walk_out(rows, emb, k=10, rooms=("home gym", "basement"))
    assert preds == ["home gym", "home gym", "basement", "basement", "home gym"]


def test_cloud_synonyms_cover_the_two_new_rooms():
    assert cloud_room("Home gym") == "home gym"
    assert cloud_room("basement or recreation room") == "basement"
    assert cloud_room("living room") == "living room"
    assert cloud_room("home gym or multipurpose room") is None  # counts as wrong
    assert cloud_room(None) is None


def test_the_labels_cover_three_rooms_across_at_least_two_walks():
    labels = load_labels()
    for room in ROOMS:
        walks = {w for w, frames in labels.items()
                 if sum(v == room for v in frames.values()) > 0}
        frames = sum(sum(v == room for v in f.values()) for f in labels.values())
        assert len(walks) >= 2 and frames >= 30, (room, walks, frames)
    assert LABELS_354.exists()


@pytest.fixture(scope="module")
def measured():
    return report(RESULTS)


def test_criteria_as_measured(measured):
    """The plan's numbers. Criteria 1, 2, 3, 5 met; 4 failed on laptop CPU."""
    k10 = measured["k10"]
    assert measured["frames"] == 1392 and measured["walks"] == 15
    assert k10["accuracy"] == pytest.approx(1128 / 1392)          # 81.0%, bar 80%
    assert k10["macro"] == pytest.approx(0.8252, abs=1e-4)          # bar 70%
    c = measured["cloud"]
    # Criterion 3 is defined on frames whose cloud answer maps to a class.
    assert c["mapped_frames"] == 189
    assert c["fewshot_accuracy_mapped"] >= c["cloud_accuracy_mapped"] - 0.05  # gap 1.1
    s = measured["smoothing"]
    assert s["switches_voted"] <= 0.5 * s["switches_raw"]          # -70%
    assert s["accuracy_voted"] >= s["accuracy_raw"] - 0.02
    assert measured["cost_ms"]["total"]["p90"] > 62                # criterion 4 failed


def test_few_shot_beats_zero_shot_on_the_same_rooms(measured):
    assert measured["zero_shot"]["accuracy"] == pytest.approx(824 / 1392)
    assert measured["k10"]["accuracy"] > measured["zero_shot"]["accuracy"] + 0.2
