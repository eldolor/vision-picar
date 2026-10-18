"""
tests/test_review_376p2.py

`PLAN-ros-alignment.md` 3.76, step P2: the bugs the review of the
evaluation tools (`tools/clip_rooms_*`, `contact_sheet.py`, `inventory_*`,
`label_prepass.py`, `person_eval.py`, `target_vlm.py`, `yoloworld_crops.py`,
whole files) confirmed. Each test was written before its fix and seen to
fail on `dev` for the reason it names. No model and no paid call: stubs.
"""

import json
import types
from pathlib import Path

import pytest

from tests.conftest import REPO_ROOT


# ---------- P2.1: frames the judge never read counted as 0 right boxes ----------


def test_right_boxes_a_frame_is_over_the_frames_the_judge_read(tmp_path, monkeypatch, capsys):
    """`cmd_score` divided by every tuning frame: a frame whose judge call
    failed counted as a frame with no right box (Gemma 4 E2B's recorded
    0.85 was ~0.97 over the frames judged or empty)."""
    import tools.inventory_vlm as iv
    monkeypatch.setattr(iv, "OUT", tmp_path)
    boxes = {f"w/f{i}.jpg": {"boxes": [{"n": 1, "label": "cup"}]} for i in range(4)}
    judged = {"w/f0.jpg": {"verdicts": {"1": "correct"}}, "w/f1.jpg": {"verdicts": {"1": "correct"}}}
    (tmp_path / "vlm_boxes_t.json").write_text(json.dumps(boxes))
    (tmp_path / "vlm_judge_t.json").write_text(json.dumps(judged))
    (tmp_path / "sample.json").write_text("[]")
    iv.cmd_score(types.SimpleNamespace(tag="t", frames=None))
    out = json.loads(capsys.readouterr().out)
    assert out["right_boxes_a_frame"] == 1.0, out
    assert out["frames_not_judged"] == 2


# ---------- P2.3: Ctrl-C still billed every queued judge call ----------


def test_an_interrupted_judge_run_stops_paying_and_keeps_its_answers(tmp_path, monkeypatch):
    """All frames were submitted up front and the pool's exit waited for the
    queue: a Ctrl-C still sent and billed them all (Thermos: 40 of 40), and
    the answers since the last checkpoint were lost."""
    import threading
    import tools.inventory_label as il
    monkeypatch.setattr(il, "OUT", tmp_path)
    monkeypatch.setattr(il, "ROOT", tmp_path)
    dets = {f"w/f{i:02d}.jpg": {"boxes": [{"label": "cup", "conf": 0.9, "xyxy": [0, 0, 1, 1]}]}
            for i in range(40)}
    (tmp_path / "detections.json").write_text(json.dumps(dets))
    monkeypatch.setattr(il, "load_vocab", lambda: {"cup": True})
    calls, gate = [], threading.Event()

    def judge_one(client, path, boxes, spend):
        calls.append(path)
        if len(calls) >= 3:
            gate.wait(2)
        return {"verdicts": {"1": "correct"}, "missed": []}

    monkeypatch.setattr(il, "judge_one", judge_one)
    monkeypatch.setattr(il, "_client", lambda: None)

    def as_completed(futures):
        # The person presses Ctrl-C once two answers have landed.
        for fut in list(futures)[:2]:
            fut.result()
            yield fut
        gate.set()
        raise KeyboardInterrupt

    monkeypatch.setattr(il, "as_completed", as_completed)
    with pytest.raises(KeyboardInterrupt):
        il.cmd_judge(types.SimpleNamespace(limit=0, budget=80.0, workers=2))
    assert len(calls) <= 4, f"{len(calls)} of 40 calls sent after the interrupt"
    saved = json.loads((tmp_path / "judge.json").read_text())
    # Every billed call's answer kept, the ones out at the interrupt too
    # (round 1, /code-review).
    assert len(saved) == len(calls), (len(saved), len(calls))


# ---------- P2.4: a label-less box was no sighting ----------


def test_a_box_without_a_label_is_a_sighting():
    """Amendment 11: a sighting when at least one box parses. The namer's
    parse drops label-less boxes, so `[{"box_2d": [...]}]` read `empty`
    (two committed Gemma 4 E2B answers: 74 false alarms, not 72)."""
    from tools.target_vlm import outcome, record, sighting_boxes
    raw = '[{"box_2d": [100, 120, 300, 340]}]'
    boxes = sighting_boxes({"boxes": [], "raw": raw})
    assert len(boxes) == 1 and outcome(raw, boxes) == "boxes"
    r = record("w", "f.jpg", False, False, boxes, 1.0)
    assert r.score == 1.0 and r.label is None


# ---------- P2.5-P2.10: the lows taken ----------


def test_a_paid_cache_is_written_in_one_step(tmp_path):
    from tools.inventory_label import write_atomic
    path = tmp_path / "judge.json"
    path.write_text('{"old": 1}')
    write_atomic(path, '{"new": 2}')
    assert json.loads(path.read_text()) == {"new": 2}
    assert not list(tmp_path.glob("*.tmp"))
    for name in ("inventory_label.py", "inventory_vlm.py", "inventory_clip.py"):
        src = (Path(REPO_ROOT) / "tools" / name).read_text()
        assert "out_path.write_text(" not in src, name


def test_a_resumed_clip_run_scores_the_rest_of_a_split_frame():
    """Resume skipped any frame already in `clip.json`; a checkpoint inside
    a frame lost the rest of its boxes (Thermos: 16 of 80)."""
    from tools.inventory_clip import pending_boxes
    dets = {"w/f.jpg": {"boxes": [{"label": "cup", "conf": 0.9, "xyxy": [0, 0, 1, 1]},
                                  {"label": "cup", "conf": 0.8, "xyxy": [1, 1, 2, 2]}]}}
    done = {"w/f.jpg": {"1": {"p_label": 0.9, "rank": 1, "top": []}}}
    left = pending_boxes(dets, {"cup": True}, done)
    assert [(fid, b["n"]) for fid, b in left] == [("w/f.jpg", 2)]


def test_two_walk_lists_with_one_name_keep_separate_embeddings(tmp_path):
    from tools.inventory_lookup import embed_path
    a, b = tmp_path / "a", tmp_path / "b"
    a.mkdir(), b.mkdir()
    (a / "walks.json").write_text('["w1"]')
    (b / "walks.json").write_text('["w1", "w2"]')
    assert embed_path(a / "walks.json") != embed_path(b / "walks.json")


def test_frame_tools_read_frames_only(tmp_path):
    """Any other .jpg crashed `contact_sheet`'s index and was scored by
    `label_prepass`."""
    from tools.contact_sheet import _frames
    for name in ("frame-0001.jpg", "frame-0000.jpg", "cover.jpg"):
        (tmp_path / name).write_bytes(b"x")
    assert _frames(str(tmp_path)) == ["frame-0000.jpg", "frame-0001.jpg"]
    assert 'fullmatch(r"frame-\\d+\\.jpg"' in (Path(REPO_ROOT) / "tools" / "label_prepass.py").read_text()


def test_room_switches_are_weighted_by_transitions():
    """Weighted by frames, the committed 3.54 report read 14.84 / 4.42 where
    the exact figures are 14.89 / 4.43 (the plan's 14.8 -> 4.4 holds)."""
    from tools.clip_rooms_fewshot import report
    s = report(Path(REPO_ROOT) / "docs" / "evaluations" / "clip-rooms-3.54-cpu.json")["smoothing"]
    assert round(s["switches_raw"], 2) == 14.89 and round(s["switches_voted"], 2) == 4.43, s
