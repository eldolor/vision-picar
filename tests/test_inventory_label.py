"""
tests/test_inventory_label.py -- control/inventory_label.py's logic (3.46
amendment 3), on fakes: no model, no cloud call, no recordings.
"""
import json
from types import SimpleNamespace

import pytest

from control import inventory_label as il


def _det(*boxes):
    return {"w": 640, "h": 480, "boxes": [
        {"label": l, "conf": c, "xyxy": [0, 0, 10, 10]} for l, c in boxes]}


VOCAB = {"chair": True, "mug": True, "wall": False, "playroom": False}


def test_only_object_names_above_the_floor_reach_the_judge():
    got = il.object_boxes(_det(("wall", 0.9), ("chair", 0.8), ("mug", 0.05),
                               ("playroom", 0.7), ("unknown", 0.9)), VOCAB)
    assert [(b["label"], b["n"]) for b in got] == [("chair", 1)]


def test_boxes_are_numbered_most_confident_first_and_capped():
    det = _det(*[("chair", 0.1 + i / 100) for i in range(30)])
    got = il.object_boxes(det, VOCAB, cap=5)
    assert [b["n"] for b in got] == [1, 2, 3, 4, 5]
    assert got[0]["conf"] == max(b["conf"] for b in det["boxes"])


def test_the_sample_is_seeded_and_spread_across_walks():
    judged = {f"long/frame-{i:04d}.jpg": {} for i in range(500)}
    judged.update({f"short{k}/frame-0000.jpg": {} for k in range(5)})
    a = il.sample_ids(judged, n=20)
    assert a == il.sample_ids(judged, n=20)
    assert all(f"short{k}/frame-0000.jpg" in a for k in range(5))


def test_threshold_is_the_lowest_confidence_meeting_the_target():
    rows = [(0.1, False), (0.2, False), (0.3, True), (0.4, True), (0.5, True), (0.6, None)]
    assert il.choose_threshold(rows, target=0.75) == 0.2       # 3 of 4 at >= 0.2
    assert il.precision_at(rows, 0.3) == (1.0, 3)              # unanswered never counts
    assert il.choose_threshold([(0.5, False)], target=0.75) is None


def _judged(rows):
    """rows: (fid, label, verdict) -> detections and judge files."""
    dets, judged = {}, {}
    for fid, label, verdict in rows:
        d = dets.setdefault(fid, {"w": 1, "h": 1, "boxes": []})
        d["boxes"].append({"label": label, "conf": 0.9 - len(d["boxes"]) / 100,
                           "xyxy": [0, 0, 1, 1]})
        n = len(d["boxes"])
        judged.setdefault(fid, {"verdicts": {}, "missed": []})["verdicts"][str(n)] = verdict
    return dets, judged


def test_reliable_names_are_counted_only_outside_the_sample():
    rows = [(f"t/{i}", "chair", "correct") for i in range(30)]
    rows += [(f"t/{i}", "mug", "correct" if i < 20 else "wrong") for i in range(30)]
    rows += [(f"s/{i}", "mug", "correct") for i in range(100)]      # the sample
    dets, judged = _judged(rows)
    sample = {f"s/{i}" for i in range(100)}
    assert il.reliable_labels(dets, VOCAB, judged, sample) == {"chair"}
    rows = [(f"t/{i}", "chair", "correct") for i in range(29)]
    dets, judged = _judged(rows)
    assert il.reliable_labels(dets, VOCAB, judged, set()) == set()   # too few


def test_the_page_shows_every_reliable_box_and_some_others():
    rows = [("a", "chair", "correct"), ("a", "mug", "wrong"), ("b", "mug", "wrong"),
            ("b", "chair", "correct")]
    dets, _ = _judged(rows)
    shown = il.page_boxes(dets, VOCAB, ["a", "b"], {"chair"}, extra=1)
    labels = [b["label"] for bs in shown.values() for b in bs]
    assert labels.count("chair") == 2 and labels.count("mug") == 1


def test_each_answer_gets_a_crop_with_context(tmp_path):
    from PIL import Image
    img = tmp_path / "f.jpg"
    Image.new("RGB", (1000, 800)).save(img)
    got = Image.open(__import__("io").BytesIO(
        il.crop(img, {"xyxy": [400, 300, 600, 500]}, width=320)))
    assert got.size == (300, 300)          # 200 px box + 25% each side, under the cap
    big = Image.open(__import__("io").BytesIO(
        il.crop(img, {"xyxy": [0, 0, 1000, 800]}, width=320)))
    assert big.width == 320


def test_verdict_words():
    assert il._correct("correct") is True and il._correct("right") is True
    assert il._correct("wrong") is False and il._correct("unanswered") is None


def test_json_is_found_inside_prose():
    assert il._json_from('Sure.\n{"boxes": {"1": "correct"}, "missed": []}\n') == {
        "boxes": {"1": "correct"}, "missed": []}
    with pytest.raises(ValueError):
        il._json_from("no json here")


def test_spend_stops_at_the_budget():
    s = il.Spend(budget=0.01)
    s.add(SimpleNamespace(input_tokens=1000, output_tokens=100))
    assert not s.over()
    s.add(SimpleNamespace(input_tokens=1000, output_tokens=400))
    assert s.over() and s.calls == 2


def test_a_refusal_is_not_read_as_an_answer():
    r = SimpleNamespace(stop_reason="refusal", content=[])
    with pytest.raises(RuntimeError):
        il._text(r)


def test_judge_one_maps_numbers_and_marks_the_unanswered(tmp_path):
    from PIL import Image
    img = tmp_path / "frame-0000.jpg"
    Image.new("RGB", (64, 48)).save(img)
    boxes = [{"n": 1, "label": "chair", "conf": 0.9, "xyxy": [1, 1, 20, 20]},
             {"n": 2, "label": "mug", "conf": 0.5, "xyxy": [5, 5, 30, 30]}]
    answer = '{"boxes": {"1": "correct"}, "missed": ["lamp"]}'

    class Fake:
        class messages:
            @staticmethod
            def create(**kw):
                assert kw["model"] == il.JUDGE_MODEL
                return SimpleNamespace(stop_reason="end_turn",
                                       content=[SimpleNamespace(type="text", text=answer)],
                                       usage=SimpleNamespace(input_tokens=10, output_tokens=5))
    spend = il.Spend(1.0)
    got = il.judge_one(Fake, img, boxes, spend)
    assert got == {"verdicts": {"1": "correct", "2": "unanswered"}, "missed": ["lamp"]}
    assert spend.calls == 1


def test_the_page_carries_only_object_boxes_and_records_the_sample(tmp_path, monkeypatch):
    from PIL import Image
    monkeypatch.setattr(il, "ROOT", tmp_path)
    monkeypatch.setattr(il, "OUT", tmp_path / "inventory_eval")
    (tmp_path / "w").mkdir()
    Image.new("RGB", (64, 48)).save(tmp_path / "w" / "frame-0000.jpg")
    il.OUT.mkdir()
    (il.OUT / "vocab.json").write_text(json.dumps(VOCAB))
    (il.OUT / "detections.json").write_text(json.dumps(
        {"w/frame-0000.jpg": _det(("chair", 0.9), ("wall", 0.9))}))
    (il.OUT / "judge.json").write_text(json.dumps(
        {"w/frame-0000.jpg": {"verdicts": {"1": "correct"}, "missed": []}}))
    monkeypatch.setattr(il, "EXTRA_BOXES", 5)
    il.cmd_sample(None)
    page = (il.OUT / "adjudicate.html").read_text()
    assert '"label": "chair"' in page and '"label": "wall"' not in page
    assert "correct" not in page          # the user answers blind
    assert json.loads((il.OUT / "sample.json").read_text()) == ["w/frame-0000.jpg"]
