"""
tests/test_inventory_label.py -- tools/inventory_label.py's logic (3.46
amendment 3), on fakes: no model, no cloud call, no recordings.
"""
import json
from types import SimpleNamespace

import pytest

from tools import inventory_label as il


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


def test_the_vlm_answer_is_parsed_scaled_and_deduplicated():
    from tools.inventory_vlm import parse_boxes
    cut = ('```json [ {"bbox_2d": [10, 20, 30, 40], "label": "Chair"}, '
           '{"bbox_2d": [10, 20, 30, 40], "label": "chair"}, '
           '{"bbox_2d": [1, 2, 3, 4], "label": "lamp"}, {"bbox_2d": [5,')
    got = parse_boxes(cut, 2.0, 1.0)
    assert [(b["label"], b["xyxy"], b["n"]) for b in got] == [
        ("chair", [20.0, 20.0, 60.0, 40.0], 1), ("lamp", [2.0, 2.0, 6.0, 4.0], 2)]
    assert parse_boxes("no objects found", 1, 1) == []


def test_gemma_boxes_are_read_rows_first_on_0_1000():
    """Amendment 9: Gemma answers [y1, x1, y2, x2] under `box_2d`; a 640x480
    frame's box at rows 500-1000, columns 0-250 is its lower-left quarter-ish."""
    from tools.inventory_vlm import parse_boxes
    text = '```json\n[{"box_2d": [500, 0, 1000, 250], "label": "Sneaker"}]\n```'
    got = parse_boxes(text, 640 / 1000, 480 / 1000, order="yxyx")
    assert got == [{"label": "sneaker", "xyxy": [0.0, 240.0, 160.0, 480.0], "n": 1}]
    # The default order is unchanged for Qwen's answers.
    assert parse_boxes(text, 1, 1)[0]["xyxy"] == [500.0, 0.0, 1000.0, 250.0]


def test_a_malformed_item_is_skipped_not_guessed():
    from tools.inventory_vlm import parse_boxes
    text = ('[{"box_2d": [1, 2, "x", 4], "label": "a"}, "stray", '
            '{"box_2d": [10, 20, 30, 40], "label": "b"}]')
    got = parse_boxes(text, 1, 1, order="yxyx")
    assert [b["label"] for b in got] == ["b"] and got[0]["n"] == 1


def test_litert_reply_text_joins_the_text_parts():
    from types import SimpleNamespace as NS

    from tools.inventory_vlm import message_text
    msg = NS(contents=[NS(text="[{"), NS(image="ignored"), NS(text='"a": 1}]'), NS(text=None)])
    assert message_text(msg) == '[{"a": 1}]'


def test_a_null_or_non_finite_box_is_skipped():
    from tools.inventory_vlm import parse_boxes
    text = ('[{"bbox_2d": null, "box_2d": [10, 20, 30, 40], "label": "a"}, '
            '{"box_2d": [NaN, 2, 3, 4], "label": "b"}]')
    got = parse_boxes(text, 1, 1, order="yxyx")
    assert [(b["label"], b["xyxy"]) for b in got] == [("a", [20.0, 10.0, 40.0, 30.0])]


def test_litert_refuses_to_resume_into_another_runs_file(tmp_path, monkeypatch):
    import pytest

    from tools import inventory_vlm as v
    monkeypatch.setattr(v, "OUT", tmp_path)
    (tmp_path / "vlm_boxes_g.json").write_text(
        '{"f.jpg": {"boxes": [], "raw": "", "model": "x.litertlm", "device": "cpu"}}')
    frames = tmp_path / "frames.json"
    frames.write_text('["f.jpg"]')
    base = ["name", "--backend", "litert", "--frames", str(frames)]
    with pytest.raises(SystemExit, match="needs --tag"):
        v.main(base + ["--model", "x.litertlm"])
    with pytest.raises(SystemExit, match="another run"):
        v.main(base + ["--model", "x.litertlm", "--device", "gpu", "--tag", "g"])


def test_judge_tuning_only_leaves_the_users_sample_unjudged(tmp_path, monkeypatch):
    from tools import inventory_vlm as v
    monkeypatch.setattr(v, "OUT", tmp_path)
    box = [{"label": "chair", "xyxy": [0, 0, 1, 1], "n": 1}]
    (tmp_path / "vlm_boxes_g.json").write_text(json.dumps(
        {"tune.jpg": {"boxes": box}, "user.jpg": {"boxes": box}, "none.jpg": {"boxes": []}}))
    (tmp_path / "sample.json").write_text('["user.jpg"]')
    asked = []
    monkeypatch.setattr(v, "_client", lambda: None)
    monkeypatch.setattr(v, "judge_one", lambda c, path, boxes, spend: asked.append(path.name) or {})
    v.main(["judge", "--tag", "g", "--tuning-only"])
    assert asked == ["tune.jpg"]
    v.main(["judge", "--tag", "g"])  # the second run resumes and adds only the user's frame
    assert asked == ["tune.jpg", "user.jpg"]


def test_judge_tuning_only_refuses_a_redrawn_sample(tmp_path, monkeypatch):
    from tools import inventory_vlm as v
    monkeypatch.setattr(v, "OUT", tmp_path)
    (tmp_path / "vlm_boxes_g.json").write_text('{"tune.jpg": {"boxes": []}}')
    (tmp_path / "sample.json").write_text('["other.jpg"]')
    monkeypatch.setattr(v, "_client", lambda: pytest.fail("no client before the check"))
    with pytest.raises(SystemExit, match="sample.json changed"):
        v.main(["judge", "--tag", "g", "--tuning-only"])
