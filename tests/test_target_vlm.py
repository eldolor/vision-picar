"""tests/test_target_vlm.py -- 3.46 amendment 11's search runner, on fakes:
no model, no recordings beyond a temp walk."""
import json

import pytest

from control import perception_eval as pe
from tools import target_vlm as t


def _walk(root, name="w", target="a red mug"):
    d = root / name
    d.mkdir()
    for i in range(3):
        (d / f"frame-000{i}.jpg").write_bytes(b"")
    (d / "labels.json").write_text(json.dumps({
        "description": target,
        "target_visible_labels": {"frame-0000.jpg": True, "frame-0001.jpg": False,
                                  "frame-0002.jpg": True}}))
    return d


class FakeNamer:
    def __init__(self, *a, **k):
        self.prompt, self.seen = None, []

    def __call__(self, fid):
        self.seen.append((fid, self.prompt))
        if fid.endswith("0002.jpg"):
            raise RuntimeError("engine hiccup")
        return {"boxes": [{"label": "mug", "xyxy": [0, 0, 1, 1]}] if fid.endswith("0000.jpg")
                else [], "raw": "[]"}


def test_records_score_with_perception_eval_and_a_failed_frame_is_retried(tmp_path, monkeypatch):
    _walk(tmp_path)
    monkeypatch.setattr(t, "ROOT", tmp_path)
    fake = FakeNamer()
    monkeypatch.setattr(t, "LiteRTNamer", lambda *a, **k: fake)
    walks, save = tmp_path / "walks.json", tmp_path / "out.json"
    walks.write_text('["w"]')
    with pytest.raises(SystemExit, match="1 frames missing"):
        t.main(["--model", "m.litertlm", "--walks", str(walks), "--save", str(save)])
    records, config = pe.load_records(save)  # the scorer reads the file as written
    assert config["detector"] == "litert:m.litertlm"
    got = {r.frame: (r.visible, r.score) for r in records}
    assert got == {"frame-0000.jpg": (True, 1.0), "frame-0001.jpg": (False, 0.0)}
    assert fake.seen[0][1] == t.prompt_for("a red mug")
    s = pe.score_at(records, 0.5)
    assert (s["tp"], s["fp"], s["fn"]) == (1, 0, 0)
    with pytest.raises(SystemExit, match="1 frames missing"):
        t.main(["--model", "m.litertlm", "--walks", str(walks), "--save", str(save)])
    assert [f for f, _ in fake.seen[3:]] == ["w/frame-0002.jpg"]  # only the failed frame


def test_a_file_from_another_model_is_refused(tmp_path, monkeypatch):
    _walk(tmp_path)
    monkeypatch.setattr(t, "ROOT", tmp_path)
    monkeypatch.setattr(t, "LiteRTNamer", FakeNamer)
    walks, save = tmp_path / "walks.json", tmp_path / "out.json"
    walks.write_text('["w"]')
    with pytest.raises(SystemExit, match="frames missing"):
        t.main(["--model", "a.litertlm", "--walks", str(walks), "--save", str(save)])
    with pytest.raises(SystemExit, match="another run"):
        t.main(["--model", "b.litertlm", "--walks", str(walks), "--save", str(save)])


def test_the_prompt_names_the_target_and_gemmas_box_order():
    p = t.prompt_for("blue bottle")
    assert "blue bottle" in p and "box_2d" in p and "[y1, x1, y2, x2]" in p


def test_an_answer_is_read_as_boxes_empty_or_unparsed():
    assert t.outcome("[]", []) == "empty"
    assert t.outcome("```json\n[]\n```", []) == "empty"
    assert t.outcome("Yes, the bottle is on the left.", []) == "unparsed"
    assert t.outcome('[{"box_2d": [1, 2, 3, 4]}]', []) == "empty"  # no usable box
    assert t.outcome("anything", [{"label": "x"}]) == "boxes"
