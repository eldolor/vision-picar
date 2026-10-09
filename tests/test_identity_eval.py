"""
tests/test_identity_eval.py

3.48's scorer (`control/identity_eval.py`) on a tiny synthetic corpus: the
cloud becomes a row, today's labels overturn stored ones, and the cloud's
confabulations are counted against every model.
"""

import json

from control.identity_eval import (
    cloud_records, confabulations, current_labels, main, relabel, row)
from control.perception_eval import FrameScore, load_corpus, save_records


def _walk(root, name, labels, cloud_says, target="blue bottle"):
    d = root / name
    d.mkdir()
    for frame in labels:
        (d / frame).write_bytes(b"\xff\xd8")  # a name on disk is all load_walk needs
    (d / "labels.json").write_text(json.dumps(
        {"description": target, "target_visible_labels": labels}))
    rows = [{"seq": i, "file": f, "navigate": {"target_visible": said,
                                                "model_id": "opus-test"}}
            for i, (f, said) in enumerate(cloud_says.items())]
    (d / "walk.jsonl").write_text("\n".join(json.dumps(r) for r in rows))


def _corpus(tmp_path):
    # f0 target, cloud yes; f1 a bin, cloud yes (a confabulation);
    # f2 empty, cloud no; f3 target, cloud said nothing.
    _walk(tmp_path, "w1",
          {"f0.jpg": True, "f1.jpg": False, "f2.jpg": False, "f3.jpg": True},
          {"f0.jpg": True, "f1.jpg": True, "f2.jpg": False})
    return load_corpus(tmp_path)


def test_the_cloud_is_scored_like_any_model_and_silence_is_never_a_yes(tmp_path):
    walks = _corpus(tmp_path)
    cloud = cloud_records(walks)
    r = row("cloud", cloud, 0.5, confabulations(cloud))
    assert (r["tp"], r["fp"], r["visible"]) == (1, 1, 2)
    assert r["precision"] == 0.5 and r["recall"] == 0.5
    assert {c.frame: c.score for c in cloud}["f3.jpg"] is None
    assert r["yes_on_confabulations"] == 1 == r["confabulation_frames"]


def test_todays_labels_win_and_flips_and_gaps_are_counted(tmp_path):
    labels = current_labels(_corpus(tmp_path))
    stored = [FrameScore("w1", "f0.jpg", visible=False, status="detected", score=0.9),
              FrameScore("w1", "f1.jpg", visible=False, status="absent", score=0.1),
              FrameScore("other", "x.jpg", visible=True, status="detected", score=0.9)]
    kept, flips, missing = relabel(stored, labels)
    assert [r.frame for r in kept] == ["f0.jpg", "f1.jpg"]
    assert kept[0].visible is True and flips == 1
    assert missing == 2  # f2 and f3 have no stored record


def test_a_model_that_says_yes_on_the_confabulation_is_caught(tmp_path):
    walks = _corpus(tmp_path)
    confab = confabulations(cloud_records(walks))
    model = [FrameScore("w1", "f0.jpg", True, "detected", 1.0),
             FrameScore("w1", "f1.jpg", False, "detected", 0.6),
             FrameScore("w1", "f2.jpg", False, "absent", 0.0),
             FrameScore("w1", "f3.jpg", True, "absent", 0.0)]
    assert row("m", model, 0.5, confab)["yes_on_confabulations"] == 1
    assert row("m", model, 0.7, confab)["yes_on_confabulations"] == 0


def test_the_command_prints_one_row_per_model_plus_the_cloud(tmp_path, capsys):
    _corpus(tmp_path)
    rec = tmp_path / "m.json"
    save_records(rec, [FrameScore("w1", "f0.jpg", True, "detected", 1.0)],
                 {"detector": "vlm:test", "device": "cuda", "dtype": "fp32"})
    out_json = tmp_path / "rows.json"
    assert main(["--recordings", str(tmp_path), "--walk", "w1",
                 "--records", f"m={rec}:0.5", "--json", str(out_json)]) == 0
    out = capsys.readouterr().out
    assert "cloud (opus-test)" in out and "| m | 0.5 |" in out
    assert "3 labelled frames not covered" in out
    assert len(json.loads(out_json.read_text())) == 2
