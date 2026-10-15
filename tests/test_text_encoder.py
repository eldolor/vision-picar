"""3.81 T1-T2: perception's text features come from a disk cache or a child
process, never from a text tower held in perception's process."""
import json
import struct

import pytest

from brain import text_encoder as te
from brain.perceive import PerceptionUnavailable


@pytest.fixture
def cache(tmp_path, monkeypatch):
    monkeypatch.setenv("PICAR_TEXT_CACHE", str(tmp_path / "cache"))
    src = tmp_path / "mobileclip_blt.ts"
    src.write_bytes(b"x" * 10)
    return tmp_path / "cache", str(src)


def counting(rows):
    calls = []

    def run(args):
        calls.append(list(args))
        return rows
    return run, calls


def test_a_miss_runs_the_child_once_then_the_cache_answers(cache):
    _, src = cache
    run, calls = counting([[0.5, -0.25]])
    assert te.yoloe_features(src, "red backpack", runner=run) == [0.5, -0.25]
    assert te.yoloe_features(src, "red backpack", runner=run) == [0.5, -0.25]
    assert calls == [["yoloe", src, "red backpack"]]


def test_features_round_trip_float32_exactly(cache):
    _, src = cache
    v = struct.unpack("f", struct.pack("f", 0.1))[0]  # a float32 value, as tolist() gives
    te.yoloe_features(src, "t", runner=counting([[v]])[0])
    assert te.yoloe_features(src, "t", runner=lambda _: pytest.fail("cache missed")) == [v]


def test_key_changes_with_text_and_with_the_mobileclip_file(cache, tmp_path):
    _, src = cache
    run, calls = counting([[1.0]])
    te.yoloe_features(src, "a", runner=run)
    te.yoloe_features(src, "b", runner=run)
    (tmp_path / "mobileclip_blt.ts").write_bytes(b"y" * 11)  # another checkpoint
    te.yoloe_features(src, "a", runner=run)
    assert [c[-1] for c in calls] == ["a", "b", "a"]


def test_clip_encodes_the_tuple_as_one_batch_and_keys_on_it(cache):
    run, calls = counting([[1.0, 0.0], [0.0, 1.0]])
    rows = te.clip_features("RN50", "openai", ("bag", "box"), runner=run)
    assert rows == [[1.0, 0.0], [0.0, 1.0]]
    te.clip_features("RN50", "openai", ("bag", "box"), runner=run)
    assert calls == [["clip", "RN50", "openai", "bag", "box"]]
    with pytest.raises(PerceptionUnavailable):  # another tuple, the child's 2 rows refused
        te.clip_features("RN50", "openai", ("box",), runner=run)
    run2, calls2 = counting([[1.0, 0.0], [0.0, 1.0]])
    te.clip_features("ViT-B-32", "openai", ("bag", "box"), runner=run2)
    assert len(calls2) == 1  # another model is another key


def test_an_unreadable_or_foreign_cache_file_is_encoded_again(cache):
    d, _ = cache
    run, calls = counting([[1.0]])
    path = d / f"{te.cache_key('m', ['a'])}.json"
    d.mkdir(parents=True)
    path.write_text("{not json")
    assert te.features("m", ["a"], [], runner=run) == [[1.0]]
    path.write_text(json.dumps({"model": "m", "texts": ["other"], "features": [[9.0]]}))
    assert te.features("m", ["a"], [], runner=run) == [[1.0]]
    path.write_text(json.dumps({"model": "m", "texts": ["a"], "features": [["x"]]}))
    assert te.features("m", ["a"], [], runner=run) == [[1.0]]
    assert len(calls) == 3
    assert json.loads(path.read_text())["features"] == [[1.0]]


@pytest.mark.parametrize("junk", [[], [[]], [[1.0], [2.0]], [[True]], [[1.0, 2.0]][0:0], "x"])
def test_a_child_that_returns_junk_is_refused_and_not_cached(cache, junk):
    d, _ = cache
    with pytest.raises(PerceptionUnavailable):
        te.features("m", ["a"], [], runner=lambda _: junk)
    assert not d.exists() or not list(d.iterdir())


def test_rows_of_unequal_length_are_refused(cache):
    with pytest.raises(PerceptionUnavailable):
        te.features("m", ["a", "b"], [], runner=lambda _: [[1.0, 2.0], [1.0]])


def test_an_unwritable_cache_still_returns_features(cache, monkeypatch):
    monkeypatch.setattr(te.tempfile, "mkstemp",
                        lambda **_: (_ for _ in ()).throw(PermissionError("ro")))
    run, calls = counting([[2.0]])
    assert te.features("m", ["a"], [], runner=run) == [[2.0]]
    assert te.features("m", ["a"], [], runner=run) == [[2.0]]
    assert len(calls) == 2


class Done:
    def __init__(self, rc, out="", err=""):
        self.returncode, self.stdout, self.stderr = rc, out, err


def test_the_child_runs_from_the_repo_without_cuda_and_its_last_line_is_the_answer(monkeypatch):
    seen = {}

    def fake_run(cmd, **kw):
        seen.update(cmd=cmd, **kw)
        return Done(0, "a warning line\n[[0.25, 0.5]]\n")
    monkeypatch.setattr(te.subprocess, "run", fake_run)
    assert te.run_child(["yoloe", "/w/m.ts", "red backpack"]) == [[0.25, 0.5]]
    assert seen["cmd"][1:] == ["-m", "brain.text_encoder", "yoloe", "/w/m.ts", "red backpack"]
    assert seen["env"]["CUDA_VISIBLE_DEVICES"] == ""
    assert (seen["cwd"] / "brain" / "text_encoder.py").exists()


@pytest.fixture(autouse=True)
def _no_failures(monkeypatch):
    monkeypatch.setattr(te, "_failed", {})


@pytest.mark.parametrize("result", [Done(1, err="boom"), Done(0, ""), Done(0, "nope\n")])
def test_a_failed_child_is_perception_unavailable(monkeypatch, result):
    monkeypatch.setattr(te.subprocess, "run", lambda *a, **k: result)
    with pytest.raises(PerceptionUnavailable):
        te.run_child(["yoloe", "m.ts", "t"])


def test_a_hung_child_is_perception_unavailable(monkeypatch):
    def hang(*a, **k):
        raise te.subprocess.TimeoutExpired("x", 1)
    monkeypatch.setattr(te.subprocess, "run", hang)
    with pytest.raises(PerceptionUnavailable):
        te.run_child(["yoloe", "m.ts", "t"])


def test_the_tower_file_sits_beside_the_full_file_and_names_its_size(tmp_path):
    full = tmp_path / "mobileclip_blt.ts"
    full.write_bytes(b"x" * 7)
    assert te.tower_path(str(full)) == tmp_path / "mobileclip_blt_text_7.ts"
    full.write_bytes(b"x" * 8)  # a replaced checkpoint gets its own cut
    assert te.tower_path(str(full)) == tmp_path / "mobileclip_blt_text_8.ts"


def test_a_failed_child_is_not_run_again_for_a_while(monkeypatch):
    monkeypatch.setattr(te, "_failed", {})
    now = [1000.0]
    monkeypatch.setattr(te.time, "monotonic", lambda: now[0])
    runs = []
    monkeypatch.setattr(te.subprocess, "run", lambda *a, **k: runs.append(1) or Done(1, err="x"))
    for _ in range(3):
        with pytest.raises(PerceptionUnavailable):
            te.run_child(["clip", "RN50", "openai", "a"])
    assert len(runs) == 1
    with pytest.raises(PerceptionUnavailable):  # other texts are not held back
        te.run_child(["clip", "RN50", "openai", "b"])
    assert len(runs) == 2
    now[0] += te.RETRY_AFTER_S
    monkeypatch.setattr(te.subprocess, "run", lambda *a, **k: Done(0, "[[1.0]]"))
    assert te.run_child(["clip", "RN50", "openai", "a"]) == [[1.0]]


def test_the_child_timeout_is_inside_the_brains_tick_guard():
    from control.brain_config import load_brain_config
    assert te.CHILD_TIMEOUT_S < load_brain_config()["tick_timeout_s"]
