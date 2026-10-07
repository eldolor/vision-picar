"""3.41 Part A's verdict, pinned against the recorded Jetson runs
(`evaluations/trt-341/`). No TensorRT needed: this reads JSON."""

import json
from pathlib import Path

from tools.jetson.bench_trt import (PROB_BAR, REBUILD_BAR_S, compare,
                                    room_for_c, verdict)

RUNS = Path(__file__).resolve().parent.parent / "evaluations" / "trt-341"


def _load(name):
    return json.loads((RUNS / f"{name}.json").read_text())


def test_the_control_reproduces_3_33():
    a = _load("A")
    assert a["frames"] == 60 and a["unavailable"] == 0
    # 3.33 measured 60.6 / 109.9 ms; same board, same frames, same 15 W.
    assert 55 <= a["median"]["total_ms"] <= 66
    assert 100 <= a["p90"]["total_ms"] <= 125


def test_b1_fp16_engines_change_the_answers_and_are_not_adopted():
    v = verdict(_load("B1"), _load("A"), _load("mem-B1"), _load("mem-A"))
    assert v["status_agree"]                  # no gate decision changed...
    assert v["max_dprob"] > PROB_BAR          # ...but probabilities moved 0.052
    assert not v["proposals_agree"]           # and one frame gained a crop
    assert not v["criterion_1"]
    assert 0.15 < v["p90_gain"] < 0.30        # 20% faster at p90: under the bar
    assert v["mem_gain_mb"] < 500             # 165 MB lighter a mission
    assert not v["adopt"]


def test_b2_gpu_resize_flips_a_verdict_and_is_not_adopted():
    v = verdict(_load("B2"), _load("A"), _load("mem-B2"), _load("mem-A"))
    assert not v["status_agree"]
    assert not v["criterion_1"] and not v["adopt"]


def test_a_new_target_is_ready_in_under_a_minute_once_the_cache_is_warm():
    b = _load("build")
    first, *rest = b["yoloe"]
    assert first["onnx_s"] + first["build_s"] > 600     # the one cold build
    assert rest and all(r["onnx_s"] + r["build_s"] < REBUILD_BAR_S for r in rest)


def test_room_left_for_an_isaac_arm():
    # Over 30% of B1's p90 is not model time, so the amendment's room test
    # does not rule C out by itself.
    assert room_for_c(_load("B1")) > 0.30


def test_compare_reports_every_arm():
    out = compare([RUNS / "A.json", RUNS / "B1.json", RUNS / "B2.json"])
    assert [a["arm"] for a in out["arms"]] == ["A", "B1", "B2"]
