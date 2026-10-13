"""3.78 amendment 1: arm T's decision from greedy tokens and its stop length."""

from tools.jetson.bench_identity_llama import MAX_TOKENS
from tools.jetson.bench_identity_trt import decide, input_doc, request, stop_length


def test_merged_box_token_is_yes_at_that_step():
    assert decide(["[{", '"bbox'])[0:3:2] == (1.0, 0)


def test_empty_list_is_no():
    score, parsed, at = decide(["[]"])
    assert (score, parsed, at) == (0.0, True, 0)


def test_bare_bracket_defers_through_whitespace_to_the_decision():
    score, parsed, at = decide(["```", "json", "\n", "[", "\n ", " {"])
    assert (score, parsed, at) == (1.0, True, 5)


def test_bare_object_before_a_list_is_a_box():
    assert decide(['{"', "bbox"])[0] == 1.0


def test_no_list_is_unparsed_and_scores_zero():
    assert decide(["I", " cannot"]) == (0.0, False, None)


def test_stop_length_includes_the_deciding_token_or_runs_full():
    assert stop_length(5) == 6
    assert stop_length(None) == MAX_TOKENS


def test_greedy_request_shape(tmp_path):
    doc = input_doc([request(tmp_path / "f.jpg", "blue bottle", 4)])
    assert doc["top_k"] == 1 and doc["batch_size"] == 1
    r = doc["requests"][0]
    assert r["max_generate_length"] == 4
    assert r["messages"][0]["content"][0] == {"type": "image", "image": str(tmp_path / "f.jpg")}
    assert "blue bottle" in r["messages"][0]["content"][1]["text"]
