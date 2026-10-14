"""3.78 amendment 3: arm T's stream parsed into the shape arm L's scorer reads."""

import math

import pytest

from tools.jetson.bench_identity_llama import decision_reached, p_localised
from tools.jetson.bench_identity_trt import (
    content_entry, parse_end_line, parse_token_line, request_doc)

VOCAB = {1: "[", 2: "{", 3: "]", 4: "\n", 5: "```", 6: "[{", 7: "[]"}


def entry(chosen, *alts):
    return content_entry(VOCAB.get, chosen, list(alts))


def test_token_line_round_trip():
    assert parse_token_line("T 2 2 2:-0.105361 3:-2.302585") == (
        2, [(2, -0.105361), (3, -2.302585)])
    assert parse_token_line("T 9 0") == (9, [])
    with pytest.raises(ValueError):
        parse_token_line("E 3 12.5 1")


def test_end_line():
    assert parse_end_line("E 3 2481.250 1") == (3, 2481.25, True)
    assert parse_end_line("E 0 0 0")[2] is False


def test_entry_carries_the_chosen_logprob_and_alternatives():
    e = entry(2, (2, math.log(0.9)), (3, math.log(0.1)))
    assert e["token"] == "{" and e["logprob"] == pytest.approx(math.log(0.9))
    assert [a["token"] for a in e["top_logprobs"]] == ["{", "]"]


def test_scored_as_arm_l_bare_bracket_then_box():
    content = [entry(5, (5, 0.0)), entry(4, (4, 0.0)),
               entry(1, (1, math.log(0.8)), (7, math.log(0.2))),
               entry(2, (2, math.log(0.75)), (3, math.log(0.25)))]
    assert not decision_reached(content[:3])
    assert decision_reached(content)
    score, parsed, at = p_localised(content)
    # P = (F + O*f) / (F + E + O) with F=0, E=0.2, O=0.8, f=0.75.
    assert parsed and at == 3 and score == pytest.approx(0.6)


def test_merged_empty_list_decides_no_at_once():
    content = [entry(7, (7, math.log(0.7)), (6, math.log(0.3)))]
    assert decision_reached(content)
    assert p_localised(content)[0] == pytest.approx(0.3)


def test_request_is_greedy_with_top20_logprobs(tmp_path):
    doc = request_doc(tmp_path / "f.jpg", "blue bottle")
    assert doc["top_k"] == 1 and doc["num_logprobs"] == 20 and doc["batch_size"] == 1
    assert doc["max_generate_length"] == 64
    c = doc["requests"][0]["messages"][0]["content"]
    assert c[0] == {"type": "image", "image": str(tmp_path / "f.jpg")}
    assert "blue bottle" in c[1]["text"]
