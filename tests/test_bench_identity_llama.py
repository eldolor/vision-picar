"""tests/test_bench_identity_llama.py -- 3.48 amendments 1-2's score: P(it
localises) read from llama-server's top_logprobs, whatever the tokenizer
merges `[` with and however much whitespace comes before the brace."""

import math

import pytest

from tools.jetson.bench_identity_llama import (
    PROMPT, compare_decisions, decision_ms, decision_reached, p_localised,
    percentile, stream_chunks)


def step(token, alts):
    return {"token": token, "logprob": math.log(dict(alts).get(token, 1e-9)),
            "top_logprobs": [{"token": t, "logprob": math.log(p)} for t, p in alts]}


def test_a_bare_bracket_is_decided_at_the_next_token():
    content = [step("[", [("[", 1.0)]), step("{", [("{", 0.8), ("]", 0.2)])]
    assert p_localised(content) == (pytest.approx(0.8), True, 1)


def test_merged_tokens_are_read_at_the_bracket_itself():
    # Qwen-style vocabularies have `[{` and `[]` as single tokens.
    content = [step("[]", [("[]", 0.7), ("[{", 0.3)])]
    assert p_localised(content) == (pytest.approx(0.3), True, 0)


def test_all_three_branches_combine():
    content = [step("[", [("[", 0.5), ("[{", 0.3), ("[]", 0.2)]),
               step("{", [("{", 0.6), ("]", 0.4)])]
    # (0.3 + 0.5 * 0.6) / 1.0
    assert p_localised(content)[0] == pytest.approx(0.6)


def test_a_code_fence_before_the_list_is_skipped():
    content = [step("```", [("```", 1.0)]), step("json", [("json", 1.0)]),
               step("\n", [("\n", 1.0)]), step(" [", [(" [", 0.9), ("[]", 0.1)]),
               step("]", [("]", 0.95), ("{", 0.05)])]
    score, parsed, decided = p_localised(content)
    # F = 0, E = 0.1 (`[]`), O = 0.9 (` [`), f = 0.05 -> 0.9 * 0.05 / 1.0
    assert parsed and decided == 4
    assert score == pytest.approx(0.045)


def test_whitespace_between_the_bracket_and_the_brace_defers_the_decision():
    """Amendment 2: the smoke test's real reply -- `[\\n`, `\\t`, `{"` --
    boxed the target and must score as found, not ~0.5."""
    content = [
        step("[\n", [("[\n", 0.9), ("[]\n", 0.09), ("[", 0.01)]),
        step("\t", [("\t", 0.55), ("   ", 0.30), (" ", 0.14), ("]\n", 0.006), ('{"', 0.001)]),
        step('{"', [('{"', 0.997), ("{\n", 0.0015)]),
    ]
    score, parsed, decided = p_localised(content)
    assert parsed and decided == 2
    assert score > 0.85, score   # 0.53 before the correction


def test_an_empty_list_after_whitespace_is_not_found():
    content = [step("[\n", [("[\n", 0.6), ("[]\n", 0.4)]),
               step("]", [("]", 0.99), ("{", 0.01)])]
    score, _, _ = p_localised(content)
    assert score == pytest.approx(0.6 * 0.01 / 1.0)


def test_a_bare_object_before_any_list_is_a_box():
    """Review of amendment 2: `{"bbox_2d": [257, ...]}` has a `[` -- the
    coordinates -- after the box opened. Reading that `[` scored a boxed
    target 0."""
    content = [step("{\"", [("{\"", 0.8), ("[]", 0.15), ("[", 0.05)]),
               step("bbox", [("bbox", 1.0)]), step("_2d", [("_2d", 1.0)]),
               step("\": [", [("\": [", 1.0)]), step("257", [("257", 1.0)])]
    score, parsed, decided = p_localised(content)
    assert parsed and decided == 0
    assert score == pytest.approx(0.8 / 0.95)


def test_a_reply_that_never_opens_a_list_scores_zero_and_says_so():
    content = [step("The", [("The", 1.0)]), step(" bottle", [(" bottle", 1.0)])]
    assert p_localised(content) == (0.0, False, None)


def test_decision_time_is_the_prompt_plus_tokens_up_to_the_decision():
    t = {"prompt_ms": 2700.0, "predicted_ms": 3000.0, "predicted_n": 40}
    assert decision_ms(t, 4) == pytest.approx(2700 + 75 * 5)
    assert decision_ms(t, None) is None
    assert decision_ms({}, 3) is None


def test_the_prompt_is_the_a10g_rows_prompt():
    assert PROMPT.format(target="blue bottle") == (
        'Output the bounding box of the blue bottle as JSON: '
        '[{"bbox_2d": [x1, y1, x2, y2]}]. If it is not visible, '
        'output []. Output JSON only.')


def test_percentile_is_nearest_rank():
    assert percentile(list(range(1, 11)), 90) == 9
    assert percentile([5.0], 90) == 5.0


# ---------------------------------------------------------------------------
# 3.58: stop at the decision.

def _stopped(content):
    """What `ask_until_decided` keeps: tokens up to the first prefix for
    which the stopping rule holds, or everything if it never does."""
    for k in range(1, len(content) + 1):
        if decision_reached(content[:k]):
            return content[:k]
    return content


REPLIES = {
    "bare bracket then brace": [step("[", [("[", 1.0)]),
                                step("{", [("{", 0.8), ("]", 0.2)]),
                                step('"bbox', [('"bbox', 1.0)])],
    "merged empty": [step("[]", [("[]", 0.7), ("[{", 0.3)]), step("\n", [("\n", 1.0)])],
    "code fence then list": [step("```", [("```", 1.0)]), step("json", [("json", 1.0)]),
                             step("\n", [("\n", 1.0)]),
                             step(" [", [(" [", 0.9), ("[]", 0.1)]),
                             step("]", [("]", 0.95), ("{", 0.05)]), step("\n```", [("\n```", 1.0)])],
    "whitespace deferral (the smoke test's reply)": [
        step("[\n", [("[\n", 0.9), ("[]\n", 0.09), ("[", 0.01)]),
        step("\t", [("\t", 0.55), ("   ", 0.30), (" ", 0.14), ("]\n", 0.006), ('{"', 0.001)]),
        step('{"', [('{"', 0.997), ("{\n", 0.0015)]),
        step("bbox", [("bbox", 1.0)])],
    "bare object": [step('{"', [('{"', 0.8), ("[]", 0.15), ("[", 0.05)]),
                    step("bbox", [("bbox", 1.0)]), step('": [', [('": [', 1.0)])],
    "never a list": [step("The", [("The", 1.0)]), step(" bottle", [(" bottle", 1.0)])],
}


@pytest.mark.parametrize("name", REPLIES)
def test_stopping_at_the_decision_changes_no_score(name):
    """3.58's stopping rule: on the same greedy stream, the score over the
    stopped reply is the score over the full reply."""
    content = REPLIES[name]
    stopped = _stopped(content)
    assert p_localised(stopped)[:2] == p_localised(content)[:2], name
    if p_localised(content)[2] is not None:
        # ...and it stops AT the deciding token, never later.
        assert len(stopped) == p_localised(content)[2] + 1, (name, len(stopped))


def test_a_bare_bracket_alone_has_not_decided():
    assert not decision_reached([step("[", [("[", 1.0)])])
    assert not decision_reached([step("[\n", [("[\n", 1.0)]), step("\t", [("\t", 1.0)])])
    assert not decision_reached([step("```", [("```", 1.0)])])


def test_stream_chunks_yields_each_token_entry_and_stops_at_done():
    import json as _json
    entry = {"token": "[", "logprob": -0.1, "top_logprobs": []}
    lines = [b"data: " + _json.dumps({"choices": [{"delta": {"role": "assistant"}}]}).encode(),
             b"",
             b"data: " + _json.dumps({"choices": [{"delta": {"content": "["},
                                                   "logprobs": {"content": [entry]}}]}).encode(),
             b"data: [DONE]",
             b"data: " + _json.dumps({"choices": [{"logprobs": {"content": [entry]}}]}).encode()]
    assert list(stream_chunks(lines)) == [entry]


def _rec(frame, score):
    return {"walk": "w", "frame": frame, "score": score}


def test_compare_decisions_counts_flips_and_the_largest_difference():
    old = [_rec("a", 0.9), _rec("b", 0.1), _rec("c", 0.6), _rec("d", 0.2)]
    new = [_rec("a", 0.92), _rec("b", 0.1), _rec("c", 0.4), _rec("d", None), _rec("e", 0.5)]
    got = compare_decisions(new, old)
    assert got["flips"] == ["w/c", "w/d"]       # c crossed the gate; d unanswered
    assert got["missing"] == ["w/e"]
    assert got["compared"] == 4
    assert got["max_score_diff"] == pytest.approx(0.02)
