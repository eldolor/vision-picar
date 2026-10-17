import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from rb_identity import decided, p_list  # noqa: E402


def step(tok, alts):
    return {"token": tok, "logprob": math.log(alts[tok]),
            "top_logprobs": [{"token": t, "logprob": math.log(p)} for t, p in alts.items()]}


def test_empty_list_merged():
    s, parsed, _ = p_list([step("[]", {"[]": 0.9, "[[": 0.1})])
    assert parsed and abs(s - 0.1) < 1e-9


def test_bare_bracket_then_digit_is_a_box():
    c = [step("[", {"[": 0.8, "[]": 0.2}), step("4", {"4": 0.7, "]": 0.3})]
    s, parsed, at = p_list(c)
    assert parsed and at == 1 and abs(s - 0.8 * 0.7 / 1.0) < 1e-9
    assert decided(c) and not decided(c[:1])


def test_json_box_still_counts():
    s, _, _ = p_list([step("[{", {"[{": 0.6, "[]": 0.4})])
    assert abs(s - 0.6) < 1e-9


def test_no_list_is_unparsed():
    assert p_list([step("I", {"I": 1.0})]) == (0.0, False, None)
