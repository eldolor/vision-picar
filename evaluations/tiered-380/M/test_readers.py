import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from readers import decided, p_box, parse_any_box  # noqa: E402


def step(tok, alts):
    return {"token": tok, "logprob": math.log(alts[tok]),
            "top_logprobs": [{"token": t, "logprob": math.log(p)} for t, p in alts.items()]}


def test_tag_generated_is_a_box():
    s, parsed, at = p_box([step("<", {"<": 1.0}), step("<box>", {"<box>": 0.7, "[]": 0.3})])
    assert parsed and at == 1 and abs(s - 0.7) < 1e-9
    assert decided([step("<box>", {"<box>": 1.0})])


def test_list_forms_as_l():
    c = [step("prefix", {"prefix": 1.0}), step("[", {"[": 0.8, "[]": 0.2}), step("4", {"4": 0.7, "]": 0.3})]
    s, parsed, at = p_box(c)
    assert parsed and at == 2 and abs(s - 0.56) < 1e-9


def test_tag_alternative_counts_as_found():
    s, _, _ = p_box([step("[]", {"[]": 0.5, "<box>": 0.5})])
    assert abs(s - 0.5) < 1e-9


def test_no_list_is_unparsed():
    assert p_box([step("No", {"No": 1.0})]) == (0.0, False, None)


def test_parse_forms():
    assert parse_any_box('[{"bbox_2d": [1, 2, 30, 40]}]') == [1, 2, 30, 40]
    assert parse_any_box("<ref>bottle</ref><box>[[100, 200, 300, 400]]</box>") == [100, 200, 300, 400]
    assert parse_any_box("<box>10 20 50 60</box>") == [10, 20, 50, 60]
    assert parse_any_box("[0.1, 0.2, 0.5, 0.6]") == [0.1, 0.2, 0.5, 0.6]
    assert parse_any_box("<box><x_129><y_190><x_205><y_233></box>") == [129, 190, 205, 233]
    assert parse_any_box('[["791, 376, 897, 516"]]<|im_end|>') == [791, 376, 897, 516]
    assert parse_any_box("[]") is None
    assert parse_any_box("[50, 20, 10, 60]") is None
