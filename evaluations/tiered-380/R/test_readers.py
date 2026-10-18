import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from readers import decided_native, p_native, parse_any_box  # noqa: E402
from native_server import boxes_reply, native_target  # noqa: E402


def step(tok, alts):
    return {"token": tok, "logprob": math.log(alts[tok]),
            "top_logprobs": [{"token": t, "logprob": math.log(p)} for t, p in alts.items()]}


def test_synthetic_native_reply_is_binary():
    _t, yes = boxes_reply([[1, 2, 3, 4]])
    _t, no = boxes_reply([])
    assert p_native(yes)[0] == 1.0 and p_native(no)[0] == 0.0


def test_native_first_token_share():
    s, parsed, at = p_native([step(" ", {" ": 1.0}),
                              step("<box>", {"<box>": 0.6, "There": 0.3, "[]": 0.1})])
    assert parsed and at == 1 and abs(s - 0.6) < 1e-9
    assert decided_native([step("<box>", {"<box>": 1.0})])
    assert not decided_native([])


def test_points_read_as_unit_boxes():
    assert parse_any_box("<ref>bottle</ref><box><12><1><25><22></box>") == [12, 1, 25, 22]
    assert parse_any_box("<ref>bottle</ref><box><500><400></box>") == [500, 400, 501, 401]
    assert parse_any_box('<points coords="1 523 412">bottle</points>') == [523, 412, 524, 413]
    assert parse_any_box("[]") is None


def test_native_prompts_only_match_their_own_template():
    assert native_target("florence", "<CAPTION_TO_PHRASE_GROUNDING>blue bottle") == ("phrase", "blue bottle")
    assert native_target("florence", "Output the bounding box of the blue bottle as JSON") is None
    assert native_target("photon", "point: red backpack") == ("point", "red backpack")


def test_locateanything_decides_after_box():
    present = [step("<ref>", {"<ref>": 1.0}), step("cup", {"cup": 1.0}), step("</ref>", {"</ref>": 1.0}),
               step("<box>", {"<box>": 1.0}), step("<932>", {"<932>": 0.5, "<931>": 0.3, "None": 0.2})]
    s, parsed, at = p_native(present)
    assert parsed and at == 4 and abs(s - 0.8) < 1e-9
    absent = present[:4] + [step("None", {"None": 0.9, "<806>": 0.1})]
    assert abs(p_native(absent)[0] - 0.1) < 1e-9
    assert not decided_native(present[:3]) and decided_native(present)
    assert parse_any_box("<ref>cup</ref><box>None</box>") is None
