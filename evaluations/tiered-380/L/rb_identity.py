"""
evaluations/tiered-380/L/rb_identity.py

3.80.L: RoboBrain2.0-3B's arrival answer on the pinned 299 frames under
Edge-LLM (RVE). `tools.jetson.bench_identity_trt` with two things swapped:
the prompt (`--arm B`, the list prompt; `--arm A`, 3.48's JSON prompt) and
the reader (`p_list`: 3.48's `p_localised`, with RoboBrain's bare-list box
forms counted as a box). Records score with `control/identity_eval.py`.

    python evaluations/tiered-380/L/rb_identity.py --arm B <bench_identity_trt args...>
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Sequence

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
import tools.jetson.bench_identity_trt as trt                # noqa: E402
from tools.jetson.bench_identity_llama import PROMPT as JSON_PROMPT  # noqa: E402

LIST_PROMPT = ("Output the bounding box of the {target} as [x1, y1, x2, y2]. "
               "If it is not visible, output []. Output the list only.")


def _alts(step):
    return step.get("top_logprobs") or [{"token": step["token"], "logprob": step["logprob"]}]


def _mass(alts, pred) -> float:
    return sum(math.exp(a["logprob"]) for a in alts if pred(a["token"].strip()))


def _opens_box(t: str) -> bool:
    """A list that holds a box: `[[`, `[(`, `[{`, or `[` and a digit."""
    return len(t) > 1 and t[0] == "[" and (t[1] in "[({" or t[1].isdigit())


def _box_body(t: str) -> bool:
    """What follows a bare `[` when a box comes next."""
    return bool(t) and (t[0].isdigit() or t[0] in "[({")


def p_list(content: Sequence[dict]) -> tuple:
    """`(score, parsed, decided_at)` like `p_localised`. At the first token
    that starts with `[` (stripped): box-opening forms F, empty `[]` forms E,
    bare `[` O; P = (F + O * f) / (F + E + O), f read from the next
    non-whitespace step when `[` was generated (box body vs `]`), else
    F / (F + E). A reply that never opens a list scores 0, unparsed."""
    for i, step in enumerate(content):
        tok = step["token"].strip()
        if not tok.startswith("["):
            continue
        alts = _alts(step)
        found = _mass(alts, _opens_box)
        empty = _mass(alts, lambda t: t.startswith("[]"))
        bare = _mass(alts, lambda t: t == "[")
        decided = i
        if tok == "[":
            j = i + 1
            while j < len(content) and content[j]["token"].strip() == "":
                j += 1
            if j < len(content):
                nxt = _alts(content[j])
                fb = _mass(nxt, _box_body)
                fe = _mass(nxt, lambda t: t.startswith("]"))
                f = fb / (fb + fe) if (fb + fe) > 0 else 0.0
                decided = j
            else:
                f = 0.0
        else:
            f = found / (found + empty) if (found + empty) > 0 else 0.0
        denom = found + empty + bare
        return ((found + bare * f) / denom if denom > 0 else 0.0), True, decided
    return 0.0, False, None


def decided(content: Sequence[dict]) -> bool:
    """Stop once `p_list` has its answer: a non-bare list token, or a bare
    `[` and the token after it."""
    for i, step in enumerate(content):
        tok = step["token"].strip()
        if tok.startswith("["):
            if tok != "[":
                return True
            return any(c["token"].strip() for c in content[i + 1:])
    return len(content) >= trt.MAX_TOKENS


def main(argv):
    arm = argv[argv.index("--arm") + 1]
    rest = argv[:argv.index("--arm")] + argv[argv.index("--arm") + 2:]
    trt.PROMPT = LIST_PROMPT if arm == "B" else JSON_PROMPT
    trt.p_localised = p_list
    trt.decision_reached = decided
    return trt.main(rest)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
