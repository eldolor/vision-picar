"""
evaluations/tiered-380/M/readers.py

3.80.M: the identity score and the box, read wider than Qwen's `bbox_2d`,
because InternVL, Youtu-VL and MiniCPM-V may answer in their own formats.
* `p_box`: 3.80.L's `p_list` with a box tag (`<box>`, `<|box_start|>`)
  counted as a box opening, like `[[`.
* `decided`: L's stopping rule, also true at a box tag.
* `parse_any_box`: `bbox_2d`, else the first four numbers of a `[...]` list,
  else the four numbers inside `<box>...</box>`. Raw numbers, any scale.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path
from typing import Optional, Sequence

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "L"))
import rb_identity as rb                                    # noqa: E402
from service.local_vlm.driver import parse_box              # noqa: E402

_NUM = r"(-?\d+(?:\.\d+)?)"
_LIST = re.compile(r"\[\s*\[?\s*[(\"']?\s*" + r"\s*,\s*".join([_NUM] * 4))
_TAG = re.compile(r"<\|?box(?:_start)?\|?>(.*?)(?:</box>|<\|box_end\|>|$)", re.S)


def is_tag(t: str) -> bool:
    t = t.strip()
    return t.startswith("<box") or t.startswith("<|box_start")


def _opens(t: str) -> bool:
    return rb._opens_box(t) or is_tag(t)


def p_box(content: Sequence[dict]) -> tuple:
    """`(score, parsed, decided_at)`. As `rb.p_list`, the first token that
    starts with `[` or is a box tag decides; a tag counts with the box
    openings (F), and when the tag itself was generated f = F / (F + E)."""
    for i, step in enumerate(content):
        tok = step["token"].strip()
        if not (tok.startswith("[") or is_tag(tok)):
            continue
        if not is_tag(tok):
            # No tag generated: L's reader, the tag forms added to F.
            # p_list stops at this same `[` token (no tag came before it).
            saved = rb._opens_box
            rb._opens_box = lambda t: saved(t) or is_tag(t)
            try:
                return rb.p_list(content)
            finally:
                rb._opens_box = saved
        alts = rb._alts(step)
        found = rb._mass(alts, _opens)
        empty = rb._mass(alts, lambda t: t.startswith("[]"))
        bare = rb._mass(alts, lambda t: t == "[")
        denom = found + empty + bare
        f = found / (found + empty) if (found + empty) > 0 else 0.0
        return ((found + bare * f) / denom if denom > 0 else 0.0), True, i
    return 0.0, False, None


def decided(content: Sequence[dict]) -> bool:
    for step in content:
        if is_tag(step["token"]):
            return True
        if step["token"].strip().startswith("["):
            break
    return rb.decided(content)


def parse_any_box(text: str) -> Optional[list]:
    box = parse_box(text)
    if box is None:
        m = _LIST.search(text)
        if m:
            box = [float(v) for v in m.groups()]
        else:
            # The four numbers inside the tag, whatever separates them
            # (`10 20 50 60`, `<x_129><y_190>...`).
            m = _TAG.search(text)
            nums = re.findall(_NUM, m.group(1)) if m else []
            if len(nums) >= 4:
                box = [float(v) for v in nums[:4]]
    if box is not None and box[2] > box[0] and box[3] > box[1]:
        return box
    return None
