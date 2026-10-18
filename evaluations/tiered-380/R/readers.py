"""
evaluations/tiered-380/R/readers.py

3.80.R: M's readers (`p_box`, `decided`, `parse_any_box`), widened for the
formats the R models answer in, each pinned in `test_readers.py`:
* `p_native`: the score of a native grounding reply. A synthetic one-step
  reply (`native_server.py`'s Florence-2 / Photon answers) scores 1 or 0.
  Otherwise the first non-blank generated token decides: the share of its
  top-20 alternatives' probability that opens a box or point
  (`opens_native`), over all of them.
  LocateAnything is read at the token after `<box>` instead: coordinate
  tokens over coordinates + `None` / `<null>`.
* `parse_any_box`: PaliGemma's `<locY1><locX1><locY2><locX2>` (0-1023, read
  as x1, y1, x2, y2 on 0-1000) first, then M's reader, then a point -- `<box><x><y></box>`
  (LocateAnything, 0-1000) or Molmo2's `coords="i x y"` (0-1000) -- as a
  1-unit box at the point, so the box centre is the point.
"""

from __future__ import annotations

import math
import re
import sys
from pathlib import Path
from typing import Optional, Sequence

import importlib.util

# M's module is also called `readers`: load it under its own name.
_spec = importlib.util.spec_from_file_location(
    "m_readers", Path(__file__).resolve().parent.parent / "M" / "readers.py")
mr = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(mr)

p_box = mr.p_box
decided = mr.decided

_NUM = r"(-?\d+(?:\.\d+)?)"
_LA_POINT = re.compile(r"<box>\s*<(\d+)>\s*<(\d+)>\s*</box>")
_LOC = re.compile(r"<loc(\d{4})><loc(\d{4})><loc(\d{4})><loc(\d{4})>")
_MOLMO = re.compile(r"coords=\"\s*\d+\s+(\d{1,4})\s+(\d{1,4})")


def opens_native(tok: str) -> bool:
    t = tok.strip()
    return ((t.startswith("[") and not t.startswith("[]")) or t.startswith("<box")
            or t.startswith("<|box_start") or t.startswith("<point") or t.startswith("<loc"))


def _first(content: Sequence[dict]):
    for i, step in enumerate(content):
        if step["token"].strip():
            return i, step
    return None, None


_COORD = re.compile(r"^<\d+>$")


def _after_box(content: Sequence[dict]):
    """LocateAnything (smoke, 2026-10-18): `<ref>label</ref><box>` always
    comes first; the token after `<box>` decides -- a coordinate `<n>` (a box
    or point follows) or `None` / `<null>` (nothing there)."""
    for i, step in enumerate(content[:-1]):
        if step["token"] == "<box>":
            return i + 1, content[i + 1]
    return None, None


def _la(content: Sequence[dict]) -> bool:
    return bool(content) and content[0]["token"] == "<ref>"


def p_native(content: Sequence[dict]) -> tuple:
    """`(score, parsed, decided_at)`, the bench's reader contract."""
    if _la(content):
        i, step = _after_box(content)
        if step is None:
            return 0.0, False, None
        alts = step.get("top_logprobs") or [{"token": step["token"], "logprob": step["logprob"]}]
        found = sum(math.exp(a["logprob"]) for a in alts if _COORD.match(a["token"].strip()))
        empty = sum(math.exp(a["logprob"]) for a in alts if a["token"].strip() in ("None", "<null>"))
        return (found / (found + empty) if found + empty > 0 else 0.0), True, i
    i, step = _first(content)
    if step is None:
        return 0.0, False, None
    alts = step.get("top_logprobs") or [{"token": step["token"], "logprob": step["logprob"]}]
    total = sum(math.exp(a["logprob"]) for a in alts)
    found = sum(math.exp(a["logprob"]) for a in alts if opens_native(a["token"]))
    return (found / total if total > 0 else 0.0), True, i


def decided_native(content: Sequence[dict]) -> bool:
    if _la(content):
        return _after_box(content)[1] is not None
    return _first(content)[1] is not None


def parse_any_box(text: str) -> Optional[list]:
    m = _LOC.search(text)
    if m:
        # PaliGemma: y1, x1, y2, x2 on 0-1023, returned on 0-1000.
        y1, x1, y2, x2 = (int(v) * 1000 / 1024 for v in m.groups())
        return [x1, y1, x2, y2] if x2 > x1 and y2 > y1 else None
    box = mr.parse_any_box(text)
    if box is not None:
        return box
    m = _LA_POINT.search(text) or _MOLMO.search(text)
    if m:
        x, y = float(m.group(1)), float(m.group(2))
        return [x, y, x + 1, y + 1]
    return None
