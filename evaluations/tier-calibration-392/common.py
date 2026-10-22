"""3.92 shared: the labelled frames, in a fixed order."""

import base64
import json
import os
from pathlib import Path

HERE = Path(__file__).parent
RECORDINGS = Path(os.environ.get("PICAR_RECORDINGS", "/Users/anshugaind/vision-picar/recordings"))


def items():
    """(key, path, target, visible) for every frame of every labelled walk."""
    for labels in sorted(RECORDINGS.glob("*/labels.json")):
        d = json.loads(labels.read_text())
        for frame, visible in sorted(d["target_visible_labels"].items()):
            yield f"{d['walk']}/{frame}", labels.parent / frame, d["description"], visible


def done(out: Path) -> set:
    if not out.exists():
        return set()
    return {json.loads(l)["key"] for l in out.read_text().splitlines() if l.strip()}


def b64(jpeg: bytes) -> str:
    return base64.b64encode(jpeg).decode()
