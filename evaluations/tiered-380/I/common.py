"""3.80.I shared helpers: the frame sets, mirroring, thirds, scoring."""

import base64
import io
import json
from pathlib import Path

from PIL import Image, ImageOps

HERE = Path(__file__).parent
RECORDINGS = Path("/Users/anshugaind/vision-picar/recordings")
SWAP = {"LEFT": "RIGHT", "RIGHT": "LEFT", "FORWARD": "FORWARD", None: None}


def sets() -> dict:
    return json.loads((HERE / "sets.json").read_text())


def labels_s1() -> list:
    return json.loads((HERE / "labels-s1.json").read_text())["rows"]


def items():
    """(set, key, path, target, truth) for S1, S2 and S3, in a fixed order."""
    s = sets()
    for r in labels_s1():
        yield "S1", f"{r['walk']}/{r['frame']}", RECORDINGS / r["walk"] / r["frame"], r["target"], r["truth"]
    for r in s["S2"]:
        yield "S2", f"{r['walk']}/{r['frame']}", RECORDINGS / r["walk"] / r["frame"], r["target"], None
    for r in s["S3"]:
        yield "S3", r["image"], HERE / r["image"], r["target"], r["truth"]


def mirrored(jpeg: bytes) -> bytes:
    out = io.BytesIO()
    ImageOps.mirror(Image.open(io.BytesIO(jpeg)).convert("RGB")).save(out, "JPEG", quality=92)
    return out.getvalue()


def b64(jpeg: bytes) -> str:
    return base64.b64encode(jpeg).decode()


def third(frac):
    if frac is None:
        return None
    return "LEFT" if frac < 1 / 3 else "RIGHT" if frac > 2 / 3 else "FORWARD"
