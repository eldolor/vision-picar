"""
evaluations/tiered-380/F/f4b_mirror.py

3.80.F diagnostic, after F4 failed (the plan's "Measured 2026-10-16"): does
Qwen VE's left/right probability carry the scene under its forward bias?

Real labelled frames where the shipped perception (YOLOE-11s-seg -> CLIP
RN50, on this CPU) detects the target off-centre (|bearing| > 5 deg), up to
MAX_FRAMES spread over the walks. Each is asked the direction question as is
and mirrored left-right. d = log(pL/pR) - log(pL'/pR') cancels any fixed
bias. Signal if sign(d) matches the side of perception's bearing on >= 75%
of >= 30 frames (chance 50%). Free (local only).

    .venv/bin/python evaluations/tiered-380/F/f4b_mirror.py [service-url]
"""

import base64
import io
import json
import math
import sys
import time
from pathlib import Path

import httpx
from PIL import Image, ImageOps

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from brain.perceive import DETECTED, pipeline_for  # noqa: E402
from control.perception_eval import frame_dict  # noqa: E402

HERE = Path(__file__).parent
RECORDINGS = Path("/Users/anshugaind/vision-picar/recordings")
MAX_FRAMES, PER_WALK, MIN_BEARING = 40, 6, 5.0
EPS = 1e-4


def mirrored(jpeg: bytes) -> bytes:
    out = io.BytesIO()
    ImageOps.mirror(Image.open(io.BytesIO(jpeg)).convert("RGB")).save(out, "JPEG", quality=92)
    return out.getvalue()


def ask(c, url, jpeg, target):
    body = {"image_b64": base64.b64encode(jpeg).decode(), "target": target, "deadline_s": 3.0}
    return c.post(f"{url}/direction", json=body).json()


def lr(got):
    p = got.get("probs") or {}
    return math.log((p.get("left", 0) + EPS) / (p.get("right", 0) + EPS))


def candidates():
    """(walk, frame path, target, bearing) for perception's off-centre detections."""
    walks = sorted(p for p in RECORDINGS.iterdir() if (p / "labels.json").is_file())
    found = []
    for walk in walks:
        labels = json.loads((walk / "labels.json").read_text())
        target = labels["description"]
        visible = [f for f, v in sorted(labels["target_visible_labels"].items()) if v]
        if not visible:
            continue
        pipe = pipeline_for(target)
        took = 0
        step = max(1, len(visible) // (PER_WALK * 3))
        for f in visible[::step]:
            path = walk / f
            if not path.is_file():
                continue
            perc = pipe.perceive(frame_dict(path))
            b = perc.bearing_deg
            if perc.status == DETECTED and b is not None and abs(b) > MIN_BEARING:
                found.append((walk.name, path, target, float(b)))
                took += 1
                if took >= PER_WALK:
                    break
        print(f"{walk.name}: {took}", flush=True)
        if len(found) >= MAX_FRAMES:
            break
    return found[:MAX_FRAMES]


def main(url="http://127.0.0.1:18005"):
    rows = []
    with httpx.Client(timeout=15) as c:
        for walk, path, target, bearing in candidates():
            jpeg = path.read_bytes()
            a, b = ask(c, url, jpeg, target), ask(c, url, mirrored(jpeg), target)
            d = lr(a) - lr(b)
            side = "left" if bearing < 0 else "right"
            rows.append({"walk": walk, "frame": path.name, "target": target,
                         "bearing_deg": round(bearing, 1), "side": side,
                         "probs": a.get("probs"), "probs_mirror": b.get("probs"),
                         "direction": a.get("direction"), "d": round(d, 3),
                         "d_side": "left" if d > 0 else "right" if d < 0 else None})
            print(json.dumps(rows[-1]), flush=True)
    n = len(rows)
    hit = sum(r["d_side"] == r["side"] for r in rows)
    raw = sum(r["direction"] == r["side"] for r in rows)
    out = {"frames": n, "walks": len({r["walk"] for r in rows}),
           "mirror_side_hits": hit, "mirror_side_rate": round(hit / n, 3) if n else None,
           "argmax_side_hits": raw,
           "argmax_counts": {w: sum(r["direction"] == w for r in rows)
                             for w in ("left", "right", "forward")},
           "signal": n >= 30 and hit / n >= 0.75, "rows": rows}
    (HERE / "f4b.json").write_text(json.dumps(out, indent=1) + "\n")
    print(json.dumps({k: v for k, v in out.items() if k != "rows"}))


if __name__ == "__main__":
    t = time.time()
    main(*sys.argv[1:])
    print(f"{time.time() - t:.0f} s")
