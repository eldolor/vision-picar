"""
evaluations/tier-calibration-392/run_perception.py

3.92.A: the shipped pipeline (`pipeline_for`: YOLOE-11s-seg crops -> CLIP
RN50, P >= 0.8 DETECTED) on every labelled frame, on this CPU (one device,
P24). Resumable. Free.

    .venv/bin/python evaluations/tier-calibration-392/run_perception.py
"""

import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from brain.perceive import pipeline_for       # noqa: E402
from common import HERE, b64, done, items      # noqa: E402


def main():
    from PIL import Image
    out = HERE / "perception.jsonl"
    seen = done(out)
    pipes = {}
    with out.open("a") as f:
        for key, path, target, visible in items():
            if key in seen:
                continue
            pipe = pipes.setdefault(target, pipeline_for(target))
            w, h = Image.open(path).size
            t0 = time.perf_counter()
            perc = pipe.perceive({"image_base64": b64(path.read_bytes()),
                                  "media_type": "image/jpeg", "image_width": w})
            best = perc.best
            box = best.detection.box if best else None
            row = {"key": key, "target": target, "visible": visible, "status": perc.status,
                   "p": round(best.probability, 4) if best else None,
                   "box": [box.x1, box.y1, box.x2, box.y2] if box else None,
                   "w": w, "h": h, "ms": round((time.perf_counter() - t0) * 1000)}
            f.write(json.dumps(row) + "\n"); f.flush()
            print(key, visible, row["status"], row["p"], row["ms"], flush=True)


if __name__ == "__main__":
    main()
