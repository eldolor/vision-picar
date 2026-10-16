"""
evaluations/tiered-380/I/rescale_boxes.py

3.80.I: the VE run scored boxes with brain/local_vlm.py's old model_size
(the stock 28*28*16384 bound), so on 1280x720 frames the box was divided by
1288 where the engine saw 840 (max_image_tokens 512). frac_x scales exactly
by old_w / new_w (no clipping: x <= 840 < 1288). Rewrites the file once,
marking each row `box_rescaled`.

    .venv/bin/python evaluations/tiered-380/I/rescale_boxes.py qwen-VE.jsonl
"""
import json
import math
import sys
from pathlib import Path

from PIL import Image

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from brain.local_vlm import model_size                 # noqa: E402  (fixed)
from common import HERE, RECORDINGS, third             # noqa: E402


def old_size(width, height):
    P, MAX = 28, 28 * 28 * 16384
    h, w = max(P, round(height / P) * P), max(P, round(width / P) * P)
    if h * w > MAX:
        b = math.sqrt(height * width / MAX)
        h, w = max(P, math.floor(height / b / P) * P), max(P, math.floor(width / b / P) * P)
    return w, h


def main(name):
    path = HERE / name
    rows = [json.loads(l) for l in path.read_text().splitlines() if l.strip()]
    for r in rows:
        if r.get("box_rescaled") or r["set"] == "S3":
            continue
        size = Image.open(RECORDINGS / r["key"]).size
        k = old_size(*size)[0] / model_size(*size)[0]
        for a in (r, r["mirror"]):
            if a.get("box_frac_x") is not None:
                a["box_frac_x"] = round(a["box_frac_x"] * k, 4)
                a["box_answer"] = third(a["box_frac_x"])
        r["box_rescaled"] = round(k, 4)
    path.write_text("".join(json.dumps(r) + "\n" for r in rows))
    print({k: sum(1 for r in rows if r.get("box_rescaled") == k) for k in {r.get("box_rescaled") for r in rows}})


if __name__ == "__main__":
    main(*sys.argv[1:])
