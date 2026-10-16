"""
evaluations/tiered-380/I/run_perception.py

3.80.I, YOLOE + CLIP's steer on S1 and S2 (each as is and mirrored): the
shipped pipeline (`pipeline_for`, YOLOE-11s-seg crops -> CLIP RN50, P >= 0.8
a detection) on this CPU -- the board runs the same models. A detection's
box centre -> its third; else a band candidate (P in [0.5, 0.8), 3.80.F's
steer) -> its third; else no answer. Free.

    .venv/bin/python evaluations/tiered-380/I/run_perception.py
"""

import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).parent))
from brain.perceive import ABSENT, DETECTED, pipeline_for     # noqa: E402
from common import HERE, b64, items, mirrored, third           # noqa: E402

BAND_LOW = 0.5


def answer(pipe, jpeg: bytes, width: int):
    perc = pipe.perceive({"image_base64": b64(jpeg), "media_type": "image/jpeg",
                          "image_width": width})
    best = perc.best
    p = round(best.probability, 4) if best else None
    frac = round(best.detection.box.centre_x / width, 4) if best else None
    if perc.status == DETECTED and best is not None:
        how = "detected"
    elif perc.status == ABSENT and best is not None and BAND_LOW <= best.probability < 0.8:
        how = "band"
    else:
        how = None
    return {"status": perc.status, "p": p, "frac": frac, "how": how,
            "answer": third(frac) if how else None}


def main():
    from PIL import Image
    pipes, rows = {}, []
    out = HERE / "perception.jsonl"
    with out.open("w") as f:
        for set_, key, path, target, truth in items():
            if set_ == "S3":
                continue   # never run on renders (1.12): no search answer by design
            pipe = pipes.setdefault(target, pipeline_for(target))
            jpeg = path.read_bytes()
            width = Image.open(path).width
            t0 = time.perf_counter()
            a = answer(pipe, jpeg, width)
            ms = round((time.perf_counter() - t0) * 1000)
            m = answer(pipe, mirrored(jpeg), width)
            row = {"set": set_, "key": key, "target": target, "truth": truth, "ms": ms,
                   **a, "mirror": m}
            f.write(json.dumps(row) + "\n"); f.flush()
            rows.append(row)
            print(set_, key, truth, a["how"], a["answer"], m["answer"], flush=True)


if __name__ == "__main__":
    main()
