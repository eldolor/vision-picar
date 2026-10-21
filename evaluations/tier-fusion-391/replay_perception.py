"""3.91 B, criterion 3: perception replayed over every frame of a recorded
walk, with no time pressure, to see where it goes blind.

Runs YOLOE + CLIP exactly as the brain does (`pipeline_for(target)`), one
frame at a time, and writes one row per frame: the live decision's status
when the frame was perceived live, the replay's status, the best
candidate's probability and bearing, and its box as a share of the frame
(width, height, area) -- how much of the picture the target filled.

    python evaluations/tier-fusion-391/replay_perception.py <walk dir> <out.json>

No cloud call.
"""

import base64
import json
import os
import sys
import time

from PIL import Image

from brain.perceive import pipeline_for


def main(walk_dir, out_path):
    meta = json.load(open(os.path.join(walk_dir, "meta.json")))
    target = meta["target_object"]
    rows = [json.loads(line) for line in open(os.path.join(walk_dir, "walk.jsonl"))]
    pipeline = pipeline_for(target)
    out = []
    for row in rows:
        path = os.path.join(walk_dir, row["file"])
        raw = open(path, "rb").read()
        width, height = Image.open(path).size
        started = time.perf_counter()
        p = pipeline.perceive({"image_base64": base64.b64encode(raw).decode(),
                               "media_type": row.get("media_type", "image/jpeg")})
        ms = (time.perf_counter() - started) * 1000
        best = p.best or (max(p.candidates, key=lambda c: c.probability)
                          if p.candidates else None)
        box = best.detection.box if best else None
        live = ((row.get("navigate") or {}).get("_missionStatus") or {}).get("perception") or {}
        out.append({
            "seq": row["seq"], "teleop_seq": row.get("teleop_seq"),
            "captured_at": row.get("captured_at"),
            "perceived_live": row.get("perceived"), "live_status": live.get("status"),
            "status": p.status, "reason": p.reason, "ms": round(ms, 1),
            "probability": round(best.probability, 3) if best else None,
            "bearing_deg": round(best.bearing_deg, 1) if best and best.bearing_deg is not None else None,
            "box_w": round((box.x2 - box.x1) / width, 3) if box else None,
            "box_h": round((box.y2 - box.y1) / height, 3) if box else None,
            "box_area": round(box.area / (width * height), 3) if box else None,
        })
    json.dump({"walk": os.path.basename(os.path.normpath(walk_dir)), "target": target,
               "frames": out}, open(out_path, "w"), indent=1)
    detected = sum(r["status"] == "detected" for r in out)
    print(f"{detected}/{len(out)} detected; median ms "
          f"{sorted(r['ms'] for r in out)[len(out) // 2]}")


if __name__ == "__main__":
    main(*sys.argv[1:3])
