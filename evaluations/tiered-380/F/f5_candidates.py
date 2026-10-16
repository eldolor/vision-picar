"""
evaluations/tiered-380/F/f5_candidates.py

3.80.F criterion F5: of the labelled real frames whose perception is not a
detection and whose best crop's CLIP P is in [0.5, 0.8), how many have the
target labelled visible? Read from P24's per-frame records of the shipped
perception (YOLOE-11s-seg crops -> CLIP RN50, no floor mask; A10G), so no
model runs here.

    .venv/bin/python evaluations/tiered-380/F/f5_candidates.py [records.json]
"""

import json
import sys
from pathlib import Path

RECORDS = "/Users/anshugaind/vision-picar/evaluations/gpu-yoloe/yoloe-11s-seg_none.json"
LOW, GATE, BAR = 0.5, 0.8, 0.60


def main(path=RECORDS):
    doc = json.loads(Path(path).read_text())
    rows = doc["records"]
    band = [r for r in rows if r["status"] != "detected" and r["score"] is not None
            and LOW <= r["score"] < GATE]
    visible = sum(r["visible"] for r in band)
    missed = [r for r in rows if r["visible"] and r["status"] != "detected"]
    out = {"records": path, "config": doc["config"], "frames": len(rows),
           "walks": len({r["walk"] for r in rows}),
           "band_frames": len(band), "band_visible": visible,
           "band_precision": round(visible / len(band), 4) if band else None,
           "bar": BAR, "met": bool(band) and visible / len(band) >= BAR,
           "visible_not_detected": len(missed),
           "visible_not_detected_in_band": sum(1 for r in missed if r in band)}
    print(json.dumps(out, indent=1))
    return out


if __name__ == "__main__":
    out = main(*sys.argv[1:])
    Path(__file__).with_name("f5.json").write_text(json.dumps(out, indent=1) + "\n")
