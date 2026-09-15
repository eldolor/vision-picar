"""Does quantization preserve the DETECTOR'S PROPOSALS?

The right question for a crop source. YOLO11s has no class for "woven
laundry basket" -- COCO does not contain one -- so scoring its own class
confidence against the corpus labels is meaningless. Its job is to
propose regions that CLIP then ranks, and in P10's split CLIP runs on the
Pi's CPU in fp32 either way. So what quantization can break is the
proposals, and that is what this measures.

Reported against the fp32 run of the same model: of the boxes fp32 found
at a given confidence, how many does the quantized model also find at
IoU >= 0.5. Run on YOLO-World too, so the two models are comparable on
one metric rather than two.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path


def _iou(a, b) -> float:
    ax, ay, aw, ah = a; bx, by, bw, bh = b
    ax1, ay1, ax2, ay2 = ax-aw/2, ay-ah/2, ax+aw/2, ay+ah/2
    bx1, by1, bx2, by2 = bx-bw/2, by-bh/2, bx+bw/2, by+bh/2
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2-ix1), max(0.0, iy2-iy1)
    inter = iw*ih
    union = aw*ah + bw*bh - inter
    return inter/union if union > 0 else 0.0


def agreement(ref: dict, got: dict, conf: float, iou: float = 0.5) -> dict:
    frames = sorted(set(ref) & set(got))
    matched = total = extra = 0
    for f in frames:
        r = [d for d in ref[f] if d["score"] >= conf]
        g = got[f]
        total += len(r)
        used = set()
        for d in r:
            best, bi = 0.0, None
            for i, e in enumerate(g):
                if i in used or e["cls"] != d["cls"]:
                    continue
                v = _iou(d["box"], e["box"])
                if v > best:
                    best, bi = v, i
            if best >= iou:
                matched += 1
                used.add(bi)
        extra += len(g) - len(used)
    return {"frames": len(frames), "fp32_boxes": total, "matched": matched,
            "recall": matched/total if total else 0.0, "unmatched_q": extra}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("reference", type=Path)
    ap.add_argument("quantized", nargs="+", type=Path)
    ap.add_argument("--confs", default="0.05,0.25,0.5")
    args = ap.parse_args(argv)
    ref = json.loads(args.reference.read_text())
    print(f"  {'quantized run':<26}{'conf':>6}{'fp32 box':>10}{'matched':>9}{'recall':>8}")
    print("  " + "-"*60)
    for q in args.quantized:
        got = json.loads(q.read_text())
        for c in (float(x) for x in args.confs.split(",")):
            a = agreement(ref, got, c)
            print(f"  {q.stem:<26}{c:>6}{a['fp32_boxes']:>10}"
                  f"{a['matched']:>9}{a['recall']:>7.0%}")
        print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
