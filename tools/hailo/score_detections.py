"""Score a detections JSON against the adjudicated corpus labels.

The INT8 question acts on the DETECTOR, which is the only part of the
split that gets quantized: in P10's arrangement CLIP runs on the Pi's CPU
in fp32 either way, so comparing detector proposals fp32 vs INT8 is
comparing the thing quantization actually touches.

Recall is reported at matched false-positive budgets, the same discipline
`control/perception_eval.py` uses and for the same reason -- a detector
run at a low enough confidence beats anything on recall while inventing
targets.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

# The vocabulary index each walk's target occupies, from
# tools/hailo/export_yoloworld_onnx.py's DEFAULT_VOCAB.
TARGET_CLASS = {"woven-laundry-basket": 0, "blue-bottle": 1,
                "blue-shoes": 2, "red-backpack": 3}


def target_index(walk: str) -> int:
    for prefix, idx in TARGET_CLASS.items():
        if walk.startswith(prefix):
            return idx
    raise KeyError(walk)


def frame_scores(det_path: Path, recordings: Path) -> list:
    """(visible, best score for the walk's own target class) per frame."""
    dets = json.loads(det_path.read_text())
    labels = {}
    out = []
    for key, frame_dets in dets.items():
        walk, name = key.split("/", 1)
        if walk not in labels:
            labels[walk] = json.loads(
                (recordings / walk / "labels.json").read_text()
            )["target_visible_labels"]
        cls = target_index(walk)
        best = max((d["score"] for d in frame_dets if d["cls"] == cls),
                   default=0.0)
        out.append((bool(labels[walk][name]), best, key))
    return out


def recall_at_fp(rows, budget: int) -> dict:
    negatives = sorted((s for vis, s, _ in rows if not vis), reverse=True)
    gate = float("-inf") if len(negatives) <= budget else \
        negatives[budget] + 1e-12
    tp = sum(1 for vis, s, _ in rows if vis and s > gate)
    fp = sum(1 for vis, s, _ in rows if not vis and s > gate)
    vis = sum(1 for vis, _, _ in rows if vis)
    return {"budget": budget, "gate": gate, "tp": tp, "fp": fp,
            "visible": vis, "recall": tp / vis if vis else 0.0}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("detections", nargs="+", type=Path)
    ap.add_argument("--recordings", type=Path, default=Path("recordings"))
    ap.add_argument("--budgets", default="0,3,16")
    args = ap.parse_args(argv)

    budgets = [int(b) for b in args.budgets.split(",")]
    print(f"  {'detections':<22}{'budget':>8}{'TP':>6}{'FP':>5}{'recall':>9}")
    print("  " + "-" * 52)
    for path in args.detections:
        rows = frame_scores(path, args.recordings)
        for b in budgets:
            s = recall_at_fp(rows, b)
            print(f"  {path.stem:<22}{b:>8}{s['tp']:>6}{s['fp']:>5}"
                  f"{s['recall']:>8.0%}")
        print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
