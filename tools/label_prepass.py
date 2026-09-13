"""Two independent opinions per frame, to bound what has to be eyeballed.

The corpus's labelling method (see any `labels.json`) auto-accepts frames
where two independent models agree and sends the rest to a human eye. The
earlier walks used a cloud VLM as the second opinion. This uses **OWLv2**
instead, for two reasons:

  * it is free and local, where the VLM is a paid call per frame; and
  * it fails differently. Walk `blue-bottle-20260907-185007` is the
    cautionary case -- the VLM called the bottle visible on 44 frames and
    34 of them were a teal storage bin, described in confident, specific,
    entirely invented detail. A second VLM shares that failure mode. P7
    measured OWLv2 at 82% recall at 3 false positives against the shipped
    pipeline's 58%, on the same corpus.

This writes signals, NOT labels. Nothing here decides `labels.json`: the
output is an ordering over frames so that adjudication starts where the
two models disagree, and a record of what each one said so a later
disagreement with the finished labels can be read as a hypothesis about
the labels (PLAN-onboard-perception.md 4670) rather than only as the
model's mistake.

    python -m tools.label_prepass <walk> [--target "..."] [--dtype fp16]
"""
from __future__ import annotations

import argparse
import base64
import json
import os
import sys
import time

DEFAULT_OUT = "prepass.json"


def _frame(path: str) -> dict:
    with open(path, "rb") as fh:
        return {"image_base64": base64.b64encode(fh.read()).decode("ascii"),
                "media_type": "image/jpeg"}


def run(walk_dir: str, target: str, dtype: str | None,
        out_path: str, owl_threshold: float) -> dict:
    from brain.perceive import pipeline_for
    from brain.perceive_lab import Owlv2

    names = sorted(f for f in os.listdir(walk_dir) if f.endswith(".jpg"))
    print(f"{len(names)} frames, target {target!r}", file=sys.stderr)

    pipe = pipeline_for(target)
    owl = Owlv2(threshold=owl_threshold, dtype=dtype)

    rows = {}
    t0 = time.time()
    for i, name in enumerate(names):
        path = os.path.join(walk_dir, name)
        frame = _frame(path)
        raw = base64.b64decode(frame["image_base64"])

        p = pipe.perceive(frame)
        dets = owl.detect_text(raw, target)
        best = max((d.confidence for d in dets), default=0.0)

        rows[name] = {
            "clip_status": p.status,
            "clip_p": (round(p.best.probability, 4)
                       if p.best is not None else None),
            "owl_max": round(best, 4),
            "owl_n": len(dets),
        }
        if i % 10 == 0 or i == len(names) - 1:
            el = time.time() - t0
            print(f"  {i + 1}/{len(names)}  {el:6.1f}s "
                  f"({el / (i + 1):.2f}s/frame)", file=sys.stderr, flush=True)
            _write(out_path, walk_dir, target, dtype, owl_threshold,
                   rows, len(names))

    _write(out_path, walk_dir, target, dtype, owl_threshold, rows, len(names))
    print(f"wrote {out_path}", file=sys.stderr)
    return _doc(walk_dir, target, dtype, owl_threshold, rows, len(names))


def _doc(walk_dir, target, dtype, owl_threshold, rows, total) -> dict:
    return {"walk": os.path.basename(walk_dir.rstrip("/")),
            "target": target,
            "owl_threshold": owl_threshold,
            "owl_dtype": dtype or "fp32",
            "frames": total,
            "scored": len(rows),
            "signals": rows,
            "note": "signals only -- not labels, and not ground truth"}


def _write(out_path, walk_dir, target, dtype, owl_threshold, rows, total):
    """Written every 10 frames, not only at the end: the first run of this
    tool buffered everything and a kill at frame ~170 cost all of it."""
    with open(out_path, "w") as fh:
        json.dump(_doc(walk_dir, target, dtype, owl_threshold, rows, total),
                  fh, indent=1)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("walk")
    ap.add_argument("--recordings", default="recordings")
    ap.add_argument("--target", default=None,
                    help="defaults to the walk's own meta.json target_object")
    # P7 measured fp16 as 1.8x faster at identical true positives -- ON AN
    # A10G. On this laptop's CPU it is **22x SLOWER** (48.35 vs 2.19 s/frame,
    # measured 2026-09-13): x86/ARM CPUs have no native fp16 compute path, so
    # torch emulates it per-op. The first run of this tool inherited fp16 from
    # P7 and a 180-frame walk never finished. Default by device, and say so.
    ap.add_argument("--dtype", default=None,
                    help="default: fp32 on CPU (fp16 is ~22x slower there), "
                         "fp16 on CUDA where P7 measured it 1.8x faster")
    ap.add_argument("--owl-threshold", type=float, default=0.02)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    walk_dir = os.path.join(args.recordings, args.walk)
    if not os.path.isdir(walk_dir):
        print(f"no such walk: {walk_dir}", file=sys.stderr)
        return 2

    target = args.target
    if target is None:
        meta_path = os.path.join(walk_dir, "meta.json")
        with open(meta_path) as fh:
            target = json.load(fh)["target_object"]

    dtype = args.dtype
    if dtype is None:
        try:
            import torch
            dtype = "fp16" if torch.cuda.is_available() else "fp32"
        except ImportError:
            dtype = "fp32"
    print(f"dtype {dtype}", file=sys.stderr)

    out = args.out or os.path.join(walk_dir, DEFAULT_OUT)
    run(walk_dir, target, dtype, out, args.owl_threshold)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
