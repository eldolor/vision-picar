"""
manual_perceive_walk.py

Run the on-board perception pipeline over a folder of real photographs.
Phase P1/P2 (`PLAN-onboard-perception.md` 4.10).

**This is the thing to run straight after a rig session.** Put the phone
on the wheeled rig at ~10-13cm, walk a room, drop the photos in a folder,
and point this at it. It answers the question the plan has been unable to
answer since the detector was chosen: *does a COCO detector plus CLIP
actually find household things from 10cm?*

    python -m tests.manual_perceive_walk ~/walks/backpack-rig-01 "red backpack"

Not in the automated suite, for the usual reason: it needs the optional
heavy dependencies (`pip install -r requirements-perception.txt`) and the
first run downloads model weights. It costs **no money** -- perception is
local and free. Only `--cloud` spends anything, and it is off by default.

## What a good result looks like, and what a bad one looks like

The plan's own caveats are the things to read the output against.

**4.2's standing-height caveat is the headline.** 4.3.1's `45.1 mAP` for
YOLO11s is a COCO-validation number, and COCO is web photography shot from
about 150cm. *"A chair from 150cm is a chair; a chair from 10cm is four
poles and the underside of a seat."* If the detector's hit rate collapses
here against the same scene shot from standing height, that is a real
finding about the part, not about this script -- and `--compare` exists to
measure exactly that pair.

**A high match rate is not automatically good.** CLIP returns a
similarity, not a probability, so `margin` is what to read: the target
string beating the distractors. A run where every frame matches at a
margin of 0.01 has found nothing and said so confidently.

**`unavailable` should be zero.** Anything else means frames that could
not be read, and 1.12's whole point is that those must never be counted as
"the target is not here".

## The one thing it cannot tell you

Throughput. 2.9 budgets these models against an 8L's 33ms frame; this
machine is not one. The `ms` column is here to show relative cost between
models, never to predict whether the pipeline fits on the robot.
"""

import argparse
import base64
import os
import sys
import time
from pathlib import Path

from brain.perceive import (
    ABSENT,
    DETECTED,
    UNAVAILABLE,
    DEFAULT_CLIP,
    DEFAULT_DETECTOR,
    PerceptionUnavailable,
    coco_class_for,
    pipeline_for,
)

SUFFIXES = {".jpg", ".jpeg", ".png", ".webp"}


def load_frames(path: Path) -> list:
    images = sorted(p for p in path.iterdir() if p.suffix.lower() in SUFFIXES)
    if not images:
        sys.exit(f"No images found in {path} "
                 f"(looked for {', '.join(sorted(SUFFIXES))})")
    frames = []
    for p in images:
        data = p.read_bytes()
        frame = {
            "image_base64": base64.b64encode(data).decode(),
            "media_type": f"image/{p.suffix.lstrip('.').replace('jpg', 'jpeg')}",
            "_name": p.name,
        }
        width = _width(data)
        if width:
            frame["image_width"] = width
        frames.append(frame)
    return frames


def _width(data: bytes):
    """The frame's own pixel width, which the bearing needs. Read from the
    file rather than assumed -- a bearing computed against a guessed width
    is wrong by exactly the ratio nobody checked."""
    try:
        import io

        from PIL import Image

        return float(Image.open(io.BytesIO(data)).size[0])
    except Exception:
        return None


def run(pipeline, frames, verbose=True) -> dict:
    rows, counts, margins = [], {}, []
    for frame in frames:
        started = time.perf_counter()
        result = pipeline.perceive(frame)
        ms = (time.perf_counter() - started) * 1000
        counts[result.status] = counts.get(result.status, 0) + 1
        if result.best:
            margins.append(result.best.margin)
        rows.append((frame["_name"], result, ms))
        if verbose:
            bearing = ("%+6.1f" % result.bearing_deg
                       if result.bearing_deg is not None else "     -")
            margin = ("%.3f" % result.best.margin) if result.best else "    -"
            label = result.best.detection.label if result.best else "-"
            print(f"  {frame['_name'][:28]:<28} {result.status:<12}"
                  f"{label[:14]:<15}{margin:>7}{bearing:>8}  {ms:6.0f}ms")
    return {"rows": rows, "counts": counts, "margins": margins}


def summarise(name: str, frames: list, out: dict) -> None:
    n = len(frames)
    counts = out["counts"]
    hits = counts.get(DETECTED, 0)
    margins = out["margins"]
    avg_ms = sum(r[2] for r in out["rows"]) / n if n else 0
    print(f"\n  {name}")
    print(f"    frames          {n}")
    print(f"    detected        {hits:>4}  ({hits / n:.0%})")
    print(f"    absent          {counts.get(ABSENT, 0):>4}")
    print(f"    unavailable     {counts.get(UNAVAILABLE, 0):>4}"
          f"{'   <-- frames that could not be read' if counts.get(UNAVAILABLE) else ''}")
    if margins:
        print(f"    margin          min {min(margins):.3f}  "
              f"mean {sum(margins) / len(margins):.3f}  max {max(margins):.3f}")
    print(f"    per frame       {avg_ms:.0f}ms  (this machine, not an 8L -- see 2.9)")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[3])
    ap.add_argument("directory", help="folder of photographs, in walk order")
    ap.add_argument("target", help='what to look for, e.g. "red backpack"')
    ap.add_argument("--weights", default=DEFAULT_DETECTOR,
                    help=f"detector weights (default {DEFAULT_DETECTOR})")
    ap.add_argument("--clip", default=DEFAULT_CLIP,
                    help=f"CLIP encoder (default {DEFAULT_CLIP})")
    ap.add_argument("--compare", metavar="WEIGHTS",
                    help="run a second detector over the same frames, e.g. "
                         "yolo11n.pt -- the n/s/m comparison 4.3.1 tables "
                         "for an 8L, measured on your own pixels instead")
    ap.add_argument("--margin", type=float, default=None,
                    help="match margin (default: the module's)")
    ap.add_argument("--hfov", type=float, default=66.0,
                    help="camera horizontal field of view in degrees; "
                         "Camera Module 3's standard lens is 66 (default 66)")
    ap.add_argument("--quiet", action="store_true", help="summary only")
    args = ap.parse_args()

    path = Path(os.path.expanduser(args.directory))
    if not path.is_dir():
        sys.exit(f"{path} is not a directory")

    frames = load_frames(path)
    coco = coco_class_for(args.target)
    print(f"\n{len(frames)} frames from {path}")
    print(f"target: {args.target!r}")
    print(f"crop path: {'label gate on ' + coco if coco else 'open vocabulary'}"
          f"  ({'in COCO' if coco else 'not a COCO class'} -- 4.2)")

    kwargs = {"hfov_deg": args.hfov}
    if args.margin is not None:
        kwargs["match_margin"] = args.margin

    try:
        pipeline = pipeline_for(args.target, weights=args.weights,
                                clip_model=args.clip, **kwargs)
    except PerceptionUnavailable as exc:
        sys.exit(f"\n{exc}")

    if not args.quiet:
        print(f"\n{args.weights} + CLIP {args.clip}")
        print(f"  {'frame':<28} {'status':<12}{'label':<15}"
              f"{'margin':>7}{'bearing':>8}{'time':>9}")
    out = run(pipeline, frames, verbose=not args.quiet)
    summarise(f"{args.weights} + CLIP {args.clip}", frames, out)

    if args.compare:
        try:
            other = pipeline_for(args.target, weights=args.compare,
                                 clip_model=args.clip, **kwargs)
        except PerceptionUnavailable as exc:
            sys.exit(f"\n{exc}")
        if not args.quiet:
            print(f"\n{args.compare} + CLIP {args.clip}")
        second = run(other, frames, verbose=not args.quiet)
        summarise(f"{args.compare} + CLIP {args.clip}", frames, second)
        a = out["counts"].get(DETECTED, 0)
        b = second["counts"].get(DETECTED, 0)
        print(f"\n  {args.weights} found the target in {a} frames; "
              f"{args.compare} in {b}.")
        print("  4.3.1 measures +7.6 mAP on-chip for s over n. Whether that "
              "survives 10cm is\n  what this comparison is for -- and it is a "
              "statement about these frames only.")

    print("\nReminder: this says nothing about throughput on a Hailo (2.9), "
          "and nothing\nabout navigation -- only about whether the thing is "
          "found. Record the rig height\nin the walk's own meta note "
          "(CLAUDE.md Stage 0).\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
