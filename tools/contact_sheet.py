"""Contact sheets for adjudicating a walk's `labels.json`, by eye.

Kept in the repo on purpose. This had been written ad hoc at least once
before (the 2026-09-08 basket walks were adjudicated "by eye from
1280x720 contact sheets") and then lost -- the same fate
`control/perception_eval.py` records for itself. Rewriting it a third
time is how a corpus ends up with two different labelling methods and no
note saying so.

The method it serves, from the existing labels.json files and
PLAN-onboard-perception.md 4.10:

  * every frame is looked at, not only the ones a model disputes;
  * adjudicate in SPANS -- visibility is contiguous in a walk, and 4670
    records that a contiguous run of false positives is almost always a
    labelling error rather than a model error;
  * boundary frames (where a span starts or ends) get opened full-size,
    because that is where a sheet-sized tile is least trustworthy.

Usage:
    python -m tools.contact_sheet <walk> [--cols 4] [--rows 3] [--tile 384]
    python -m tools.contact_sheet <walk> --frames 12,13,14   # full-size crops
"""
from __future__ import annotations

import argparse
import os
import sys

from PIL import Image, ImageDraw

LABEL_H = 18


def _frames(walk_dir: str) -> list[str]:
    return sorted(f for f in os.listdir(walk_dir) if f.endswith(".jpg"))


def build_sheets(walk_dir: str, out_dir: str, cols: int, rows: int,
                 tile_w: int) -> list[str]:
    names = _frames(walk_dir)
    tile_h = round(tile_w * 9 / 16)
    per = cols * rows
    os.makedirs(out_dir, exist_ok=True)
    made = []
    for start in range(0, len(names), per):
        chunk = names[start:start + per]
        sheet = Image.new("RGB",
                          (cols * tile_w, rows * (tile_h + LABEL_H)),
                          (24, 24, 24))
        draw = ImageDraw.Draw(sheet)
        for i, name in enumerate(chunk):
            c, r = i % cols, i // cols
            x, y = c * tile_w, r * (tile_h + LABEL_H)
            with Image.open(os.path.join(walk_dir, name)) as im:
                sheet.paste(im.convert("RGB").resize((tile_w, tile_h),
                                                     Image.LANCZOS), (x, y))
            # The index is the whole point: a tile nobody can name is a
            # tile nobody can label.
            idx = int(name.split("-")[1].split(".")[0])
            draw.text((x + 4, y + tile_h + 3), f"{idx:04d}", fill=(230, 230, 90))
        path = os.path.join(out_dir, f"sheet-{start // per:03d}.jpg")
        sheet.save(path, quality=88)
        made.append(path)
    return made


def build_closeups(walk_dir: str, out_dir: str, idxs: list[int]) -> list[str]:
    """One frame per file at native resolution -- for boundary frames."""
    os.makedirs(out_dir, exist_ok=True)
    made = []
    for i in idxs:
        src = os.path.join(walk_dir, f"frame-{i:04d}.jpg")
        if not os.path.exists(src):
            print(f"missing {src}", file=sys.stderr)
            continue
        dst = os.path.join(out_dir, f"closeup-{i:04d}.jpg")
        with Image.open(src) as im:
            im.convert("RGB").save(dst, quality=95)
        made.append(dst)
    return made


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("walk")
    ap.add_argument("--recordings", default="recordings")
    ap.add_argument("--out", default=None)
    ap.add_argument("--cols", type=int, default=4)
    ap.add_argument("--rows", type=int, default=3)
    ap.add_argument("--tile", type=int, default=384)
    ap.add_argument("--frames", default=None,
                    help="comma-separated indices -> native-resolution crops")
    args = ap.parse_args()

    walk_dir = os.path.join(args.recordings, args.walk)
    if not os.path.isdir(walk_dir):
        print(f"no such walk: {walk_dir}", file=sys.stderr)
        return 2
    out = args.out or os.path.join("/tmp/sheets", args.walk)

    if args.frames:
        idxs = [int(x) for x in args.frames.split(",") if x.strip()]
        made = build_closeups(walk_dir, out, idxs)
    else:
        made = build_sheets(walk_dir, out, args.cols, args.rows, args.tile)
    for p in made:
        print(p)
    print(f"{len(made)} file(s), {len(_frames(walk_dir))} frames", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
