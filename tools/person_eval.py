"""
tools/person_eval.py -- 3.52 criteria 1-2: does a person detector find the
people in the corpus, and how often does it fire on furniture?

The corpus labels (`labels.json`) record only the target. The person truth
here was adjudicated by eye on 2026-10-09 (`docs/plans/ros-alignment/
3.52-privacy.md`, part B): every frame of the six 2026-09-08 basket walks,
then every candidate at 640 px. One person, lying on a sofa under a red
blanket, 3-6 m away. Frames where only the blanket shows are excluded from
both sides rather than guessed.

A frame FIRES when the detector returns any box. On a person frame a firing
is a hit only if a box overlaps the person, which this tool cannot know --
it writes every box so the hits can be checked by eye, and `--verified`
takes the frames confirmed that way.

    python -m tools.person_eval --recordings ../vision-picar/recordings --out person-eval.json
"""

import argparse
import json
from pathlib import Path

WALKS = (
    "woven-laundry-basket-20260908-212419",
    "woven-laundry-basket-20260908-212549",
    "woven-laundry-basket-20260908-212719",
    "woven-laundry-basket-20260908-215140",
    "woven-laundry-basket-20260908-215252",
    "woven-laundry-basket-20260908-215351",
)
# walk -> inclusive frame ranges where the person is clearly visible.
PERSON = {
    "woven-laundry-basket-20260908-212419": [(66, 78)],
    "woven-laundry-basket-20260908-212549": [(52, 62)],
    "woven-laundry-basket-20260908-215140": [(42, 52)],
    "woven-laundry-basket-20260908-215252": [(41, 48)],
}
# Only the red blanket shows: neither a person frame nor a person-free one.
BLANKET_ONLY = {
    "woven-laundry-basket-20260908-215140": [(53, 54)],
    "woven-laundry-basket-20260908-215252": [(49, 50)],
}
# The four arms criterion 1 names, none run before the criteria were written.
ARMS = (
    ("yoloe-11s-seg.pt", 0.05, 640),
    ("yoloe-11s-seg.pt", 0.05, 1280),
    ("yolo11s.pt", 0.05, 640),
    ("yolo11s.pt", 0.05, 1280),
)


def _expand(ranges: dict) -> set:
    return {(w, n) for w, rr in ranges.items() for a, b in rr for n in range(a, b + 1)}


def person_frames() -> set:
    return _expand(PERSON)


def excluded_frames() -> set:
    return _expand(BLANKET_ONLY)


def score(fired: dict, all_frames: set, verified_hits: set = None) -> dict:
    """`fired` maps (walk, frame number) -> list of boxes for every frame the
    detector returned anything on. `all_frames` is every frame looked at.
    Recall counts a person frame only if it is in `verified_hits` (when
    given) -- a box on the gaming chair beside the person is not a hit."""
    people = person_frames() & all_frames
    skip = excluded_frames()
    negatives = all_frames - people - skip
    fired_people = {k for k in people if fired.get(k)}
    hits = fired_people if verified_hits is None else fired_people & verified_hits
    false = {k for k in negatives if fired.get(k)}
    return {
        "person_frames": len(people),
        "person_fired": len(fired_people),
        "person_hits": len(hits),
        "recall": round(len(hits) / len(people), 3) if people else None,
        "negative_frames": len(negatives),
        "false_fires": len(false),
        "false_rate": round(len(false) / len(negatives), 3) if negatives else None,
    }


def _frames(recordings: Path) -> list:
    out = []
    for w in WALKS:
        for p in sorted((recordings / w).glob("frame-*.jpg")):
            out.append((w, int(p.stem.split("-")[1]), p))
    return out


def is_yoloe(weights: str) -> bool:
    """By file NAME: a full path starts with its directory, and testing that
    loaded YOLOE as plain YOLO with COCO class 0 -- which fired on nothing
    (found 2026-10-09, the first run of this tool)."""
    return Path(weights).name.startswith("yoloe")


def run_arm(frames: list, weights: str, conf: float, imgsz: int, device=None) -> dict:  # pragma: no cover
    from ultralytics import YOLO, YOLOE

    if is_yoloe(weights):
        model = YOLOE(weights)
        model.set_classes(["person"], model.get_text_pe(["person"]))
        classes = None
    else:
        model = YOLO(weights)
        classes = [0]  # COCO person
    fired = {}
    for w, n, p in frames:
        r = model.predict(str(p), conf=conf, imgsz=imgsz, classes=classes,
                          verbose=False, device=device)[0]
        boxes = [[round(v) for v in b] + [round(float(c), 3)]
                 for b, c in zip(r.boxes.xyxy.tolist(), r.boxes.conf.tolist())]
        if boxes:
            fired[(w, n)] = boxes
    return fired


def main(argv=None) -> int:  # pragma: no cover - needs the models
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--recordings", type=Path, required=True)
    ap.add_argument("--weights-dir", type=Path, default=Path("."))
    ap.add_argument("--device", default=None)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args(argv)
    frames = _frames(args.recordings)
    all_frames = {(w, n) for w, n, _ in frames}
    result = {"frames": len(frames), "arms": []}
    for weights, conf, imgsz in ARMS:
        fired = run_arm(frames, str(args.weights_dir / weights), conf, imgsz, args.device)
        row = {"weights": weights, "conf": conf, "imgsz": imgsz,
               "unverified": score(fired, all_frames),
               "fired": {f"{w}/{n:04d}": b for (w, n), b in sorted(fired.items())}}
        result["arms"].append(row)
        print(weights, conf, imgsz, row["unverified"], flush=True)
    args.out.write_text(json.dumps(result, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
