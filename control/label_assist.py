"""Propose per-frame visibility labels for a walk, for a human to adjudicate.

**This never writes `labels.json`.** It writes `labels.candidate.json`, and
the distinction is the whole point: `control/perception_eval.py` refuses an
unlabelled walk precisely because a model's own claim is not ground truth --
34 of 44 `target_visible` claims on one search walk were a storage bin. A
tool that auto-filled `labels.json` would quietly convert that refusal into
a corpus of the detector's opinions about itself.

What it does instead is cut the review down. Adjudicating 422 frames by eye
is the reason five recorded walks sit unscorable; sorting them by a
confidence a DIFFERENT model assigns turns that into confirming a handful of
boundary cases. The bands are the deliverable:

  * `visible`   -- high confidence, spot-check a few
  * `review`    -- the band where the model is unsure. THIS is the work.
  * `absent`    -- low confidence, spot-check a few

The proposer is OWLv2 as a crop source, chosen because it is **not** the
shipped detector (P24 ships `yoloe-11s`). A labeller that shares the
detector under test marks its own homework, and every recall number
measured against it would be inflated by exactly the errors they agree on.
OWLv2 reads 90% at a 3-FP budget on the existing corpus (P23), so it is
strong enough to be worth sorting by and wrong often enough to need a human.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

LABELS = "labels.json"
CANDIDATE = "labels.candidate.json"
# Above this, the proposer is confident the target is there; below the lower
# bound, confident it is not. Between them is what a person should actually
# look at. Deliberately wide -- a narrow band hides the disagreements that
# make adjudication worth doing.
HIGH, LOW = 0.90, 0.30


def propose(walk_dir: Path, detector: str = "crops:owlv2",
            clip: str = "RN50") -> dict:
    from brain.perceive_lab import pipeline_for_spec
    from control.perception_eval import frame_dict

    meta = json.loads((walk_dir / "meta.json").read_text())
    target = (meta.get("target_object") or "").strip()
    if not target:
        raise SystemExit(f"{walk_dir.name}: meta.json has no target_object")

    pipeline = pipeline_for_spec(target.lower(), detector=detector,
                                 clip_model=clip, proposer="none",
                                 max_crops=16, crop_path="low_confidence")
    frames = sorted(p for p in walk_dir.iterdir() if p.suffix.lower() == ".jpg")
    scores, labels = {}, {}
    for p in frames:
        r = pipeline.perceive(frame_dict(p))
        s = float(r.best.probability) if r.best else 0.0
        scores[p.name] = round(s, 4)
        labels[p.name] = s >= HIGH
    band = lambda s: "visible" if s >= HIGH else ("absent" if s < LOW else "review")
    bands = {"visible": [], "review": [], "absent": []}
    for name, s in scores.items():
        bands[band(s)].append(name)
    return {
        "walk": walk_dir.name,
        "description": target,
        # Same key `labels.json` uses, so a reviewer can rename the file
        # after checking it rather than reshaping it.
        "target_visible_labels": labels,
        # Empty on purpose: `adjudicated` means a HUMAN looked. Nothing here
        # has been looked at, and pre-filling it would erase the difference
        # perception_eval reports as its `adjudicated` count.
        "adjudicated": [],
        "proposed_by": {"detector": detector, "clip": clip,
                        "high": HIGH, "low": LOW},
        "candidate_scores": scores,
        "review_bands": {k: sorted(v) for k, v in bands.items()},
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("walks", nargs="+", help="walk directories")
    ap.add_argument("--detector", default="crops:owlv2",
                    help="the PROPOSER. Keep it different from the detector "
                         "under test (default crops:owlv2; shipped is yoloe)")
    ap.add_argument("--clip", default="RN50")
    args = ap.parse_args(argv)

    for w in args.walks:
        d = Path(w)
        if (d / LABELS).exists():
            print(f"{d.name}: already has {LABELS} -- skipped, never overwritten")
            continue
        doc = propose(d, detector=args.detector, clip=args.clip)
        (d / CANDIDATE).write_text(json.dumps(doc, indent=1))
        b = doc["review_bands"]
        print(f"{d.name}: {len(doc['target_visible_labels'])} frames -> "
              f"{len(b['visible'])} visible, {len(b['review'])} REVIEW, "
              f"{len(b['absent'])} absent  -> {CANDIDATE}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
