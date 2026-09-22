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
import base64
import json
import os
from concurrent.futures import ThreadPoolExecutor
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


# ---------------------------------------------------------------------------
# A cloud VLM as the proposer.
#
# Read the docstring's warning first, because it applies here HARDER than it
# does to OWLv2: these models are the ones /navigate actually runs, so a
# corpus labelled by one of them is a corpus of that model's opinions about
# itself. Measured on 2026-09-22 over the 452 adjudicated frames, Opus 4.5
# reads 88% recall with a colour-strictness failure -- it finds a charcoal
# backpack and rejects it as "black, not grey" -- and 4 of the 11 unlabelled
# walks target a "red backpack", the exact axis that bias runs along.
#
# It is still worth having, for the reason the docstring gives: the bands are
# the deliverable, and sorting 1,512 frames by a model's confidence turns an
# impossible review into a tractable one. What it must never do is write
# labels.json, and main() below is what enforces that.
#
# A PURPOSE-BUILT prompt, not /navigate's. /navigate answers "what should the
# robot do", and its target_visible rides along with navigation semantics
# (it has a bearing, an obstacle question, a step budget). A labeller wants
# one question asked cleanly, plus the confidence the bands are built from.
_LABEL_PROMPT = """You are labelling a frame from a floor-level robot camera for a dataset.

Question: is the target object "{target}" visible anywhere in this image?

Rules:
  * Partial visibility counts as visible -- an edge, a corner, or a piece
    seen behind or under something else is still visible.
  * Extreme close-ups count: if the object fills the frame, it is visible.
  * The target description is approximate, especially its COLOUR. Judge the
    object itself. If the object is plainly the one being looked for but a
    word in the description is arguable (a charcoal bag called "grey"), it
    IS visible -- lower your confidence rather than answering no.
  * Answer no only when you believe the object is genuinely not in view.
  * Express every doubt as a LOWER CONFIDENCE, never as a false no or yes.

Reply with JSON only:
{{"visible": true|false,
  "confidence": 0.0-1.0,
  "seen": "<a few words naming what you based that on>"}}"""
# The last two rules are the whole prompt, and they were MEASURED into it on
# 2026-09-22 against the two walks a human has adjudicated.
#
# A first version said "a similar but DIFFERENT object does not count. Be
# specific about which." That reads as reasonable and it was the single worst
# thing in the file: it invites the model to adjudicate the target's ADJECTIVE.
# Opus 4.5 then found the charcoal backpack in every frame, wrote "black
# backpack visible... but no grey backpack detected", and answered no --
# CONFIDENTLY, so the frames landed in the `absent` band nobody reviews.
#
#   prompt          recall   FN   FP   silent errors   review band
#   v1 ("different")   77%   16   12              17            23
#   v2 (this one)     100%    0   13               0            49
#
# Same model, same frames, same cost ($0.78 vs $0.81). The number that
# matters is SILENT ERRORS -- 17 to 0. v2 is not more accurate so much as
# honestly uncertain: it makes its mistakes where a reviewer is looking.
# The review band nearly doubles, and that is the trade being bought.
#
# Checked against the opposite failure too, on the verified-empty walk where
# a red luggage tag sits at the edge of frame: v2 scores 6 false positives
# against v1's 5, ALL of them in the review band, none silent. Loosening the
# colour rule did not cost precision.
#
# A note for whoever changes this next: the corpus cannot tell you whether a
# rule helps. Run it against recordings/grey-backpack-20260921-203748 and
# recordings/red-backpack-20260921-203152 with --force, and count silent
# errors, not accuracy.


def propose_cloud(walk_dir: Path, model_id: str, region: str | None = None,
                  workers: int = 6) -> dict:
    """Propose labels with a Bedrock vision model. Same shape as propose()."""
    import boto3

    meta = json.loads((walk_dir / "meta.json").read_text())
    target = (meta.get("target_object") or "").strip()
    if not target:
        raise SystemExit(f"{walk_dir.name}: meta.json has no target_object")

    client = boto3.client("bedrock-runtime", region_name=region)
    prompt = _LABEL_PROMPT.format(target=target)
    frames = sorted(p for p in walk_dir.iterdir() if p.suffix.lower() == ".jpg")

    def one(path: Path):
        body = path.read_bytes()
        for attempt in range(4):
            try:
                r = client.converse(
                    modelId=model_id,
                    messages=[{"role": "user", "content": [
                        {"image": {"format": "jpeg", "source": {"bytes": body}}},
                        {"text": prompt}]}],
                    inferenceConfig={"maxTokens": 200})
                text = "".join(b["text"] for b in r["output"]["message"]["content"] if "text" in b)
                d = json.loads(text[text.index("{"):text.rindex("}") + 1])
                u = r.get("usage") or {}
                return path.name, {
                    "visible": bool(d.get("visible")),
                    # An absent confidence must not read as 0.0 ("certainly
                    # absent") -- that would file it in the band nobody
                    # reviews. Unknown sorts into `review` instead.
                    "confidence": float(d["confidence"]) if "confidence" in d else 0.5,
                    "seen": str(d.get("seen", ""))[:120],
                    "in_tok": u.get("inputTokens", 0), "out_tok": u.get("outputTokens", 0),
                }
            except Exception as e:  # noqa: BLE001 -- one bad frame must not lose the walk
                if attempt == 3:
                    return path.name, {"error": f"{type(e).__name__}: {e}"[:140]}
                import time as _t
                _t.sleep(2 ** attempt)

    with ThreadPoolExecutor(max_workers=workers) as pool:
        results = dict(pool.map(one, frames))

    labels, scores, seen, errors = {}, {}, {}, 0
    in_tok = out_tok = 0
    for name, r in results.items():
        if r.get("error"):
            errors += 1
            # Unscored rather than guessed: a frame the labeller never saw
            # goes to a human, it does not get a default.
            scores[name] = 0.5; labels[name] = False; seen[name] = r["error"]
            continue
        # Confidence is in the CLAIM, so turn it into confidence that the
        # target is present -- otherwise a confident "absent" (0.95) lands
        # in the `visible` band.
        p = r["confidence"] if r["visible"] else 1.0 - r["confidence"]
        scores[name] = round(p, 4); labels[name] = bool(r["visible"]); seen[name] = r["seen"]
        in_tok += r["in_tok"]; out_tok += r["out_tok"]

    bands = {"visible": [], "review": [], "absent": []}
    for name, s in scores.items():
        bands["visible" if s >= HIGH else ("absent" if s < LOW else "review")].append(name)
    return {
        "walk": walk_dir.name,
        "description": target,
        "target_visible_labels": labels,
        "adjudicated": [],
        "proposed_by": {"model": model_id, "prompt": "label_assist/v2",
                        "high": HIGH, "low": LOW, "errors": errors},
        "candidate_scores": scores,
        "candidate_seen": seen,
        "review_bands": {k: sorted(v) for k, v in bands.items()},
        "usage": {"input_tokens": in_tok, "output_tokens": out_tok},
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("walks", nargs="+", help="walk directories")
    ap.add_argument("--detector", default="crops:owlv2",
                    help="the PROPOSER. Keep it different from the detector "
                         "under test (default crops:owlv2; shipped is yoloe)")
    ap.add_argument("--clip", default="RN50")
    ap.add_argument("--model", default=None,
                    help="Bedrock model id -- propose with a cloud VLM instead "
                         "of the local detector. Read this module's warning "
                         "about proposers that are also under test.")
    ap.add_argument("--region", default=None, help="region for --model")
    ap.add_argument("--force", action="store_true",
                    help="re-propose a walk that already has labels.json. Still "
                         "writes only the candidate file; used for CALIBRATION, "
                         "to score a proposer against labels a human adjudicated.")
    args = ap.parse_args(argv)

    for w in args.walks:
        d = Path(w)
        if (d / LABELS).exists() and not args.force:
            print(f"{d.name}: already has {LABELS} -- skipped, never overwritten")
            continue
        doc = (propose_cloud(d, args.model, region=args.region) if args.model
               else propose(d, detector=args.detector, clip=args.clip))
        (d / CANDIDATE).write_text(json.dumps(doc, indent=1))
        b = doc["review_bands"]
        print(f"{d.name}: {len(doc['target_visible_labels'])} frames -> "
              f"{len(b['visible'])} visible, {len(b['review'])} REVIEW, "
              f"{len(b['absent'])} absent  -> {CANDIDATE}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
