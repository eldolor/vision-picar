"""Does a candidate target STRING fire spuriously? A probe, not a scorer.

Pre-flight for a rig session. The target string is a first-class variable
and not a caption -- it moved recall **7x** on the shoes walk, and P22
measured a colour+noun worth ~5x the margin of a bare noun -- so a string
that cannot discriminate wastes the whole walk. This rejects one in about
thirty seconds instead of after 200 frames of pushing a rig around.

It runs the shipped pipeline over a sample of the EXISTING corpus, asking
for a target that is not a labelled object in any of it. Every detection is
therefore either a spurious fire or an unlabelled real object in the room,
which is why this reports a RATE and a peak score and never a recall.

**Read two numbers, not one.**

  * A high firing rate means the string cannot discriminate. "a black
    dumbbell" fires on 22% of random home frames at P 0.998 -- dark compact
    objects are everywhere. Reject it.
  * A *zero* rate with a max P of 0.000 is not specificity, it is an INERT
    prompt: the detector proposes nothing for those words at all. "a green
    watering can" and "a blue recycling bin" both do this. A walk against an
    inert prompt fails trivially and tests the prompt rather than the
    architecture, which is worth nothing to a falsifier.

What you want is **live but specific**: a non-zero peak proving the prompt
grounds, with a low firing rate. "a red toolbox" (0.896 / 2%) and "a white
phone charger cable" (0.971 / 2%) both qualify; those are the two targets
`PLAN-onboard-perception.md`'s "What to record next" now names.

Also check the COCO gate, which this prints: `coco_class_for()` maps "an
orange extension cord" to `orange` (the FRUIT -- the colour adjective is
itself a COCO noun), "a purple dog leash" to `dog`, and "a beige cat litter
tray" to `cat`. Any of those quietly puts the target back in the vocabulary
the walk was meant to escape.
"""
from __future__ import annotations

import argparse
import glob
import random
from pathlib import Path

DEFAULT_SAMPLE = 200
SEED = 7   # fixed, so two candidate strings are compared on the same frames


def probe(targets, recordings="recordings", sample=DEFAULT_SAMPLE,
          gate=0.8, seed=SEED):
    from brain.perceive import coco_class_for, pipeline_for
    from control.perception_eval import frame_dict

    frames = sorted(glob.glob(f"{recordings}/*/frame-*.jpg"))
    if not frames:
        raise SystemExit(f"no frames under {recordings}/")
    rng = random.Random(seed)
    chosen = rng.sample(frames, min(sample, len(frames)))

    print(f"{len(chosen)} frames sampled from {len(frames)}  (seed {seed})\n")
    print(f"{'target string':34s} {'COCO gate':>12s} {'fires':>6s} {'rate':>6s} "
          f"{'max P':>7s}  verdict")
    rows = []
    for t in targets:
        pipeline = pipeline_for(t)
        hits, peak = 0, 0.0
        for f in chosen:
            r = pipeline.perceive(frame_dict(Path(f)))
            # Over EVERY candidate, not `r.best` (handoff 4h): `best` is set
            # only on a DETECTED frame, P >= the pipeline's own 0.8, so a
            # prompt grounding at P 0.6 read as "inert" and a --gate below
            # 0.8 could never count a hit.
            top = max((float(c.probability) for c in r.candidates), default=0.0)
            peak = max(peak, top)
            if top >= gate:
                hits += 1
        rate = hits / len(chosen)
        coco = coco_class_for(t)
        if coco:
            verdict = f"REJECT -- gated to COCO `{coco}`"
        elif peak < 0.01:
            verdict = "REJECT -- inert, the detector never grounds it"
        elif rate > 0.10:
            verdict = "REJECT -- fires on unrelated frames"
        else:
            verdict = "usable -- live and specific"
        print(f"{t:34s} {str(coco or '-'):>12s} {hits:>6d} {100*rate:>5.0f}% "
              f"{peak:>7.3f}  {verdict}")
        rows.append({"target": t, "coco_class": coco, "fires": hits,
                     "rate": rate, "max_probability": peak,
                     "verdict": verdict})
    return rows


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("targets", nargs="+", help="candidate target strings")
    ap.add_argument("--recordings", default="recordings")
    ap.add_argument("--sample", type=int, default=DEFAULT_SAMPLE)
    ap.add_argument("--gate", type=float, default=0.8)
    args = ap.parse_args(argv)
    probe(args.targets, recordings=args.recordings, sample=args.sample,
          gate=args.gate)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
