"""
tools/gpu/sweep.py

The soft-gate sweep, run against the whole labelled corpus on a rented GPU.

**The question.** 4.2's crop gate has had exactly two settings and both are
measured: the hard `label_gate` (keep crops YOLO labelled with the target's
COCO word) and, since 2026-09-07, the open-vocabulary `low_confidence`
(keep everything, let CLIP decide). The hard gate lost 11 of 18 true
positives on the bottle walk because at close range the detector *relabels*
the target, so the default moved. That fixed recall by deleting the gate,
and 2.9's per-frame budget now pays for it: every surviving proposal is one
more CLIP image encode, and on an 8L the three models together have 3.1x
headroom rather than the 10-30x this plan assumed for a while.

`brain/perceive.py`'s `soft_gate` is the interior of that axis. The COCO
class names are ranked against the mission's target string in CLIP's *text*
space, once at mission start, and the top `k` become the accept set. k=1 is
close to the hard gate; k=80 is the open-vocabulary path exactly. So this
sweeps ONE parameter between two already-measured endpoints.

**What would count as a win**, pre-registered here so the result is
readable either way:

  * **the interesting one** -- open-vocabulary recall at materially fewer
    crops per frame. That is 2.9 budget bought for nothing, and it is the
    only outcome that changes what runs on the car.
  * **the weaker one** -- fewer false positives at equal recall.
  * **the null** -- recall rises monotonically with k and crops rise with
    it, so there is no interior point and the axis is a straight line
    between two known ends. Then the shipped default is already right and
    this is one config's worth of GPU time spent confirming it.

**The known risk, stated up front.** CLIP was trained to align image with
text, not text with text. Text-text cosine here is a ranking signal and
nothing stronger -- short noun phrases crowd together near the top of the
range -- which is why `label_affinity()` takes a top-k rather than
thresholding an absolute similarity, and why the accept sets are printed:
if the ordering is nonsense, that is the finding and the null above is not
the right reading of a flat curve.

**One model instance, every config.** The detector and the encoder are
built once and shared, so nothing here can be a difference between two
model loads. Only the target string, the crop path and k change.

    python sweep.py --out out --device cuda
    python sweep.py --out out --device cuda --ks 1,4,8 --with-floor
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from brain.perceive import (COCO_CLASSES, CROP_LOW_CONFIDENCE,
                            CROP_PATH_AUTO, CROP_SOFT_GATE, LOW_CONFIDENCE,
                            PerceptionPipeline)
from control.perception_eval import (METRIC_PROBABILITY, load_corpus,
                                     recall_at_fp_budget, run_walk,
                                     save_records, totals)

# The endpoints are the two paths already measured; the interior is what is
# being asked about. 80 is included so the sweep contains its own control:
# soft_gate at k=80 must reproduce low_confidence exactly, and if it does
# not, the gate is broken rather than the model interesting.
DEFAULT_KS = (1, 2, 4, 8, 16, 32, 80)


def build(detector, scorer, target, *, crop_path, affinity_k=None,
          proposer=None):
    kwargs = dict(crop_path=crop_path, proposer=proposer,
                  # Pinned, for every config including the baselines. The
                  # hard gate would otherwise ask the detector for 0.25 and
                  # the open path for 0.05, and a sweep that moves the
                  # proposal threshold along with the gate cannot attribute
                  # its own result.
                  confidence=LOW_CONFIDENCE)
    if affinity_k is not None:
        kwargs["affinity_k"] = affinity_k
    return PerceptionPipeline(detector, scorer, target, **kwargs)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--recordings", default="recordings")
    ap.add_argument("--out", default="out")
    ap.add_argument("--device", default=None)
    ap.add_argument("--ks", default=",".join(str(k) for k in DEFAULT_KS))
    ap.add_argument("--with-floor", action="store_true",
                    help="also run the floor-mask union at each k. Doubles "
                         "the run; the mask is the crop source 4.2 says an "
                         "out-of-vocabulary target actually needs")
    ap.add_argument("--only", default=None,
                    help="comma-separated config names to run, e.g. "
                         "soft-k4+floor. Everything else is skipped -- for "
                         "resuming a run that died part way rather than "
                         "paying for the configs already on disk")
    ap.add_argument("--fp-budgets", default="0,3,16")
    args = ap.parse_args(argv)

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    ks = [int(k) for k in args.ks.split(",") if k.strip()]
    budgets = [int(b) for b in args.fp_budgets.split(",")]

    walks = load_corpus(Path(args.recordings))
    print(f"corpus: {len(walks)} walks, "
          f"{sum(len(w.frames) for w in walks)} labelled frames, "
          f"{sum(w.visible for w in walks)} visible", flush=True)

    from brain.perceive_lab import ClipScorer, SegformerFloorProposer, YoloDetector
    print("loading models once, shared across every config below", flush=True)
    detector = YoloDetector("yolo11s.pt", device=args.device)
    scorer = ClipScorer("RN50", device=args.device)
    proposer = (SegformerFloorProposer(device=args.device)
                if args.with_floor else None)

    configs = [
        ("auto", dict(crop_path=CROP_PATH_AUTO), None),
        ("shipped-lowconf", dict(crop_path=CROP_LOW_CONFIDENCE), None),
    ]
    for k in ks:
        configs.append((f"soft-k{k}",
                        dict(crop_path=CROP_SOFT_GATE, affinity_k=k), None))
    if args.with_floor:
        configs.append(("shipped-lowconf+floor",
                        dict(crop_path=CROP_LOW_CONFIDENCE), proposer))
        for k in ks:
            configs.append((f"soft-k{k}+floor",
                            dict(crop_path=CROP_SOFT_GATE, affinity_k=k),
                            proposer))

    if args.only:
        wanted = {n.strip() for n in args.only.split(",") if n.strip()}
        unknown = wanted - {n for n, _, _ in configs}
        if unknown:
            print(f"unknown config(s): {sorted(unknown)}", flush=True)
            return 2
        configs = [c for c in configs if c[0] in wanted]
        print(f"--only: running {[c[0] for c in configs]}", flush=True)

    summary = []
    for name, kwargs, prop in configs:
        started = time.perf_counter()
        print(f"\n=== {name} " + "=" * (60 - len(name)), flush=True)
        records = []
        accept_sets = {}
        for walk in walks:
            try:
                pipeline = build(detector, scorer, walk.target, proposer=prop,
                                 **kwargs)
            except ValueError as exc:
                # `label_gate` on a target with no COCO word is refused at
                # construction, by design. Under `auto` that never happens
                # -- auto falls back -- so this is a real stop, not a skip.
                print(f"  {walk.name}: REFUSED -- {exc}", flush=True)
                return 2
            vocab = pipeline.vocabulary
            accept_sets[walk.name] = {
                "target": walk.target,
                "coco_class": vocab.coco_class,
                "in_vocabulary": vocab.in_vocabulary,
                "affinity": list(vocab.affinity),
                # The top of the ranking regardless of k, so a flat result
                # can be read as "the ordering is nonsense" rather than
                # "the gate does not matter".
                "top10": [[c, round(s, 4)] for c, s in vocab.ranked[:10]],
            }
            walk_records = run_walk(pipeline, walk, METRIC_PROBABILITY)
            records += walk_records
            scored = [r for r in walk_records if r.scored]
            print(f"  {walk.name:<42} {len(walk_records):>4} frames  "
                  f"{walk.visible:>3} visible  "
                  f"crops/frame {sum(r.candidates for r in walk_records) / max(1, len(walk_records)):5.2f}  "
                  f"scored {len(scored):>4}", flush=True)

        t = totals(records)
        crops = sum(r.candidates for r in records) / max(1, len(records))
        row = {"config": name, "crops_per_frame": round(crops, 3),
               "ms_per_frame": round(t["ms_per_frame"], 1),
               "frames": len(records),
               "seconds": round(time.perf_counter() - started, 1)}
        for b in budgets:
            r = recall_at_fp_budget(records, b)
            row[f"recall@{b}fp"] = r.get("recall")
            row[f"gate@{b}fp"] = r.get("gate")
            row[f"tp@{b}fp"] = r.get("tp")
            row[f"fp@{b}fp"] = r.get("fp")
        summary.append(row)
        print(f"  -> crops/frame {crops:.2f}   " +
              "   ".join(f"@{b}FP {row[f'recall@{b}fp']}" for b in budgets),
              flush=True)

        save_records(out / f"{name}.json", records,
                     {"config": name, "device": args.device or "cpu",
                      "detector": "yolo11s.pt", "clip": "RN50",
                      "confidence": LOW_CONFIDENCE,
                      "proposer": "floor" if prop is not None else "none",
                      "crop_path": kwargs["crop_path"],
                      "affinity_k": kwargs.get("affinity_k"),
                      "metric": METRIC_PROBABILITY,
                      "accept_sets": accept_sets})

    name = "summary.json" if not args.only else "summary-resumed.json"
    (out / name).write_text(json.dumps(summary, indent=2))
    print("\n" + json.dumps(summary, indent=2), flush=True)
    print(f"\nrecords -> {out}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
