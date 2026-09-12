"""
perception_eval.py

Scores the **on-board perception tier** over a whole corpus of recorded
rig walks, against each walk's adjudicated `labels.json`.

Phase **P3** of `PLAN-onboard-perception.md` 4.10. `control/walk_eval.py`
is its sibling and scores a different thing: that one grades the *walk*
(did the policy navigate sensibly), this one grades the *perception*
(did the local tier see the target). They share a corpus, a directory
layout, and the rule that a score is only as good as its reference.

    python -m control.perception_eval score --floor-mask
    python -m control.perception_eval score --walk blue-shoes-20260907-152528
    python -m control.perception_eval compare shipped.json sam.json --fp-budget 3

## The reference is `labels.json`, never `walk.jsonl`

This is the whole reason the module exists. Every recall and precision
number in 4.10 up to 2026-09-07 used the VLM's own `target_visible` as
ground truth, and it is not ground truth: on the 209-frame search walk it
claimed the bottle on 44 frames, **34 of which are a teal storage bin in
the wrong room**, and on the backpack walk it rejected the target twice
over a colour word. `walk.jsonl` records what a model said at the time.
It is evidence about the model, not about the room.

So: `labels.json` beside each walk carries the per-frame human flag, which
frames were adjudicated by eye, and a note saying plainly that the VLM's
answer is not the reference. **This module reads that file and refuses to
run without it** -- a corpus-wide score computed against a model's own
opinion is worse than no score, because it looks like one.

## Score once, threshold afterwards

`PerceptionPipeline` applies its own gate and returns a tri-state. That is
right for a mission and wrong for a sweep: re-running three models over
299 frames for each candidate threshold costs minutes per gate and
measures nothing new, because **the gate is arithmetic over numbers the
pipeline already computed.**

So a run records `max(candidate.probability)` for every frame, gate-free,
and every table below is computed from those numbers. One pass over the
corpus, any number of operating points. `--save` writes the per-frame
records out so a second config can be compared without paying for either
again -- which matters as soon as a model is slow enough to care about
(SAM's automatic mask generation is seconds per frame, not milliseconds).

## Recall without its false-positive count is not a result

4.11 is built this way on purpose and says why: an open-vocabulary
detector run at a low enough confidence will beat anything on recall while
inventing targets, and **that is exactly how YOLO-World would have looked
like a win.** Every table here prints TP, FP and both rates together, and
`compare` matches on the false-positive budget before it reports a recall
at all -- `recall_at_fp_budget()` finds the exact threshold that admits at
most B false positives, so two configs are always read at the same cost.

## What it cannot tell you

Throughput, for 2.9's reason -- a laptop is not an 8L, and the `ms` column
compares models to each other and never to the robot's frame budget. And
finding is not navigating: Stage 0's failure was never recognition, so a
perfect score here leaves the actual gap (`brain/planner.py`, C8)
untouched.
"""

from __future__ import annotations

import base64
import json
import math
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Optional, Sequence

# The tri-state, imported rather than duplicated: `unavailable` has to mean
# the same thing here as it does in the pipeline, or a corpus score would
# quietly count a wedged capture as "the target is not here" -- the exact
# failure 1.12 designed the third state to prevent.
# `_image_width` rather than a second copy reading the same header: it is
# already the thing that reads a frame's width off the pixels, and
# duplicating it here would have put PIL in control/ -- which
# tests/test_lambda_packaging.py caught, correctly, because the walks
# Lambda copies every file in this directory.
from brain.perceive import ABSENT, DETECTED, UNAVAILABLE, _image_width

SUFFIXES = {".jpg", ".jpeg", ".png", ".webp"}
LABELS_FILE = "labels.json"

# The gates every table sweeps unless told otherwise. 0.80 is the shipped
# `DEFAULT_MATCH_PROBABILITY`; 0.50 is 1.11a's proposed corroboration bar;
# the rest are there so the shape of the curve between them is visible
# rather than inferred from two points.
DEFAULT_GATES = (0.90, 0.80, 0.70, 0.60, 0.50, 0.40, 0.30)

# What `score` means. The probability is the shipped gate and the only one
# comparable across target strings; the margin is kept because every
# measurement before 2026-09-07 used it, and reproducing an old number is
# how you tell a real change from a changed metric.
METRIC_PROBABILITY = "probability"
METRIC_MARGIN = "margin"
# An open-vocabulary detector (4.11) has no CLIP stage and no distractors:
# the number it thresholds on is its own box confidence. Recorded under its
# own name so no table can silently compare a probability against one.
METRIC_CONFIDENCE = "confidence"

# The smallest gap between the weakest true positive and the strongest false
# one that still counts as a separation. Below this the model is saturating
# rather than ranking, and an operating point derived from it is arithmetic
# rather than a threshold anyone could set. 1e-4 is four decimal places --
# the precision these scores are reported and reasoned about at.
MIN_USABLE_MARGIN = 1e-4
METRICS = (METRIC_PROBABILITY, METRIC_MARGIN, METRIC_CONFIDENCE)


class CorpusError(Exception):
    """A corpus that cannot be scored -- missing labels, empty walk."""


@dataclass
class FrameScore:
    """One frame, scored, with the gate deliberately not yet applied."""

    walk: str
    frame: str
    visible: bool                 # ground truth, from labels.json
    status: str                   # the pipeline's own tri-state, at its own gate
    score: Optional[float]        # None when nothing was scored at all
    metric: str = METRIC_PROBABILITY
    label: Optional[str] = None
    candidates: int = 0
    adjudicated: bool = False
    ms: float = 0.0

    @property
    def scored(self) -> bool:
        return self.score is not None and not math.isnan(self.score)


@dataclass
class Walk:
    """A recorded rig walk plus its adjudicated labels."""

    name: str
    path: Path
    target: str
    frames: list = field(default_factory=list)   # [(Path, visible, adjudicated)]

    @property
    def visible(self) -> int:
        return sum(1 for _, vis, _ in self.frames if vis)


# ---------------------------------------------------------------------------
# Loading. Pure filesystem, no models.
# ---------------------------------------------------------------------------

def load_walk(path: Path) -> Walk:
    """One walk directory -> a `Walk`, or `CorpusError` explaining what is
    missing. The labels file is required, not optional: see the module
    docstring for why an unlabelled walk must not be scorable."""
    labels_path = path / LABELS_FILE
    if not labels_path.is_file():
        raise CorpusError(
            f"{path.name} has no {LABELS_FILE}. The VLM's own target_visible "
            "is not ground truth (34 of 44 claims on the search walk were a "
            "storage bin), so a walk without adjudicated labels cannot be "
            "scored -- adjudicate it first.")
    doc = json.loads(labels_path.read_text())
    labels = doc.get("target_visible_labels") or {}
    if not labels:
        raise CorpusError(f"{path.name}/{LABELS_FILE} carries no per-frame labels")
    target = (doc.get("description") or "").strip()
    if not target:
        raise CorpusError(
            f"{path.name}/{LABELS_FILE} has no `description` -- that string is "
            "the target the pipeline is asked for, and it moved recall 7x on "
            "the shoes walk, so it may not be guessed from the walk name.")
    adjudicated = set(doc.get("adjudicated") or [])

    images = sorted(p for p in path.iterdir() if p.suffix.lower() in SUFFIXES)
    frames = [(p, bool(labels[p.name]), p.name in adjudicated)
              for p in images if p.name in labels]
    if not frames:
        raise CorpusError(f"{path.name}: no image on disk is named in {LABELS_FILE}")
    return Walk(name=path.name, path=path, target=target, frames=frames)


def load_corpus(root: Path, only: Sequence[str] = ()) -> list:
    """Every scorable walk under `root`, in name order."""
    if not root.is_dir():
        raise CorpusError(f"{root} is not a directory")
    dirs = sorted(p for p in root.iterdir()
                  if p.is_dir() and (p / LABELS_FILE).is_file())
    if only:
        wanted = set(only)
        dirs = [p for p in dirs if p.name in wanted]
        missing = wanted - {p.name for p in dirs}
        if missing:
            raise CorpusError(
                f"no labelled walk named {', '.join(sorted(missing))} under {root}")
    if not dirs:
        raise CorpusError(
            f"no walk under {root} has a {LABELS_FILE}. The corpus is four rig "
            "walks recorded 2026-09-07; see CLAUDE.md Stage 0 for where they live.")
    return [load_walk(p) for p in dirs]


def frame_dict(path: Path) -> dict:
    """A `RobotInterface`-shaped frame, which is what `perceive()` takes.

    `image_width` is read off the file rather than assumed -- the bearing is
    computed against it, and a bearing scaled by a guessed width is wrong by
    exactly the ratio nobody checked (which is how `bearing_deg` managed to
    never once be a number before 2026-09-07)."""
    data = path.read_bytes()
    frame = {
        "image_base64": base64.b64encode(data).decode(),
        "media_type": f"image/{path.suffix.lstrip('.').lower().replace('jpg', 'jpeg')}",
    }
    width = _image_width(data)
    if width:
        frame["image_width"] = width
    return frame


# ---------------------------------------------------------------------------
# Scoring. Pure arithmetic over FrameScores -- no models, no filesystem.
# ---------------------------------------------------------------------------

def best_score(perception, metric: str = METRIC_PROBABILITY) -> Optional[float]:
    """The number this frame would be thresholded on, **independent of the
    gate the pipeline happened to be built with.**

    Taken over every candidate rather than off `perception.best`, because
    `best` is the argmax under whichever metric the pipeline was
    configured for, and a sweep must not inherit that choice.
    """
    if perception.status == UNAVAILABLE:
        return None
    scores = [getattr(c, metric) for c in perception.candidates]
    return max(scores) if scores else None


def score_at(records: Sequence[FrameScore], gate: float) -> dict:
    """TP/FP/FN and both rates at one operating point.

    `unavailable` frames are counted separately and are never detections:
    "I could not tell" is not "it is not there" (1.12), and folding them
    into FN would quietly blame the matcher for a dead camera.
    """
    tp = fp = fn = tn = unavailable = 0
    for r in records:
        if r.status == UNAVAILABLE:
            unavailable += 1
        hit = r.scored and r.score >= gate
        if r.visible and hit:
            tp += 1
        elif r.visible:
            fn += 1
        elif hit:
            fp += 1
        else:
            tn += 1
    visible = tp + fn
    return {
        "gate": gate,
        "frames": len(records),
        "visible": visible,
        "tp": tp, "fp": fp, "fn": fn, "tn": tn,
        "unavailable": unavailable,
        "recall": (tp / visible) if visible else None,
        "precision": (tp / (tp + fp)) if (tp + fp) else None,
    }


def sweep(records: Sequence[FrameScore],
          gates: Sequence[float] = DEFAULT_GATES) -> list:
    return [score_at(records, g) for g in gates]


def by_walk(records: Sequence[FrameScore]) -> dict:
    out: dict = {}
    for r in records:
        out.setdefault(r.walk, []).append(r)
    return out


def recall_at_fp_budget(records: Sequence[FrameScore], budget: int) -> dict:
    """The operating point that admits **at most `budget` false positives**,
    and the recall there. This is the only fair way to compare two configs.

    4.11's table is built this way and its own caution says why: a detector
    run at a low enough confidence beats anything on recall while inventing
    targets, and reporting that recall without its false-positive count is
    how YOLO-World would have looked like a win.

    Exact rather than swept: the threshold is set just above the
    (budget+1)-th highest score among non-target frames, so no grid
    resolution is involved and the answer cannot fall between two gates.
    """
    negatives = sorted((r.score for r in records if not r.visible and r.scored),
                       reverse=True)
    if len(negatives) <= budget:
        # Even admitting everything scored costs fewer FPs than the budget.
        gate = float("-inf")
    else:
        # Strictly above the first score we are not allowed to admit.
        gate = math.nextafter(negatives[budget], math.inf)
    out = score_at(records, gate)
    out["budget"] = budget
    # **How much room the operating point actually has.**
    #
    # A saturated model scores 0.9999999999856 on its true sightings and
    # 0.9999999999766 on the frames it invented, and the arithmetic above
    # will happily separate those -- reporting 10/10 at zero false positives
    # off a gap of 9e-12. That number is real and completely useless: no
    # threshold anyone can set lives there, and at any usable gate the same
    # model fires on 34 of 34 confabulated frames.
    #
    # Measured on Qwen3-VL-4B, 2026-09-09, and it flattered exactly the model
    # the hardware decision was leaning toward. `separable` is the caller's
    # guard: a margin this thin means the score does not rank, it saturates.
    positives = [r.score for r in records if r.visible and r.scored
                 and r.score >= gate]
    margin = (min(positives) - negatives[budget]) if (positives and
              len(negatives) > budget) else None
    out["margin"] = margin
    out["separable"] = margin is None or margin >= MIN_USABLE_MARGIN
    return out


def totals(records: Sequence[FrameScore]) -> dict:
    return {
        "walks": len({r.walk for r in records}),
        "frames": len(records),
        "visible": sum(1 for r in records if r.visible),
        "adjudicated": sum(1 for r in records if r.adjudicated),
        "unavailable": sum(1 for r in records if r.status == UNAVAILABLE),
        "detected": sum(1 for r in records if r.status == DETECTED),
        "absent": sum(1 for r in records if r.status == ABSENT),
        "ms_per_frame": (sum(r.ms for r in records) / len(records)) if records else 0.0,
    }


# ---------------------------------------------------------------------------
# Serialisation, so a slow config is paid for once.
# ---------------------------------------------------------------------------

def decisions_by_frame(walk_dir: Path) -> dict:
    """Map each frame FILE to the mission decision actually made on it.

    Under "Drive via brain" the twin pushes frames on one timer, polls
    `/mission/status` on another, and the mission ticks on a third -- so the
    status stored beside a frame in `walk.jsonl` usually belongs to an
    EARLIER frame. Scoring that pairing per-frame is how a correct answer
    gets read as a confabulation, which nearly happened on 2026-09-08.

    Both halves of the fix meet here: every recorded frame carries the robot's
    own `teleop_seq`, and every mission status carries `last_frame_seq` --
    the id the decision was actually taken on. Joining on those is exact.

    Returns `{frame_name: decision}` for the frames a decision can be tied
    to, and simply omits the rest. An unaligned walk yields `{}` rather than
    a plausible-looking guess.
    """
    path = walk_dir / "walk.jsonl"
    if not path.is_file():
        return {}
    by_teleop, decisions = {}, {}
    for line in path.read_text().splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        seq = row.get("seq")
        name = row.get("file") or ("frame-%04d.jpg" % seq if seq is not None else None)
        if name is None:
            continue
        if row.get("teleop_seq") is not None:
            by_teleop[row["teleop_seq"]] = name
        nav = row.get("navigate") or {}
        status = nav.get("_missionStatus") or {}
        decided_on = status.get("last_frame_seq")
        if decided_on is not None:
            decisions[decided_on] = status
    return {by_teleop[t]: d for t, d in decisions.items() if t in by_teleop}


def save_records(path: Path, records: Sequence[FrameScore], config: dict) -> None:
    path.write_text(json.dumps({
        "config": config,
        "records": [asdict(r) for r in records],
    }, indent=1))


def load_records(path: Path) -> tuple:
    doc = json.loads(Path(path).read_text())
    return ([FrameScore(**r) for r in doc["records"]], doc.get("config") or {})


# ---------------------------------------------------------------------------
# Running the models. The only part that needs the optional heavy deps.
# ---------------------------------------------------------------------------

def run_walk(pipeline, walk: Walk, metric: str = METRIC_PROBABILITY,
             on_frame=None) -> list:
    """Score every labelled frame of one walk. `pipeline` is anything with
    `perceive(frame) -> Perception` -- the shipped one, or one of
    `brain/perceive_lab.py`'s candidates."""
    records = []
    for path, visible, adjudicated in walk.frames:
        started = time.perf_counter()
        perception = pipeline.perceive(frame_dict(path))
        ms = (time.perf_counter() - started) * 1000
        record = FrameScore(
            walk=walk.name, frame=path.name, visible=visible,
            status=perception.status, score=best_score(perception, metric),
            metric=metric,
            label=perception.best.detection.label if perception.best else None,
            candidates=len(perception.candidates),
            adjudicated=adjudicated, ms=ms,
        )
        records.append(record)
        if on_frame:
            on_frame(record)
    return records


# ---------------------------------------------------------------------------
# Formatting
# ---------------------------------------------------------------------------

def _pct(x) -> str:
    return "  --" if x is None else f"{x:4.0%}"


def format_gate_table(records: Sequence[FrameScore],
                      gates: Sequence[float] = DEFAULT_GATES) -> str:
    lines = ["  gate   visible    TP    FP    FN   recall  precision",
             "  " + "-" * 55]
    for row in sweep(records, gates):
        lines.append(
            f"  {row['gate']:.2f}   {row['visible']:>7}  {row['tp']:>4}  "
            f"{row['fp']:>4}  {row['fn']:>4}   {_pct(row['recall'])}"
            f"      {_pct(row['precision'])}")
    return "\n".join(lines)


def format_per_walk(records: Sequence[FrameScore], gate: float) -> str:
    lines = [f"  per walk at P >= {gate:.2f}",
             f"  {'walk':<34}{'frames':>7}{'visible':>8}{'TP':>5}{'FP':>5}"
             f"{'recall':>9}",
             "  " + "-" * 68]
    for name, rows in sorted(by_walk(records).items()):
        s = score_at(rows, gate)
        lines.append(f"  {name[:33]:<34}{s['frames']:>7}{s['visible']:>8}"
                     f"{s['tp']:>5}{s['fp']:>5}{_pct(s['recall']):>9}")
    s = score_at(records, gate)
    lines.append("  " + "-" * 68)
    lines.append(f"  {'TOTAL':<34}{s['frames']:>7}{s['visible']:>8}"
                 f"{s['tp']:>5}{s['fp']:>5}{_pct(s['recall']):>9}")
    return "\n".join(lines)


def format_comparison(named: Sequence[tuple], budgets: Sequence[int]) -> str:
    """`named` is [(label, records), ...]. One row per config per budget,
    every recall printed beside the false positives it cost."""
    lines = [f"  {'config':<38}{'FP budget':>10}{'gate':>8}{'TP':>6}{'FP':>5}"
             f"{'recall':>9}",
             "  " + "-" * 76]
    for budget in budgets:
        for label, records in named:
            s = recall_at_fp_budget(records, budget)
            gate = "-inf" if s["gate"] == float("-inf") else f"{s['gate']:.3f}"
            lines.append(f"  {label[:37]:<38}{budget:>10}{gate:>8}{s['tp']:>6}"
                         f"{s['fp']:>5}{_pct(s['recall']):>9}")
        lines.append("")
    return "\n".join(lines).rstrip()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _add_score_args(ap) -> None:
    from brain.perceive import (DEFAULT_CLIP, DEFAULT_CROP_PATH,
                                DEFAULT_DETECTOR, DEFAULT_SEGMENTER)

    ap.add_argument("--recordings", default="recordings",
                    help="corpus root (default: recordings/)")
    ap.add_argument("--walk", action="append", default=[],
                    help="score only this walk; repeatable")
    ap.add_argument("--vlm-max-pixels", type=int, default=None,
                    help="tiling budget for --detector vlm, in pixels. This "
                         "is the variable under test: it decides how many "
                         "pixels a small distant object survives the vision "
                         "encoder with. Default: the model's own")
    ap.add_argument("--no-vlm-boxes", action="store_true",
                    help="skip the VLM's grounding pass. Halves the cost and "
                         "loses the bearing; the score is unaffected, because "
                         "it comes from the yes/no logits rather than the box")
    ap.add_argument("--detector", default=DEFAULT_DETECTOR,
                    help=f"detector: a YOLO weights file (default "
                         f"{DEFAULT_DETECTOR}), `none` for proposer-only, or "
                         f"an open-vocabulary model -- gdino, owlv2, "
                         f"yoloworld, or vlm:<model id> (4.11). See "
                         f"brain/perceive_lab.py")
    ap.add_argument("--clip", default=DEFAULT_CLIP,
                    help=f"CLIP encoder (default {DEFAULT_CLIP})")
    ap.add_argument("--proposer", default="none",
                    choices=("none", "floor", "sam"),
                    help="class-agnostic crop source unioned with the "
                         "detector's boxes (default none)")
    ap.add_argument("--floor-mask", action="store_true",
                    help="shorthand for --proposer floor")
    ap.add_argument("--segmenter", default=DEFAULT_SEGMENTER,
                    help="floor-mask model")
    ap.add_argument("--crop-path", default=DEFAULT_CROP_PATH,
                    help=f"auto | label_gate | low_confidence "
                         f"(default {DEFAULT_CROP_PATH})")
    ap.add_argument("--metric", default=METRIC_PROBABILITY, choices=METRICS,
                    help="what to threshold on (default probability -- the "
                         "only one comparable across target strings)")
    ap.add_argument("--gate", type=float, default=0.8,
                    help="the operating point the per-walk table is read at "
                         "(default 0.8, the shipped gate)")
    ap.add_argument("--gates", default=None,
                    help="comma-separated sweep, e.g. 0.9,0.8,0.5")
    ap.add_argument("--device", default=None,
                    help="torch device for the models: cpu, cuda, mps. "
                         "Default cpu -- every published record was scored "
                         "that way and stays reproducible")
    ap.add_argument("--dtype", default=None, choices=["fp32", "fp16", "bf16"],
                    help="inference precision. fp16 is what an edge GPU would "
                         "most plausibly run a ViT at, and its accuracy cost "
                         "is assumed rather than measured everywhere in "
                         "PLAN-onboard-perception.md")
    ap.add_argument("--imgsz", type=int, default=None,
                    help="detector input resolution (default 640, which is "
                         "what 4.3.1's 92 FPS on the 8L was measured at). "
                         "Raising it is the standard fix for small distant "
                         "objects and the 8L has the headroom for ~4x")
    ap.add_argument("--max-crops", type=int, default=None,
                    help="per-frame crop budget (default: the module's, which "
                         "scales with the number of crop sources)")
    ap.add_argument("--save", default=None, metavar="FILE",
                    help="write the per-frame records for `compare`")
    ap.add_argument("--quiet", action="store_true", help="no per-frame lines")


def _build_pipeline(args, target: str):
    from brain.perceive_lab import pipeline_for_spec

    return pipeline_for_spec(
        target,
        device=getattr(args, "device", None),
        dtype=getattr(args, "dtype", None),
        detector=args.detector,
        clip_model=args.clip,
        proposer=("floor" if args.floor_mask else args.proposer),
        segmenter=args.segmenter,
        crop_path=args.crop_path,
        max_crops=args.max_crops,
        imgsz=args.imgsz,
        vlm_max_pixels=args.vlm_max_pixels,
        vlm_ground=not args.no_vlm_boxes,
    )


def _config_of(args) -> dict:
    return {
        "detector": args.detector,
        # Recorded because a latency figure is meaningless without them, and
        # because an accuracy figure at reduced precision is a DIFFERENT
        # measurement from the fp32 one it will be compared against.
        "device": getattr(args, "device", None) or "cpu",
        "dtype": getattr(args, "dtype", None) or "fp32",
        "clip": args.clip,
        "proposer": "floor" if args.floor_mask else args.proposer,
        "crop_path": args.crop_path,
        "metric": args.metric,
        "max_crops": args.max_crops,
        "imgsz": args.imgsz,
        "vlm_max_pixels": args.vlm_max_pixels,
        "vlm_ground": not args.no_vlm_boxes,
    }


def _label_for(config: dict) -> str:
    proposer = config.get("proposer") or "none"
    bits = [str(config.get("detector"))]
    if proposer != "none":
        bits.append(proposer)
    return " + ".join(bits)


def cmd_score(args) -> int:
    from brain.perceive import PerceptionUnavailable

    root = Path(args.recordings).expanduser()
    walks = load_corpus(root, args.walk)
    total_frames = sum(len(w.frames) for w in walks)
    total_visible = sum(w.visible for w in walks)
    print(f"\ncorpus: {len(walks)} walk(s), {total_frames} labelled frames, "
          f"{total_visible} visible   ({root})")
    print(f"config: {_label_for(_config_of(args))}, crop path "
          f"{args.crop_path}, metric {args.metric}")
    print("reference: labels.json (adjudicated) -- never walk.jsonl\n")

    records = []
    for walk in walks:
        print(f"  {walk.name}  target {walk.target!r}  "
              f"{len(walk.frames)} frames, {walk.visible} visible")
        try:
            pipeline = _build_pipeline(args, walk.target)
        except PerceptionUnavailable as exc:
            return _fail(str(exc))

        def on_frame(r, _n=len(walk.frames)):
            if args.quiet:
                return
            mark = "*" if r.visible else " "
            score = "    -" if not r.scored else f"{r.score:5.2f}"
            print(f"    {mark} {r.frame:<18}{r.status:<12}"
                  f"{(r.label or '-')[:16]:<17}{score}  {r.ms:5.0f}ms")

        records += run_walk(pipeline, walk, args.metric, on_frame=on_frame)

    gates = ([float(g) for g in args.gates.split(",")] if args.gates
             else DEFAULT_GATES)
    t = totals(records)
    print(f"\n{format_per_walk(records, args.gate)}\n")
    print(f"{format_gate_table(records, gates)}\n")
    print(f"  unavailable {t['unavailable']}"
          f"{'   <-- frames that could not be read (1.12)' if t['unavailable'] else ''}")
    print(f"  {t['ms_per_frame']:.0f}ms per frame on this machine -- compares "
          f"models to each other,\n  never to the robot's budget (2.9).")

    if args.save:
        save_records(Path(args.save), records, _config_of(args))
        print(f"\n  records -> {args.save}  "
              f"(`compare` reads these without re-running the models)")
    print()
    return 0


def cmd_compare(args) -> int:
    named = []
    for spec in args.files:
        label, _, path = spec.rpartition("=")
        records, config = load_records(Path(path))
        named.append((label or _label_for(config), records))
    budgets = [int(b) for b in args.fp_budget.split(",")]
    print("\n  Recall at matched precision. A recall without its "
          "false-positive count is\n  not a result -- it is how an "
          "open-vocabulary detector run at low confidence\n  looks like a "
          "win (4.11).\n")
    print(format_comparison(named, budgets))
    first = named[0][1]
    print(f"\n  {len({r.walk for r in first})} walk(s), {len(first)} frames, "
          f"{sum(1 for r in first if r.visible)} visible.\n")
    return 0


def _fail(message: str) -> int:
    print(f"\n{message}\n")
    return 1


def main(argv=None) -> int:
    import argparse

    ap = argparse.ArgumentParser(
        prog="python -m control.perception_eval",
        description="Score the on-board perception tier over a corpus of "
                    "adjudicated rig walks (P3).")
    sub = ap.add_subparsers(dest="command")

    score = sub.add_parser("score", help="run the models over the corpus")
    _add_score_args(score)
    score.set_defaults(func=cmd_score)

    compare = sub.add_parser(
        "compare", help="read saved records and match them on false positives")
    compare.add_argument("files", nargs="+",
                         help="record files from `score --save`, optionally "
                              "labelled as label=path")
    compare.add_argument("--fp-budget", default="0,3,16",
                         help="comma-separated false-positive budgets "
                              "(default 0,3,16 -- 3 and 16 are 4.11's own "
                              "operating points)")
    compare.set_defaults(func=cmd_compare)

    args = ap.parse_args(argv)
    if not getattr(args, "func", None):
        ap.print_help()
        return 2
    try:
        return args.func(args)
    except CorpusError as exc:
        return _fail(str(exc))


__all__ = [
    "FrameScore", "Walk", "CorpusError",
    "load_walk", "load_corpus", "frame_dict",
    "best_score", "score_at", "sweep", "by_walk", "totals",
    "recall_at_fp_budget", "run_walk", "decisions_by_frame",
    "save_records", "load_records",
    "format_gate_table", "format_per_walk", "format_comparison",
    "DEFAULT_GATES", "METRICS", "METRIC_PROBABILITY", "METRIC_MARGIN",
    "METRIC_CONFIDENCE", "MIN_USABLE_MARGIN",
]


if __name__ == "__main__":
    raise SystemExit(main())
