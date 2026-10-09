"""
identity_eval.py

3.48 (`docs/plans/ros-alignment/3.48-local-identity.md`): **could a local
model answer the arrival question -- "is the target in this frame?" -- in
place of the cloud?** Scored from records `perception_eval` already saved,
against today's `labels.json`, beside the cloud's own recorded answers.

    python -m control.identity_eval \\
        --walk blue-bottle-20260907-142454 --walk blue-bottle-20260907-185007 \\
        --walk blue-shoes-20260907-152528 --walk red-backpack-20260907-144856 \\
        --records qwen25-3b=evaluations/gpu/a10g-qwen25-3b-8walk.json:0.5 \\
        --records gdino-tiny=evaluations/gpu/a10g-gdino-8walk.json:0.35

The arrival question costs most when the answer is a wrong YES: it turns a
local false positive into `found` on the wrong object. So every row leads
with precision at a FIXED operating point (written into the plan before the
run), counts YES on the frames where the cloud confabulated (the teal bin,
P3), and reports `@ 0 FP` only beside it, as a fitted number.

What this adds to `perception_eval` and nothing more:

* **Relabelling.** A stored record carries the `visible` flag of the day it
  was scored; labels have been corrected since (P7's basket span). Today's
  `labels.json` wins, and the number of flips is reported.
* **The cloud as a row.** Opus's `navigate.target_visible` from each frame's
  `walk.jsonl` becomes a record (1.0 / 0.0), so the cloud is scored by the
  same `score_at` as every local model. That is evidence about the model,
  never the reference (`perception_eval`'s first rule).
* **The confabulation subset:** frames the cloud called visible that the
  labels say are not.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import replace
from pathlib import Path
from typing import Optional, Sequence

from brain.perceive import ABSENT, DETECTED
from control.perception_eval import (
    CorpusError, FrameScore, load_corpus, load_records, recall_at_fp_budget,
    score_at)

CLOUD_GATE = 0.5


def frame_key(walk: str, frame: str) -> str:
    return f"{walk}/{frame}"


def current_labels(walks) -> dict:
    """`{walk/frame: visible}` from today's labels.json, for `walks` as
    `perception_eval.load_corpus` returns them."""
    return {frame_key(w.name, p.name): vis for w in walks for p, vis, _ in w.frames}


def relabel(records: Sequence[FrameScore], labels: dict) -> tuple:
    """Keep the records whose frame is in `labels`, with `visible` taken
    from `labels`. Returns `(records, flips, missing)`: how many stored
    flags today's labels overturned, and how many labelled frames the
    records do not cover (a coverage gap, which would otherwise read as a
    smaller corpus rather than a missing one)."""
    kept, flips, seen = [], 0, set()
    for r in records:
        key = frame_key(r.walk, r.frame)
        if key not in labels:
            continue
        seen.add(key)
        if r.visible != labels[key]:
            flips += 1
        kept.append(replace(r, visible=labels[key]))
    return kept, flips, len(set(labels) - seen)


def cloud_records(walks) -> list:
    """The cloud's recorded answer on every labelled frame, as records.

    A frame whose row carries no `navigate.target_visible` is unscored
    (score None), so it can never count as a YES or a NO."""
    out = []
    for w in walks:
        answers, models = {}, set()
        path = w.path / "walk.jsonl"
        if path.is_file():
            for line in path.read_text().splitlines():
                if not line.strip():
                    continue
                row = json.loads(line)
                nav = row.get("navigate") or {}
                name = row.get("file")
                if name is None or not isinstance(nav.get("target_visible"), bool):
                    continue
                answers[name] = nav["target_visible"]
                if nav.get("model_id"):
                    models.add(nav["model_id"])
        for p, vis, adj in w.frames:
            said = answers.get(p.name)
            out.append(FrameScore(
                walk=w.name, frame=p.name, visible=vis,
                status=DETECTED if said else ABSENT,
                score=None if said is None else (1.0 if said else 0.0),
                metric="cloud_target_visible",
                label=",".join(sorted(models)) or None,
                adjudicated=adj))
    return out


def confabulations(cloud: Sequence[FrameScore]) -> set:
    """Frames the cloud called visible that the labels say are not."""
    return {frame_key(r.walk, r.frame) for r in cloud
            if r.scored and r.score >= CLOUD_GATE and not r.visible}


def row(name: str, records: Sequence[FrameScore], gate: float,
        confab: set) -> dict:
    """One table row: the fixed operating point, YES on the confabulation
    subset, and the fitted zero-false-positive point."""
    at = score_at(records, gate)
    fitted = recall_at_fp_budget(records, 0)
    yes_on_confab = sum(1 for r in records
                        if frame_key(r.walk, r.frame) in confab
                        and r.scored and r.score >= gate)
    return {
        "name": name, "gate": gate, "frames": at["frames"], "visible": at["visible"],
        "tp": at["tp"], "fp": at["fp"], "precision": at["precision"],
        "recall": at["recall"],
        "yes_on_confabulations": yes_on_confab, "confabulation_frames": len(confab),
        "recall_at_0fp_fitted": fitted["recall"], "separable_at_0fp": fitted["separable"],
    }


def _pct(x: Optional[float]) -> str:
    return "--" if x is None else f"{100 * x:.1f}%"


def format_rows(rows: Sequence[dict]) -> str:
    lines = ["| model | gate | precision (TP/YES) | recall (TP/visible) | YES on cloud confabulations | @ 0 FP (fitted) |",
             "|---|---|---|---|---|---|"]
    for r in rows:
        sep = "" if r["separable_at_0fp"] else " (not separable)"
        lines.append(
            f"| {r['name']} | {r['gate']} | {_pct(r['precision'])} ({r['tp']}/{r['tp'] + r['fp']}) "
            f"| {_pct(r['recall'])} ({r['tp']}/{r['visible']}) "
            f"| {r['yes_on_confabulations']}/{r['confabulation_frames']} "
            f"| {_pct(r['recall_at_0fp_fitted'])}{sep} |")
    return "\n".join(lines)


def _parse_records_arg(text: str) -> tuple:
    name, _, rest = text.partition("=")
    path, _, gate = rest.rpartition(":")
    if not name or not path or not gate:
        raise argparse.ArgumentTypeError(f"--records wants NAME=PATH:GATE, got {text!r}")
    return name, Path(path), float(gate)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[1])
    ap.add_argument("--recordings", default="recordings")
    ap.add_argument("--walk", action="append", default=[], required=True,
                    help="pin the frame set; repeatable")
    ap.add_argument("--records", action="append", default=[], type=_parse_records_arg,
                    help="NAME=PATH:GATE -- a saved perception_eval run and its fixed gate")
    ap.add_argument("--json", default=None, help="also write the rows here")
    args = ap.parse_args(argv)
    try:
        walks = load_corpus(Path(args.recordings), only=args.walk)
    except CorpusError as e:
        print(f"error: {e}")
        return 2
    labels = current_labels(walks)
    cloud = cloud_records(walks)
    confab = confabulations(cloud)
    models = sorted({r.label for r in cloud if r.label})
    print(f"frames: {len(labels)} in {len(walks)} walks, "
          f"{sum(labels.values())} visible (today's labels.json)")
    rows = [row(f"cloud ({', '.join(models) or 'unknown'})", cloud, CLOUD_GATE, confab)]
    for name, path, gate in args.records:
        records, config = load_records(path)
        records, flips, missing = relabel(records, labels)
        print(f"{name}: {config.get('detector')} on {config.get('device')} "
              f"{config.get('dtype')} -- {len(records)} frames, {flips} labels "
              f"flipped since scoring, {missing} labelled frames not covered")
        rows.append(row(name, records, gate, confab))
    print()
    print(format_rows(rows))
    if args.json:
        Path(args.json).write_text(json.dumps(rows, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
