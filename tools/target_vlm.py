"""3.46 amendment 11: a vision model searching for one named object.

    python -m tools.target_vlm --model gemma-4-E2B-it.litertlm \\
        --walks search-walks.json --save search-gemma4e2b.json

Asks Gemma 4 (LiteRT-LM, `tools.inventory_vlm.LiteRTNamer`) whether each
labelled frame of the given walks shows that walk's target, using the same
description `control/perception_eval` gives YOLOE + CLIP. A frame is a
sighting when the answer holds at least one parsable box. The records are
`perception_eval.FrameScore`s with a score of 1 or 0, so

    python -m control.perception_eval compare search-shipped.json \\
        search-gemma4e2b.json --fp-budget 0,3

reads both. Resumable: frames already in `--save` are kept.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import asdict
from pathlib import Path

from brain.perceive import ABSENT, DETECTED
from control.perception_eval import FrameScore, load_walk
from tools.inventory_label import ROOT  # RECORDINGS_DIR, as LiteRTNamer reads it
from tools.inventory_vlm import LiteRTNamer

PROMPT = (
    "Look for this object in the photo: {target}. If it is there, output JSON with one "
    "entry for each one you see: [{{\"box_2d\": [y1, x1, y2, x2], \"label\": \"{target}\"}}]. "
    "If there is none, output [].")

METRIC = "vlm_sighting"


def outcome(raw: str, boxes: list) -> str:
    """How the answer read: `boxes`, `empty` (a JSON list with no usable
    box) or `unparsed` (prose, or no JSON list at all). Amendment 11
    counts the last, which scores as no sighting."""
    if boxes:
        return "boxes"
    text = raw.strip().removeprefix("```json").removeprefix("```").removesuffix("```").strip()
    try:
        return "empty" if isinstance(json.loads(text), list) else "unparsed"
    except json.JSONDecodeError:
        return "unparsed"


def prompt_for(target: str) -> str:
    return PROMPT.format(target=target)


def record(walk: str, frame: str, visible: bool, adjudicated: bool,
           boxes: list, ms: float) -> FrameScore:
    """One frame's answer as a scorer record: 1.0 a sighting, 0.0 none."""
    return FrameScore(walk=walk, frame=frame, visible=visible,
                      status=DETECTED if boxes else ABSENT,
                      score=1.0 if boxes else 0.0, metric=METRIC,
                      label=boxes[0]["label"] if boxes else None,
                      candidates=len(boxes), adjudicated=adjudicated, ms=ms)


def _write(path: Path, doc: dict) -> None:
    """Atomically: a run is hours long, and a torn file loses all of it."""
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(doc, indent=1))
    os.replace(tmp, path)


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(prog="python -m tools.target_vlm")
    ap.add_argument("--model", required=True, help="the .litertlm file")
    ap.add_argument("--device", choices=("gpu", "cpu"), default="gpu")
    ap.add_argument("--walks", required=True, help="a JSON list of walk names")
    ap.add_argument("--save", required=True)
    args = ap.parse_args(argv)

    config = {"detector": f"litert:{os.path.basename(args.model)}", "device": args.device,
              "prompt": PROMPT, "metric": METRIC}
    save = Path(args.save)
    # `raw` sits beside `records`, not in them: `load_records` rebuilds
    # each record as a FrameScore and would refuse an extra key.
    doc = (json.loads(save.read_text()) if save.exists()
           else {"config": config, "records": [], "raw": {}, "outcome": {}})
    if doc["config"] != config:
        sys.exit(f"{save} holds another run ({doc['config'].get('detector')}); use a new --save")
    done = {(r["walk"], r["frame"]) for r in doc["records"]}
    walks = [load_walk(ROOT / w) for w in dict.fromkeys(json.loads(open(args.walks).read()))]
    namer, errors = None, 0
    for walk in walks:
        todo = [f for f in walk.frames if (walk.name, f[0].name) not in done]
        if not todo:
            continue
        if namer is None:
            namer = LiteRTNamer(args.model, args.device)
        namer.prompt = prompt_for(walk.target)
        print(f"{walk.name}: {len(todo)} frames, target {walk.target!r}", file=sys.stderr)
        for k, (path, visible, adjudicated) in enumerate(todo, 1):
            t = time.perf_counter()
            try:
                got = namer(f"{walk.name}/{path.name}")
            except Exception as exc:  # noqa: BLE001 -- not saved, so a rerun retries it
                errors += 1
                print(f"{walk.name}/{path.name}: {exc}", file=sys.stderr)
                continue
            ms = (time.perf_counter() - t) * 1000
            r = record(walk.name, path.name, visible, adjudicated, got["boxes"], ms)
            doc["records"].append(asdict(r))
            doc["raw"][f"{walk.name}/{path.name}"] = got["raw"][:500]
            doc["outcome"][f"{walk.name}/{path.name}"] = outcome(got["raw"], got["boxes"])
            print(f"{time.strftime('%T')} {walk.name}/{path.name} {ms / 1000:.1f} s "
                  f"visible={visible} boxes={len(got['boxes'])}", file=sys.stderr, flush=True)
            if k % 10 == 0 or k == len(todo):
                _write(save, doc)
    _write(save, doc)
    labelled = sum(len(w.frames) for w in walks)
    counts = {k: list(doc["outcome"].values()).count(k) for k in ("boxes", "empty", "unparsed")}
    print(f"{len(doc['records'])} of {labelled} labelled frames -> {save}; answers {counts}; "
          f"{errors} failed calls (not saved)")
    if len(doc["records"]) < labelled:
        sys.exit(f"{labelled - len(doc['records'])} frames missing: rerun to retry them "
                 "before comparing, or the comparison is on fewer frames than the shipped run")


if __name__ == "__main__":
    main()
