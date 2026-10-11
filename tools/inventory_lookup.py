"""3.46 amendment 12: remember what objects look like, then look them up.

    python -m tools.inventory_lookup embed    # CLIP RN50 on every stored crop (free, local)
    python -m tools.inventory_lookup score    # criterion 1: frame recall, perception_eval records
    python -m tools.inventory_lookup lookup   # criteria 2-3: rank a house's crops by P(target)

The stored objects are the prompt-free YOLOE boxes already saved for every
frame (`recordings/inventory_eval/detections.json`), on amendment 11's six
walks. A request is matched against them with the shipped rule
(`brain/perceive.py`): softmax over the target and `DEFAULT_DISTRACTORS`
at `CLIP_LOGIT_SCALE`, gate P >= 0.8.

Two box sets, fixed by the amendment before measuring:

* `objects` (the primary): `inventory_label.object_boxes` at conf >= 0.05,
  the 20 most confident -- names the vocabulary marks as objects only;
* `all` (reported, not judged): every box, the 20 most confident.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from control.perception_eval import FrameScore, load_walk, save_records
from tools.inventory_label import DETECT_CONF, MAX_BOXES, OUT, ROOT, load_vocab, object_boxes

WALKS = Path("evaluations/inventory-346/search-walks.json")
EMBED = OUT / "lookup_embeddings.npz"
METRIC = "memory_probability"
VARIANTS = ("objects", "all")
BATCH = 64


def stored_boxes(det: dict, vocab: dict, variant: str) -> list:
    """The boxes a memory keeps for one frame, under one variant."""
    if variant == "objects":
        return object_boxes(det, vocab, floor=DETECT_CONF)
    boxes = sorted((b for b in det["boxes"] if b["conf"] >= DETECT_CONF),
                   key=lambda b: -b["conf"])
    return boxes[:MAX_BOXES]


def walks() -> list:
    return [load_walk(ROOT / w) for w in dict.fromkeys(json.loads(WALKS.read_text()))]


def cmd_embed(args) -> None:
    import numpy as np
    import torch
    from PIL import Image

    from brain.perceive import ClipScorer

    vocab = load_vocab()
    dets = json.loads((OUT / "detections.json").read_text())
    # One crop per distinct box, tagged with the variants that keep it.
    meta, index = [], {}
    for walk in walks():
        for path, visible, _ in walk.frames:
            fid = f"{walk.name}/{path.name}"
            if fid not in dets:
                sys.exit(f"{fid} has no saved prompt-free boxes; run inventory_label detect")
            for variant in VARIANTS:
                for b in stored_boxes(dets[fid], vocab, variant):
                    key = (fid, tuple(b["xyxy"]))
                    if key not in index:
                        index[key] = len(meta)
                        meta.append({"walk": walk.name, "frame": path.name, "visible": visible,
                                     "label": b["label"], "conf": b["conf"],
                                     "xyxy": b["xyxy"], "variants": []})
                    meta[index[key]]["variants"].append(variant)
    clip = ClipScorer()
    feats, t0 = [], time.perf_counter()
    for start in range(0, len(meta), BATCH):
        crops = []
        for m in meta[start:start + BATCH]:
            img = Image.open(ROOT / m["walk"] / m["frame"]).convert("RGB")
            x1, y1, x2, y2 = (int(v) for v in m["xyxy"])
            crops.append(clip.preprocess(img.crop((x1, y1, max(x2, x1 + 1), max(y2, y1 + 1)))))
        with torch.no_grad():
            f = clip.model.encode_image(torch.stack(crops).to(clip.device))
            f /= f.norm(dim=-1, keepdim=True)
        feats.append(f.float().cpu().numpy())
        if (start // BATCH) % 20 == 0:
            print(f"{start + len(crops)}/{len(meta)} crops", file=sys.stderr)
    np.savez_compressed(EMBED, feats=np.concatenate(feats).astype(np.float32),
                        meta=json.dumps(meta))
    print(f"{len(meta)} crops in {time.perf_counter() - t0:.0f} s -> {EMBED}")


def load_embeddings():
    import numpy as np
    z = np.load(EMBED)
    return z["feats"], json.loads(str(z["meta"]))


def text_features(texts):
    from brain.perceive import ClipScorer
    clip = ClipScorer(device="cpu")
    return clip._text_features(tuple(texts)).float().numpy()


def probabilities(feats, text):
    """P(target | crop) for every row: the shipped softmax, target first."""
    import numpy as np

    from brain.perceive import CLIP_LOGIT_SCALE
    x = CLIP_LOGIT_SCALE * feats @ text.T
    x -= x.max(axis=1, keepdims=True)
    e = np.exp(x)
    return e[:, 0] / e.sum(axis=1)


def cmd_score(args) -> None:
    from brain.perceive import ABSENT, DEFAULT_DISTRACTORS, DEFAULT_MATCH_PROBABILITY, DETECTED

    feats, meta = load_embeddings()
    for variant in VARIANTS:
        records = []
        for walk in walks():
            text = text_features((walk.target,) + tuple(DEFAULT_DISTRACTORS))
            rows = [i for i, m in enumerate(meta)
                    if m["walk"] == walk.name and variant in m["variants"]]
            p = probabilities(feats[rows], text) if rows else []
            best: dict = {}
            for i, pi in zip(rows, p):
                f = meta[i]["frame"]
                if f not in best or pi > best[f][0]:
                    best[f] = (float(pi), meta[i]["label"])
            counts = {}
            for i in rows:
                counts[meta[i]["frame"]] = counts.get(meta[i]["frame"], 0) + 1
            for path, visible, adjudicated in walk.frames:
                score, label = best.get(path.name, (None, None))
                records.append(FrameScore(
                    walk=walk.name, frame=path.name, visible=visible,
                    status=DETECTED if score is not None and score >= DEFAULT_MATCH_PROBABILITY
                    else ABSENT,
                    score=score, metric=METRIC, label=label,
                    candidates=counts.get(path.name, 0), adjudicated=adjudicated))
        out = Path(f"evaluations/inventory-346/memory-{variant}.json")
        save_records(out, records, {"detector": "yoloe-11s-seg-pf (saved boxes)",
                                    "clip": "RN50", "variant": variant,
                                    "conf": DETECT_CONF, "cap": MAX_BOXES, "metric": METRIC})
        hit = sum(r.visible and r.status == DETECTED for r in records)
        fa = sum(not r.visible and r.status == DETECTED for r in records)
        vis = sum(r.visible for r in records)
        print(f"{variant}: P >= 0.8 finds {hit}/{vis} = {hit / vis:.1%} with {fa} false alarms "
              f"-> {out}")


def cmd_lookup(args) -> None:
    import numpy as np
    from PIL import Image

    from brain.perceive import DEFAULT_DISTRACTORS

    feats, meta = load_embeddings()
    rows = [i for i, m in enumerate(meta) if "objects" in m["variants"]]
    house = np.ascontiguousarray(feats[rows])
    out_dir = Path(args.crops)
    out_dir.mkdir(parents=True, exist_ok=True)
    report = []
    for walk in walks():
        text = text_features((walk.target,) + tuple(DEFAULT_DISTRACTORS))
        t = time.perf_counter()
        p = probabilities(house, text)
        top = np.argsort(-p)[:args.k]
        ms = (time.perf_counter() - t) * 1000
        hits = []
        for rank, j in enumerate(top, 1):
            m = meta[rows[j]]
            own = m["walk"] == walk.name
            # The walk name keeps two walks with one target apart.
            name = f"{walk.name}-{rank}.jpg"
            img = Image.open(ROOT / m["walk"] / m["frame"]).convert("RGB")
            x1, y1, x2, y2 = m["xyxy"]
            pad_x, pad_y = (x2 - x1) * 0.25, (y2 - y1) * 0.25
            img.crop((max(0, x1 - pad_x), max(0, y1 - pad_y),
                      min(img.width, x2 + pad_x), min(img.height, y2 + pad_y))).save(out_dir / name)
            hits.append({"rank": rank, "p": round(float(p[j]), 4), "walk": m["walk"],
                         "frame": m["frame"], "own_walk": own,
                         "labelled_visible": m["visible"] if own else None,
                         "yoloe_label": m["label"], "xyxy": m["xyxy"], "crop": name})
        report.append({"target": walk.target, "walk": walk.name, "crops": len(rows),
                       "lookup_ms": round(ms, 2), "top": hits})
        print(f"{walk.target!r}: {ms:.1f} ms over {len(rows)} crops; top "
              + ", ".join(f"{h['p']:.2f}{'' if h['own_walk'] else '*'}"
                          f"{'+' if h['labelled_visible'] else ''}" for h in hits))
    Path(args.save).write_text(json.dumps(report, indent=1))
    print(f"-> {args.save}; crops for the by-eye check in {out_dir}")


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(prog="python -m tools.inventory_lookup")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("embed").set_defaults(fn=cmd_embed)
    sub.add_parser("score").set_defaults(fn=cmd_score)
    lk = sub.add_parser("lookup")
    lk.add_argument("--k", type=int, default=5)
    lk.add_argument("--crops", required=True, help="where to write the top crops")
    lk.add_argument("--save", default="evaluations/inventory-346/lookup-top5.json")
    lk.set_defaults(fn=cmd_lookup)
    args = ap.parse_args(argv)
    args.fn(args)


if __name__ == "__main__":
    main()
