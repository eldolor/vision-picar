"""3.46 amendment 5: CLIP as the second opinion on every object box.

    python -m tools.inventory_clip score      # every object box, once (free, local)
    python -m tools.inventory_clip verify     # VERIFY on the tuning frames, against the judge

The car already runs "detect, then verify" for its target (`brain/perceive.py`):
a detector proposes, CLIP RN50 checks the crop. Here the same CLIP scores
every object box YOLOE-pf drew against ALL 1,691 object names
(`recordings/inventory_eval/vocab.json`), as "a photo of a {name}", and
keeps per box:

* `p_label` -- CLIP's probability for the detector's own name (softmax
  over the 1,691 at `CLIP_LOGIT_SCALE`, the shipped scale), and its rank;
* `top` -- CLIP's own best five names and their probabilities.

Two ways to use it, measured separately:

* **verify** keeps a box under the DETECTOR's name only when CLIP agrees.
  The judge already ruled on every detector name, so this is measured on
  the existing verdicts -- no new cloud call.
* **relabel** names the box with CLIP's top name instead. Those names were
  never judged, so measuring it is a new judge run (amendment 5's cost).
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from tools.inventory_label import (JUDGE_FLOOR, OUT, ROOT, _correct, choose_threshold,
                                     load_vocab, object_boxes, precision_at)

PROMPT = "a photo of a {}"
TOP = 5
BATCH = 64


def cmd_score(args) -> None:
    import torch
    from PIL import Image

    from brain.perceive import CLIP_LOGIT_SCALE, ClipScorer

    vocab = load_vocab()
    names = sorted(n for n, ok in vocab.items() if ok)
    index = {n: i for i, n in enumerate(names)}
    dets = json.loads((OUT / "detections.json").read_text())
    clip = ClipScorer()
    text = clip._text_features(tuple(PROMPT.format(n) for n in names))
    out_path = OUT / "clip.json"
    done = json.loads(out_path.read_text()) if out_path.exists() else {}
    jobs = [(fid, b) for fid in sorted(dets) if fid not in done
            for b in object_boxes(dets[fid], vocab)]
    t0 = time.perf_counter()
    for start in range(0, len(jobs), BATCH):
        part = jobs[start:start + BATCH]
        crops = []
        for fid, b in part:
            img = Image.open(ROOT / fid).convert("RGB")
            x1, y1, x2, y2 = (int(v) for v in b["xyxy"])
            crops.append(clip.preprocess(img.crop((x1, y1, max(x2, x1 + 1), max(y2, y1 + 1)))))
        with torch.no_grad():
            feats = clip.model.encode_image(torch.stack(crops).to(clip.device))
            feats /= feats.norm(dim=-1, keepdim=True)
            probs = (CLIP_LOGIT_SCALE * feats @ text.T).softmax(dim=-1).cpu()
        for (fid, b), p in zip(part, probs):
            mine = index[b["label"]]
            order = p.argsort(descending=True)
            rank = int((order == mine).nonzero()[0][0]) + 1
            done.setdefault(fid, {})[str(b["n"])] = {
                "p_label": round(float(p[mine]), 4), "rank": rank,
                "top": [[names[int(i)], round(float(p[i]), 4)] for i in order[:TOP]]}
        if (start // BATCH) % 20 == 0:
            out_path.write_text(json.dumps(done))
            rate = (time.perf_counter() - t0) / (start + len(part)) * 1000
            print(f"{start + len(part)}/{len(jobs)} boxes, {rate:.1f} ms each", file=sys.stderr)
    out_path.write_text(json.dumps(done))
    print(f"{sum(len(v) for v in done.values())} boxes -> {out_path}")


def tuning_rows(dets, vocab, judged, clip, sample):
    """(box, CLIP's view, the judge's verdict) for every judged object box
    on the frames OUTSIDE the user's sample."""
    for fid, j in judged.items():
        if fid in sample:
            continue
        for b in object_boxes(dets[fid], vocab):
            c = (clip.get(fid) or {}).get(str(b["n"]))
            ok = _correct(j["verdicts"].get(str(b["n"])))
            if c is not None and ok is not None:
                yield b, c, ok


def cmd_verify(args) -> None:
    vocab = load_vocab()
    dets = json.loads((OUT / "detections.json").read_text())
    judged = json.loads((OUT / "judge.json").read_text())
    clip = json.loads((OUT / "clip.json").read_text())
    sample = set(json.loads((OUT / "sample.json").read_text()))
    rows = list(tuning_rows(dets, vocab, judged, clip, sample))
    right = sum(ok for _, _, ok in rows)
    print(f"{len(rows)} judged boxes on the tuning frames, {right} right "
          f"({right / len(rows):.1%})")
    for k in (1, 2, 3, 5):
        kept = [ok for _, c, ok in rows if c["rank"] <= k]
        print(f"  CLIP ranks the detector's name in its top {k}: precision "
              f"{sum(kept) / len(kept):.1%} over {len(kept)} boxes, keeps "
              f"{sum(kept) / right:.0%} of the right ones")
    # The gate as a threshold on CLIP's probability for the detector's name,
    # chosen by amendment 3's rule (lowest value reaching 75% on the judge).
    pr = [(c["p_label"], ok) for _, c, ok in rows]
    g = choose_threshold(pr)
    if g is not None:
        p, n = precision_at(pr, g)
        kept = sum(ok for c, ok in pr if c >= g)
        print(f"  gate p_label >= {g}: precision {p:.1%} over {n} boxes, keeps "
              f"{kept / right:.0%} of the right ones")
    else:
        print("  no p_label gate reaches 75% on the judge")


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(prog="python -m tools.inventory_clip")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("score")
    sub.add_parser("verify")
    args = ap.parse_args(argv)
    {"score": cmd_score, "verify": cmd_verify}[args.cmd](args)


if __name__ == "__main__":
    main()
