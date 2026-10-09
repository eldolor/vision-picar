"""
tools/clip_rooms_fewshot.py

3.54 (`docs/plans/ros-alignment/3.54-clip-rooms-fewshot.md`): **few-shot
room recognition with the CLIP RN50 the tiered policy already loads.** A
frame goes to the room whose K labelled example frames, from OTHER walks,
it looks most like (nearest centroid on whole-frame image embeddings).
Measurement only: nothing here is on the mission path.

    python -m tools.clip_rooms_fewshot embed --root recordings --device cpu --out emb.npz
    python -m tools.clip_rooms_fewshot predict emb.npz --out docs/evaluations/clip-rooms-3.54.json
    python -m tools.clip_rooms_fewshot report docs/evaluations/clip-rooms-3.54.json

`embed` runs CLIP once per frame and times it; `predict` applies the
protocol fixed in the plan before any embedding existed (leave-one-walk-out,
K evenly spaced references per room, nearest centroid) and writes the
per-frame predictions, which are committed; `report` is arithmetic over
that file, so the numbers reproduce without CLIP.
"""

import argparse
import json
import time
from pathlib import Path
from typing import Optional, Sequence

from brain.tiered import percentiles
from tools.clip_rooms_eval import (
    COMMITTED_LABELS, TEMPLATES, UNSURE, WALKS as WALKS_351, WARMUP,
    _CLOUD_SYNONYMS, _sync, accuracy, macro_accuracy, per_room, smooth,
    switches_per_100)

LABELS_354 = (Path(__file__).resolve().parent.parent
              / "docs" / "evaluations" / "room-labels-3.54.json")
# The rooms with >= 30 decided frames across >= 2 walks (3.54's survey).
ROOMS = ("living room", "basement", "home gym")
K_PRIMARY = 10
K_REPORTED = (3, 10, 30)
SMOOTH_N = 5
# 3.51's synonyms, plus the two rooms 3.51's walks never reached. Fixed in
# the plan before measuring.
CLOUD_SYNONYMS = {
    **_CLOUD_SYNONYMS,
    "home gym": "home gym", "gym": "home gym", "exercise room": "home gym",
    "workout room": "home gym",
    "basement": "basement", "basement or recreation room": "basement",
    "recreation room": "basement", "rec room": "basement",
}


def cloud_room(text: Optional[str]) -> Optional[str]:
    if not isinstance(text, str):
        return None
    return CLOUD_SYNONYMS.get(text.strip().lower())


def load_labels() -> dict:
    """{walk: {frame: room}} for every walk in the protocol."""
    out = {w: v for w, v in json.loads(COMMITTED_LABELS.read_text())["walks"].items()
           if w in WALKS_351}
    out.update(json.loads(LABELS_354.read_text())["walks"])
    return out


def pick_evenly(items: Sequence, k: int) -> list:
    """K items at evenly spaced positions, ends included; all if fewer."""
    n = len(items)
    if n <= k:
        return list(items)
    if k == 1:
        return [items[0]]
    return [items[round(i * (n - 1) / (k - 1))] for i in range(k)]


def centroid(vectors):
    import numpy as np
    c = np.mean(vectors, axis=0)
    return c / np.linalg.norm(c)


def leave_one_walk_out(rows: Sequence[dict], emb, k: int,
                       rooms: Sequence[str] = ROOMS) -> list:
    """Predicted room per row. `rows[i]` is {walk, frame, label}; `emb[i]` its
    L2-normalised embedding. The walk under test never supplies a reference;
    a room with no reference outside it is not predicted for that walk."""
    import numpy as np
    preds = [None] * len(rows)
    for walk in sorted({r["walk"] for r in rows}):
        cents, names = [], []
        for room in rooms:
            pool = [i for i, r in enumerate(rows)
                    if r["walk"] != walk and r["label"] == room]
            if pool:
                cents.append(centroid(emb[pick_evenly(pool, k)]))
                names.append(room)
        mat = np.stack(cents)
        for i, r in enumerate(rows):
            if r["walk"] == walk:
                preds[i] = names[int(np.argmax(mat @ emb[i]))]
    return preds


def embed(root: Path, device: Optional[str], out: Path) -> None:  # pragma: no cover - needs CLIP
    import numpy as np
    import torch
    from PIL import Image

    from brain.perceive import ClipScorer

    scorer = ClipScorer(device=device)
    text = torch.stack([scorer._text_features(tuple(t.format(room=r) for r in ROOMS))
                        for t in TEMPLATES]).mean(dim=0)
    text = text / text.norm(dim=-1, keepdim=True)

    def encode(img):
        with torch.no_grad():
            f = scorer.model.encode_image(scorer.preprocess(img).unsqueeze(0).to(scorer.device))
            f = f / f.norm(dim=-1, keepdim=True)
        _sync(torch, scorer.device)
        return f

    rows, vecs, enc_ms, cls_ms = [], [], [], []
    warm = False
    for walk, labels in sorted(load_labels().items()):
        cloud = {}
        for line in (root / walk / "walk.jsonl").read_text().splitlines():
            if line.strip():
                row = json.loads(line)
                cloud[row["file"]] = (row.get("navigate") or {}).get("room_guess")
        for frame in sorted(labels):
            img = Image.open(root / walk / frame).convert("RGB")
            if not warm:
                for _ in range(WARMUP):
                    encode(img)
                warm = True
            t0 = time.perf_counter()
            f = encode(img)
            enc_ms.append((time.perf_counter() - t0) * 1000)
            v = f[0].float().cpu().numpy()
            # The classify step on its own: one dot product per room.
            cents = np.stack([v] * len(ROOMS))
            t0 = time.perf_counter()
            int(np.argmax(cents @ v))
            cls_ms.append((time.perf_counter() - t0) * 1000)
            zero = ROOMS[int((f @ text.T).argmax())]
            rows.append({"walk": walk, "frame": frame, "label": labels[frame],
                         "cloud_raw": cloud.get(frame), "zero_shot": zero})
            vecs.append(v)
    np.savez_compressed(out, emb=np.stack(vecs), meta=json.dumps({
        "device": scorer.device, "model": scorer.model_name, "rows": rows,
        "encode_ms": enc_ms, "classify_ms": cls_ms}))
    p = percentiles(enc_ms)
    print(f"{len(rows)} frames on {scorer.device}: encode p50 {p['p50']} ms, p90 {p['p90']} ms")


def predict(emb_path: Path, out: Path) -> None:  # pragma: no cover - needs the embeddings
    import numpy as np
    data = np.load(emb_path)
    meta = json.loads(str(data["meta"]))
    rows, emb = meta["rows"], data["emb"]
    test = [i for i, r in enumerate(rows) if r["label"] in ROOMS]
    scored = [rows[i] for i in test]
    preds = {k: leave_one_walk_out(scored, emb[test], k) for k in K_REPORTED}
    result = [{**r, **{f"k{k}": preds[k][j] for k in K_REPORTED}}
              for j, r in enumerate(scored)]
    out.write_text(json.dumps({
        "device": meta["device"], "model": meta["model"], "rooms": list(ROOMS),
        "encode_ms": [round(t, 2) for t in meta["encode_ms"]],
        "classify_ms": [round(t, 4) for t in meta["classify_ms"]],
        "rows": result}, indent=1))
    print(f"{len(result)} test frames written to {out}")


def report(path: Path) -> dict:
    data = json.loads(path.read_text())
    rows = data["rows"]
    out = {"frames": len(rows), "walks": len({r["walk"] for r in rows})}
    for key in [f"k{k}" for k in K_REPORTED] + ["zero_shot"]:
        pairs = [(r["label"], r[key]) for r in rows]
        out[key] = {"accuracy": accuracy(pairs), "macro": macro_accuracy(pairs),
                    "per_room": per_room(pairs)}
    key = f"k{K_PRIMARY}"
    # Frames with a recorded room_guess; one that maps to no class counts as
    # wrong, as in 3.51. The mapped-only subset is reported beside it.
    with_cloud = [r for r in rows if r["cloud_raw"] is not None]
    mapped = [r for r in with_cloud if cloud_room(r["cloud_raw"]) is not None]
    out["cloud"] = {
        "frames": len(with_cloud),
        "cloud_accuracy": accuracy([(r["label"], cloud_room(r["cloud_raw"]))
                                    for r in with_cloud]),
        "fewshot_accuracy": accuracy([(r["label"], r[key]) for r in with_cloud]),
        "mapped_frames": len(mapped),
        "cloud_accuracy_mapped": accuracy([(r["label"], cloud_room(r["cloud_raw"]))
                                           for r in mapped]),
        "fewshot_accuracy_mapped": accuracy([(r["label"], r[key]) for r in mapped]),
    }
    # Smoothing runs within a walk, in frame order: a vote never spans walks.
    raw, voted = [], []
    sw_raw = sw_voted = 0.0
    walks = sorted({r["walk"] for r in rows})
    for w in walks:
        seq = sorted((r for r in rows if r["walk"] == w), key=lambda r: r["frame"])
        p = [r[key] for r in seq]
        s = smooth(p, SMOOTH_N)
        raw += [(r["label"], x) for r, x in zip(seq, p)]
        voted += [(r["label"], x) for r, x in zip(seq, s)]
        sw_raw += switches_per_100(p) * len(p)
        sw_voted += switches_per_100(s) * len(s)
    n = len(rows)
    out["smoothing"] = {"n": SMOOTH_N, "switches_raw": sw_raw / n,
                        "switches_voted": sw_voted / n,
                        "accuracy_raw": accuracy(raw), "accuracy_voted": accuracy(voted)}
    enc, cls = data["encode_ms"], data["classify_ms"]
    total = [e + c for e, c in zip(enc, cls)]
    out["cost_ms"] = {"device": data["device"], "encode": percentiles(enc),
                      "classify": percentiles(cls), "total": percentiles(total)}
    return out


def main(argv=None) -> None:  # pragma: no cover - CLI
    ap = argparse.ArgumentParser(prog="python -m tools.clip_rooms_fewshot")
    sub = ap.add_subparsers(dest="cmd", required=True)
    e = sub.add_parser("embed")
    e.add_argument("--root", type=Path, default=Path("recordings"))
    e.add_argument("--device", default=None)
    e.add_argument("--out", type=Path, required=True)
    p = sub.add_parser("predict")
    p.add_argument("emb", type=Path)
    p.add_argument("--out", type=Path, required=True)
    r = sub.add_parser("report")
    r.add_argument("path", type=Path)
    a = ap.parse_args(argv)
    if a.cmd == "embed":
        embed(a.root, a.device, a.out)
    elif a.cmd == "predict":
        predict(a.emb, a.out)
    else:
        print(json.dumps(report(a.path), indent=1))


if __name__ == "__main__":  # pragma: no cover
    main()
