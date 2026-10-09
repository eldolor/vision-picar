"""
tools/clip_rooms_eval.py

3.51 (`docs/plans/ros-alignment/3.51-clip-rooms.md`): **can the CLIP model
the tiered policy already loads tell which room a frame was taken in?**
An early read on the four adjudicated rig walks of 2026-09-07, which cover
two or three rooms of one house. Measurement only: nothing here is on the
mission path.

    python -m tools.clip_rooms_eval score --device cpu --out cpu.json
    python -m tools.clip_rooms_eval score --device mps --out mps.json
    python -m tools.clip_rooms_eval report cpu.json

`score` runs the whole frame through CLIP RN50 once per frame, records the
probability of every room class under every prompt set, and times the
image encode. `report` is arithmetic over that file: accuracy, macro
accuracy, agreement with the cloud's recorded `room_guess`, smoothing. The
split is perception_eval's "score once, threshold afterwards".

The reference is the committed `docs/evaluations/room-labels-3.51.json`
(made by eye for 3.51; `--labels ''` reads a `room_labels.json` beside each
walk instead), never `walk.jsonl`: the cloud's `room_guess` there is what a model said,
and it is scored against the labels like CLIP is.
"""

import argparse
import json
import time
from collections import Counter
from pathlib import Path
from typing import Optional, Sequence

from brain.tiered import percentiles

# The reference, committed: {"walks": {walk: {frame: room}}}. `score` reads
# it by default, so the measurement reproduces from a fresh checkout.
COMMITTED_LABELS = (Path(__file__).resolve().parent.parent
                    / "docs" / "evaluations" / "room-labels-3.51.json")

WALKS = (
    "blue-bottle-20260907-142454",
    "red-backpack-20260907-144856",
    "blue-shoes-20260907-152528",
    "blue-bottle-20260907-185007",
)
# Tuning happens on these three; the 209-frame walk, the only one that
# crosses rooms, is held out (3.51's method).
HELD_OUT = "blue-bottle-20260907-185007"

# Fixed before any CLIP output was seen (3.51): seven room types a house
# has, whatever the walks contain, so CLIP must pick against real
# alternatives.
ROOMS = ("living room", "kitchen", "bedroom", "hallway", "bathroom",
         "dining room", "home office")
TEMPLATES = ("a photo of a {room}.",
             "a photo of a {room}, taken from the floor.",
             "the inside of a {room}.")
ENSEMBLE = "ensemble"
UNSURE = "unsure"
LABELS_FILE = "room_labels.json"

# The cloud answers in free text. Mapped onto the seven classes; anything
# else, "unclear" included, maps to None and counts as wrong.
_CLOUD_SYNONYMS = {
    "living room": "living room", "family room": "living room",
    "sitting room": "living room", "lounge": "living room", "den": "living room",
    "hallway": "hallway", "hall": "hallway", "foyer": "hallway",
    "entryway": "hallway", "entry": "hallway", "entrance": "hallway",
    "home office": "home office", "office": "home office", "study": "home office",
    "kitchen": "kitchen", "bedroom": "bedroom", "bathroom": "bathroom",
    "dining room": "dining room",
}


def normalise_room(text: Optional[str]) -> Optional[str]:
    """A free-text room answer -> one of ROOMS, or None."""
    if not isinstance(text, str):
        return None
    return _CLOUD_SYNONYMS.get(text.strip().lower())


def load_walk(root: Path, walk: str, labels: Optional[dict] = None) -> list:
    """[{frame, label, cloud}] in frame order. `cloud` is the normalised
    recorded room_guess (None when absent or unmapped). `labels` is the
    walk's {frame: room}; without it, the walk's own `room_labels.json`."""
    if labels is None:
        labels = json.loads((root / walk / LABELS_FILE).read_text())["labels"]
    cloud = {}
    for line in (root / walk / "walk.jsonl").read_text().splitlines():
        if line.strip():
            row = json.loads(line)
            cloud[row["file"]] = normalise_room((row.get("navigate") or {}).get("room_guess"))
    return [{"frame": f, "label": labels[f], "cloud": cloud.get(f)}
            for f in sorted(labels)]


def accuracy(pairs: Sequence[tuple]) -> Optional[float]:
    """Share of (truth, prediction) pairs that agree, `unsure` truths
    excluded. None when nothing is left."""
    scored = [(t, p) for t, p in pairs if t != UNSURE]
    if not scored:
        return None
    return sum(t == p for t, p in scored) / len(scored)


def macro_accuracy(pairs: Sequence[tuple], min_frames: int = 10) -> Optional[float]:
    """Mean per-room recall over rooms with at least `min_frames` decided
    frames -- the guard against answering the majority room every time."""
    by_room: dict = {}
    for t, p in pairs:
        if t != UNSURE:
            by_room.setdefault(t, []).append(t == p)
    recalls = [sum(v) / len(v) for v in by_room.values() if len(v) >= min_frames]
    return sum(recalls) / len(recalls) if recalls else None


def per_room(pairs: Sequence[tuple]) -> dict:
    out: dict = {}
    for t, p in pairs:
        if t != UNSURE:
            hit, n = out.get(t, (0, 0))
            out[t] = (hit + (t == p), n + 1)
    return out


def smooth(predictions: Sequence[str], n: int) -> list:
    """Causal majority vote over the last `n` predictions (what a robot
    could run live); a tie goes to the most recent of the tied rooms."""
    out = []
    for i in range(len(predictions)):
        window = list(predictions[max(0, i - n + 1): i + 1])
        counts = Counter(window)
        top = max(counts.values())
        out.append(next(r for r in reversed(window) if counts[r] == top))
    return out


def switches_per_100(predictions: Sequence[str]) -> float:
    if len(predictions) < 2:
        return 0.0
    changes = sum(a != b for a, b in zip(predictions, predictions[1:]))
    return 100.0 * changes / (len(predictions) - 1)


# ---------------------------------------------------------------------------
# The model half. Needs requirements-perception.txt; never imported by the
# suite.

WARMUP = 3  # as tools/jetson/bench_perception.py


def _sync(torch, device: str) -> None:  # pragma: no cover - needs torch
    """Wait for queued GPU work, so a timer measures the work, not its launch."""
    if device == "cuda":
        torch.cuda.synchronize()
    elif device == "mps":
        torch.mps.synchronize()


def score(root: Path, device: Optional[str], out: Path,
          labels_path: Optional[Path] = None) -> None:  # pragma: no cover - needs CLIP
    import torch
    from PIL import Image

    from brain.perceive import ClipScorer

    committed = (json.loads(labels_path.read_text())["walks"]
                 if labels_path is not None else {})
    scorer = ClipScorer(device=device)
    text = {tmpl: scorer._text_features(tuple(tmpl.format(room=r) for r in ROOMS))
            for tmpl in TEMPLATES}
    mean = torch.stack(list(text.values())).mean(dim=0)
    text[ENSEMBLE] = mean / mean.norm(dim=-1, keepdim=True)
    scale = scorer.model.logit_scale.exp()

    def encode(img):
        with torch.no_grad():
            feats = scorer.model.encode_image(
                scorer.preprocess(img).unsqueeze(0).to(scorer.device))
            feats = feats / feats.norm(dim=-1, keepdim=True)
        _sync(torch, scorer.device)
        return feats

    rows, timings = [], []
    warm = False
    for walk in WALKS:
        for row in load_walk(root, walk, committed.get(walk)):
            img = Image.open(root / walk / row["frame"]).convert("RGB")
            if not warm:  # first-call compilation and allocation, not timed
                for _ in range(WARMUP):
                    encode(img)
                warm = True
            started = time.perf_counter()
            feats = encode(img)
            timings.append((time.perf_counter() - started) * 1000)
            probs = {}
            with torch.no_grad():
                for name, tf in text.items():
                    p = (scale * feats @ tf.T).softmax(dim=-1)[0]
                    probs[name] = [round(float(v), 8) for v in p]
            rows.append({**row, "walk": walk, "probs": probs})
    out.write_text(json.dumps({
        "device": scorer.device, "model": scorer.model_name, "rooms": list(ROOMS),
        "templates": list(TEMPLATES), "warmup": WARMUP,
        "encode_ms": [round(t, 2) for t in timings], "rows": rows}, indent=1))
    p = percentiles(timings)
    print(f"{len(rows)} frames on {scorer.device}: encode p50 {p['p50']} ms, p90 {p['p90']} ms")


def predictions(rows: Sequence[dict], rooms: Sequence[str], prompt: str) -> list:
    return [rooms[max(range(len(rooms)), key=lambda i: r["probs"][prompt][i])] for r in rows]


def report(path: Path) -> dict:
    data = json.loads(path.read_text())
    rows, rooms = data["rows"], data["rooms"]
    out: dict = {"device": data["device"], "frames": len(rows),
                 "decided": sum(r["label"] != UNSURE for r in rows)}
    truth = [r["label"] for r in rows]
    for prompt in [ENSEMBLE] + list(data["templates"]):
        pairs = list(zip(truth, predictions(rows, rooms, prompt)))
        out[prompt] = {"accuracy": accuracy(pairs), "macro": macro_accuracy(pairs),
                       "per_room": per_room(pairs)}
    cloud = [r["cloud"] for r in rows]
    clip = predictions(rows, rooms, ENSEMBLE)
    cloud_pairs = list(zip(truth, cloud))
    out["cloud"] = {"accuracy": accuracy(cloud_pairs), "macro": macro_accuracy(cloud_pairs),
                    "per_room": per_room(cloud_pairs)}
    decided = [(c, k) for t, c, k in zip(truth, cloud, clip) if t != UNSURE]
    out["clip_cloud_agreement"] = (sum(c == k for c, k in decided) / len(decided)
                                   if decided else None)
    # The repo's one p90 definition (brain/tiered.py), integer ms.
    out["encode_ms"] = percentiles(data["encode_ms"])

    # Smoothing: N chosen on the three short walks, judged on the held-out one.
    def smoothed(sel, n):
        res = []
        for w in dict.fromkeys(r["walk"] for r in sel):  # per walk, in order
            preds = predictions([r for r in sel if r["walk"] == w], rooms, ENSEMBLE)
            res.extend(smooth(preds, n) if n > 1 else preds)
        return res

    def judged(sel, n):
        preds = smoothed(sel, n)
        return {"accuracy": accuracy(list(zip([r["label"] for r in sel], preds))),
                "switches_per_100": switches_per_100(preds)}

    tune = [r for r in rows if r["walk"] != HELD_OUT]
    held = [r for r in rows if r["walk"] == HELD_OUT]
    base = judged(tune, 1)
    out["tune"] = {n: judged(tune, n) for n in (3, 5, 7)}
    eligible = [n for n, j in out["tune"].items()
                if None not in (j["accuracy"], base["accuracy"])
                and j["accuracy"] >= base["accuracy"] - 0.02]
    # Fewest switches among the eligible; a tie goes to the smaller N.
    choice = min(eligible, key=lambda n: (out["tune"][n]["switches_per_100"], n)) \
        if eligible else None
    out["smoothing"] = {
        "n": choice,
        "chosen_by_rule": choice is not None,
        "held_out_raw": judged(held, 1),
        "held_out_smoothed": judged(held, choice) if choice else None,
    }
    hp = list(zip([r["label"] for r in held], predictions(held, rooms, ENSEMBLE)))
    out["held_out"] = {"accuracy": accuracy(hp), "macro": macro_accuracy(hp),
                       "per_room": per_room(hp)}
    return out


def main(argv=None) -> None:  # pragma: no cover - CLI
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[1])
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("score")
    s.add_argument("--recordings", default="recordings")
    s.add_argument("--labels", default=str(COMMITTED_LABELS),
                   help="the committed labels; '' reads each walk's room_labels.json")
    s.add_argument("--device", default=None)
    s.add_argument("--out", required=True)
    r = sub.add_parser("report")
    r.add_argument("path")
    args = ap.parse_args(argv)
    if args.cmd == "score":
        score(Path(args.recordings), args.device, Path(args.out),
              Path(args.labels) if args.labels else None)
    else:
        print(json.dumps(report(Path(args.path)), indent=1))


if __name__ == "__main__":  # pragma: no cover
    main()
