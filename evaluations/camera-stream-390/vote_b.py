"""
evaluations/camera-stream-390/vote_b.py

3.90.B (criterion 6): k-of-n over consecutive frames on the labelled walks,
against single frames. No model runs: the shipped pipeline's per-frame
results are 3.92's (`tier-calibration-392/perception.jsonl`, pinned by
hash). A frame is a positive under (k, n) when it is itself detected and
at least k of the last n frames of its walk (itself included) are --
`brain/tiered.py` `_tr_candidate`'s rule, counted in frames. Free.

    .venv/bin/python evaluations/camera-stream-390/vote_b.py
"""

import hashlib
import json
import re
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).parent
ROWS = HERE.parent / "tier-calibration-392" / "perception.jsonl"
SHA256 = "944fabe51a2e495a7adf2cc14807b0ab50b63da84abe28bf2c90391250737dbf"
VOTES = [(2, 2), (2, 3), (3, 5)]
BAR_PRECISION_PTS = 5.0
BAR_RECALL_PTS = 2.0


def walks():
    """{walk: [(frame number, visible, detected, key)]} in frame order."""
    data = ROWS.read_bytes()
    got = hashlib.sha256(data).hexdigest()
    if got != SHA256:
        raise SystemExit(f"frame set changed: sha256 {got}, pinned {SHA256}")
    out = defaultdict(list)
    for line in data.decode().splitlines():
        r = json.loads(line)
        walk, frame = r["key"].split("/")
        n = int(re.search(r"(\d+)", frame).group(1))
        out[walk].append((n, r["visible"], r["status"] == "detected", r["key"]))
    for frames in out.values():
        frames.sort()
    return out


def voted(frames, k, n):
    """Per-frame decision under k-of-n (None: single frames)."""
    hits = [d for _, _, d, _ in frames]
    if k is None:
        return hits
    return [hits[i] and sum(hits[max(0, i - n + 1):i + 1]) >= k for i in range(len(hits))]


def score(all_walks, k=None, n=None):
    tp = fp = fn = 0
    false_pos = []
    for frames in all_walks.values():
        for (_, visible, _, key), pos in zip(frames, voted(frames, k, n)):
            if pos and visible:
                tp += 1
            elif pos:
                fp += 1
                false_pos.append(key)
            elif visible:
                fn += 1
    return {"tp": tp, "fp": fp, "fn": fn,
            "precision": round(100 * tp / (tp + fp), 2) if tp + fp else None,
            "recall": round(100 * tp / (tp + fn), 2),
            "false_positives": false_pos,
            "runs_caught": runs_caught(all_walks, k, n)}


def runs_caught(all_walks, k, n):
    """Supplementary, decides nothing: of the in-view runs (consecutive
    visible frames), how many have at least one positive -- a sighting the
    candidate trigger would still see."""
    caught = total = 0
    for frames in all_walks.values():
        pos = voted(frames, k, n)
        run_hit = None
        for (_, visible, _, _), p in zip(frames, pos):
            if visible:
                run_hit = (run_hit or False) or p
            elif run_hit is not None:
                total += 1
                caught += run_hit
                run_hit = None
        if run_hit is not None:
            total += 1
            caught += run_hit
    return [caught, total]


def main():
    all_walks = walks()
    base = score(all_walks)
    result = {"frames": sum(len(f) for f in all_walks.values()), "walks": len(all_walks),
              "rows": str(ROWS.relative_to(HERE.parent.parent)), "sha256": SHA256,
              "single": base, "votes": {}}
    for k, n in VOTES:
        s = score(all_walks, k, n)
        s["d_precision"] = round(s["precision"] - base["precision"], 2)
        s["d_recall"] = round(s["recall"] - base["recall"], 2)
        s["meets_bar"] = (s["d_precision"] >= BAR_PRECISION_PTS
                          and -s["d_recall"] <= BAR_RECALL_PTS)
        result["votes"][f"{k}of{n}"] = s
    result["criterion_6"] = "met" if any(v["meets_bar"] for v in result["votes"].values()) else "FAILED"
    (HERE / "B-votes.json").write_text(json.dumps(result, indent=1) + "\n")
    print(f"{result['frames']} frames, {result['walks']} walks")
    print(f"single   P {base['precision']:6.2f}  R {base['recall']:6.2f}  tp {base['tp']} fp {base['fp']}")
    for name, s in result["votes"].items():
        print(f"{name:8} P {s['precision']:6.2f} ({s['d_precision']:+.2f})  "
              f"R {s['recall']:6.2f} ({s['d_recall']:+.2f})  tp {s['tp']} fp {s['fp']}  "
              f"bar {'met' if s['meets_bar'] else 'not met'}  runs {s['runs_caught']}")
    print(f"single runs {base['runs_caught']}")
    print("criterion 6:", result["criterion_6"])


if __name__ == "__main__":
    main()
