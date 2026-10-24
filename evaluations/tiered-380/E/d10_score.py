"""3.80.E, D10's two offline parts, free: on 3.48's pinned 299 frames, does
asking for the box (`box: true`, the frame question) change VE's score or
decision, and where perception also detects the target (P >= 0.8), how far
is the box's bearing from perception's?

Both from records made on the board by `service/local_vlm` with VE:
* without the box: 3.80.C's rebuilt driver, `C/driver/B1.json`;
* with it: 3.92.A's `qwen-VE.jsonl` (the 2,261 labelled frames, these 299
  among them), beside 3.92.A's YOLOE + CLIP records (`perception.jsonl`).

A bearing is `brain/tiered.py`'s: (box centre - 0.5) x the horizontal field
of view, the VLM's box read in the model's resized frame as
`brain/local_vlm._box_frac` reads it. D10's latency part (a yes with its box
under load) is measured on the board, not here.

    python evaluations/tiered-380/E/d10_score.py --out d10.json
"""

import argparse
import json
import statistics
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT))

from brain.local_vlm import model_size  # noqa: E402
from brain.tiered import DEFAULT_HFOV_DEG, LOCAL_YES_AT  # noqa: E402

CAL = ROOT / "evaluations/tier-calibration-392"
NO_BELOW = 0.1        # tier_local_no_below
GATE_P = 0.8          # perception's match gate
SCORE_BAR, BEARING_BAR = 0.01, 5.0


def decision(score):
    return "yes" if score >= LOCAL_YES_AT else "no" if score <= NO_BELOW else "band"


def jsonl(path):
    return {r["key"]: r for r in (json.loads(l) for l in open(path))}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    plain = {f"{r['walk']}/{r['frame']}": r
             for r in json.load(open(HERE.parent / "C/driver/B1.json"))["records"]}
    boxed, perc = jsonl(CAL / "qwen-VE.jsonl"), jsonl(CAL / "perception.jsonl")
    missing = sorted(k for k in plain if k not in boxed or k not in perc)
    rows, diffs, bearings = [], [], []
    for k, p in sorted(plain.items()):
        if k in missing:
            continue
        b, q = boxed[k], perc[k]
        row = {"key": k, "visible": p["visible"], "score": p["score"],
               "score_box": b.get("score"), "status_box": b.get("status")}
        if b.get("status") == "answered" and isinstance(b.get("score"), (int, float)):
            row["diff"] = abs(b["score"] - p["score"])
            row["flip"] = decision(b["score"]) != decision(p["score"])
            diffs.append(row["diff"])
        if (b.get("box") and q.get("box") and q.get("status") == "detected"
                and (q.get("p") or 0) >= GATE_P and (b.get("score") or 0) >= LOCAL_YES_AT):
            mw, _ = model_size(q["w"], q["h"])
            vx = (float(b["box"][0]) + float(b["box"][2])) / 2 / mw
            px = (q["box"][0] + q["box"][2]) / 2 / q["w"]
            row["bearing_vlm_deg"] = round((vx - 0.5) * DEFAULT_HFOV_DEG, 2)
            row["bearing_perception_deg"] = round((px - 0.5) * DEFAULT_HFOV_DEG, 2)
            row["bearing_diff_deg"] = abs(row["bearing_vlm_deg"] - row["bearing_perception_deg"])
            bearings.append(row["bearing_diff_deg"])
        rows.append(row)
    flips = [r["key"] for r in rows if r.get("flip")]
    over = [r["key"] for r in rows if r.get("diff", 0) > SCORE_BAR]
    unanswered = [r["key"] for r in rows if "diff" not in r]
    med = statistics.median(bearings) if bearings else None
    summary = {
        "frames": len(rows), "missing_from_392": missing, "hfov_deg": DEFAULT_HFOV_DEG,
        "max_score_diff": max(diffs) if diffs else None, "over_0.01": over,
        "flips": flips, "unanswered_with_box": unanswered,
        "bearing_pairs": len(bearings), "bearing_diff_median_deg": med,
        "bearing_diff_p90_deg": (sorted(bearings)[int(0.9 * (len(bearings) - 1))]
                                 if bearings else None),
        "scores_met": not over and not flips and not unanswered and not missing,
        "bearing_met": med is not None and med <= BEARING_BAR,
    }
    Path(args.out).write_text(json.dumps({"summary": summary, "rows": rows}, indent=1))
    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
