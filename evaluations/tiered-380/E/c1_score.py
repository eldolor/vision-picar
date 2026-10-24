"""3.80.E criterion 1: the tiered identity answer at arrival on 3.48's pinned
299 frames -- the local VLM first, the cloud only after
`tier_local_max_unable` unable answers (or with the local tier absent).

Free and offline. Each frame is one arrival, put to the real
`TieredVision.confirm_arrival` (a fresh one per frame, the config's own
thresholds), asked again while the verdict is `undecided`, as the agent asks
again on the next arrival frame. The local tier replays the board's recorded
answer for that frame (`--records`, a 3.80.B-style `B1.json` from
`service/local_vlm` on the Jetson): VE is deterministic on a frame (3.80.B's
B1: 299 of 299 within 0.01 across runs), so asking it twice gives the same
answer.

The cloud makes no call. Its recorded answers (Opus 4.5, September) are in the
walks on the Mac, not here; what is known of them without the walks is 3.48's
row: every false yes (34) is on the teal-bin subset of
`blue-bottle-20260907-185007`, and it missed 2 of the 74 visible frames. So a
frame escalated to the cloud is scored "no" when the label is no and the frame
is outside that walk (its recorded answer was no), and is otherwise left
`cloud_unknown` -- reported, never guessed.

    python evaluations/tiered-380/E/c1_score.py \\
        --records evaluations/tiered-380/C/driver/B1.json --out c1.json
"""

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from brain.perceive import FrameReportedPipeline  # noqa: E402
from brain.tiered import TieredVision  # noqa: E402
from control.brain_config import load_brain_config  # noqa: E402

TEAL_WALK = "blue-bottle-20260907-185007"
PRECISION_BAR, RECALL_BAR = 0.95, 0.85


class CloudAsked(Exception):
    pass


def score_frame(rec: dict, cfg: dict) -> dict:
    asked = {"local": 0, "cloud": 0}

    def local(frame):
        asked["local"] += 1
        return {"status": rec["service_status"], "score": rec["score"]}

    def cloud(frame):
        asked["cloud"] += 1
        raise CloudAsked

    tier = TieredVision(FrameReportedPipeline("target"), lambda f: {},
                        confirm_vision_fn=cloud, local_identity_fn=local,
                        local_no_below=cfg["tier_local_no_below"],
                        local_max_unable=cfg["tier_local_max_unable"])
    verdict, by = None, "local"
    for _ in range(cfg["tier_local_max_unable"] + 1):
        try:
            v = tier.confirm_arrival({"image": b""})
        except CloudAsked:
            by = "cloud"
            break
        if not v.get("undecided"):
            verdict = bool(v["confirmed"])
            break
    if by == "cloud":
        # The cloud's recorded answer, where 3.48's row pins it (docstring).
        verdict = (False if not rec["visible"] and rec["walk"] != TEAL_WALK
                   else None)
    return {"key": f"{rec['walk']}/{rec['frame']}", "visible": rec["visible"],
            "local_status": rec["service_status"], "local_score": rec["score"],
            "by": by, "yes": verdict, "local_asks": asked["local"],
            "cloud_asks": asked["cloud"]}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--records", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    cfg = load_brain_config()
    recs = json.load(open(args.records))["records"]
    rows = [score_frame(r, cfg) for r in recs]
    tp = sum(1 for r in rows if r["yes"] and r["visible"])
    fp = sum(1 for r in rows if r["yes"] and not r["visible"])
    vis = sum(1 for r in rows if r["visible"])
    unknown = [r["key"] for r in rows if r["yes"] is None]
    prec = tp / (tp + fp) if tp + fp else None
    rec = tp / vis if vis else None
    summary = {
        "records": args.records, "frames": len(rows), "visible": vis,
        "thresholds": {k: cfg[k] for k in ("tier_local_no_below", "tier_local_max_unable")},
        "yes": tp + fp, "tp": tp, "fp": fp, "precision": prec, "recall": rec,
        "decided_by": {"local": sum(r["by"] == "local" for r in rows),
                       "cloud": sum(r["by"] == "cloud" for r in rows)},
        "cloud_escalations": sum(r["cloud_asks"] for r in rows),
        "cloud_unknown": unknown,
        "teal_bin_walk_yes": sum(1 for r in rows if r["yes"] and not r["visible"]
                                 and r["key"].startswith(TEAL_WALK)),
        "met": (not unknown and prec is not None and prec >= PRECISION_BAR
                and rec >= RECALL_BAR),
    }
    Path(args.out).write_text(json.dumps({"summary": summary, "rows": rows}, indent=1))
    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
