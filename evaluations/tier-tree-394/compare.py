"""
evaluations/tier-tree-394/compare.py

3.94 criterion 2b: the baseline checkout's sweep against HEAD's, arm by arm,
mission by mission, frame by frame (action, turn size, trigger, local
question, call counts, reasoning) and on the missions' rows. Writes
`summary.json`.

    tar xzf evaluations/tier-tree-394/traces.tar.gz -C evaluations/tier-tree-394   # the committed traces
    .venv/bin/python evaluations/tier-tree-394/compare.py
"""

import json
from pathlib import Path

HERE = Path(__file__).parent
ARMS = ("pace6", "pace1", "decoy", "cloudno", "sync")


def main():
    summary = {}
    for arm in ARMS:
        base = json.loads((HERE / f"base-{arm}.json").read_text())
        head = json.loads((HERE / f"head-{arm}.json").read_text())
        frames = differ = rows_differ = 0
        first = None
        for b, h in zip(base["missions"], head["missions"], strict=True):
            assert b["house"] == h["house"]
            frames += len(h["frames"])
            if b["row"] != h["row"]:
                rows_differ += 1
            for i, (fb, fh) in enumerate(zip(b["frames"], h["frames"])):
                if fb != fh:
                    differ += 1
                    first = first or (h["house"], i, fb, fh)
            if len(b["frames"]) != len(h["frames"]):
                differ += abs(len(b["frames"]) - len(h["frames"]))
                first = first or (h["house"], "length", len(b["frames"]), len(h["frames"]))
        outcomes = {}
        for m in head["missions"]:
            outcomes[m["row"]["outcome"]] = outcomes.get(m["row"]["outcome"], 0) + 1
        summary[arm] = {"missions": len(head["missions"]), "frames": frames,
                        "frames_differ": differ, "missions_rows_differ": rows_differ,
                        "outcomes": outcomes, "first_difference": first,
                        "met": differ == 0 and rows_differ == 0}
        print(arm, json.dumps({k: v for k, v in summary[arm].items() if k != "first_difference"}),
              "first:", first)
    (HERE / "summary.json").write_text(json.dumps(summary, indent=1) + "\n")


if __name__ == "__main__":
    main()
