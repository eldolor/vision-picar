"""
evaluations/tiered-380/M/decide_scale.py

3.80.M: reads each model's raw boxes in the scale its numbers are in, decided
on S1 before the box arm is scored (the plan's rule): for pixels of the
frame, 0-1000 and 0-1, the yes-with-a-box frames' box-centre third is compared
with S1's labels; a reading the numbers cannot fit is ruled out first; the
reading with the most agreement is used. Writes `<name>.scaled.jsonl` (the
box fields re-read) and `<name>.scale.json` (the decision).

    .venv/bin/python evaluations/tiered-380/M/decide_scale.py <name>
"""

import json
import sys
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent / "I"))
from common import third                                    # noqa: E402

YES_AT = 0.5
SCALES = {"pixels": lambda w: w, "0-1000": lambda w: 1000.0, "0-1": lambda w: 1.0}


def reread(a, scale):
    """The answer `a` (one side of a row) with its box read in `scale`."""
    box = a.get("box")
    frac = None
    if box is not None and a.get("id_status") == "answered" and a["id_score"] >= YES_AT:
        frac = round(min(1.0, max(0.0, (box[0] + box[2]) / 2 / SCALES[scale](a["img"][0]))), 4)
    return {**a, "box_frac_x": frac, "box_answer": third(frac), "box_scale": scale}


def fits(rows, scale):
    xs = [(v, a["img"][0]) for r in rows for a in (r, r["mirror"])
          if a.get("box") for v in a["box"]]
    return all(v <= SCALES[scale](w) * 1.02 for v, w in xs)


def main(name):
    rows = [json.loads(l) for l in (HERE / f"{name}.jsonl").read_text().splitlines() if l.strip()]
    s1 = [r for r in rows if r["set"] == "S1"]
    agree = {}
    for scale in SCALES:
        ans = [reread(r, scale)["box_answer"] for r in s1]
        agree[scale] = {"fits": fits(rows, scale),
                        "answered": sum(a is not None for a in ans),
                        "agree": sum(a == r["truth"] for a, r in zip(ans, s1))}
    ok = {k: v for k, v in agree.items() if v["fits"]} or agree
    ranked = sorted(ok, key=lambda k: -ok[k]["agree"])
    chosen = ranked[0]
    close = len(ranked) > 1 and ok[ranked[0]]["agree"] - ok[ranked[1]]["agree"] <= 2
    doc = {"chosen": chosen, "close_second": ranked[1] if close else None, "by_scale": agree}
    (HERE / f"{name}.scale.json").write_text(json.dumps(doc, indent=1) + "\n")
    with (HERE / f"{name}.scaled.jsonl").open("w") as f:
        for r in rows:
            f.write(json.dumps({**reread(r, chosen), "mirror": reread(r["mirror"], chosen)}) + "\n")
    print(json.dumps(doc))


if __name__ == "__main__":
    main(sys.argv[1])
