"""
evaluations/tiered-380/S/score.py

3.80.S: composes the four pipelines (A, G, F, FS) from each resolution's
board rows (`S/board/out/<res>.jsonl`) and scores them with 3.80.I's own
`accuracy` and `mirror`; latency from the rows, memory from
`S/board/out/<res>.mem`. Writes `S/scores.json`.

    .venv/bin/python evaluations/tiered-380/S/score.py
"""

import json
import sys
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent / "I"))
import score as I                                # noqa: E402

OUT = HERE / "board" / "out"
DETECTED = "detected"   # brain/perceive.py's DETECTED


def recent(row) -> bool:
    return any(p["status"] == DETECTED for p in row["prev"])


def a_ans(perc, _flo, _row):
    return perc["answer"]


def g_asks(perc, row) -> bool:
    """G asks Florence: no detection on this frame, one in the window."""
    return perc["status"] != DETECTED and recent(row)


def g_ans(perc, flo, row):
    if perc["status"] == DETECTED:
        return perc["answer"]
    if recent(row):
        return flo["answer"]
    return perc["answer"] if perc["how"] == "band" else None


def f_ans(_perc, flo, _row):
    return flo["answer"]


def fs_ans(perc, flo, _row):
    return perc["answer"] if perc["answer"] is not None else flo["answer"]


PIPES = {"A": a_ans, "G": g_ans, "F": f_ans, "FS": fs_ans}


def pct(xs, q):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(q * len(xs)))] if xs else None


def memory(res):
    p = OUT / f"{res}.mem"
    if not p.exists():
        return None
    vals = [int(line.split()[1]) for line in p.read_text().splitlines() if len(line.split()) == 2]
    return {"before": vals[0], "min": min(vals), "drop": vals[0] - min(vals), "samples": len(vals)}


def score_res(res):
    rows = [json.loads(line) for line in (OUT / f"{res}.jsonl").read_text().splitlines() if line.strip()]
    out = {"rows": len(rows), "memory": memory(res)}
    for name, fn in PIPES.items():
        built = [{"set": r["set"], "key": r["key"], "truth": r["truth"],
                  "ans": fn(r["perc"], r["florence"], r),
                  "mirror": {"ans": fn(r["mirror"]["perc"], r["mirror"]["florence"], r)}}
                 for r in rows]
        s1 = [b for b in built if b["set"] == "S1"]
        s2 = [b for b in built if b["set"] == "S2"]
        out[name] = {"S1": I.accuracy(s1, "ans"), "S1_mirror": I.mirror(s1, "ans"),
                     "S2_answered": sum(b["ans"] is not None for b in s2),
                     "S2_mirror_answered": sum(b["mirror"]["ans"] is not None for b in s2)}
    # G's gate and its latency: a frame's pipeline time (perception, plus
    # Florence when G asks it), originals and mirrors both.
    asked, perc_ms = [], []
    for r in rows:
        for perc, flo in ((r["perc"], r["florence"]), (r["mirror"]["perc"], r["mirror"]["florence"])):
            perc_ms.append(perc["ms"])
            if g_asks(perc, r):
                asked.append(perc["ms"] + flo["ms"])
    out["G"]["asked_S1"] = sum(g_asks(r["perc"], r) for r in rows if r["set"] == "S1")
    out["G"]["asked_S2"] = sum(g_asks(r["perc"], r) for r in rows if r["set"] == "S2")
    out["G"]["asked_frames"] = len(asked)
    out["G"]["asked_p50_ms"], out["G"]["asked_p90_ms"] = pct(asked, 0.5), pct(asked, 0.9)
    out["perception_ms"] = {"p50": pct(perc_ms, 0.5), "p90": pct(perc_ms, 0.9)}
    flo_ms = [x for r in rows for x in (r["florence"]["ms"], r["mirror"]["florence"]["ms"])]
    out["florence_ms"] = {"p50": pct(flo_ms, 0.5), "p90": pct(flo_ms, 0.9)}
    return out


def main():
    res_out = {}
    for res in ("orig", "640", "320"):
        if (OUT / f"{res}.jsonl").exists():
            res_out[res] = score_res(res)
    (HERE / "scores.json").write_text(json.dumps(res_out, indent=1) + "\n")
    for res, t in res_out.items():
        print(f"== {res} rows {t['rows']} mem {t['memory']} perception {t['perception_ms']} "
              f"florence {t['florence_ms']}")
        for name in PIPES:
            p = t[name]
            print(f"  {name:3s} S1 {p['S1']['right']}/{p['S1']['n']} cov {p['S1']['coverage']} "
                  f"mirror {p['S1_mirror']['rate']} | S2 answered {p['S2_answered']}/40 "
                  f"(mirror {p['S2_mirror_answered']})")
        g = t["G"]
        print(f"  G asked S1 {g['asked_S1']} S2 {g['asked_S2']} | asked-frame p50/p90 "
              f"{g['asked_p50_ms']}/{g['asked_p90_ms']} ms")


if __name__ == "__main__":
    main()
