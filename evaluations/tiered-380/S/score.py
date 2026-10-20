"""
evaluations/tiered-380/S/score.py

3.80.S: composes the four pipelines (A, G, F, FS) from each resolution's
board rows (`S/board/out/<res>.jsonl`) and scores them with 3.80.I's own
`accuracy` and `mirror`; latency from the rows, memory from
`S/board/out/<res>.mem`. Writes `S/scores.json`.

    .venv/bin/python evaluations/tiered-380/S/score.py
"""

import importlib.util
import json
import sys
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent / "I"))
sys.path.insert(0, str(HERE.resolve().parents[2]))
from brain.tiered import percentiles             # noqa: E402

# 3.80.I's scorer by its path: its module name is this file's own.
_spec = importlib.util.spec_from_file_location("score_380i", HERE.parent / "I" / "score.py")
I = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(I)

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


def memory(res):
    p = OUT / f"{res}.mem"
    if not p.exists():
        return None
    vals = [int(line.split()[1]) for line in p.read_text().splitlines()
            if len(line.split()) == 2 and line.split()[1].isdigit()]
    if not vals:
        return None
    return {"before": vals[0], "min": min(vals), "drop": vals[0] - min(vals), "samples": len(vals)}


def score_res(res):
    rows = [json.loads(line) for line in (OUT / f"{res}.jsonl").read_text().splitlines() if line.strip()]
    sizes = {s: sum(r["set"] == s for r in rows) for s in ("S1", "S2")}
    if sizes != {"S1": 60, "S2": 40}:
        print(f"WARNING {res}: a partial run, {sizes} (want S1 60, S2 40)")
    out = {"rows": len(rows), "sizes": sizes, "memory": memory(res)}
    for name, fn in PIPES.items():
        built = [{"set": r["set"], "key": r["key"], "truth": r["truth"],
                  "ans": fn(r["perc"], r["florence"], r),
                  "mirror": {"ans": fn(r["mirror"]["perc"], r["mirror"]["florence"], r)}}
                 for r in rows]
        s1 = [b for b in built if b["set"] == "S1"]
        s2 = [b for b in built if b["set"] == "S2"]
        out[name] = {"S1": I.accuracy(s1, "ans"), "S1_mirror": I.mirror(s1, "ans"),
                     "S2_n": len(s2), "S2_answered": sum(b["ans"] is not None for b in s2),
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
    out["G"]["asked_ms"] = percentiles(asked)
    out["perception_ms"] = percentiles(perc_ms)
    out["florence_ms"] = percentiles([x for r in rows
                                      for x in (r["florence"]["ms"], r["mirror"]["florence"]["ms"])])
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
                  f"mirror {p['S1_mirror']['rate']} | S2 answered {p['S2_answered']}/{p['S2_n']} "
                  f"(mirror {p['S2_mirror_answered']})")
        g = t["G"]
        print(f"  G asked S1 {g['asked_S1']} S2 {g['asked_S2']} | asked-frame {g['asked_ms']}")


if __name__ == "__main__":
    main()
