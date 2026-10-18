"""
evaluations/tiered-380/R/score_steer.py

3.80.R: scores every R steering row (`R/<name>.scaled.jsonl`, board, and
`R/mac/<name>.scaled.jsonl`, Mac; never merged, P24) with 3.80.I's own
scorer functions, so the numbers compare with I/M/N's. Writes
`R/steer_scores.json`; prints S1 right of 60 (R2), S3 accuracy (I2) and the
mirror rates (I3).

    .venv/bin/python evaluations/tiered-380/R/score_steer.py
"""

import json
import sys
from collections import Counter
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent / "I"))
import score as I                                # noqa: E402


def rows_for(path):
    """The R rows in `path`, keyed as I's loader keys them."""
    return I.load(str(Path("..") / path.relative_to(HERE.parent)))


def score(rows, field, sets_):
    t = {}
    for s in sets_:
        rs = [r for r in rows if r["set"] == s]
        if not rs or all(I.pick(r, field) is None for r in rs):
            continue
        e = {"mirror": I.mirror(rs, field)}
        if s in ("S1", "S3"):
            e["all"] = I.accuracy(rs, field)
        else:
            e["answers"] = dict(Counter(str(I.pick(r, field)) for r in rs))
        t[s] = e
    return t


def main():
    out = {}
    for where, d in (("board", HERE), ("Mac", HERE / "mac")):
        for p in sorted(d.glob("*.scaled.jsonl")):
            rows = rows_for(p)
            name = p.name[: -len(".scaled.jsonl")]
            for field, sets_ in (("direction", ("S1", "S2", "S3")), ("box_answer", ("S1", "S2"))):
                t = score(rows, field, sets_)
                if t:
                    out[f"{where} {name}: {field}"] = {"rows": len(rows), **t}
    (HERE / "steer_scores.json").write_text(json.dumps(out, indent=1) + "\n")
    for tier, t in out.items():
        s1 = t.get("S1", {}).get("all", {})
        s3 = t.get("S3", {}).get("all", {})
        print(f"{tier:40s} rows {t['rows']} S1 {s1.get('right')}/{s1.get('n')} cov {s1.get('coverage')} | "
              f"S3 {s3.get('acc_all')} | mirror S1 {t.get('S1', {}).get('mirror', {}).get('rate')} "
              f"S3 {t.get('S3', {}).get('mirror', {}).get('rate')}")


if __name__ == "__main__":
    main()
