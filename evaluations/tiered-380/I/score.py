"""
evaluations/tiered-380/I/score.py

3.80.I: scores every tier's answers against S1's labels and S3's path truth,
and the mirror test on S1-S3. Writes `scores.json`.

    .venv/bin/python evaluations/tiered-380/I/score.py
"""

import json
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from common import HERE, SWAP, labels_s1       # noqa: E402

FLAGS = {f"{r['walk']}/{r['frame']}": r for r in labels_s1()}


def load(name):
    p = HERE / name
    if not p.exists():
        return []
    rows = {}
    for line in p.read_text().splitlines():
        if line.strip():
            r = json.loads(line)
            rows[r["key"]] = r
    return list(rows.values())


def pick(row, field):
    a = row
    for part in field.split("."):
        a = (a or {}).get(part)
    return a


def accuracy(rows, field, keep=lambda r: True):
    rows = [r for r in rows if keep(r)]
    n = len(rows)
    ans = [r for r in rows if pick(r, field) is not None]
    hit = sum(pick(r, field) == r["truth"] for r in ans)
    sides = [r for r in rows if r["truth"] in ("LEFT", "RIGHT")]
    side_hit = sum(pick(r, field) == r["truth"] for r in sides)
    return {"n": n, "answered": len(ans), "right": hit,
            "acc_all": round(hit / n, 3) if n else None,
            "acc_answered": round(hit / len(ans), 3) if ans else None,
            "coverage": round(len(ans) / n, 3) if n else None,
            "side_right": side_hit, "side_n": len(sides),
            "answers": dict(Counter(str(pick(r, field)) for r in rows))}


def mirror(rows, field):
    """Consistent: the mirrored answer is the original with LEFT/RIGHT swapped.
    Over frames with an answer both times."""
    both = [r for r in rows if pick(r, field) is not None
            and pick(r, "mirror." + field) is not None]
    ok = sum(pick(r, "mirror." + field) == SWAP[pick(r, field)] for r in both)
    lr = [r for r in both if pick(r, field) != "FORWARD"]
    ok_lr = sum(pick(r, "mirror." + field) == SWAP[pick(r, field)] for r in lr)
    return {"pairs": len(both), "consistent": ok,
            "rate": round(ok / len(both), 3) if both else None,
            "turn_pairs": len(lr), "turn_consistent": ok_lr}


def lean(rows):
    """The direction question's probabilities under its forward bias: on
    frames whose truth is LEFT or RIGHT, does d = log(pL/pR) minus the same
    for the mirror pick the truth's side (3.80.F's f4b statistic)? And does
    the plain left-vs-right lean (pL > pR) pick it?"""
    import math
    eps = 1e-4
    lr = lambda p: math.log(((p or {}).get("left", 0) + eps) / ((p or {}).get("right", 0) + eps))  # noqa: E731
    side = [r for r in rows if r["truth"] in ("LEFT", "RIGHT") and r.get("probs")]
    hit = sum(((lr(r["probs"]) - lr(r["mirror"].get("probs"))) > 0) == (r["truth"] == "LEFT") for r in side)
    raw = sum((lr(r["probs"]) > 0) == (r["truth"] == "LEFT") for r in side)
    return {"n": len(side), "mirror_d_right": hit, "lean_right": raw}


TIERS = [  # (tier, file, field, sets)
    ("YOLOE + CLIP (detection or band candidate)", "perception.jsonl", "answer", ("S1", "S2")),
    ("Qwen VE: direction question", "qwen-VE.jsonl", "direction", ("S1", "S2", "S3")),
    ("Qwen VE: identity box", "qwen-VE.jsonl", "box_answer", ("S1", "S2")),
    ("Qwen full: direction question", "qwen-full.jsonl", "direction", ("S1", "S2", "S3")),
    ("Qwen full: identity box", "qwen-full.jsonl", "box_answer", ("S1", "S2")),
    ("llama.cpp Q4 Qwen2.5-VL-3B: direction", "../J/qwen-q4.jsonl", "direction", ("S1", "S2", "S3")),
    ("llama.cpp Q4 Qwen2.5-VL-3B: box", "../J/qwen-q4.jsonl", "box_answer", ("S1", "S2")),
    ("llama.cpp Q4 RoboBrain2.0-3B: direction", "../J/robobrain-q4.jsonl", "direction", ("S1", "S2", "S3")),
    ("llama.cpp Q4 RoboBrain2.0-3B: box", "../J/robobrain-q4.jsonl", "box_answer", ("S1", "S2")),
    ("Edge-LLM RoboBrain RVE: direction", "qwen-RVE.jsonl", "direction", ("S1", "S2", "S3")),
    ("Edge-LLM RoboBrain RVE: box", "qwen-RVE.jsonl", "box_answer", ("S1", "S2")),
    ("Edge-LLM RoboBrain RT: direction", "qwen-RT.jsonl", "direction", ("S1", "S2", "S3")),
    ("Edge-LLM RoboBrain RT: box", "qwen-RT.jsonl", "box_answer", ("S1", "S2")),
    ("llama.cpp Q4 InternVL3.5-2B: direction", "../M/ivl2b.scaled.jsonl", "direction", ("S1", "S2", "S3")),
    ("llama.cpp Q4 InternVL3.5-2B: box", "../M/ivl2b.scaled.jsonl", "box_answer", ("S1", "S2")),
    ("llama.cpp Q4 InternVL3.5-4B: direction", "../M/ivl4b.scaled.jsonl", "direction", ("S1", "S2", "S3")),
    ("llama.cpp Q4 InternVL3.5-4B: box", "../M/ivl4b.scaled.jsonl", "box_answer", ("S1", "S2")),
    ("llama.cpp Q4 Youtu-VL-4B: direction", "../M/youtu.scaled.jsonl", "direction", ("S1", "S2", "S3")),
    ("llama.cpp Q4 Youtu-VL-4B: box", "../M/youtu.scaled.jsonl", "box_answer", ("S1", "S2")),
    ("llama.cpp Q4 MiniCPM-V 4.0: direction", "../M/minicpm.scaled.jsonl", "direction", ("S1", "S2", "S3")),
    ("llama.cpp Q4 MiniCPM-V 4.0: box", "../M/minicpm.scaled.jsonl", "box_answer", ("S1", "S2")),
    ("Edge-LLM InternVL3.5-4B NVE V512: direction", "../N/NVE-V512.scaled.jsonl", "direction", ("S1", "S2", "S3")),
    ("Edge-LLM InternVL3.5-4B NVE V512: box", "../N/NVE-V512.scaled.jsonl", "box_answer", ("S1", "S2")),
    ("Edge-LLM InternVL3.5-4B NVE V1024: direction", "../N/NVE-V1024.scaled.jsonl", "direction", ("S1", "S2", "S3")),
    ("Edge-LLM InternVL3.5-4B NVE V1024: box", "../N/NVE-V1024.scaled.jsonl", "box_answer", ("S1", "S2")),
    ("Opus 4.5: action", "opus.jsonl", "action", ("S1", "S2", "S3")),
    ("Opus 4.5: target_direction", "opus.jsonl", "target_third", ("S1",)),
]


def main():
    out = {}
    clean = lambda r: not (FLAGS[r["key"]]["border"] or FLAGS[r["key"]]["wide"])  # noqa: E731
    for tier, name, field, sets_ in TIERS:
        rows = load(name)
        if not rows:
            continue
        t = {}
        for s in sets_:
            rs = [r for r in rows if r["set"] == s]
            if not rs:
                continue
            e = {"mirror": mirror(rs, field)}
            if field == "direction" and s in ("S1", "S3"):
                e["lean"] = lean(rs)
            if s == "S1":
                e["all"] = accuracy(rs, field)
                e["clear_frames"] = accuracy(rs, field, clean)
            elif s == "S3":
                e["all"] = accuracy(rs, field)
                e["majority_share"] = round(max(Counter(r["truth"] for r in rs).values()) / len(rs), 3)
            else:
                e["answers"] = dict(Counter(str(pick(r, field)) for r in rs))
            t[s] = e
        out[tier] = t
    (HERE / "scores.json").write_text(json.dumps(out, indent=1) + "\n")
    for tier, t in out.items():
        s1 = t.get("S1", {}).get("all", {})
        s3 = t.get("S3", {}).get("all", {})
        print(f"{tier:45s} S1 {s1.get('right')}/{s1.get('n')} acc {s1.get('acc_all')} "
              f"cov {s1.get('coverage')} | S3 {s3.get('acc_all')} | "
              f"mirror S1 {t.get('S1', {}).get('mirror', {}).get('rate')} "
              f"S2 {t.get('S2', {}).get('mirror', {}).get('rate')} "
              f"S3 {t.get('S3', {}).get('mirror', {}).get('rate')}")


if __name__ == "__main__":
    main()
