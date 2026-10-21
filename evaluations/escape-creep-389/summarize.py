"""Summarise measure.py's lines: the creeps that started at the bar, by
verb. A creep is every `/action` of it (pieces, since 3.89); its gain is
their sum."""
import json
import statistics
import sys
from collections import defaultdict

rows = [json.loads(l) for l in open(sys.argv[1]) if l.strip()]
groups = defaultdict(list)
for r in rows:
    if r.get("creep"):
        groups[(r["trial"], r["verb"], r.get("room_cm"))].append(r)
out = {}
for verb in ("FORWARD", "REVERSE"):
    cs = []
    for (trial, v, room), pieces in groups.items():
        moved = sum(abs((p["out"].get("result") or {}).get("moved_m") or 0) for p in pieces)
        if v == verb and ((room or 0) >= 15 or moved >= 0.12):
            cs.append((pieces, moved))
    if not cs:
        continue
    secs = [p["seconds"] for pieces, _ in cs for p in pieces]
    speeds = [p["wheels"]["speed_m_s"] for pieces, _ in cs for p in pieces if p.get("wheels")]
    out[verb] = {"creeps": len(cs), "posts": len(secs),
                 "median_s": round(statistics.median(secs), 2), "max_s": max(secs),
                 "over_8": sum(s > 8 for s in secs), "over_10": sum(s > 10 for s in secs),
                 "errors": sum("error" in p["out"] for pieces, _ in cs for p in pieces),
                 "median_speed_m_s": round(statistics.median(speeds), 4) if speeds else None,
                 "median_gain_cm": round(100 * statistics.median(m for _, m in cs), 1),
                 "two_or_more_pieces": sum(len(pieces) > 1 for pieces, _ in cs),
                 "end_gap_cm_max": max(pieces[-1]["gap_cm"] for pieces, _ in cs),
                 "min_gap_cm": min(p["gap_cm"] for pieces, _ in cs for p in pieces)}
verbs = [r for r in rows if "gap_cm" in r]
out["all_verbs"] = {"n": len(verbs), "min_gap_cm": min(r["gap_cm"] for r in verbs),
                    "inside": sum(bool(r["inside"]) for r in verbs),
                    "contacts": sum(r["gap_cm"] < 1.0 for r in verbs)}
print(json.dumps(out, indent=1))
