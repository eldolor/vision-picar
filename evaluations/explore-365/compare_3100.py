"""
evaluations/explore-365/compare_3100.py -- 3.100: the evolutionary (CEM,
`learned`) model against the rule and the gradient (PPO, `network`) model
on amendment 3's 613 fresh houses, plus the 2026-10-12 files for
reproducibility.

    python evaluations/explore-365/compare_3100.py
"""
import json
import math
import os
import statistics

HERE = os.path.dirname(__file__)


def load(name: str) -> dict:
    return json.load(open(os.path.join(HERE, name)))


def rows(d: dict) -> dict:
    return {r["index"]: r for r in d["rows"]}


def found(r: dict) -> bool:
    return r["outcome"] == "found"


def pct(new, old) -> str:
    return f"{100 * (new - old) / old:+.1f}%"


def describe(name: str, d: dict) -> dict:
    rs = list(rows(d).values())
    f = [r for r in rs if found(r)]
    return {"arm": name, "n": len(rs), "found": len(f),
            "clean": d["summary"]["clean"],
            "median_goals": statistics.median(r["goals"] for r in f),
            "median_metres": statistics.median(r["metres"] for r in f),
            "mean_goals": round(statistics.mean(r["goals"] for r in f), 2),
            "mean_metres": round(statistics.mean(r["metres"] for r in f), 2),
            "mean_reward": d["summary"]["mean_reward"]}


def head_to_head(a: dict, b: dict) -> dict:
    """Per house, on the houses both find: who sends fewer goals."""
    ra, rb = rows(a), rows(b)
    both = [h for h in ra if h in rb and found(ra[h]) and found(rb[h])]
    return {"both_found": len(both),
            "a_fewer": sum(ra[h]["goals"] < rb[h]["goals"] for h in both),
            "b_fewer": sum(ra[h]["goals"] > rb[h]["goals"] for h in both),
            "tied": sum(ra[h]["goals"] == rb[h]["goals"] for h in both),
            "a_only": sorted(h for h in ra if found(ra[h]) and not found(rb[h])),
            "b_only": sorted(h for h in rb if found(rb[h]) and not found(ra[h]))}


def diff(a: dict, b: dict) -> list:
    ra, rb = rows(a), rows(b)
    keys = ("outcome", "goals", "metres", "ticks")
    return [h for h in ra if tuple(ra[h].get(k) for k in keys) != tuple(rb[h].get(k) for k in keys)]


def main() -> None:
    today = {"rule": load("fresh-rule-3100.json"), "learned": load("fresh-learned.json"),
             "network": load("fresh-network-3100.json")}
    old = {"rule": load("fresh-rule.json"), "network": load("fresh-network.json")}
    n = len(today["rule"]["rows"])

    print("== Arms on today's dev (613 fresh houses) ==")
    desc = {k: describe(k, v) for k, v in today.items()}
    for d in desc.values():
        print(d)

    rule, cem = desc["rule"], desc["learned"]
    floor = rule["found"] - math.ceil(n / 200)
    print("\n== Criterion 1 (clean) ==")
    for d in desc.values():
        print(f"{d['arm']}: {d['clean']}/{d['n']}", "met" if d["clean"] == d["n"] else "FAILED")
    print("\n== Criterion 2 (learned vs rule) ==")
    print(f"found {cem['found']} vs bar >= {floor}:", "met" if cem["found"] >= floor else "FAILED")
    g, m = cem["median_goals"], cem["median_metres"]
    print(f"median goals {g} vs {rule['median_goals']} ({pct(g, rule['median_goals'])}); "
          f"metres {m} vs {rule['median_metres']} ({pct(m, rule['median_metres'])})")
    ok = g <= 0.8 * rule["median_goals"] or m <= 0.8 * rule["median_metres"]
    print("20% half:", "met" if ok else "FAILED")

    print("\n== Criterion 3 (learned vs network, per house) ==")
    print("learned=a, network=b:", head_to_head(today["learned"], today["network"]))
    print("learned=a, rule=b:", head_to_head(today["learned"], today["rule"]))
    print("network=a, rule=b:", head_to_head(today["network"], today["rule"]))

    print("\n== Criterion 4 (median-goal saving: 200 test houses -> 613 fresh) ==")
    for arm in ("learned", "network"):
        t = [load(f"test-{arm}-{lo}.json") for lo in (0, 200)]
        r = [load(f"test-rule-{lo}.json") for lo in (0, 200)]
        tg = statistics.median(x["goals"] for d in t for x in d["rows"] if found(x))
        rg = statistics.median(x["goals"] for d in r for x in d["rows"] if found(x))
        print(f"{arm}: test {pct(tg, rg)} -> fresh {pct(desc[arm]['median_goals'], rule['median_goals'])}")

    print("\n== Criterion 5 (today vs 2026-10-12, house by house) ==")
    for arm, d in old.items():
        changed = diff(today[arm], d)
        print(f"{arm}: {len(changed)} of {n} houses differ; recorded {d['summary']}", changed[:20])


if __name__ == "__main__":
    main()
