"""
evaluations/goal-learn-3101/test.py -- 3.101: the gate, then the 588 test
houses once, then the criteria.

    python evaluations/goal-learn-3101/test.py gate OUT_DIR    # criterion S
    python evaluations/goal-learn-3101/test.py run OUT_DIR     # every arm, resumable (refuses unless S is met)
    python evaluations/goal-learn-3101/test.py score OUT_DIR   # criteria 1-4 from OUT_DIR

Arms, each in modes R and U, on today's code: `rule`, `F-large` (3.96's
best.json), `F-rich` (this section's best.json), and `F-rich-drop` (50% of
each house's room labels withheld; reported only).
"""
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..")))
sys.path.insert(0, os.path.join(HERE, "..", "goal-learn-396"))
sys.path.insert(0, HERE)
os.environ.setdefault("WORKERS", str(os.cpu_count() or 4))
import c as C396                                              # noqa: E402
import train as T                                             # noqa: E402
from rich import rich_features                                # noqa: E402
from run import F_LARGE                                       # noqa: E402

ARMS = ("rule", "F-large", "F-rich", "F-rich-drop")
F_RICH = os.path.join(HERE, "run", "F-rich", "best.json")
BASELINE = os.path.join(HERE, "run", "baseline-select.json")


def _spec(arm: str):
    if arm == "rule":
        return "rule", 0.0
    b = json.load(open(F_LARGE if arm == "F-large" else F_RICH))
    return ("greedy", b["kind"], b["params"]), (C396.DROP if arm.endswith("-drop") else 0.0)


def _job(args):
    arm, s, h, mode, spec, drop = args
    T.EXTRA_FEATURES = rich_features if arm.startswith("F-rich") else None
    try:
        r = T.mission(s, h, mode, spec, 0, drop)
    except Exception as e:  # noqa: BLE001 -- a house that cannot start is a row, not a lost run
        r = {"split": s, "index": h, "mode": mode, "outcome": "exception",
             "error": f"{type(e).__name__}: {e}", "goals": 0, "metres": 0.0,
             "reward": -T.CRASH_COST}
    r.pop("choices", None)
    return r


def gate() -> dict:
    """Criterion S: F-rich's best selection check against F-large's today."""
    rich = json.load(open(F_RICH))["best_selection"]["R"]["mean_reward"]
    large = json.load(open(BASELINE))["today"]["R"]["mean_reward"]
    return {"F-rich": rich, "F-large": large, "met": rich > large}


def cmd_gate(out: str) -> dict:
    g = gate()
    os.makedirs(out, exist_ok=True)
    T.save_atomic(os.path.join(out, "gate.json"), lambda p: json.dump(g, open(p, "w"), indent=1))
    print(json.dumps(g))
    return g


def cmd_run(out: str) -> None:
    if not gate()["met"]:
        raise SystemExit("criterion S not met: F-rich does not go to the test houses")
    os.makedirs(out, exist_ok=True)
    hs = C396.test_houses()
    print(f"{len(hs)} test houses", flush=True)
    with ProcessPoolExecutor(T.WORKERS) as ex:
        for arm in ARMS:
            spec, drop = _spec(arm)
            for mode in ("R", "U"):
                path = os.path.join(out, f"{arm}-{mode}.json")
                if os.path.exists(path):
                    continue
                rows = list(ex.map(_job, [(arm, s, h, mode, spec, drop) for s, h in hs],
                                   chunksize=2))
                T.save_atomic(path, lambda p: json.dump(rows, open(p, "w")))
                print(f"{arm} {mode}: {T.summary(rows)}", flush=True)


def cmd_score(out: str) -> dict:
    have = [a for a in ARMS if all(os.path.exists(os.path.join(out, f"{a}-{m}.json"))
                                   for m in ("R", "U"))]
    missing = [a for a in ("rule", "F-large", "F-rich") if a not in have]
    if missing:
        raise SystemExit(f"arms not run yet: {missing} -- not scoring")
    rows = {(a, m): json.load(open(os.path.join(out, f"{a}-{m}.json"))) for a in have
            for m in ("R", "U")}
    want = sorted(h for _, h in C396.test_houses())
    for k, rs in rows.items():
        if sorted(r["index"] for r in rs) != want:
            raise SystemExit(f"{k}: not the {len(want)} test houses -- not scoring")
    res = {f"{a}-{m}": T.summary(r) for (a, m), r in rows.items()}
    L = {m: res[f"F-large-{m}"] for m in ("R", "U")}
    N = {m: res[f"F-rich-{m}"] for m in ("R", "U")}
    unclean = sum(1 for m in ("R", "U") for r in rows[("F-rich", m)] if r["outcome"] != "no_target"
                  and (r["outcome"] == "exception" or r.get("capped")))
    crit = {
        "1_reward": {"F-rich": N["R"]["mean_reward"], "bar": round(L["R"]["mean_reward"] + 0.02, 4),
                     "met": N["R"]["mean_reward"] >= L["R"]["mean_reward"] + 0.02},
        "2_finds": {m: {"F-rich": N[m]["found"], "bar": round(0.99 * L[m]["found"], 2),
                        "met": N[m]["found"] >= 0.99 * L[m]["found"]} for m in ("R", "U")},
        "3_trips": {m: {"F-rich": N[m]["median_goals_found"], "bar": L[m]["median_goals_found"],
                        "met": N[m]["median_goals_found"] <= L[m]["median_goals_found"]}
                    for m in ("R", "U")},
        "4_clean": {"unclean": unclean, "met": unclean == 0},
    }
    crit["won"] = (crit["1_reward"]["met"] and all(v["met"] for v in crit["2_finds"].values())
                   and all(v["met"] for v in crit["3_trips"].values()) and crit["4_clean"]["met"])

    def found(arm, m):
        return {r["index"] for r in rows[(arm, m)] if r["outcome"] == "found"}
    vs_large = {m: {"gained": sorted(found("F-rich", m) - found("F-large", m)),
                    "lost": sorted(found("F-large", m) - found("F-rich", m))} for m in ("R", "U")}
    vs_rule = {}
    for m in ("R", "U"):
        rule = res[f"rule-{m}"]
        for arm in ("F-large", "F-rich"):
            new = res[f"{arm}-{m}"]
            vs_rule[f"{arm}-{m}"] = {"goal_saving": C396._saving(new["median_goals_found"],
                                                                 rule["median_goals_found"]),
                                     "metre_saving": C396._saving(new["median_metres_found"],
                                                                  rule["median_metres_found"]),
                                     "found_vs_rule": new["found"] - rule["found"]}
    report = {"summary": res, "criteria": crit, "vs_F_large": vs_large, "vs_rule": vs_rule}
    T.save_atomic(os.path.join(out, "score.json"), lambda p: json.dump(report, open(p, "w"), indent=1))
    print(json.dumps(report, indent=1))
    return report


if __name__ == "__main__":
    import multiprocessing
    multiprocessing.set_start_method("fork", force=True)
    {"gate": cmd_gate, "run": cmd_run, "score": cmd_score}[sys.argv[1]](sys.argv[2])
