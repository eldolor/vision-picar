"""
evaluations/goal-learn-396/c.py -- 3.96 C: the test houses, once.

    python evaluations/goal-learn-396/c.py run OUT_DIR     # every arm below, resumable
    python evaluations/goal-learn-396/c.py score OUT_DIR   # criteria 2-5 from OUT_DIR

The test houses are ProcTHOR `train` 1000-1999 (588 usable), never used to
train or choose. Arms, each in target modes R and U:

* **rule**: today's default.
* **net365**: 3.65's shipped network (`brain/explore_goal_net.json`).
* **F-large**: B4's winner, greedy, at its best selection check
  (`cloud/ckpt/F-large/best.json`).
* **F-large-drop**: the same, with the labels of 50% of each house's rooms
  withheld (criterion 3).

Each arm-mode pair is one file (`<arm>-<mode>.json`), written once it is
complete, so a rerun skips what is done.
"""
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import train as T                                             # noqa: E402
from sim.maps.procthor import usable                          # noqa: E402

ARMS = ("rule", "net365", "F-large", "F-large-drop")
DROP = 0.5
BEST = os.path.join(HERE, "cloud", "ckpt", "F-large", "best.json")


def test_houses() -> list:
    hs = [("train", h) for h in usable("train") if 1000 <= h < 2000]
    return hs[:3] if os.environ.get("SMOKE") == "1" else hs


def _spec(arm: str):
    if arm in ("rule", "net365"):
        return arm, 0.0
    b = json.load(open(BEST))
    return ("greedy", b["kind"], b["params"]), (DROP if arm.endswith("-drop") else 0.0)


def _job(args):
    s, h, mode, spec, drop = args
    r = T.mission(s, h, mode, spec, 0, drop)
    r.pop("choices", None)
    return r


def cmd_run(out: str) -> None:
    os.makedirs(out, exist_ok=True)
    hs = test_houses()
    print(f"{len(hs)} test houses", flush=True)
    with ProcessPoolExecutor(T.WORKERS) as ex:
        for arm in ARMS:
            spec, drop = _spec(arm)
            for mode in ("R", "U"):
                path = os.path.join(out, f"{arm}-{mode}.json")
                if os.path.exists(path):
                    continue
                rows = list(ex.map(_job, [(s, h, mode, spec, drop) for s, h in hs], chunksize=2))
                T.save_atomic(path, lambda p: json.dump(rows, open(p, "w")))
                print(f"{arm} {mode}: {T.summary(rows)}", flush=True)


def cmd_score(out: str) -> dict:
    rows = {(a, m): json.load(open(os.path.join(out, f"{a}-{m}.json")))
            for a in ARMS for m in ("R", "U")}
    res = {f"{a}-{m}": T.summary(r) for (a, m), r in rows.items()}
    verdict = {}
    for arm, saving in (("F-large", 0.20), ("F-large-drop", 0.10)):
        for m in ("R", "U"):
            rule, new = res[f"rule-{m}"], res[f"{arm}-{m}"]
            found_ok = new["found"] >= 0.99 * rule["found"]
            g = 1 - new["median_goals_found"] / rule["median_goals_found"]
            d = 1 - new["median_metres_found"] / rule["median_metres_found"]
            # Mode U asks only for no more median goals (amendment 2);
            # criterion 3 asks for its saving in mode R.
            fewer = (g >= saving or d >= saving) if m == "R" else g >= 0
            verdict[f"{arm}-{m}"] = {"found_ok": found_ok, "goal_saving": round(g, 4),
                                     "metre_saving": round(d, 4), "saving_ok": fewer,
                                     "met": found_ok and fewer}
    # Criterion 4: houses the rule finds and F-large does not.
    def found(arm, m):
        return {r["index"] for r in rows[(arm, m)] if r["outcome"] == "found"}
    lost = {m: sorted(found("rule", m) - found("F-large", m)) for m in ("R", "U")}
    # Criterion 5: every mission ends with an outcome under the tick cap.
    clean = all(r["outcome"] and r["outcome"] != "exception" and not r.get("capped")
                for rs in rows.values() for r in rs if r["outcome"] != "no_target")
    report = {"summary": res, "verdict": verdict, "lost_vs_rule": lost, "clean": clean}
    json.dump(report, open(os.path.join(out, "score.json"), "w"), indent=1)
    print(json.dumps(report, indent=1))
    return report


if __name__ == "__main__":
    {"run": cmd_run, "score": cmd_score}[sys.argv[1]](sys.argv[2])
