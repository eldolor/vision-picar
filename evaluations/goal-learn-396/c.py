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
    try:
        r = T.mission(s, h, mode, spec, 0, drop)
    except Exception as e:  # noqa: BLE001 -- a house that cannot even start is a row, not a lost run
        r = {"split": s, "index": h, "mode": mode, "outcome": "exception",
             "error": f"{type(e).__name__}: {e}", "goals": 0, "metres": 0.0,
             "reward": -T.CRASH_COST}
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


def _saving(new, rule):
    """1 - new/rule, rounded so a saving exactly at a bar (4 vs 5 goals)
    reads as the bar; None when either side found nothing."""
    if new is None or not rule:
        return None
    return round(1 - new / rule, 6)


def cmd_score(out: str) -> dict:
    rows = {(a, m): json.load(open(os.path.join(out, f"{a}-{m}.json")))
            for a in ARMS for m in ("R", "U")}
    # Run once: every file must hold the same houses, the whole test set.
    want = sorted(h for _, h in test_houses())
    for k, rs in rows.items():
        got = sorted(r["index"] for r in rs)
        if got != want:
            raise SystemExit(f"{k}: {len(got)} houses, not the {len(want)} test houses -- not scoring")
    res = {f"{a}-{m}": T.summary(r) for (a, m), r in rows.items()}
    verdict = {}
    for m in ("R", "U"):
        rule = res[f"rule-{m}"]
        for arm in ("F-large", "F-large-drop"):
            new = res[f"{arm}-{m}"]
            g = _saving(new["median_goals_found"], rule["median_goals_found"])
            d = _saving(new["median_metres_found"], rule["median_metres_found"])
            found_ok = new["found"] >= 0.99 * rule["found"]
            if arm == "F-large":
                # Criterion 2 (amendment 2): R saves 20% of goals or metres; U no more goals.
                saving_ok = (((g or -1) >= 0.20 or (d or -1) >= 0.20) if m == "R"
                             else g is not None and g >= 0)
            else:
                # Criterion 3 (amendment 4): the found bar in both modes; a 10% goal
                # saving in mode R; mode U's goals reported only.
                saving_ok = (g is not None and g >= 0.10) if m == "R" else None
            verdict[f"{arm}-{m}"] = {"found_ok": found_ok, "goal_saving": g, "metre_saving": d,
                                     "saving_ok": saving_ok,
                                     "met": found_ok and saving_ok is not False}
    # Criterion 4: houses the rule finds and F-large does not.

    def found(arm, m):
        return {r["index"] for r in rows[(arm, m)] if r["outcome"] == "found"}
    lost = {m: sorted(found("rule", m) - found("F-large", m)) for m in ("R", "U")}
    # Criterion 5, per arm: every mission ends with an outcome under the tick cap.
    clean = {a: sum(1 for m in ("R", "U") for r in rows[(a, m)] if r["outcome"] != "no_target"
                    and (not r["outcome"] or r["outcome"] == "exception" or r.get("capped")))
             for a in ARMS}
    drop_rows = [r for m in ("R", "U") for r in rows[("F-large-drop", m)] if r.get("rooms")]
    dropped = round(sum(r["dropped"] for r in drop_rows) / max(1, sum(r["rooms"] for r in drop_rows)), 4)
    report = {"summary": res, "verdict": verdict, "lost_vs_rule": lost,
              "unclean_missions": clean, "dropped_room_share": dropped}
    T.save_atomic(os.path.join(out, "score.json"), lambda p: json.dump(report, open(p, "w"), indent=1))
    print(json.dumps(report, indent=1))
    return report


if __name__ == "__main__":
    {"run": cmd_run, "score": cmd_score}[sys.argv[1]](sys.argv[2])
