"""
evaluations/goal-learn-3125/run.py -- 3.125: `explore_goal_policy: rooms`
with no room labels against the rule, on 3.96's 588 test houses, once.

    python evaluations/goal-learn-3125/run.py run OUT_DIR     # both arms, resumable
    python evaluations/goal-learn-3125/run.py score OUT_DIR   # criteria 1-4

`run` holds the desktop's heavy-run lock (`.git/heavy-run.lock`,
PARALLEL-SESSIONS rule 7) until it exits. Each arm-mode pair is one file
(`<arm>-<mode>.json`), written once it is complete.
"""
import fcntl
import json
import os
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "evaluations", "goal-learn-396"))
import c as C                                                 # noqa: E402
import train as T                                             # noqa: E402

ARMS = {"rule": "rule", "rooms": "wired"}
MODES = ("R", "U")


def heavy_lock():
    """tests/demo_explore.heavy_lock's lock, held for this process's life."""
    common = subprocess.run(["git", "rev-parse", "--git-common-dir"], capture_output=True,
                            text=True, cwd=ROOT).stdout.strip()
    path = os.path.join(os.path.abspath(os.path.join(ROOT, common)), "heavy-run.lock")
    f = open(path, "a+")
    try:
        fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        f.seek(0)
        print(f"waiting for the heavy-run lock ({path}): {f.read().strip() or 'held'}", flush=True)
        fcntl.flock(f, fcntl.LOCK_EX)
    f.seek(0)
    f.truncate()
    f.write(f"pid {os.getpid()} 3.125 {' '.join(sys.argv)[:200]} since {time.ctime()}\n")
    f.flush()
    return f


def _job(args):
    s, h, mode, spec = args
    try:
        r = T.mission(s, h, mode, spec, 0, 0.0)
    except Exception as e:  # noqa: BLE001 -- a house that cannot start is a row
        r = {"split": s, "index": h, "mode": mode, "outcome": "exception",
             "error": f"{type(e).__name__}: {e}", "goals": 0, "metres": 0.0,
             "reward": -T.CRASH_COST}
    r.pop("choices", None)
    return r


def cmd_run(out: str) -> None:
    lock = heavy_lock()  # noqa: F841 -- held until exit
    os.makedirs(out, exist_ok=True)
    hs = C.test_houses()
    print(f"{len(hs)} test houses, {T.WORKERS} workers, lock held", flush=True)
    with ProcessPoolExecutor(T.WORKERS) as ex:
        for arm, spec in ARMS.items():
            for mode in MODES:
                path = os.path.join(out, f"{arm}-{mode}.json")
                if os.path.exists(path):
                    continue
                t0 = time.time()
                rows = list(ex.map(_job, [(s, h, mode, spec) for s, h in hs], chunksize=2))
                T.save_atomic(path, lambda p: json.dump(rows, open(p, "w")))
                print(f"{arm} {mode} ({time.time() - t0:.0f} s): {T.summary(rows)}", flush=True)


def cmd_score(out: str) -> dict:
    rows = {(a, m): json.load(open(os.path.join(out, f"{a}-{m}.json"))) for a in ARMS for m in MODES}
    want = sorted(h for _, h in C.test_houses())
    for k, rs in rows.items():
        if sorted(r["index"] for r in rs) != want:
            raise SystemExit(f"{k}: not the {len(want)} test houses -- not scoring")
    res = {f"{a}-{m}": T.summary(r) for (a, m), r in rows.items()}
    verdict = {}
    for m in MODES:
        rule, new = res[f"rule-{m}"], res[f"rooms-{m}"]
        unclean = sum(1 for r in rows[("rooms", m)] if r["outcome"] != "no_target"
                      and (not r["outcome"] or r["outcome"] == "exception" or r.get("capped")))
        c = {"1_finds": new["found"] >= 0.99 * rule["found"],
             "2_goals": (new["median_goals_found"] is not None
                         and new["median_goals_found"] < rule["median_goals_found"]),
             "3_metres": (new["median_metres_found"] is not None
                          and new["median_metres_found"] <= rule["median_metres_found"]),
             "4_clean": unclean == 0}
        verdict[m] = {**c, "unclean": unclean,
                      "goal_saving": C._saving(new["median_goals_found"], rule["median_goals_found"]),
                      "metre_saving": C._saving(new["median_metres_found"], rule["median_metres_found"]),
                      "met": all(c.values())}

    def found(arm, m):
        return {r["index"] for r in rows[(arm, m)] if r["outcome"] == "found"}
    only = {m: {"rule_only": sorted(found("rule", m) - found("rooms", m)),
                "rooms_only": sorted(found("rooms", m) - found("rule", m))} for m in MODES}
    # The rule arm against 3.96.C's recorded rows (information).
    old = os.path.join(ROOT, "evaluations", "goal-learn-396", "test-c")
    drift = {}
    for m in MODES:
        then = {r["index"]: (r["outcome"], r["goals"], r["metres"])
                for r in json.load(open(os.path.join(old, f"rule-{m}.json")))}
        drift[m] = sum(1 for r in rows[("rule", m)]
                       if then.get(r["index"]) != (r["outcome"], r["goals"], r["metres"]))
    report = {"summary": res, "verdict": verdict, "all_met": all(v["met"] for v in verdict.values()),
              "found_by_one_arm": only, "rule_rows_differing_from_396": drift}
    T.save_atomic(os.path.join(out, "score.json"), lambda p: json.dump(report, open(p, "w"), indent=1))
    print(json.dumps(report, indent=1))
    return report


if __name__ == "__main__":
    {"run": cmd_run, "score": cmd_score}[sys.argv[1]](sys.argv[2])
