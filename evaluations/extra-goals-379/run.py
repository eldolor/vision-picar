"""3.79: the speed-only A/B, F S F S F S (docs/plans/ros-alignment/3.79-extra-goals.md).

    eval "$(python tools/plan_section.py env)"   # run with the repo .venv python
    python evaluations/extra-goals-379/run.py [ARMS]      # default FSFSFS

Each run is `tests/demo_explore.py one` (rule, home_first_floor); arm S puts
slow/ on PYTHONPATH with PICAR_SLOW_CHOICE_MS=450. Appends one line per run
to runs.jsonl: the arm, the 1-minute load at start, and demo_explore's result."""
import json
import os
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SLOW_MS = "450"
QUIET_LOAD = 4.0          # 1-minute load on this 8-core laptop (amendment 1)
QUIET_CHECKS = 3          # in a row, a minute apart


def heavy_lock_free() -> bool:
    import fcntl
    path = ROOT / subprocess.run(["git", "rev-parse", "--git-common-dir"], cwd=ROOT,
                                 capture_output=True, text=True).stdout.strip() / "heavy-run.lock"
    with open(path, "a+") as f:
        try:
            fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return False
        fcntl.flock(f, fcntl.LOCK_UN)
        return True


def wait_quiet() -> float:
    """Amendment 1: start a run only when no other live run holds the lock
    and the load has stayed under QUIET_LOAD for QUIET_CHECKS minutes."""
    quiet = 0
    while True:
        ok = heavy_lock_free() and os.getloadavg()[0] < QUIET_LOAD
        quiet = quiet + 1 if ok else 0
        if quiet >= QUIET_CHECKS:
            break
        time.sleep(60)
    return os.getloadavg()[0]


def main(arms: str) -> None:
    for arm in arms:
        env = dict(os.environ)
        paths = [str(ROOT)] + ([str(HERE / "slow")] if arm == "S" else [])
        env["PYTHONPATH"] = os.pathsep.join(paths + [p for p in [env.get("PYTHONPATH")] if p])
        if arm == "S":
            env["PICAR_SLOW_CHOICE_MS"] = SLOW_MS
        load = wait_quiet()
        out = subprocess.run([sys.executable, "tests/demo_explore.py", "one"], cwd=ROOT,
                             env=env, capture_output=True, text=True)
        lines = [l for l in out.stdout.splitlines() if l.startswith("{")]
        rec = {"arm": arm, "load_1m": round(load, 2), "exit": out.returncode}
        rec.update(json.loads(lines[-1]) if lines else {"error": out.stderr[-500:]})
        with open(HERE / "runs.jsonl", "a") as f:
            f.write(json.dumps(rec) + "\n")
        print(json.dumps({k: rec.get(k) for k in ("arm", "load_1m", "outcome", "seconds",
                                                  "steps", "contact_samples", "travel_m")}),
              flush=True)
        if "error" in rec:
            raise SystemExit(rec["error"])


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "FSFSFS")
