"""3.96 criterion 7: live explore missions on nav2, the rule against
`explore_goal_policy: rooms`, alternated (default R N R N R N).

    eval "$(python tools/plan_section.py env)"   # run with the repo .venv python
    python evaluations/goal-learn-396/live.py [ARMS]

Each run is `tests/demo_explore.py one` (home_first_floor, a fresh stack);
arm N sets EXPLORE_GOAL_POLICY=rooms for its brain. A run starts only when
no other live run holds the heavy-run lock and the 1-minute load has stayed
under QUIET_LOAD for QUIET_CHECKS minutes (3.79's amendment 1). Appends one
line per run to live/runs.jsonl."""
import json
import os
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
POLICY = {"R": "rule", "N": "rooms"}
QUIET_LOAD = 4.0
QUIET_CHECKS = 3          # in a row, a minute apart


def heavy_lock_free() -> bool:
    import fcntl
    # The same file, and the same /tmp fallback, as tests/demo_explore.heavy_lock().
    common = subprocess.run(["git", "rev-parse", "--git-common-dir"], cwd=ROOT,
                            capture_output=True, text=True).stdout.strip() or "/tmp"
    path = (ROOT / common).resolve() / "heavy-run.lock"
    with open(path, "a+") as f:
        try:
            fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return False
        fcntl.flock(f, fcntl.LOCK_UN)
        return True


def wait_quiet() -> float:
    quiet = 0
    while True:
        ok = heavy_lock_free() and os.getloadavg()[0] < QUIET_LOAD
        quiet = quiet + 1 if ok else 0
        if quiet >= QUIET_CHECKS:
            return os.getloadavg()[0]
        time.sleep(60)


def main(arms: str) -> None:
    out_dir = HERE / "live"
    out_dir.mkdir(exist_ok=True)
    for arm in arms:
        env = dict(os.environ, EXPLORE_GOAL_POLICY=POLICY[arm])
        # Run as a script, demo_explore.py has tests/ on its path, not the
        # repo: `import sim` needs the root (3.79's runner does the same).
        env["PYTHONPATH"] = os.pathsep.join([str(ROOT)] + [p for p in [env.get("PYTHONPATH")] if p])
        load = wait_quiet()
        started = time.strftime("%Y-%m-%dT%H:%M:%S")
        out = subprocess.run([sys.executable, "tests/demo_explore.py", "one"], cwd=ROOT,
                             env=env, capture_output=True, text=True)
        lines = [ln for ln in out.stdout.splitlines() if ln.startswith("{")]
        rec = {"arm": arm, "policy": POLICY[arm], "started": started,
               "load_1m": round(load, 2), "exit": out.returncode}
        rec.update(json.loads(lines[-1]) if lines else {"error": out.stderr[-500:]})
        if "waiting for the heavy-run lock" in out.stderr:
            # Another live run took the lock between the gate and the run's own.
            rec["invalid"] = "waited for another live run after the quiet gate"
        with open(out_dir / "runs.jsonl", "a") as f:
            f.write(json.dumps(rec) + "\n")
        print(json.dumps({k: rec.get(k) for k in ("arm", "load_1m", "outcome", "seconds",
                                                  "steps", "contact_samples", "travel_m",
                                                  "invalid")}), flush=True)
        if "error" in rec:
            raise SystemExit(rec["error"])


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "RNRNRN")
