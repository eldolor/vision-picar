"""3.79: the speed-only A/B, F S F S F S (docs/plans/ros-alignment/3.79-extra-goals.md).

    eval "$(python tools/plan_section.py env)"
    python evaluations/extra-goals-379/run.py [ARMS]      # default FSFSFS

Each run is `tests/demo_explore.py one` (rule, home_first_floor); arm S puts
slow/ on PYTHONPATH with PICAR_SLOW_CHOICE_MS=450. Appends one line per run
to runs.jsonl: the arm, the 1-minute load at start, and demo_explore's result."""
import json
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SLOW_MS = "450"


def main(arms: str) -> None:
    for arm in arms:
        env = dict(os.environ)
        if arm == "S":
            env["PYTHONPATH"] = os.pathsep.join(
                p for p in (str(HERE / "slow"), env.get("PYTHONPATH")) if p)
            env["PICAR_SLOW_CHOICE_MS"] = SLOW_MS
        load = os.getloadavg()[0]
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


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "FSFSFS")
