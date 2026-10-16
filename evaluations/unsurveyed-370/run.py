"""3.70 M5, live half: 3.31 criterion 1 on an unsurveyed house
(docs/plans/ros-alignment/3.70-survey-and-mission-ends.md, M5).

    eval "$(python tools/plan_section.py env)"   # run with the repo .venv python
    python evaluations/unsurveyed-370/run.py [ROOM ...]   # default: every room

Each room is `tests/demo_explore.py rooms home_first_floor <room>`: a fresh
stack (empty map, no survey), the backpack moved into that room, one
`explore` mission from the foyer. A room starts only when no other live run
holds the heavy-run lock and three load checks a minute apart are all under
4 (3.79's amendment 1: runs at a higher load are not valid); a room that
then had to wait for another run's lock is marked invalid. Refuses to start
without the branch's environment, so a run never lands on the shared
`:latest` image or another session's ports. Appends one line per room to
rooms.jsonl (or $ROOMS_OUT): the image, the 1-minute load at start, the exit code and
demo_explore's row."""
import json
import os
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "evaluations" / "extra-goals-379"))

from run import wait_quiet                     # noqa: E402  3.79's quiet wait
from sim.maps import build_world               # noqa: E402

HOUSE = "home_first_floor"


def main(rooms):
    missing = [k for k in ("ROS_IMAGE", "PICAR_ROBOT_PORT", "PICAR_BRAIN_PORT",
                           "PICAR_BRIDGE_PORT", "PICAR_ROS_CONTAINER") if not os.environ.get(k)]
    if missing:
        raise SystemExit(f"run `eval \"$(python tools/plan_section.py env)\"` first: {missing} unset")
    rooms = rooms or sorted(build_world(HOUSE).rooms)
    for room in rooms:
        load = wait_quiet()
        out = subprocess.run([sys.executable, "-m", "tests.demo_explore", "rooms", HOUSE, room],
                             cwd=ROOT, capture_output=True, text=True)
        rows = [json.loads(line) for line in out.stdout.splitlines() if line.startswith("{")]
        rec = {"image": os.environ["ROS_IMAGE"], "load_at_start": round(load, 2),
               "at": time.strftime("%Y-%m-%dT%H:%M:%S"), "exit": out.returncode}
        rec.update(rows[-1] if rows else {"room": room, "error": out.stderr[-2000:]})
        if out.returncode and rows:
            rec["stderr_tail"] = out.stderr[-2000:]
        if "waiting for the heavy-run lock" in out.stderr:
            # Another live run took the lock between the gate and this run's
            # own lock: it started without amendment 1's quiet checks.
            rec["invalid"] = "amendment 1: waited for another live run after the gate"
        with open(os.environ.get("ROOMS_OUT") or HERE / "rooms.jsonl", "a") as f:
            f.write(json.dumps(rec) + "\n")


if __name__ == "__main__":
    main(sys.argv[1:])
