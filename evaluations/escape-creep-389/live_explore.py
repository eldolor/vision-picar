"""3.89 criteria 3-4 live: explore missions in home_first_floor under
`drive: ros` through the brain's HTTP API, each on a fresh stack
(docs/plans/ros-alignment/3.89-escape-creep-timeout.md).

    eval "$(python tools/plan_section.py env)"
    ROS_IMAGE=vision-picar-ros:latest python evaluations/escape-creep-389/live_explore.py 3 > runs.jsonl

One JSON line per mission: the outcome, ground-truth contacts (as
`tests/demo_explore.run_mission` counts them), every escape `creep` event
from the brain's log (piece, cut by time) and any request that timed out."""
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from tests import demo_explore as demo         # noqa: E402

HOUSE = "home_first_floor"


def brain_events(run_dir):
    creeps, timeouts = [], 0
    for line in open(os.path.join(run_dir, "explore_brain.log")):
        if "timed out" in line and "POST /action" in line:
            timeouts += 1
        if '"event": "creep"' in line:
            creeps.append(json.loads(line[line.index("{"):]))
    return creeps, timeouts


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    for i in range(n):
        robot = demo.stack(HOUSE)
        try:
            result = demo.run_mission(robot, HOUSE)
        finally:
            demo._kill_ports()
            os.system(f"docker rm -f {demo.CONTAINER} >/dev/null 2>&1")
        creeps, timeouts = brain_events(demo.RUN_DIR)
        print(json.dumps({"run": i + 1, "house": HOUSE,
                          **{k: result.get(k) for k in ("outcome", "seconds", "travel_m", "steps",
                                                         "min_gap_cm", "contact_samples",
                                                         "inside_samples", "stuck_episodes")},
                          "creeps": [{k: c.get(k) for k in ("way", "executed", "piece",
                                                            "cut_by_time", "forward")}
                                     for c in creeps],
                          "action_timeouts": timeouts, "last_log": result.get("last_log"),
                          "run_dir": demo.RUN_DIR}), flush=True)


if __name__ == "__main__":
    main()
