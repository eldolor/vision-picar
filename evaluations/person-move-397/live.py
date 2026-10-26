"""
evaluations/person-move-397/live.py -- 3.97 criterion 4, live.

A person's D-pad move during an active nav2 goal ends the goal, and nav2
does not drive on after it, under `drive: ros` and under `drive: direct`
with the ROS world. One fresh stack per drive (tests/demo_explore.stack()),
R5's mapping lap, then two taps (a 15 deg LEFT, 3.15's, and a FORWARD),
each on a fresh goal once nav2 reports it active and 1.5 s more. The method
and bars are in docs/plans/ros-alignment/3.97-nudge-goal-label.md.

    eval "$(python tools/plan_section.py env)"
    PICAR_ROS_NETWORK=host SIM_MAP=scaled_house python evaluations/person-move-397/live.py [ros direct]

Writes evaluations/person-move-397/live/<UTC stamp>/runs.jsonl.
"""

import json
import math
import os
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
os.environ.setdefault("SIM_MAP", "scaled_house")

from tests import demo_explore as ex  # noqa: E402
from tests import demo_nav_goals as nav  # noqa: E402

TAPS = [("LEFT", {"angle": 15}), ("FORWARD", {"speed": 50, "duration": 0.5})]
WATCH_S = 5.0          # after the verb: does nav2 drive on?
SETTLE_S = 1.0         # the verb's own coast before the watch starts


def truth(robot):
    t = robot.get("/world/truth").json()
    return t["x_m"], t["y_m"]


def goal_state(robot):
    return (robot.get("/world/goal").json().get("goal") or {}).get("state")


def farthest_goal(robot):
    x, y = truth(robot)
    return max(nav.GOALS, key=lambda g: math.hypot(g[1] - x, g[2] - y))


def tap_once(robot, action, kw):
    name, gx, gy = farthest_goal(robot)
    sent = robot.post("/world/goal", json={"x_m": gx, "y_m": gy}).json()
    deadline = time.time() + 30
    while time.time() < deadline and goal_state(robot) != "active":
        time.sleep(0.1)
    if goal_state(robot) != "active":
        return {"tap": action, "goal": name, "invalid": "goal never active", "sent": sent}
    time.sleep(1.5)
    seen = {}
    poller = nav.client()
    t_tap = time.time()

    def poll():
        while time.time() - t_tap < 6 and "canceled" not in seen:
            st = (poller.get("/world/goal").json().get("goal") or {}).get("state")
            seen.setdefault(st, round(time.time() - t_tap, 3))
            time.sleep(0.03)
    th = threading.Thread(target=poll)
    th.start()
    reply = robot.post("/action", json={"action": action, **kw},
                       headers={"x-driver": "twin-dpad"}).json()
    replied = round(time.time() - t_tap, 3)
    th.join()
    time.sleep(SETTLE_S)
    x0, y0 = truth(robot)
    moved, states = 0.0, set()
    end = time.time() + WATCH_S
    while time.time() < end:
        x, y = truth(robot)
        moved = max(moved, math.hypot(x - x0, y - y0))
        states.add(goal_state(robot))
        time.sleep(0.1)
    canceled = seen.get("canceled")
    row = {"tap": action, "goal": name, "executed": reply.get("executed"),
           "reason": reply.get("reason"), "verb_reply_s": replied,
           "tap_to_canceled_s": canceled, "states_seen": seen,
           "moved_after_cm": round(moved * 100, 1), "states_after": sorted(map(str, states))}
    row["met"] = bool(row["executed"] and canceled is not None and canceled <= 1.0
                      and row["moved_after_cm"] < 3.0
                      and not ({"active", "pending"} & states))
    return row


def run_drive(drive, out):
    robot = ex.stack(os.environ["SIM_MAP"], drive=drive)
    rows = []
    try:
        br = nav.bridge()
        end = time.time() + 35.0
        while not br.get("/slam/pose").json().get("start_truth"):
            if time.time() > end:
                raise RuntimeError("the bridge recorded no start truth")
            time.sleep(0.2)
        nav.map_first(robot)
        for action, kw in TAPS:
            row = {"drive": drive, "run_dir": ex.RUN_DIR, **tap_once(robot, action, kw)}
            rows.append(row)
            print(json.dumps(row), flush=True)
            out.write(json.dumps(row) + "\n")
            out.flush()
            time.sleep(1.5)                  # the D-pad's authority lapses
    finally:
        ex._kill_ports()
    return rows


def main(drives):
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    out_dir = os.path.join(ROOT, "evaluations", "person-move-397", "live", stamp)
    os.makedirs(out_dir)
    with open(os.path.join(out_dir, "runs.jsonl"), "w") as out:
        for drive in drives:
            for attempt in (1, 2):
                try:
                    run_drive(drive, out)
                    break
                except Exception as e:  # noqa: BLE001 -- recorded, re-run once (the method)
                    rec = {"drive": drive, "attempt": attempt, "died": repr(e)[:300]}
                    print(json.dumps(rec), flush=True)
                    out.write(json.dumps(rec) + "\n")
                    out.flush()
    print("written", out_dir)


if __name__ == "__main__":
    main(sys.argv[1:] or ["ros", "direct"])
