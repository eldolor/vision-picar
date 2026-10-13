"""
python -m tests.demo_survey [house]

PLAN-ros-alignment.md 3.70 S1, live: one survey through the brain's HTTP API
on the ROS stack (`tests/demo_explore.stack()`: robot server, brain and the
ROS container, sim only, no paid calls -- the sim's frames carry their room,
which stands in for the label source).

S1's bars, judged here on ground truth:
* ends `exhausted`, within 30 min (the survey's own limit, 1800 s);
* coverage >= 95% of truth-reachable floor (`demo_explore.coverage()`);
* no stretch >= 120 s parked (within 5 cm of one spot, the last 15 s left
  out: `evaluations/slam-343/parked.py`'s rule);
* 0 contacts (chassis within 1 cm of a wall or object, `footprint_sweep`).

Prints one JSON line; the numbers go in 3.70's records.
"""

import json
import math
import sys
import time

import httpx

from sim.grid_world import CELL_WALL
from sim.maps import build_world
from tests import demo_explore as dx
from tests.footprint_sweep import CELL_CM, chassis, gap, penetration, square

SURVEY_LIMIT_S = 1800.0
PARKED_M, PARKED_TAIL_S = 0.05, 15


def longest_parked(series):
    """parked.py's rule on (t, x_m, y_m) samples."""
    s = [r for r in series if r[0] <= series[-1][0] - PARKED_TAIL_S] if series else []
    best, i = 0.0, 0
    for j in range(len(s)):
        while math.hypot(s[j][1] - s[i][1], s[j][2] - s[i][2]) > PARKED_M:
            i += 1
        best = max(best, s[j][0] - s[i][0])
    return round(best, 1)


def run(robot, house):
    world = build_world(house)
    walls = {(x, y) for y, row in enumerate(world.layout) for x, c in enumerate(row)
             if c == CELL_WALL}
    brain = httpx.Client(base_url=f"http://127.0.0.1:{dx.BRAIN}", timeout=60)
    r = brain.post("/mission/start", json={"job": "survey", "policy": "explore",
                                           "mission_time_limit_s": SURVEY_LIMIT_S})
    if r.status_code != 200:
        return {"start": r.status_code, "detail": r.text[:300]}
    t0 = time.time()
    series, contacts, inside, min_gap, status = [], 0, 0, math.inf, {}
    while time.time() - t0 < SURVEY_LIMIT_S + 120:
        truth = robot.get("/world/truth").json()
        objs = {(o["x"], o["y"]) for o in robot.get("/sim/objects").json()["objects"]}
        x, y = truth["x_m"] / 0.3, truth["y_m"] / 0.3
        series.append((time.time() - t0, truth["x_m"], truth["y_m"]))
        a = chassis(x, y, math.radians(truth["heading_deg"]) - math.pi / 2)
        for c in walls | objs:
            if abs(c[0] + 0.5 - x) > 1.6 or abs(c[1] + 0.5 - y) > 1.6:
                continue
            g = gap(a, square(*c)) * CELL_CM
            min_gap = min(min_gap, g)
            contacts += g < 1.0
            inside += penetration(a, square(*c)) > 0
        status = brain.get("/mission/status").json()
        if not status.get("running") and status.get("outcome") not in (None, "idle", "running"):
            break
        time.sleep(0.5)
    else:
        status = brain.post("/mission/stop").json()["status"]
    survey = status.get("survey") or {}
    tag = f"survey-{house}-{int(t0)}"
    with open(f"{dx.RUN_DIR or dx.LOGDIR}/{tag}.json", "w") as f:
        json.dump({**status, "series": series,
                   "final_map": robot.get("/world/map").json()}, f)
    elapsed = (status.get("budget") or {}).get("elapsed_s")
    cov = dx.coverage(robot, house)
    parked = longest_parked(series)
    row = {
        "house": house, "outcome": status.get("outcome"), "end_reason": status.get("end_reason"),
        "elapsed_s": elapsed, "coverage_truth": cov, "longest_parked_s": parked,
        "contact_samples": contacts, "inside_samples": inside,
        "min_gap_cm": round(min_gap, 1), "samples": len(series),
        "goals": (status.get("explore") or {}).get("goals_sent"),
        "label_calls": survey.get("label_calls"), "rooms_named": survey.get("rooms_named"),
        "status_file": f"{tag}.json",
    }
    row["S1"] = {
        "exhausted": row["outcome"] == "exhausted",
        "within_30_min": elapsed is not None and elapsed <= SURVEY_LIMIT_S,
        "coverage_95": cov is not None and cov >= 0.95,
        "parked_under_120": parked < 120,
        "no_contacts": contacts == 0,
    }
    return row


def main():
    house = sys.argv[1] if len(sys.argv) > 1 else "home_first_floor"
    robot = dx.stack(house)          # takes the heavy-run lock; a fresh, empty map
    print(json.dumps(run(robot, house)), flush=True)


if __name__ == "__main__":
    main()
