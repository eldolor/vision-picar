"""
python -m tests.demo_mission_ends [house] [time_limit_s]

PLAN-ros-alignment.md 3.70 E4/E5, live: two missions through the brain's HTTP
API on the ROS stack (`tests/demo_explore.stack()`: robot server, brain and
the ROS container, sim only, no paid calls).

* **E4** -- an `explore` mission for an absent target with the time limit
  set low. It must end `out_of_budget` within the limit plus one tick, the
  wheels at zero within 0.5 s, and report the rooms the robot's centre
  entered (equal to truth), coverage within 2 points of
  `demo_explore.coverage()`, the goals reached and the map.
* **E5** -- the same mission, cancelled by `POST /mission/stop` part way:
  `cancelled`, `cancelled_by: stop`, the wheels at zero within 0.5 s of the
  Stop's answer, and the same report.

Prints one JSON line per mission; the numbers go in 3.70's records.
"""

import json
import sys
import time

import httpx

from sim.maps import build_world
from tests import demo_explore as dx

POLL_S = 0.1


def _truth_room(world, truth):
    return world.room_at(int(truth["x_m"] / 0.3), int(truth["y_m"] / 0.3))


def _wheels_zero(robot):
    w = robot.get("/wheels").json()
    if not w.get("usable"):
        return None
    return w["left"]["velocity_rad_s"] == 0 and w["right"]["velocity_rad_s"] == 0


def _wait_wheels_zero(robot, since, limit_s=3.0):
    while time.monotonic() - since < limit_s:
        if _wheels_zero(robot):
            return round(time.monotonic() - since, 3)
        time.sleep(0.02)
    return None


def run(robot, house, limit_s, stop_after_s=None):
    world = build_world(house)
    brain = httpx.Client(base_url=f"http://127.0.0.1:{dx.BRAIN}", timeout=60)
    r = brain.post("/mission/start", json={"target_object": "purple elephant",
                                           "policy": "explore",
                                           "mission_time_limit_s": limit_s})
    r.raise_for_status()
    t0 = time.monotonic()
    truth_rooms, status, last_tick = set(), {}, None
    stop_answer_at, wheels_after = None, None
    while time.monotonic() - t0 < limit_s + 120:
        truth = robot.get("/world/truth").json()
        room = _truth_room(world, truth)
        if room != "unknown":
            truth_rooms.add(room)
        if stop_after_s is not None and stop_answer_at is None \
                and time.monotonic() - t0 >= stop_after_s:
            resp = brain.post("/mission/stop")
            stop_answer_at = time.monotonic()
            status = resp.json()["status"]
            wheels_after = _wait_wheels_zero(robot, stop_answer_at)
            break
        status = brain.get("/mission/status").json()
        if status.get("running"):
            last_tick = status.get("seconds_since_last_tick")
        if not status.get("running") and status.get("outcome") not in ("idle", "running"):
            seen_end = time.monotonic()
            wheels_after = _wait_wheels_zero(robot, seen_end)
            break
        time.sleep(POLL_S)
    # The report lands just after the stop; read it once it has.
    for _ in range(50):
        if status.get("report") is not None:
            break
        time.sleep(0.1)
        status = brain.get("/mission/status").json()
    report = status.get("report") or {}
    cov_truth = dx.coverage(robot, house)
    return {
        "house": house, "limit_s": limit_s, "stop_after_s": stop_after_s,
        "outcome": status.get("outcome"), "end_state": status.get("end_state"),
        "end_reason": status.get("end_reason"), "cancelled_by": status.get("cancelled_by"),
        "elapsed_s": (status.get("budget") or {}).get("elapsed_s"),
        "over_limit_s": (round((status.get("budget") or {}).get("elapsed_s", 0) - limit_s, 2)
                         if stop_after_s is None else None),
        "last_tick_age_s": last_tick,
        "wheels_zero_after_s": wheels_after,
        "rooms_reported": report.get("rooms_entered"),
        "rooms_truth": sorted(truth_rooms),
        "rooms_equal": sorted(truth_rooms) == (report.get("rooms_entered") or []),
        "coverage_reported": report.get("coverage"), "coverage_truth": cov_truth,
        "coverage_diff_pts": (round((report["coverage"] - cov_truth) * 100, 1)
                              if report.get("coverage") is not None and cov_truth is not None
                              else None),
        "goals_reached": len(report.get("goals_reached") or []),
        "map_id": report.get("map_id"), "map_version": report.get("map_version"),
        "steps": status.get("step"), "max_steps": status.get("max_steps"),
        "log_tail": (status.get("log_tail") or [])[-3:],
    }


def main():
    house = sys.argv[1] if len(sys.argv) > 1 else "home_first_floor"
    limit_s = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
    try:
        robot = dx.stack(house)          # a fresh stack per mission: an empty map
        print(json.dumps({"E4": run(robot, house, limit_s)}), flush=True)
        robot = dx.stack(house)
        print(json.dumps({"E5": run(robot, house, 600.0, stop_after_s=limit_s / 2)}),
              flush=True)
    finally:
        dx._kill_ports()


if __name__ == "__main__":
    main()
