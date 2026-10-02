"""
python -m tests.demo_slam_home [n] [--slam path/to/slam.yaml] [--drift L,R] [--limit S] [--tour 1]

PLAN-ros-alignment.md 3.34's instrument: does slam_toolbox keep the robot
where it is in the furnished home? Each run is a fresh stack (tests/
demo_explore.stack) and one `explore` mission for a target that is not in
the house -- the workload that broke SLAM in 3.31's batch: long, wandering,
past many chair and table legs. /world/error (SLAM's pose against the sim's
truth) is sampled at 1 Hz for the whole mission. One JSON line per run.

`--tour 1` replaces the mission with R6's nine-goal nav2 tour of every room
(tests/demo_nav_goals.py HOME_GOALS): a search that never leaves two rooms
exercises SLAM in two rooms.

A JUMP is the error growing by more than 0.5 m between two samples at most
2 s apart: odometry cannot do that, so it is the map frame moving under the
robot -- a scan matched, or a loop closed, to the wrong place.
"""

import json
import sys
import threading
import time

import httpx

from tests import demo_explore as dx

HOUSE = "home_first_floor"
JUMP_M = 0.5


def run(slam_yaml=None, drift="", limit_s=1200, max_steps=400, tour=False):
    robot = dx.stack(HOUSE, slam_yaml=slam_yaml, odom_drift=drift)
    brain = httpx.Client(base_url=f"http://127.0.0.1:{dx.BRAIN}", timeout=60)
    if not tour:
        r = brain.post("/mission/start", json={"target_object": "purple elephant",
                                               "policy": "explore", "max_steps": max_steps})
        if r.status_code != 200:
            return {"start": r.status_code, "detail": r.text[:300]}
    samples, stop = [], threading.Event()

    def sampler():
        c = httpx.Client(base_url=f"http://127.0.0.1:{dx.ROBOT}", timeout=5)
        while not stop.is_set():
            try:
                e = c.get("/world/error").json()
                if e.get("usable"):
                    t = e.get("truth") or {}
                    samples.append((time.time(), e["position_error_m"], e["heading_error_deg"],
                                    e["odom_position_error_m"], t.get("x_m"), t.get("y_m")))
            except Exception:  # noqa: BLE001 -- a missed sample is a gap, not an end
                pass
            stop.wait(1.0)
    th = threading.Thread(target=sampler, daemon=True)
    th.start()
    t0, status, full_log = time.time(), {}, []
    if tour:
        import os
        os.environ.update(SIM_MAP=HOUSE, PICAR_ROBOT_URL=f"http://127.0.0.1:{dx.ROBOT}")
        from tests import demo_nav_goals as ng
        goals = [ng.run_goal(robot, x, y)["state"] for _name, x, y in ng.HOME_GOALS]
        status = {"outcome": "tour", "step": goals}
    while not tour and time.time() - t0 < limit_s:
        status = brain.get("/mission/status").json()
        tail = status.get("log_tail") or []
        k = len(tail)
        while k and tail[:k] != full_log[-k:]:
            k -= 1
        full_log.extend(tail[k:])
        if not status.get("running") and status.get("outcome") not in (None, "idle", "running"):
            break
        time.sleep(2)
    if not tour and status.get("running"):
        brain.post("/mission/stop")
    time.sleep(3)                      # at rest: the error now is not latency
    stop.set()
    th.join()
    final = robot.get("/world/error").json()
    jumps = [(round(b[0] - t0), round(b[1] - a[1], 2), round(b[2] - a[2], 1))
             for a, b in zip(samples, samples[1:])
             if b[0] - a[0] <= 2.0 and b[1] - a[1] > JUMP_M]
    out = {"slam": slam_yaml or "image", "drift": drift, "outcome": status.get("outcome"),
           "seconds": round(time.time() - t0), "steps": status.get("step"),
           "coverage": dx.coverage(robot, HOUSE), "samples": len(samples),
           "max_error_m": round(max((s[1] for s in samples), default=0), 3),
           "max_heading_error_deg": round(max((abs(s[2]) for s in samples), default=0), 1),
           "max_odom_error_m": round(max((s[3] for s in samples), default=0), 3),
           "final_error_m": final.get("position_error_m"),
           "final_heading_error_deg": final.get("heading_error_deg"),
           "jumps": jumps}
    with open(f"{dx.LOGDIR}/slam_home_{int(t0)}.json", "w") as f:
        json.dump({**out, "series": samples, "full_log": full_log}, f)
    return out


def main():
    args = sys.argv[1:]
    n = int(args.pop(0)) if args and args[0].isdigit() else 1
    opt = dict(zip(args[::2], args[1::2]))
    for _ in range(n):
        print(json.dumps(run(opt.get("--slam"), opt.get("--drift", ""),
                             int(opt.get("--limit", 1200)), tour=opt.get("--tour") == "1")),
              flush=True)
    dx._kill_ports()


if __name__ == "__main__":
    main()
