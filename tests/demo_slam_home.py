"""
python -m tests.demo_slam_home [n] [--slam path/to/slam.yaml] [--drift L,R] [--limit S] [--tour 1]
    [--explore-first S]   # 3.39: explore (an absent target) for up to S s, then tour
    [--nav path/to/nav2.yaml]   # 3.39: another nav2 config
    [--tour-speed LIN,ANG]      # 3.39: RPP's desired_linear_vel / rotate_to_heading_angular_vel,
                                #   set live AFTER exploring, so only the tour is slowed
    [--goal-timeout S]          # per tour goal (default 120)

PLAN-ros-alignment.md 3.38's instrument: does slam_toolbox keep the robot
where it is in the furnished home? Each run is a fresh stack (tests/
demo_explore.stack) and one `explore` mission for a target that is not in
the house -- the workload that broke SLAM in 3.31's batch: long, wandering,
past many chair and table legs. /world/error (SLAM's pose against the sim's
truth) is sampled at 1 Hz for the whole mission. One JSON line per run.

`--tour 1` replaces the mission with R6's nine-goal nav2 tour of every room
(tests/demo_nav_goals.py HOME_GOALS): a search that never leaves two rooms
exercises SLAM in two rooms.

A JUMP is the error growing by more than 0.3 m (0.5 until 3.40) between two samples at most
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
JUMP_M = 0.3                 # 3.40: was 0.5; a 0.43 m closure step passed it (2026-10-05)
CELL = 0.3


def score_map(robot, house=HOUSE):
    """SLAM's map (house frame, through the start anchor) against the true
    house: the share of OCCUPIED cells within 10 cm of a true wall or solid
    object, overall and per room. 3.39: a pose error alone cannot tell a
    robot misplaced on a good map from a map that bent -- this can. A good
    map with a bad pose points at localisation; a bent one at the map."""
    from sim.grid_world import CELL_WALL
    from sim.maps import build_world
    g = build_world(house)
    solid = {(x, y) for y, row in enumerate(g.layout) for x, c in enumerate(row)
             if c == CELL_WALL} | set(g.solid_cells)
    room_of = {c: name for name, cells in g.rooms.items() for c in cells}
    m = robot.get("/world/map").json()
    if not m.get("usable"):
        return None

    def near_solid(px, py):
        cx, cy = int(px // CELL), int(py // CELL)
        for x in range(cx - 1, cx + 2):
            for y in range(cy - 1, cy + 2):
                if (x, y) in solid:
                    dx = max(x * CELL - px, 0, px - (x + 1) * CELL)
                    dy = max(y * CELL - py, 0, py - (y + 1) * CELL)
                    if dx * dx + dy * dy <= 0.01:
                        return True
        return False
    res, w = m["resolution_m"], m["width"]
    tot, ok, rooms = 0, 0, {}
    for k, v in enumerate(m["cells"]):
        if v != 1:
            continue
        px = m["origin_x_m"] + (k % w + 0.5) * res
        py = m["origin_y_m"] + (k // w + 0.5) * res
        good = near_solid(px, py)
        tot += 1
        ok += good
        r = rooms.setdefault(room_of.get((int(px // CELL), int(py // CELL)), "?"), [0, 0])
        r[0] += 1
        r[1] += good
    return {"occupied": tot, "precision": round(ok / max(tot, 1), 3),
            "rooms": {n: (c, round(g_ / max(c, 1), 3)) for n, (c, g_) in sorted(rooms.items())}}


def ros_param(name, value=None):
    """Set (or get) one controller_server parameter inside the container."""
    import subprocess
    verb = f"set /controller_server {name} {value}" if value is not None else \
        f"get /controller_server {name}"
    r = subprocess.run(["docker", "exec", dx.CONTAINER, "/entrypoint.sh", "bash", "-c",
                        f"ros2 param {verb}"], capture_output=True, text=True, timeout=60)
    return (r.stdout + r.stderr).strip()


def run(slam_yaml=None, drift="", limit_s=1200, max_steps=400, tour=False, explore_first_s=0,
        nav_yaml=None, tour_speed=None, goal_timeout_s=120):
    robot = dx.stack(HOUSE, slam_yaml=slam_yaml, odom_drift=drift, nav_yaml=nav_yaml)
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

    def keep_log(st):
        # The status keeps the last 20 lines; keep every one seen, in order.
        tail = st.get("log_tail") or []
        k = len(tail)
        while k and tail[:k] != full_log[-k:]:
            k -= 1
        full_log.extend(tail[k:])
    if tour:
        import os
        os.environ.update(SIM_MAP=HOUSE, PICAR_ROBOT_URL=f"http://127.0.0.1:{dx.ROBOT}")
        from tests import demo_nav_goals as ng
        # 3.39: map before navigating. At the start nav2's costmap holds only
        # what SLAM has seen from the start pose, and the east of the house is
        # not on it -- a goal there cannot be planned to. Explore first (an
        # absent target, so the mission ends "searched" or at the cap), then
        # tour the mapped house.
        explore = None
        if explore_first_s:
            r = brain.post("/mission/start", json={"target_object": "purple elephant",
                                                   "policy": "explore", "max_steps": max_steps})
            t_ex = time.time()
            while time.time() - t_ex < explore_first_s:
                st = brain.get("/mission/status").json()
                keep_log(st)
                if not st.get("running") and st.get("outcome") not in (None, "idle", "running"):
                    break
                time.sleep(2)
            st = brain.get("/mission/status").json()
            keep_log(st)
            if st.get("running"):
                brain.post("/mission/stop")
            time.sleep(2)
            explore = {"outcome": st.get("outcome"), "seconds": round(time.time() - t_ex),
                       "started_at": t_ex, "ended_at": time.time(),
                       "counts": st.get("explore"),
                       "coverage": dx.coverage(robot, HOUSE), "map": score_map(robot),
                       "error": robot.get("/world/error").json().get("position_error_m")}
        speed = None
        if tour_speed:
            lin, ang = tour_speed.split(",")
            ros_param("FollowPath.desired_linear_vel", lin)
            ros_param("FollowPath.rotate_to_heading_angular_vel", ang)
            speed = [ros_param("FollowPath.desired_linear_vel"),
                     ros_param("FollowPath.rotate_to_heading_angular_vel")]
        t_tour = time.time()
        goals = []
        for name, x, y in ng.HOME_GOALS:
            # 3.45: when each goal ran, so a stall can be put in its phase and goal.
            t_g = time.time()
            goals.append({"name": name, "started_at": t_g,
                          **ng.run_goal(robot, x, y, timeout_s=goal_timeout_s),
                          "ended_at": time.time()})
        status = {"outcome": "tour", "explore": explore, "tour_speed": speed,
                  "tour_started_s": round(t_tour - t0),
                  "step": [g["state"] for g in goals],
                  "end_error_m": [g.get("end_error_m") for g in goals],
                  "goals": goals, "tour_started_at": t_tour}
    while not tour and time.time() - t0 < limit_s:
        status = brain.get("/mission/status").json()
        keep_log(status)
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
    out = {"t0": t0, "run_dir": dx.RUN_DIR, "tour_started_at": status.get("tour_started_at"),
           "goals": status.get("goals"), "slam": slam_yaml or "image", "nav": nav_yaml or "image", "drift": drift, "outcome": status.get("outcome"),
           "seconds": round(time.time() - t0), "steps": status.get("step"),
           "explore": status.get("explore"), "tour_speed": status.get("tour_speed"),
           "tour_started_s": status.get("tour_started_s"), "goal_states": status.get("step"),
           "goal_errors_m": status.get("end_error_m"),
           "coverage": dx.coverage(robot, HOUSE), "samples": len(samples),
           "max_error_m": round(max((s[1] for s in samples), default=0), 3),
           "max_heading_error_deg": round(max((abs(s[2]) for s in samples), default=0), 1),
           "max_odom_error_m": round(max((s[3] for s in samples), default=0), 3),
           "final_error_m": final.get("position_error_m"),
           "final_heading_error_deg": final.get("heading_error_deg"),
           "jumps": jumps, "map": score_map(robot)}
    with open(f"{dx.RUN_DIR or dx.LOGDIR}/slam_home_{int(t0)}.json", "w") as f:
        json.dump({**out, "series": samples, "full_log": full_log}, f)
    return out


def main():
    args = sys.argv[1:]
    n = int(args.pop(0)) if args and args[0].isdigit() else 1
    opt = dict(zip(args[::2], args[1::2]))
    for _ in range(n):
        print(json.dumps(run(opt.get("--slam"), opt.get("--drift", ""),
                             int(opt.get("--limit", 1200)), tour=opt.get("--tour") == "1",
                             explore_first_s=int(opt.get("--explore-first", 0)),
                             nav_yaml=opt.get("--nav"), tour_speed=opt.get("--tour-speed"),
                             goal_timeout_s=float(opt.get("--goal-timeout", 120)))),
              flush=True)
    dx._kill_ports()


if __name__ == "__main__":
    main()
