"""
python -m tests.demo_explore <rooms|absent|door|rule|one> [house] [room]

PLAN-ros-alignment.md 3.31's live instrument: missions through the brain's
HTTP API on the full ROS stack (nav2 + slam_toolbox), judged on the sim's
ground truth. Each run gets a FRESH stack -- robot server, brain and ROS
container -- so every mission starts on an empty map.

    rooms   criterion 1: the backpack moved to each room in turn
    absent  criterion 2: a target that is not in the house
    door    criterion 3: someone in the kitchen door for 60 s (scaled house)
    rule    criterion 4: the rule-based policy, someone crossing the hallway
    one     criterion 6: one explore mission, as the house ships

The stack runs beside any other on spare ports (robot 8100, brain 8101,
bridge 8190) and its own ROS domain (73), so it never touches a stack
another session has up on the defaults. One JSON line per mission.
"""

import json
import math
import os
import subprocess
import sys
import time
from collections import deque

import httpx

from sim.grid_world import CELL_WALL
from sim.maps import build_world
from tests.footprint_sweep import CELL_CM, chassis, gap, penetration, square

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PY = sys.executable
ROBOT, BRAIN, BRIDGE = 8100, 8101, 8190
CONTAINER = "picar-ros-explore"
TARGET = "red backpack"
MISSION_LIMIT_S = 1800
LOGDIR = os.environ.get("EXPLORE_LOGDIR", "/tmp")


def _kill_ports():
    # Keep nav2's own account of a run before the container goes.
    if subprocess.run(["docker", "inspect", CONTAINER], capture_output=True).returncode == 0:
        with open(f"{LOGDIR}/explore_ros_{int(time.time())}.log", "w") as f:
            subprocess.run(["docker", "logs", CONTAINER], stdout=f, stderr=subprocess.STDOUT)
    # A server that outlives its run keeps answering on the port, and the
    # next run's own server then fails to bind: the 2026-10-02 breakfast run
    # chased the den run's backpack. Kill, escalate, and wait until free.
    for sig in ("-TERM", "-KILL"):
        for port in (ROBOT, BRAIN):
            for pid in _listeners(port):
                subprocess.run(["kill", sig, pid])
        deadline = time.time() + 10
        while time.time() < deadline and any(_listeners(p) for p in (ROBOT, BRAIN)):
            time.sleep(0.5)
    if any(_listeners(p) for p in (ROBOT, BRAIN)):
        raise RuntimeError("ports still held after SIGKILL")
    subprocess.run(["docker", "rm", "-f", CONTAINER], capture_output=True)
    time.sleep(2)


def _listeners(port):
    return subprocess.run(["lsof", "-nP", f"-tiTCP:{port}", "-sTCP:LISTEN"],
                          capture_output=True, text=True).stdout.split()


def stack(house, movers="", slam_yaml=None, odom_drift=""):
    """A fresh robot server, brain and ROS container. `slam_yaml` mounts
    another slam_toolbox config over the image's (it is symlink-installed);
    `odom_drift` is SIM_ODOM_DRIFT ("left,right")."""
    _kill_ports()
    env = {k: v for k, v in os.environ.items()
           if k not in ("APP_SHARED_SECRET", "LOCAL_SECRET")}
    env.update(SIM_MAP=house, SIM_MOVERS=movers, ROBOT_DRIVE="ros", WORLD_MODE="ros",
               SIM_ODOM_DRIFT=odom_drift,
               ROS_BRIDGE_URL=f"http://127.0.0.1:{BRIDGE}")
    procs = [subprocess.Popen([PY, "-m", "uvicorn", "robot.server:app", "--port", str(ROBOT),
                      "--host", "127.0.0.1", "--log-level", "warning"], cwd=ROOT, env=env,
                     stdout=open(f"{LOGDIR}/explore_robot.log", "w"), stderr=subprocess.STDOUT)]
    benv = dict(env, ROBOT_URL=f"http://127.0.0.1:{ROBOT}")
    procs.append(subprocess.Popen([PY, "-m", "uvicorn", "control.brain_server:app", "--port", str(BRAIN),
                      "--host", "127.0.0.1", "--log-level", "warning"], cwd=ROOT, env=benv,
                     stdout=open(f"{LOGDIR}/explore_brain.log", "w"), stderr=subprocess.STDOUT))
    time.sleep(4)
    for proc, port in zip(procs, (ROBOT, BRAIN)):
        if proc.poll() is not None or str(proc.pid) not in _listeners(port):
            raise RuntimeError(f"port {port} is not served by this run's own server")
    mount = (["-v", f"{os.path.abspath(slam_yaml)}:/ws/src/picar_bringup/config/slam.yaml:ro"]
             if slam_yaml else [])
    subprocess.run(["docker", "run", "-d", "--name", CONTAINER, *mount,
                    "-p", f"127.0.0.1:{BRIDGE}:8090", "-e", "ROS_DOMAIN_ID=73",
                    "-e", f"ROBOT_URL=http://host.docker.internal:{ROBOT}",
                    "-e", f"BRAIN_URL=http://host.docker.internal:{BRAIN}",
                    "vision-picar-ros:latest", "ros2", "launch", "picar_bringup",
                    "picar.launch.py"], capture_output=True, check=True)
    robot = httpx.Client(base_url=f"http://127.0.0.1:{ROBOT}", timeout=30)
    deadline = time.time() + 120
    while time.time() < deadline:
        try:
            if robot.get("/health").json()["drive"]["ros_up"] and \
                    robot.get("/world/map").json().get("usable"):
                return robot
        except Exception:  # noqa: BLE001 -- still coming up
            pass
        time.sleep(2)
    raise RuntimeError("stack did not come up")


def free_cells(world):
    return {(x, y) for y, row in enumerate(world.layout) for x, c in enumerate(row)
            if c != CELL_WALL and (x, y) not in world.objects}


def reachable_cells(world):
    """Truth floor cells the CHASSIS can get to from the start: any heading,
    connected in configuration space (3.21's instrument, tests/chassis_fit.py).
    A cell counts if the robot's centre can stand anywhere in it."""
    import math as _m
    from robot.safety import FOOTPRINT_LENGTH_M, FOOTPRINT_WIDTH_M
    from tests import chassis_fit as cf
    occ = cf.occupancy(world)
    free = cf.free_space(occ, FOOTPRINT_LENGTH_M, FOOTPRINT_WIDTH_M)
    start_px = (round((world.y + 1) * cf.PX), round((world.x + 1) * cf.PX))
    start_k = round(world.theta / (2 * _m.pi / cf.HEADINGS)) % cf.HEADINGS
    where = cf.reachable(free, start_px, start_k).any(axis=0)
    out = set()
    for (cx, cy) in free_cells(world):
        block = where[(cy + 1) * cf.PX:(cy + 2) * cf.PX, (cx + 1) * cf.PX:(cx + 2) * cf.PX]
        if block.any():
            out.add((cx, cy))
    return out


def room_spot(world, room, reach):
    """A reachable cell near the middle of `room`, away from the start."""
    # Floor nav2 can plan to: reachable, and no furniture in the eight cells
    # around it. A room's middle is often under its table (the breakfast
    # room's is, between the legs) -- a test of furniture, not of search.
    cells = [c for c in world.rooms[room] if c in reach
             and math.hypot(c[0] - world.robot_x, c[1] - world.robot_y) > 4
             and not any((c[0] + dx, c[1] + dy) in world.objects
                         for dx in (-1, 0, 1) for dy in (-1, 0, 1))]
    if not cells:
        return None
    mx = sum(c[0] for c in cells) / len(cells)
    my = sum(c[1] for c in cells) / len(cells)
    return min(cells, key=lambda c: math.hypot(c[0] - mx, c[1] - my))


def run_mission(robot, house, policy="explore", target=TARGET, max_steps=200):
    world = build_world(house)
    walls = {(x, y) for y, row in enumerate(world.layout) for x, c in enumerate(row)
             if c == CELL_WALL}
    brain = httpx.Client(base_url=f"http://127.0.0.1:{BRAIN}", timeout=60)
    r = brain.post("/mission/start", json={"target_object": target, "policy": policy,
                                           "max_steps": max_steps})
    if r.status_code != 200:
        return {"start": r.status_code, "detail": r.text[:300]}
    target_at = [(o["x"], o["y"]) for o in robot.get("/sim/objects").json()["objects"]
                 if o["name"] == target]
    t0, last, travel = time.time(), None, 0.0
    min_gap, contacts, inside, samples = math.inf, 0, 0, 0
    status, full_log = {}, []
    while time.time() - t0 < MISSION_LIMIT_S:
        truth = robot.get("/world/truth").json()
        objs = {(o["x"], o["y"]) for o in robot.get("/sim/objects").json()["objects"]}
        x, y = truth["x_m"] / 0.3, truth["y_m"] / 0.3
        th = math.radians(truth["heading_deg"]) - math.pi / 2
        if last:
            travel += math.hypot(x - last[0], y - last[1]) * 0.3
        last = (x, y)
        A = chassis(x, y, th)
        for c in walls | objs:
            if abs(c[0] + 0.5 - x) > 1.6 or abs(c[1] + 0.5 - y) > 1.6:
                continue
            g = gap(A, square(*c)) * CELL_CM
            min_gap = min(min_gap, g)
            if g < 1.0:
                contacts += 1
            if penetration(A, square(*c)) > 0:
                inside += 1
        samples += 1
        status = brain.get("/mission/status").json()
        tail = status.get("log_tail") or []
        # The status keeps 20 lines; keep every one seen, in order.
        k = len(tail)
        while k and tail[:k] != full_log[-k:]:
            k -= 1
        full_log.extend(tail[k:])
        if not status.get("running") and status.get("outcome") not in (None, "idle", "running"):
            break
        time.sleep(0.5)
    else:
        brain.post("/mission/stop")
    tag = f"{policy}-{target}-{int(t0)}".replace(" ", "_")
    with open(f"{LOGDIR}/explore_status_{tag}.json", "w") as f:
        json.dump({**status, "full_log": full_log,
                   "final_map": robot.get("/world/map").json(),
                   "final_pose": robot.get("/world/pose").json()}, f)
    return {"outcome": status.get("outcome"), "seconds": round(time.time() - t0),
            "target_at": target_at,
            "status_file": f"explore_status_{tag}.json",
            "steps": status.get("step"), "explore": status.get("explore"),
            "stuck_episodes": status.get("stuck_episodes"), "travel_m": round(travel, 1),
            "min_gap_cm": round(min_gap, 1), "contact_samples": contacts,
            "inside_samples": inside, "samples": samples,
            "last_log": (status.get("log_tail") or [])[-3:]}


def coverage(robot, house):
    """Share of truth-reachable floor whose centre is no longer unknown on the
    robot's map."""
    world = build_world(house)
    reach = reachable_cells(world)
    m = robot.get("/world/map").json()
    if not m.get("usable"):
        return None
    res, w, h = m["resolution_m"], m["width"], m["height"]
    known = 0
    for cx, cy in reach:
        i = int(((cx + 0.5) * 0.3 - m["origin_x_m"]) / res)
        j = int(((cy + 0.5) * 0.3 - m["origin_y_m"]) / res)
        if 0 <= i < w and 0 <= j < h and m["cells"][j * w + i] != -1:
            known += 1
    return round(known / len(reach), 4)


def main():
    mode = sys.argv[1]
    house = sys.argv[2] if len(sys.argv) > 2 else "home_first_floor"
    if mode == "rooms":
        world = build_world(house)
        reach = reachable_cells(world)
        rooms = sys.argv[3:] or sorted(world.rooms)
        for room in rooms:
            spot = room_spot(world, room, reach)
            if spot is None:
                print(json.dumps({"room": room, "skipped": "no reachable spot"}), flush=True)
                continue
            robot = stack(house)
            src = [(o["x"], o["y"]) for o in robot.get("/sim/objects").json()["objects"]
                   if o["name"] == TARGET][0]
            moved = robot.post("/sim/objects/move", json={"src": list(src), "dst": list(spot)})
            res = run_mission(robot, house)
            print(json.dumps({"room": room, "spot": spot, "moved": moved.status_code, **res}),
                  flush=True)
    elif mode == "absent":
        robot = stack(house)
        res = run_mission(robot, house, target="purple elephant", max_steps=400)
        print(json.dumps({"mode": "absent", "coverage": coverage(robot, house), **res}), flush=True)
    elif mode == "door":
        robot = stack("scaled_house", "kitchen_door_sitter")
        print(json.dumps({"mode": "door", **run_mission(robot, "scaled_house")}), flush=True)
    elif mode == "rule":
        robot = stack("scaled_house", "hallway_crossing")
        print(json.dumps({"mode": "rule", **run_mission(robot, "scaled_house", policy="frontier",
                                                        max_steps=150)}), flush=True)
    elif mode == "one":
        robot = stack(house)
        print(json.dumps({"mode": "one", **run_mission(robot, house)}), flush=True)
    _kill_ports()


if __name__ == "__main__":
    main()
