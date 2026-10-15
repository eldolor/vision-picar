"""
python evaluations/person-stop-384/live_check.py [--code DIR] [--out FILE]

`PLAN-ros-alignment.md` 3.84 criterion 7: a person's Stop during an explore
mission, through the real stack -- the robot server (`ROBOT_DRIVE=ros`,
`WORLD_MODE=ros`, `SIM_MAP=scaled_house`), the brain (`control/brain_server`),
the ROS container (`vision-picar-ros:latest`: slam_toolbox, nav2, the bridge)
-- judged on the simulator's ground truth (`GET /world/truth`).

  (a) An explore mission started through the brain's HTTP API; once its goal
      is active and the robot moving (true speed > 5 cm/s), the D-pad's Stop
      (`POST /stop`, `x-driver: twin-dpad`). The robot's TRUE displacement
      from its pose at the Stop's reply over the next 10 s (bar <= 5 cm).
  (b) The mission's end state within those 10 s (bar: `cancelled`,
      `cancelled_by: "stop"`).
  (c) A D-pad FORWARD right after: executes, true displacement > 0.
  (d) A new explore mission: the robot moves >= 20 cm within 60 s.

`--code DIR` runs the robot server and brain from another checkout (the `dev`
baseline). Ports, ROS domain and container come from
`python tools/plan_section.py env`. One JSON line per run.
"""

import argparse
import json
import math
import os
import subprocess
import sys
import time

import httpx

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from tests.demo_explore import _listeners, heavy_lock  # noqa: E402

PY = sys.executable
ROBOT = int(os.environ["PICAR_ROBOT_PORT"])
BRAIN = int(os.environ["PICAR_BRAIN_PORT"])
BRIDGE = int(os.environ["PICAR_BRIDGE_PORT"])
CONTAINER = os.environ["PICAR_ROS_CONTAINER"]
DOMAIN = os.environ["ROS_DOMAIN_ID"]
IMAGE = "vision-picar-ros:latest"
LOGDIR = os.environ.get("EXPLORE_LOGDIR", "/tmp/vision-picar-3.84")
TARGET = "red backpack"
WATCH_S = 10.0


def stop_all(procs):
    for p in procs:
        p.terminate()
    for p in procs:
        try:
            p.wait(10)
        except subprocess.TimeoutExpired:
            p.kill()
    if subprocess.run(["docker", "inspect", CONTAINER], capture_output=True).returncode == 0:
        with open(f"{LOGDIR}/ros_{int(time.time())}.log", "w") as f:
            subprocess.run(["docker", "logs", CONTAINER], stdout=f, stderr=subprocess.STDOUT)
    subprocess.run(["docker", "rm", "-f", CONTAINER], capture_output=True)


def start(code_dir, procs):
    os.makedirs(LOGDIR, exist_ok=True)
    for port in (ROBOT, BRAIN, BRIDGE):
        if _listeners(port):
            raise RuntimeError(f"port {port} is already served")
    subprocess.run(["docker", "rm", "-f", CONTAINER], capture_output=True)
    env = {k: v for k, v in os.environ.items() if k not in ("APP_SHARED_SECRET", "LOCAL_SECRET")}
    env.update(SIM_MAP="scaled_house", SIM_MOVERS="", ROBOT_DRIVE="ros", WORLD_MODE="ros",
               ROS_BRIDGE_URL=f"http://127.0.0.1:{BRIDGE}", PICAR_LOG_LEVEL="WARNING")
    tag = f"{os.path.basename(code_dir.rstrip('/'))}_{int(time.time())}"
    procs.append(subprocess.Popen(
        [PY, "-m", "uvicorn", "robot.server:app", "--port", str(ROBOT), "--host", "127.0.0.1",
         "--log-level", "warning"], cwd=code_dir, env=env,
        stdout=open(f"{LOGDIR}/robot_{tag}.log", "w"), stderr=subprocess.STDOUT))
    benv = dict(env, ROBOT_URL=f"http://127.0.0.1:{ROBOT}", PICAR_LOG_LEVEL="INFO")
    procs.append(subprocess.Popen(
        [PY, "-m", "uvicorn", "control.brain_server:app", "--port", str(BRAIN), "--host",
         "127.0.0.1", "--log-level", "warning"], cwd=code_dir, env=benv,
        stdout=open(f"{LOGDIR}/brain_{tag}.log", "w"), stderr=subprocess.STDOUT))
    time.sleep(4)
    for proc, port in zip(procs, (ROBOT, BRAIN)):
        if proc.poll() is not None or str(proc.pid) not in _listeners(port):
            raise RuntimeError(f"port {port} is not served by this run's own server")
    subprocess.run(["docker", "run", "-d", "--name", CONTAINER,
                    "-p", f"127.0.0.1:{BRIDGE}:8090", "-e", f"ROS_DOMAIN_ID={DOMAIN}",
                    "-e", f"ROBOT_URL=http://host.docker.internal:{ROBOT}",
                    "-e", f"BRAIN_URL=http://host.docker.internal:{BRAIN}",
                    IMAGE, "ros2", "launch", "picar_bringup", "picar.launch.py"],
                   capture_output=True, check=True)
    robot = httpx.Client(base_url=f"http://127.0.0.1:{ROBOT}", timeout=30)
    brain = httpx.Client(base_url=f"http://127.0.0.1:{BRAIN}", timeout=60)
    end = time.time() + 150
    while time.time() < end:
        try:
            if (robot.get("/health").json()["drive"]["ros_up"]
                    and robot.get("/world/map").json().get("usable")
                    and robot.get("/world/goal").status_code == 200):
                return robot, brain
        except Exception:  # noqa: BLE001 -- still coming up
            pass
        time.sleep(2)
    raise RuntimeError("stack did not come up")


def truth(robot):
    t = robot.get("/world/truth").json()
    return t["x_m"], t["y_m"]


def goal_state(robot):
    return ((robot.get("/world/goal").json() or {}).get("goal") or {}).get("state")


def start_explore(brain):
    r = brain.post("/mission/start", json={"target_object": TARGET, "policy": "explore",
                                           "mission_time_limit_s": 600})
    return r.status_code, r.text[:300]


def wait_driving(robot, timeout=120.0):
    """Until a goal is active and the true speed over 0.5 s is > 5 cm/s."""
    end = time.time() + timeout
    while time.time() < end:
        p = truth(robot)
        time.sleep(0.5)
        q = truth(robot)
        if goal_state(robot) == "active" and math.dist(p, q) / 0.5 > 0.05:
            return round(math.dist(p, q) / 0.5, 3)
        time.sleep(0.2)
    return None


def run(robot, brain):
    row = {}
    row["start"] = start_explore(brain)
    speed = wait_driving(robot)
    if speed is None:
        row["error"] = "the mission never drove"
        row["status"] = brain.get("/mission/status").json()
        return row
    row["speed_before_stop_m_s"] = speed
    stop = robot.post("/stop", headers={"x-driver": "twin-dpad"}).json()
    t0, p0 = time.time(), truth(robot)
    worst, path, ended_at, status = 0.0, [], None, None
    while time.time() - t0 < WATCH_S:
        p = truth(robot)
        worst = max(worst, math.dist(p, p0))
        path.append([round(time.time() - t0, 2), round(p[0], 3), round(p[1], 3)])
        if ended_at is None:
            status = brain.get("/mission/status").json()
            if not status.get("running"):
                ended_at = round(time.time() - t0, 2)
        time.sleep(0.1)
    status = brain.get("/mission/status").json()
    row.update(stop=stop.get("executed"), max_true_displacement_m=round(worst, 4),
               final_displacement_m=round(math.dist(truth(robot), p0), 4),
               goal_state_after=goal_state(robot), running_after=status.get("running"),
               outcome=status.get("outcome"), cancelled_by=status.get("cancelled_by"),
               end_reason=status.get("end_reason"), ended_after_s=ended_at,
               path=path[::10])
    row["pass_a"] = worst <= 0.05
    row["pass_b"] = (status.get("outcome") == "cancelled"
                     and status.get("cancelled_by") == "stop" and ended_at is not None)
    if status.get("running"):
        brain.post("/mission/stop")                 # the baseline: end it before (c)
        time.sleep(2.0)
    # (c) the person's own next command.
    p = truth(robot)
    fwd = robot.post("/action", json={"action": "FORWARD"},
                     headers={"x-driver": "twin-dpad"}).json()
    time.sleep(0.5)
    row["forward"] = {"executed": fwd.get("executed"), "reason": fwd.get("reason"),
                      "moved_m": round(math.dist(truth(robot), p), 4)}
    row["pass_c"] = bool(fwd.get("executed")) and row["forward"]["moved_m"] > 0.0
    # (d) a new mission.
    time.sleep(2.0)                                 # the person's claim lapses
    p = truth(robot)
    row["restart"] = start_explore(brain)
    end, moved = time.time() + 60.0, 0.0
    while time.time() < end and moved < 0.20:
        moved = max(moved, math.dist(truth(robot), p))
        time.sleep(0.25)
    row["restart_moved_m"] = round(moved, 3)
    row["restart_status"] = {k: brain.get("/mission/status").json().get(k)
                             for k in ("running", "outcome", "cancelled_by", "end_reason")}
    row["pass_d"] = moved >= 0.20
    brain.post("/mission/stop")
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--code", default=ROOT)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    heavy_lock()
    procs = []
    try:
        robot, brain = start(args.code, procs)
        row = run(robot, brain)
        git = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=args.code,
                             capture_output=True, text=True).stdout.strip()
        row.update(code=git, at=time.strftime("%Y-%m-%dT%H:%M:%S"))
        out = open(args.out, "a") if args.out else sys.stdout
        print(json.dumps(row), file=out, flush=True)
    finally:
        stop_all(procs)


if __name__ == "__main__":
    main()
