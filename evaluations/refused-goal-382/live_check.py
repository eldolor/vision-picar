"""
python evaluations/refused-goal-382/live_check.py [--code DIR] [--out FILE]

`PLAN-ros-alignment.md` 3.82 criterion 6: a goal after a Stop, through the
real stack -- the robot server (`ROBOT_DRIVE=ros`, `WORLD_MODE=ros`,
`SIM_MAP=scaled_house`), the ROS container (`vision-picar-ros:latest`:
slam_toolbox, nav2, the bridge) -- judged on the simulator's ground truth
(`GET /world/truth`).

A pass-through proxy sits between the robot server and the bridge, so the
two refusals this section is about can be provoked on the real path:
`DELETE /goal` answered 503 (the Stop's first cancels fail, as on a bridge
blip) and `POST /goal` answered 503 ("nav2 is not up").

  (a) goal A driving; Stop; at once a goal B the bridge refuses, with the
      Stop's cancels failing for 2 s. The robot's TRUE displacement over the
      next 8 s, and goal A's state at the end.
  (b) Stop, then an accepted goal B: does nav2 reach it?

`--code DIR` runs the robot server from another checkout (the `dev`
baseline). Ports, ROS domain and container come from
`python tools/plan_section.py env`. One JSON line per scenario.
"""

import argparse
import json
import math
import os
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import httpx

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from tests.demo_explore import _listeners, heavy_lock  # noqa: E402

PY = sys.executable
ROBOT = int(os.environ["PICAR_ROBOT_PORT"])
BRIDGE = int(os.environ["PICAR_BRIDGE_PORT"])
PROXY = int(os.environ["PICAR_BODY_PORT"])          # unused by a plain-sim stack
CONTAINER = os.environ["PICAR_ROS_CONTAINER"]
DOMAIN = os.environ["ROS_DOMAIN_ID"]
IMAGE = "vision-picar-ros:latest"
LOGDIR = os.environ.get("EXPLORE_LOGDIR", "/tmp/vision-picar-3.82")

# Run 1 (2026-10-15 11:50): nav2 rejected a first goal into the unmapped
# hallway (4.35, 0.75); goal A now stays in the start room, and is retried.
GOAL_A = (3.45, 1.35)          # the start room, east, from the start (1.95, 1.35)
GOAL_B_REFUSED = (1.95, 3.15)
GOAL_B = (1.95, 1.35)          # back to the start
TERMINAL = {"succeeded", "aborted", "canceled", "rejected"}


class Faults:
    fail_deletes = False
    refuse_posts = False
    deletes_failed = 0
    posts_refused = 0


def proxy_server():
    upstream = httpx.Client(base_url=f"http://127.0.0.1:{BRIDGE}", timeout=10)

    class Handler(BaseHTTPRequestHandler):
        def _relay(self):
            path = self.path.split("?")[0]
            if path == "/goal" and self.command == "DELETE" and Faults.fail_deletes:
                Faults.deletes_failed += 1
                return self._answer(503, b'{"error": "injected: cancel failed"}')
            if path == "/goal" and self.command == "POST" and Faults.refuse_posts:
                Faults.posts_refused += 1
                n = int(self.headers.get("content-length") or 0)
                self.rfile.read(n)
                return self._answer(503, b'{"error": "nav2 is not up (injected)"}')
            n = int(self.headers.get("content-length") or 0)
            body = self.rfile.read(n) if n else None
            headers = {k: v for k, v in self.headers.items()
                       if k.lower() not in ("host", "content-length", "connection")}
            try:
                r = upstream.request(self.command, self.path, content=body, headers=headers)
            except httpx.HTTPError as e:
                return self._answer(502, json.dumps({"error": f"proxy: {e}"}).encode())
            return self._answer(r.status_code, r.content)

        def _answer(self, status, content):
            self.send_response(status)
            self.send_header("content-type", "application/json")
            self.send_header("content-length", str(len(content)))
            self.end_headers()
            self.wfile.write(content)

        do_GET = do_POST = do_DELETE = do_PUT = _relay

        def log_message(self, *a):
            pass

    srv = ThreadingHTTPServer(("127.0.0.1", PROXY), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


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


def start(code_dir):
    os.makedirs(LOGDIR, exist_ok=True)
    for port in (ROBOT, BRIDGE):            # the proxy is ours, already up
        if _listeners(port):
            raise RuntimeError(f"port {port} is already served")
    subprocess.run(["docker", "rm", "-f", CONTAINER], capture_output=True)
    env = {k: v for k, v in os.environ.items() if k not in ("APP_SHARED_SECRET", "LOCAL_SECRET")}
    env.update(SIM_MAP="scaled_house", SIM_MOVERS="", ROBOT_DRIVE="ros", WORLD_MODE="ros",
               ROS_BRIDGE_URL=f"http://127.0.0.1:{PROXY}", PICAR_LOG_LEVEL="WARNING")
    log = open(f"{LOGDIR}/robot_{os.path.basename(code_dir.rstrip('/'))}_{int(time.time())}.log", "w")
    robot_proc = subprocess.Popen([PY, "-m", "uvicorn", "robot.server:app", "--port", str(ROBOT),
                                   "--host", "127.0.0.1", "--log-level", "warning"],
                                  cwd=code_dir, env=env, stdout=log, stderr=subprocess.STDOUT)
    time.sleep(4)
    if robot_proc.poll() is not None or str(robot_proc.pid) not in _listeners(ROBOT):
        raise RuntimeError("the robot server did not come up")
    subprocess.run(["docker", "run", "-d", "--name", CONTAINER,
                    "-p", f"127.0.0.1:{BRIDGE}:8090", "-e", f"ROS_DOMAIN_ID={DOMAIN}",
                    "-e", f"ROBOT_URL=http://host.docker.internal:{ROBOT}",
                    IMAGE, "ros2", "launch", "picar_bringup", "picar.launch.py"],
                   capture_output=True, check=True)
    robot = httpx.Client(base_url=f"http://127.0.0.1:{ROBOT}", timeout=30)
    bridge = httpx.Client(base_url=f"http://127.0.0.1:{BRIDGE}", timeout=30)
    end = time.time() + 150
    while time.time() < end:
        try:
            if (robot.get("/health").json()["drive"]["ros_up"]
                    and robot.get("/world/map").json().get("usable")
                    and bridge.get("/slam/pose").json().get("start_truth")
                    and robot.get("/world/goal").status_code == 200):
                return robot, robot_proc
        except Exception:  # noqa: BLE001 -- still coming up
            pass
        time.sleep(2)
    raise RuntimeError("stack did not come up")


def truth(robot):
    t = robot.get("/world/truth").json()
    return t["x_m"], t["y_m"]


def goal_state(robot):
    return ((robot.get("/world/goal").json() or {}).get("goal") or {}).get("state")


def wait_moving(robot, timeout=60.0):
    t0, p0 = time.time(), truth(robot)
    while time.time() - t0 < timeout:
        if goal_state(robot) == "active" and math.dist(truth(robot), p0) > 0.10:
            return True
        time.sleep(0.2)
    return False


def scenario_a(robot):
    Faults.fail_deletes = Faults.refuse_posts = False
    for attempt in range(3):
        r = robot.post("/world/goal", json={"x_m": GOAL_A[0], "y_m": GOAL_A[1]}).json()
        if r.get("accepted") and wait_moving(robot, timeout=20.0):
            break
        time.sleep(5.0)                              # the costmap still coming up
    else:
        return {"scenario": "a", "error": "goal A never drove", "reply": r,
                "state": goal_state(robot)}
    Faults.fail_deletes = Faults.refuse_posts = True
    stop = robot.post("/stop", headers={"x-driver": "twin-dpad"}).json()
    refused = robot.post("/world/goal", json={"x_m": GOAL_B_REFUSED[0], "y_m": GOAL_B_REFUSED[1]}).json()
    t0 = time.time()
    p0 = truth(robot)
    worst, path = 0.0, []
    while time.time() - t0 < 8.0:
        if Faults.fail_deletes and time.time() - t0 > 2.0:
            Faults.fail_deletes = False             # the bridge takes cancels again
        p = truth(robot)
        worst = max(worst, math.dist(p, p0))
        path.append([round(time.time() - t0, 2), round(p[0], 3), round(p[1], 3)])
        time.sleep(0.1)
    Faults.fail_deletes = Faults.refuse_posts = False
    end_state = goal_state(robot)
    return {"scenario": "a", "stop": stop.get("executed"), "refused_reply": refused,
            "deletes_failed": Faults.deletes_failed, "posts_refused": Faults.posts_refused,
            "max_true_displacement_m": round(worst, 4),
            "final_displacement_m": round(math.dist(truth(robot), p0), 4),
            "goal_a_end_state": end_state,
            "pass": worst <= 0.03 and end_state not in ("active", "succeeded"),
            "path": path[::10]}


def scenario_b(robot):
    Faults.fail_deletes = Faults.refuse_posts = False
    time.sleep(2.0)                                  # the D-pad stop's authority lapses
    robot.post("/stop", headers={"x-driver": "twin-dpad"})
    time.sleep(0.1)
    r = robot.post("/world/goal", json={"x_m": GOAL_B[0], "y_m": GOAL_B[1]}).json()
    t0, state = time.time(), None
    while time.time() - t0 < 60.0:
        state = goal_state(robot)
        if state in TERMINAL:
            break
        time.sleep(0.25)
    p = truth(robot)
    return {"scenario": "b", "accepted": r.get("accepted"), "end_state": state,
            "seconds": round(time.time() - t0, 1),
            "end_error_m": round(math.dist(p, GOAL_B), 3),
            "pass": bool(r.get("accepted")) and state == "succeeded"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--code", default=ROOT)
    ap.add_argument("--out", default=None)
    ap.add_argument("--skip-b", action="store_true")
    args = ap.parse_args()
    heavy_lock()
    srv = proxy_server()
    procs = []
    try:
        robot, proc = start(args.code)
        procs.append(proc)
        rows = [scenario_a(robot)]
        if not args.skip_b:
            rows.append(scenario_b(robot))
        git = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=args.code,
                             capture_output=True, text=True).stdout.strip()
        out = open(args.out, "a") if args.out else sys.stdout
        for row in rows:
            row.update(code=git, at=time.strftime("%Y-%m-%dT%H:%M:%S"))
            print(json.dumps(row), file=out, flush=True)
    finally:
        stop_all(procs)
        srv.shutdown()


if __name__ == "__main__":
    main()
