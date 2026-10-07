"""The home tour, with nav2's log kept and each goal's reply recorded."""
import json
import os
import subprocess
import sys
import time

sys.path.insert(0, ".")
os.environ.update(SIM_MAP="home_first_floor")
from tests import demo_explore as dx

S = sys.argv[1]
robot = dx.stack("home_first_floor")
t_up = time.time()
os.environ.update(PICAR_ROBOT_URL=f"http://127.0.0.1:{dx.ROBOT}",
                  PICAR_BRIDGE_URL=f"http://127.0.0.1:{dx.BRIDGE}")
from tests import demo_nav_goals as ng

rows = []
for name, x, y in ng.HOME_GOALS:
    t = time.time() - t_up
    pose = robot.get("/world/pose").json()
    truth = robot.get("/world/truth").json()
    mp = robot.get("/world/map").json()
    known = sum(1 for c in (mp.get("cells") or []) if c != -1) if isinstance(mp.get("cells"), list) else None
    r = ng.run_goal(robot, x, y)
    rows.append({"goal": name, "x": x, "y": y, "sent_at_s": round(t, 1), **r,
                 "truth_before": [round(truth["x_m"], 2), round(truth["y_m"], 2)],
                 "slam_before": [round(pose.get("x_m", 0), 2), round(pose.get("y_m", 0), 2)],
                 "known_cells_before": known})
    print(json.dumps(rows[-1]), flush=True)
subprocess.run(f"docker logs {dx.CONTAINER} > {S}/tour-diag-ros.log 2>&1", shell=True)
subprocess.run(["docker", "rm", "-f", dx.CONTAINER], capture_output=True)
dx._kill_ports()
