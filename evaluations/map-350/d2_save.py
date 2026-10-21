"""3.50.D2-a: a map is saved when a mission ends, and survives the container.

A robot server, a brain and this branch's container with a host directory
at /maps (a bind mount, so the files can be hashed here). A frontier
mission in the starter house runs ~25 s and is stopped; the bridge should
save v1 within 5 s of `running` turning false. Then the container is
removed and a new one started on the same directory: the store must still
offer v1, byte for byte.

    eval "$(python tools/plan_section.py env)"
    python evaluations/map-350/d2_save.py
"""
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "service/slam/src/picar_bridge"))

import httpx  # noqa: E402

from picar_bridge.mapstore import GRAPH_FILES, MapStore  # noqa: E402
from tests import demo_explore as de  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
HOUSE = "starter_house"
IMAGE = os.environ.get("ROS_IMAGE", "vision-picar-ros:plan-3.50")


def container(maps):
    subprocess.run(["docker", "rm", "-f", de.CONTAINER], capture_output=True)
    subprocess.run(["docker", "run", "-d", "--name", de.CONTAINER, "-v", f"{maps}:/maps",
                    "-p", f"127.0.0.1:{de.BRIDGE}:8090", "-e", f"ROS_DOMAIN_ID={de.ROS_DOMAIN}",
                    "-e", f"ROBOT_URL=http://host.docker.internal:{de.ROBOT}",
                    "-e", f"BRAIN_URL=http://host.docker.internal:{de.BRAIN}",
                    IMAGE, "ros2", "launch", "picar_bringup", "picar.launch.py"],
                   capture_output=True, check=True)


def wait_up(robot, bridge, timeout=120):
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            if robot.get("/health").json()["drive"]["ros_up"] and \
                    bridge.get("/slam/pose").json().get("start_truth_pending") is False:
                return
        except Exception:  # noqa: BLE001 -- still coming up
            pass
        time.sleep(2)
    raise RuntimeError("stack did not come up")


def digest(prefix):
    return {ext: hashlib.sha256(open(prefix + ext, "rb").read()).hexdigest() for ext in GRAPH_FILES}


def main():
    de.heavy_lock()
    de._kill_ports()
    de._new_run_dir()
    maps = os.path.join(de.RUN_DIR, "maps")
    os.makedirs(maps)
    env = {k: v for k, v in os.environ.items() if k not in ("APP_SHARED_SECRET", "LOCAL_SECRET")}
    env.update(SIM_MAP=HOUSE, ROBOT_DRIVE="ros", WORLD_MODE="ros",
               ROS_BRIDGE_URL=f"http://127.0.0.1:{de.BRIDGE}", PICAR_LOG_LEVEL="WARNING")
    subprocess.Popen([de.PY, "-m", "uvicorn", "robot.server:app", "--port", str(de.ROBOT),
                      "--host", "127.0.0.1", "--log-level", "warning"], cwd=ROOT, env=env,
                     stdout=open(f"{de.RUN_DIR}/robot.log", "w"), stderr=subprocess.STDOUT)
    subprocess.Popen([de.PY, "-m", "uvicorn", "control.brain_server:app", "--port", str(de.BRAIN),
                      "--host", "127.0.0.1", "--log-level", "warning"], cwd=ROOT,
                     env=dict(env, ROBOT_URL=f"http://127.0.0.1:{de.ROBOT}", PICAR_LOG_LEVEL="INFO"),
                     stdout=open(f"{de.RUN_DIR}/brain.log", "w"), stderr=subprocess.STDOUT)
    time.sleep(4)
    container(maps)
    robot = httpx.Client(base_url=f"http://127.0.0.1:{de.ROBOT}", timeout=30)
    brain = httpx.Client(base_url=f"http://127.0.0.1:{de.BRAIN}", timeout=30)
    bridge = httpx.Client(base_url=f"http://127.0.0.1:{de.BRIDGE}", timeout=30)
    wait_up(robot, bridge)
    out = {"house": HOUSE, "image": IMAGE}
    h0 = bridge.get("/health").json()
    out["store_mounted"] = h0["map"]["store"]
    map_id, start_truth = h0["map"]["map_id"], bridge.get("/slam/pose").json()["start_truth"]

    r = brain.post("/mission/start", json={"target_object": "red backpack", "policy": "frontier"})
    out["start"] = r.status_code
    time.sleep(25)
    brain.post("/mission/stop")
    while brain.get("/mission/status").json().get("running"):
        time.sleep(0.1)
    ended = time.time()
    store, cur = MapStore(maps), None
    while time.time() - ended < 15:
        cur = store.current()
        if cur is not None:
            break
        time.sleep(0.1)
    out["saved_after_s"] = round(time.time() - ended, 2) if cur else None
    h = bridge.get("/health").json()["map"]
    out["bridge_map"] = h
    ok = {"within_5s": cur is not None and out["saved_after_s"] <= 5.0, "failed_saves_0": h["failed"] == 0}
    if cur:
        out["meta"] = cur.meta
        ok.update(version_1=cur.version == 1, map_id=cur.map_id == map_id,
                  dock=cur.meta["dock_pose"] == {"x": 0.0, "y": 0.0, "theta": 0.0},
                  start_truth=cur.meta["start_truth"] == start_truth,
                  files_nonempty=all(os.path.getsize(cur.prefix + e) > 0 for e in GRAPH_FILES))
        before = digest(cur.prefix)
        subprocess.run(["docker", "logs", de.CONTAINER], stdout=open(f"{de.RUN_DIR}/ros1.log", "w"),
                       stderr=subprocess.STDOUT)
        container(maps)
        wait_up(robot, bridge)
        again = MapStore(maps).current()
        ok["survives_restart"] = (again is not None and again.version == 1
                                  and digest(again.prefix) == before)
    out["criteria"] = ok
    out["D2a"] = "MET" if all(ok.values()) else "FAILED"
    subprocess.run(["docker", "logs", de.CONTAINER], stdout=open(f"{de.RUN_DIR}/ros.log", "w"),
                   stderr=subprocess.STDOUT)
    de._kill_ports()
    with open(os.path.join(HERE, "d2a-save.json"), "w") as f:
        json.dump(out, f, indent=1)
    print(json.dumps(out, indent=1))
    shutil.copytree(maps, os.path.join(de.RUN_DIR, "maps-kept"), dirs_exist_ok=True)


if __name__ == "__main__":
    main()
