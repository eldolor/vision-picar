"""3.50.D1: does the pinned image save and reload a map, by hand?

Not a criterion run -- the feasibility check D2-D4 are designed on. Three
stacks in the starter house, each a fresh robot server (the robot back at
its start, the "dock") and a fresh container:

  1. map: 3.14's lap under the stock async node, then serialise the graph.
  2. arm A: the stock async node (mapping); deserialise START_AT_FIRST_NODE.
  3. arm B: map_and_localization_slam_toolbox_node in localisation mode;
     deserialise LOCALIZE_AT_POSE (0, 0, 0) -- localisation refuses the
     dock -- then switch to mapping and serialise again (D's re-map save).

For each reload: the share of the saved map's cells the reloaded /map agrees
with (tri-state: unknown / free / occupied; criterion 10's 99%), and the
pose error at rest against the truth after 5 s (criterion 1's 10 cm, 3 deg).

    eval "$(python tools/plan_section.py env)"
    python -m evaluations.map-350.d1_roundtrip   # or: python evaluations/map-350/d1_roundtrip.py
"""
import json
import os
import subprocess
import sys
import tempfile
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, ROOT)

import httpx  # noqa: E402

from tests import demo_explore as de  # noqa: E402
from tests import demo_slam_lap as lap  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
HOUSE = "starter_house"
# D1 only reads the shared image (the same Dockerfile pin); nothing rebuilds it.
IMAGE = os.environ.get("D1_IMAGE", "vision-picar-ros:latest")
SAVE_DIR = "/tmp/picar-maps"          # inside the container
ROS = ("source /opt/ros/humble/setup.bash && source /underlay/install/setup.bash && "
       "source /ws/install/setup.bash && ")


def up(slam_exec="async_slam_toolbox_node", slam_mode="mapping"):
    """stack() with the slam node and mode swapped by mounting variants."""
    os.makedirs(de.LOGDIR, exist_ok=True)
    tmp = tempfile.mkdtemp(prefix="d1-mount-", dir=de.LOGDIR)   # a path Docker Desktop shares
    launch = open(os.path.join(ROOT, "service/slam/src/picar_bringup/launch/picar.launch.py")).read()
    assert launch.count('executable="async_slam_toolbox_node"') == 1
    with open(os.path.join(tmp, "picar.launch.py"), "w") as f:
        f.write(launch.replace('executable="async_slam_toolbox_node"', f'executable="{slam_exec}"'))
    slam = open(os.path.join(ROOT, "service/slam/src/picar_bringup/config/slam.yaml")).read()
    assert slam.count("    mode: mapping\n") == 1
    with open(os.path.join(tmp, "slam.yaml"), "w") as f:
        f.write(slam.replace("    mode: mapping\n", f"    mode: {slam_mode}\n"))
    de.heavy_lock()
    de._kill_ports()
    de._new_run_dir()
    env = {k: v for k, v in os.environ.items() if k not in ("APP_SHARED_SECRET", "LOCAL_SECRET")}
    env.update(SIM_MAP=HOUSE, ROBOT_DRIVE="ros", WORLD_MODE="ros",
               ROS_BRIDGE_URL=f"http://127.0.0.1:{de.BRIDGE}", PICAR_LOG_LEVEL="WARNING")
    subprocess.Popen([de.PY, "-m", "uvicorn", "robot.server:app", "--port", str(de.ROBOT),
                      "--host", "127.0.0.1", "--log-level", "warning"], cwd=ROOT, env=env,
                     stdout=open(f"{de.RUN_DIR}/robot.log", "w"), stderr=subprocess.STDOUT)
    time.sleep(4)
    subprocess.run(["docker", "run", "-d", "--name", de.CONTAINER,
                    "-v", f"{tmp}/picar.launch.py:/ws/src/picar_bringup/launch/picar.launch.py:ro",
                    "-v", f"{tmp}/slam.yaml:/ws/src/picar_bringup/config/slam.yaml:ro",
                    "-p", f"127.0.0.1:{de.BRIDGE}:8090", "-e", f"ROS_DOMAIN_ID={de.ROS_DOMAIN}",
                    "-e", f"ROBOT_URL=http://host.docker.internal:{de.ROBOT}",
                    IMAGE,
                    "ros2", "launch", "picar_bringup", "picar.launch.py"],
                   capture_output=True, check=True)
    robot = httpx.Client(base_url=f"http://127.0.0.1:{de.ROBOT}", timeout=60)
    bridge = httpx.Client(base_url=f"http://127.0.0.1:{de.BRIDGE}", timeout=30)
    deadline = time.time() + 120
    while time.time() < deadline:
        try:
            if robot.get("/health").json()["drive"]["ros_up"] and \
                    bridge.get("/slam/pose").json().get("start_truth_pending") is False:
                return robot, bridge
        except Exception:  # noqa: BLE001 -- still coming up
            pass
        time.sleep(2)
    raise RuntimeError("stack did not come up")


def ros(cmd, timeout=60):
    r = subprocess.run(["docker", "exec", de.CONTAINER, "bash", "-lc", ROS + cmd],
                       capture_output=True, text=True, timeout=timeout)
    return r.returncode, (r.stdout + r.stderr).strip()


def call(service, srv_type, body):
    return ros(f"ros2 service call {service} {srv_type} \"{body}\"")


def tri(v):
    return "u" if v < 0 else ("o" if v >= 50 else "f")


def agreement(saved, now):
    """Share of the saved map's cells whose state the reloaded map matches,
    looked up by world position (the two grids' origins may differ)."""
    same = total = 0
    known_same = known = 0
    r = saved["resolution_m"]
    for j in range(saved["height"]):
        for i in range(saved["width"]):
            s = tri(saved["data"][j * saved["width"] + i])
            x = saved["origin_x_m"] + (i + 0.5) * r
            y = saved["origin_y_m"] + (j + 0.5) * r
            ii = int((x - now["origin_x_m"]) / now["resolution_m"])
            jj = int((y - now["origin_y_m"]) / now["resolution_m"])
            n = (tri(now["data"][jj * now["width"] + ii])
                 if 0 <= ii < now["width"] and 0 <= jj < now["height"] else "u")
            total += 1
            same += s == n
            if s != "u":
                known += 1
                known_same += s == n
    return {"all_cells": round(same / total, 4), "known_cells": round(known_same / max(known, 1), 4),
            "saved_known": known, "saved_cells": total}


def at_rest(robot, bridge, wait_s=5.0):
    time.sleep(wait_s)
    e = robot.get("/world/error").json()
    return {k: e.get(k) for k in ("usable", "position_error_m", "heading_error_deg")}


def fresh_map(bridge, after_version, timeout=20):
    deadline = time.time() + timeout
    while time.time() < deadline:
        m = bridge.get("/slam/map").json()
        if m.get("version", 0) > after_version:
            return m
        time.sleep(0.5)
    return bridge.get("/slam/map").json()


def main():
    out = {"house": HOUSE, "image": IMAGE}
    host_maps = os.path.join(de.LOGDIR, "d1-maps")
    os.makedirs(host_maps, exist_ok=True)

    # 1. Map and save.
    robot, bridge = up()
    lap.run_lap(robot)
    time.sleep(3)
    saved = bridge.get("/slam/map").json()
    out["lap_end_error"] = at_rest(robot, bridge, 0)
    code, text = ros(f"mkdir -p {SAVE_DIR}")
    code, text = call("/slam_toolbox/serialize_map", "slam_toolbox/srv/SerializePoseGraph",
                      f"{{filename: '{SAVE_DIR}/v1'}}")
    out["serialise"] = {"code": code, "reply": text[-300:]}
    subprocess.run(["docker", "cp", f"{de.CONTAINER}:{SAVE_DIR}/.", host_maps], check=True)
    out["files"] = {f: os.path.getsize(os.path.join(host_maps, f)) for f in sorted(os.listdir(host_maps))}
    with open(os.path.join(host_maps, "saved_map.json"), "w") as f:
        json.dump(saved, f)

    # 2. Arm A: async (mapping) node, reload from the first node.
    arms = {}
    for name, exec_, mode, match, after in (
            ("A_async_first_node", "async_slam_toolbox_node", "mapping", 1, None),
            ("B_map_and_loc_at_pose", "map_and_localization_slam_toolbox_node", "localization", 3,
             "remap_save")):
        robot, bridge = up(exec_, mode)
        arm = {"before": at_rest(robot, bridge, 0)}
        ros(f"mkdir -p {SAVE_DIR}")
        subprocess.run(["docker", "cp", f"{host_maps}/.", f"{de.CONTAINER}:{SAVE_DIR}"], check=True)
        v0 = bridge.get("/slam/map").json().get("version", 0)
        t0 = time.time()
        code, text = call("/slam_toolbox/deserialize_map", "slam_toolbox/srv/DeserializePoseGraph",
                          f"{{filename: '{SAVE_DIR}/v1', match_type: {match}, "
                          f"initial_pose: {{x: 0.0, y: 0.0, theta: 0.0}}}}")
        arm["deserialise"] = {"code": code, "s": round(time.time() - t0, 2), "reply": text[-300:]}
        now = fresh_map(bridge, v0)
        arm["map_agreement"] = agreement(saved, now) if now.get("data") else "no map"
        arm["at_rest_5s"] = at_rest(robot, bridge)
        if after == "remap_save":
            code, text = call("/slam_toolbox/set_localization_mode", "std_srvs/srv/SetBool",
                              "{data: false}")
            arm["to_mapping"] = {"code": code, "reply": text[-200:]}
            code, text = call("/slam_toolbox/serialize_map", "slam_toolbox/srv/SerializePoseGraph",
                              f"{{filename: '{SAVE_DIR}/v2'}}")
            arm["remap_serialise"] = {"code": code, "reply": text[-300:]}
            code, text = ros(f"ls -l {SAVE_DIR}")
            arm["remap_files"] = text
        arms[name] = arm
        subprocess.run(["docker", "logs", de.CONTAINER], stdout=open(f"{de.RUN_DIR}/ros.log", "w"),
                       stderr=subprocess.STDOUT)
    out["arms"] = arms
    de._kill_ports()
    with open(os.path.join(HERE, "d1-roundtrip.json"), "w") as f:
        json.dump(out, f, indent=1)
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
