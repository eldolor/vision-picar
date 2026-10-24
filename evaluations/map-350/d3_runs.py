"""3.50.D3: the reload, measured on the criteria written in the plan
("3.50.D3 criteria") before this ran.

Per house (scaled_house, home_first_floor), on one host directory at /maps:

  map    an explore mission for an absent target, stopped at a time limit;
         the bridge saves v1 at its end (D2)
  a, b   three sessions: robot server, brain and container restarted (the
         robot on the dock); the reload's verdict, the reloaded /map against
         the map run's last /map, the error at rest, then after a drive of
         >= 1 m and a 90 deg turn                     (D3-a = session 1, D3-b)
  d      three objects next to the dock moved, then a session as above (D3-d)
  e      off the dock: after a matched session, a drive >= 1 m and a turn of
         45 deg, then only the container restarted; five times     (D3-e)

and in starter_house, 3.14's lap on a fresh map, then on a reload of it (D3-c).

The maps are a floor plan of the SIM house, kept under EXPLORE_LOGDIR, never
committed (only numbers are). Raw output: evaluations/map-350/D3/.

    eval "$(python tools/plan_section.py env)"
    python evaluations/map-350/d3_runs.py [scaled_house home_first_floor lap]
"""
import json
import math
import os
import subprocess
import sys
import threading
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, ROOT)

import httpx  # noqa: E402

from tests import demo_explore as de  # noqa: E402
from tests import demo_slam_lap as lap  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "D3")
IMAGE = os.environ.get("ROS_IMAGE", "")
assert IMAGE.endswith("plan-3.50"), "run with this branch's env"
MAP_RUN_S = {"scaled_house": 420, "home_first_floor": 900}
ABSENT = "purple unicorn"          # never found: explore maps until its time limit
BAR_M, BAR_DEG, WITHIN_S = 0.10, 3.0, 5.0


def _env(house):
    env = {k: v for k, v in os.environ.items() if k not in ("APP_SHARED_SECRET", "LOCAL_SECRET")}
    env.update(SIM_MAP=house, ROBOT_DRIVE="ros", WORLD_MODE="ros",
               ROS_BRIDGE_URL=f"http://127.0.0.1:{de.BRIDGE}", PICAR_LOG_LEVEL="WARNING")
    return env


def servers(house):
    """A fresh robot server (the robot back on the dock) and brain."""
    subprocess.run(["docker", "rm", "-f", de.CONTAINER], capture_output=True)
    de._kill_ports()
    env = _env(house)
    tag = int(time.time())
    subprocess.Popen([de.PY, "-m", "uvicorn", "robot.server:app", "--port", str(de.ROBOT),
                      "--host", "127.0.0.1", "--log-level", "warning"], cwd=ROOT, env=env,
                     stdout=open(f"{de.RUN_DIR}/robot-{tag}.log", "w"), stderr=subprocess.STDOUT)
    subprocess.Popen([de.PY, "-m", "uvicorn", "control.brain_server:app", "--port", str(de.BRAIN),
                      "--host", "127.0.0.1", "--log-level", "warning"], cwd=ROOT,
                     env=dict(env, ROBOT_URL=f"http://127.0.0.1:{de.ROBOT}", PICAR_LOG_LEVEL="INFO"),
                     stdout=open(f"{de.RUN_DIR}/brain-{tag}.log", "w"), stderr=subprocess.STDOUT)
    time.sleep(4)
    return (httpx.Client(base_url=f"http://127.0.0.1:{de.ROBOT}", timeout=60),
            httpx.Client(base_url=f"http://127.0.0.1:{de.BRAIN}", timeout=30))


_logs = [0]


def _network():
    """Docker Desktop (the Mac) reaches the host's 127.0.0.1 servers as
    host.docker.internal; Linux does not (the servers bind 127.0.0.1), so
    there the container shares the host's network, as on the Jetson
    (tools/jetson/README.md), with the bridge on this section's own port."""
    if sys.platform.startswith("linux"):
        return ["--network", "host", "-e", f"BRIDGE_PORT={de.BRIDGE}",
                "-e", f"ROBOT_URL=http://127.0.0.1:{de.ROBOT}",
                "-e", f"BRAIN_URL=http://127.0.0.1:{de.BRAIN}"]
    return ["-p", f"127.0.0.1:{de.BRIDGE}:8090",
            "-e", f"ROBOT_URL=http://host.docker.internal:{de.ROBOT}",
            "-e", f"BRAIN_URL=http://host.docker.internal:{de.BRAIN}"]


def container(maps, bridge):
    """Restart only the container; watch it come up at 10 Hz. Returns the
    watch: when the reload was judged, and whether anything leaked before."""
    if subprocess.run(["docker", "inspect", de.CONTAINER], capture_output=True).returncode == 0:
        _logs[0] += 1
        subprocess.run(["docker", "logs", de.CONTAINER],
                       stdout=open(f"{de.RUN_DIR}/ros-{_logs[0]}.log", "w"), stderr=subprocess.STDOUT)
    subprocess.run(["docker", "rm", "-f", de.CONTAINER], capture_output=True)
    subprocess.run(["docker", "run", "-d", "--name", de.CONTAINER, "-v", f"{maps}:/maps",
                    *_network(), "-e", f"ROS_DOMAIN_ID={de.ROS_DOMAIN}",
                    IMAGE, "ros2", "launch", "picar_bringup", "picar.launch.py"],
                   capture_output=True, check=True)
    w = {"leaked_pose": 0, "leaked_map": 0, "goal_during_check": None, "states": []}
    t0 = time.time()
    while time.time() - t0 < 180:
        try:
            h = bridge.get("/health").json()
        except Exception:  # noqa: BLE001 -- still coming up
            time.sleep(0.1)
            continue
        st = h["slam"]["state"]
        if not w["states"] or w["states"][-1][0] != st:
            w["states"].append((st, round(time.time() - t0, 2)))
        if st == "failed":
            w["health"] = h
            return w
        if st != "ready":
            p = bridge.get("/slam/pose").json()
            m = bool(bridge.get("/slam/map").json().get("data"))
            # Bracketed (2026-10-23): a sample counts only if the state after it
            # is still not ready -- the first home run counted the one read that
            # fell across the verdict.
            if bridge.get("/health").json()["slam"]["state"] != "ready":
                w["leaked_pose"] += p.get("map") is not None
                w["leaked_map"] += m
            else:
                w["straddled"] = w.get("straddled", 0) + 1
            if st == "checking" and w["goal_during_check"] is None:
                # At the pose localisation assumes: a refusal is the test.
                w["goal_during_check"] = bridge.post("/goal", json={"x_m": 0.0, "y_m": 0.0}).status_code
        elif st == "ready" and bridge.get("/slam/pose").json().get("start_truth_pending") is False:
            w["health"] = h
            w["ready_at"] = time.time()
            return w
        time.sleep(0.1)
    raise RuntimeError(f"container did not become ready: {w['states']}")


def error(robot):
    e = robot.get("/world/error").json()
    return {k: e.get(k) for k in ("usable", "position_error_m", "heading_error_deg")}


def ok_error(e):
    return bool(e.get("usable")) and e["position_error_m"] <= BAR_M and abs(e["heading_error_deg"]) <= BAR_DEG


def act(robot, action, **kw):
    r = robot.post("/action", json={"action": action, **kw}, headers={"x-driver": "twin-dpad"})
    time.sleep(0.6)
    return r.json().get("executed", False)


def drive(robot, metres=1.0, turn_deg=90):
    """Forward until the truth has moved `metres` (turning left where the
    way is blocked), then turn `turn_deg` right; returns what was done."""
    t0 = robot.get("/world/truth").json()
    for _ in range(30):
        t = robot.get("/world/truth").json()
        if math.hypot(t["x_m"] - t0["x_m"], t["y_m"] - t0["y_m"]) >= metres:
            break
        if not act(robot, "FORWARD"):
            act(robot, "LEFT", angle=90)
    h0 = robot.get("/world/truth").json()["heading_deg"]
    if turn_deg:
        act(robot, "RIGHT", angle=turn_deg)
    t = robot.get("/world/truth").json()
    time.sleep(2.0)                                  # at rest
    return {"moved_m": round(math.hypot(t["x_m"] - t0["x_m"], t["y_m"] - t0["y_m"]), 2),
            "turned_deg": round(abs((t["heading_deg"] - h0 + 180) % 360 - 180), 1)}


def tri(v):
    return "u" if v < 0 else ("o" if v >= 65 else ("f" if v < 25 else "m"))


def agreement(saved, now):
    """Tri-state share of the saved /map's cells the reloaded one matches,
    by world position."""
    same = total = 0
    r = saved["resolution_m"]
    for j in range(saved["height"]):
        for i in range(saved["width"]):
            x = saved["origin_x_m"] + (i + 0.5) * r
            y = saved["origin_y_m"] + (j + 0.5) * r
            ii = int((x - now["origin_x_m"]) // now["resolution_m"])
            jj = int((y - now["origin_y_m"]) // now["resolution_m"])
            n = (tri(now["data"][jj * now["width"] + ii])
                 if 0 <= ii < now["width"] and 0 <= jj < now["height"] else "u")
            total += 1
            same += tri(saved["data"][j * saved["width"] + i]) == n
    return round(same / total, 4)


def first_map(bridge, timeout=15):
    deadline = time.time() + timeout
    while time.time() < deadline:
        m = bridge.get("/slam/map").json()
        if m.get("data"):
            return m
        time.sleep(0.2)
    return None


def session(house, maps, bridge_c, saved_map, saved_id, moved=None):
    """A session on the dock: servers and container fresh."""
    robot, brain = servers(house)
    for src, dst in (moved or []):
        r = robot.post("/sim/objects/move", json={"src": src, "dst": dst})
        assert r.status_code == 200, f"move {src} -> {dst}: {r.status_code} {r.text[:200]}"
    w = container(maps, bridge_c)
    rel = w["health"]["slam"]["reload"]
    out = {"watch": {k: v for k, v in w.items() if k != "health"}, "reload": rel,
           "map_id": w["health"]["map"]["map_id"]}
    if rel.get("result") != "matched":
        return out, robot
    m = first_map(bridge_c) if saved_map is not None else None
    out["map_agreement"] = agreement(saved_map, m) if m else None
    time.sleep(max(0.0, w["ready_at"] + 2.0 - time.time()))
    out["at_rest"] = error(robot)
    out["at_rest_s_after_verdict"] = round(time.time() - w["ready_at"], 1)
    out["drive"] = drive(robot, 1.0, 90)
    out["after_drive"] = error(robot)
    out["criteria"] = {"map_id_same": out["map_id"] == saved_id,
                       "at_rest": ok_error(out["at_rest"]),
                       "after_drive": ok_error(out["after_drive"]),
                       "drive_done": out["drive"]["moved_m"] >= 1.0 and out["drive"]["turned_deg"] >= 80,
                       "nothing_leaked": w["leaked_pose"] == 0 and w["leaked_map"] == 0
                                         and w["goal_during_check"] in (None, 503)}
    if out["map_agreement"] is not None:
        out["criteria"]["map_99"] = out["map_agreement"] >= 0.99
    return out, robot


def objects_near_dock(robot, n=3):
    t = robot.get("/world/truth").json()
    cx, cy = t["x_m"] / 0.3, t["y_m"] / 0.3
    objs = [o for o in robot.get("/sim/objects").json()["objects"] if not o["mover"]]
    objs.sort(key=lambda o: math.hypot(o["x"] + 0.5 - cx, o["y"] + 0.5 - cy))
    moves = []
    for o in objs:
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            dst = [o["x"] + dx, o["y"] + dy]
            r = robot.post("/sim/objects/move", json={"src": [o["x"], o["y"]], "dst": dst})
            if r.status_code == 200:
                robot.post("/sim/objects/move", json={"src": dst, "dst": [o["x"], o["y"]]})
                moves.append(([o["x"], o["y"]], dst, o["name"]))
                break
        if len(moves) == n:
            break
    return moves


def house_runs(house):
    res = {"house": house, "image": IMAGE}
    maps = os.path.join(de.RUN_DIR, f"maps-{house}")
    os.makedirs(maps, exist_ok=True)
    bridge = httpx.Client(base_url=f"http://127.0.0.1:{de.BRIDGE}", timeout=30)

    # The map run: explore for an absent target, stopped at its time limit.
    robot, brain = servers(house)
    w = container(maps, bridge)
    de.MISSION_LIMIT_S = MAP_RUN_S[house]
    res["map_run"] = {"watch_reload": w["health"]["slam"]["reload"],
                      "mission": de.run_mission(robot, house, policy="explore", target=ABSENT,
                                                max_steps=100000)}
    t_end = time.time()
    while time.time() - t_end < 20 and bridge.get("/health").json()["map"]["saved"] < 1:
        time.sleep(0.2)
    h = bridge.get("/health").json()["map"]
    saved_map = bridge.get("/slam/map").json()
    res["map_run"].update(saved=h, final_error=error(robot))
    assert h["saved"] >= 1, f"the map run saved nothing: {h}"
    saved_id = h["map_id"]

    sessions = []
    for i in range(3):
        s, robot = session(house, maps, bridge, saved_map, saved_id)
        sessions.append(s)
    res["sessions"] = sessions                               # D3-a is sessions[0]

    robot, _ = servers(house)
    moves = objects_near_dock(robot)
    res["moved_objects"] = moves
    res["clutter"], robot = session(house, maps, bridge, saved_map, saved_id,
                                    moved=[(m[0], m[1]) for m in moves])

    res["off_dock_drive"], res["off_dock"] = off_dock(house, maps, bridge, saved_map, saved_id)
    res["verdicts"] = verdicts(res)
    return res


def truth(robot):
    t = robot.get("/world/truth").json()
    return t["x_m"], t["y_m"], t["heading_deg"]


def off_dock(house, maps, bridge, saved_map, saved_id):
    """D3-e: the robot driven off the dock -- judged on the TRUTH, from the
    dock to where it stands at the reload (the first run of 2026-10-23 judged
    only the last leg's 0.70 m / 29.6 deg, under the bar: not a result)."""
    trials = []
    robot, _ = servers(house)
    dock = truth(robot)
    s, robot = session(house, maps, bridge, saved_map, saved_id)
    assert s["reload"].get("result") == "matched", f"the on-dock session did not match: {s['reload']}"
    for _ in range(3):
        x, y, h = truth(robot)
        if math.hypot(x - dock[0], y - dock[1]) >= 1.0 and abs((h - dock[2] + 180) % 360 - 180) >= 30:
            break
        drive(robot, 1.0, 60)
    x, y, h = truth(robot)
    off = {"from_dock_m": round(math.hypot(x - dock[0], y - dock[1]), 2),
           "from_dock_deg": round(abs((h - dock[2] + 180) % 360 - 180), 1)}
    for _ in range(5):
        w = container(maps, bridge)
        rel = w["health"]["slam"]["reload"]
        st = dict(w["states"])
        # From the localisation node's start (~checking) to the FRESH node's
        # ready, not only to the verdict (harness review).
        took = (round(st["ready"] - st["checking"], 2)
                if "ready" in st and "checking" in st else None)
        trials.append({"reload": rel, "watch": {k: v for k, v in w.items() if k != "health"},
                       "map_id": w["health"]["map"]["map_id"], "fresh_ready_s": took,
                       "criteria": {"fell_back": rel.get("result") == "fell_back",
                                    "within_10s": took is not None and took <= 10.0,
                                    "new_map_id": w["health"]["map"]["map_id"] != saved_id,
                                    "nothing_leaked": w["leaked_pose"] == 0 and w["leaked_map"] == 0
                                                      and w["goal_during_check"] in (None, 503)}})
    return off, trials


def verdicts(res):
    s = res["sessions"]
    a = s[0].get("criteria", {})
    v = {"D3-a": all(a.get(k) for k in ("map_99", "map_id_same", "at_rest", "nothing_leaked"))
         and bool(a) and s[0]["at_rest_s_after_verdict"] <= WITHIN_S,
         "D3-b": all(x.get("criteria") and all(x["criteria"].get(k) for k in
                     ("map_id_same", "at_rest", "after_drive", "drive_done", "nothing_leaked"))
                     and x["at_rest_s_after_verdict"] <= WITHIN_S for x in s),
         "D3-d": bool(res["clutter"].get("criteria")) and all(
             res["clutter"]["criteria"].get(k) for k in ("at_rest", "after_drive", "drive_done")),
         "D3-e": res["off_dock_drive"]["from_dock_m"] >= 1.0 and res["off_dock_drive"]["from_dock_deg"] >= 30
         and all(all(t["criteria"].values()) for t in res["off_dock"])}
    return {k: "MET" if ok else "FAILED" for k, ok in v.items()}


def lap_runs():
    """D3-c: 3.14's lap on a fresh map, then on a reload of that map."""
    house = "starter_house"
    maps = os.path.join(de.RUN_DIR, "maps-lap")
    os.makedirs(maps, exist_ok=True)
    bridge = httpx.Client(base_url=f"http://127.0.0.1:{de.BRIDGE}", timeout=30)
    out = {}
    for arm in ("fresh", "reload"):
        robot, brain = servers(house)
        w = container(maps, bridge)
        out[arm] = {"reload": w["health"]["slam"]["reload"]}
        os.environ["PICAR_ROBOT_URL"] = f"http://127.0.0.1:{de.ROBOT}"
        samples, blocked = lap.run_lap(robot)
        out[arm].update(final=error(robot), blocked=blocked,
                        max_m=max((x[1] for x in samples), default=None))
        if arm == "fresh":
            # A save needs a mission's end; a short frontier mission, stopped.
            brain.post("/mission/start", json={"target_object": ABSENT, "policy": "frontier"})
            time.sleep(1.5)
            brain.post("/mission/stop")
            t = time.time()
            while time.time() - t < 20 and bridge.get("/health").json()["map"]["saved"] < 1:
                time.sleep(0.2)
            out[arm]["saved"] = bridge.get("/health").json()["map"]
    f, r = out["fresh"]["final"], out["reload"]["final"]
    out["D3-c"] = ("MET" if out["reload"]["reload"].get("result") == "matched" and f.get("usable")
                   and r.get("usable") and abs(r["position_error_m"] - f["position_error_m"]) <= 0.02
                   else "FAILED")
    return out


def main(which):
    os.makedirs(OUT, exist_ok=True)
    de.heavy_lock()
    de._kill_ports()
    de._new_run_dir()
    for name in which:
        if name.startswith("ab:"):
            # ab:<house>:<maps dir>: D3-b's three sessions again on that saved map
            # (the map's agreement was measured in the first run; not re-judged).
            _, house, maps = name.split(":", 2)
            bridge = httpx.Client(base_url=f"http://127.0.0.1:{de.BRIDGE}", timeout=30)
            saved_id = json.load(open(os.path.join(maps, "current.json")))["map_id"]
            sessions = [session(house, maps, bridge, None, saved_id)[0] for _ in range(3)]
            res = {"house": house, "sessions": sessions,
                   "D3-b": "MET" if all(x.get("criteria") and all(x["criteria"].values())
                                        and x["at_rest_s_after_verdict"] <= WITHIN_S
                                        for x in sessions) else "FAILED"}
            name = f"{house}-ab"
        elif name.startswith("e:"):
            # e:<house>:<maps dir of an earlier run>: D3-e again on its saved map.
            _, house, maps = name.split(":", 2)
            bridge = httpx.Client(base_url=f"http://127.0.0.1:{de.BRIDGE}", timeout=30)
            robot, _ = servers(house)
            w = container(maps, bridge)
            saved_map, saved_id = first_map(bridge), w["health"]["map"]["map_id"]
            assert w["health"]["slam"]["reload"]["result"] == "matched"
            off, trials = off_dock(house, maps, bridge, saved_map, saved_id)
            res = {"house": house, "off_dock_drive": off, "off_dock": trials,
                   "D3-e": "MET" if off["from_dock_m"] >= 1.0 and off["from_dock_deg"] >= 30
                   and all(all(t["criteria"].values()) for t in trials) else "FAILED"}
            name = f"{house}-e"
        else:
            res = lap_runs() if name == "lap" else house_runs(name)
        with open(os.path.join(OUT, f"{name}.json"), "w") as f:
            json.dump(res, f, indent=1)
        print(name, json.dumps(res.get("verdicts") or res.get("D3-c") or res.get("D3-e")
                               or res.get("D3-b")), flush=True)
    subprocess.run(["docker", "logs", de.CONTAINER], stdout=open(f"{de.RUN_DIR}/ros-last.log", "w"),
                   stderr=subprocess.STDOUT)
    subprocess.run(["docker", "rm", "-f", de.CONTAINER], capture_output=True)
    de._kill_ports()


if __name__ == "__main__":
    main(sys.argv[1:] or ["scaled_house", "home_first_floor", "lap"])
