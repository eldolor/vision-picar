"""3.89 criteria 1-2: how long a creep verb (speed 5) toward a wall takes on
the live stack, under `drive: ros` and under the sim's own drive
(docs/plans/ros-alignment/3.89-escape-creep-timeout.md).

    eval "$(python tools/plan_section.py env)"
    ROS_IMAGE=vision-picar-ros:latest python evaluations/escape-creep-389/measure.py ros 20 > ros.jsonl
    python evaluations/escape-creep-389/measure.py direct 20 > direct.jsonl

Each trial turns by a seeded angle, drives FORWARD at speed 50 until the
safety layer refuses it (the robot then stands at the 20 cm bar, as the
3.85 escape leaves it), and creeps FORWARD at speed 5 to the 3 cm line;
then the same backwards. Every `/action` is timed here, on the caller's
side (an upper bound on the server's own time), with `/wheels` sampled at
20 Hz during each creep for the speed the wheels actually made. Ground truth
(the chassis' gap to every wall and object) is sampled after every verb.
One JSON line per verb; a summary line last."""
import json
import math
import os
import random
import re
import subprocess
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

import httpx                                   # noqa: E402

from brain.explore import CREEP_PIECES           # noqa: E402
from tests import demo_explore as demo         # noqa: E402

HOUSE = os.environ.get("SIM_MAP", "home_first_floor")
HDR = {"x-driver": "brain"}


def direct_stack(house):
    """The robot server alone, on the sim's own drive (no ROS)."""
    demo.heavy_lock()
    demo._kill_ports()
    run_dir = demo._new_run_dir()
    env = {k: v for k, v in os.environ.items()
           if k not in ("APP_SHARED_SECRET", "LOCAL_SECRET", "ROBOT_DRIVE", "WORLD_MODE")}
    env.update(SIM_MAP=house, PICAR_LOG_LEVEL="WARNING")
    subprocess.Popen([demo.PY, "-m", "uvicorn", "robot.server:app", "--port", str(demo.ROBOT),
                      "--host", "127.0.0.1", "--log-level", "warning"], cwd=ROOT, env=env,
                     stdout=open(f"{run_dir}/explore_robot.log", "w"), stderr=subprocess.STDOUT)
    robot = httpx.Client(base_url=f"http://127.0.0.1:{demo.ROBOT}", timeout=60)
    for _ in range(60):
        try:
            robot.get("/health")
            return robot
        except Exception:  # noqa: BLE001 -- still coming up
            time.sleep(0.5)
    raise RuntimeError("robot server did not come up")


def make_truth(house):
    world = demo.build_world(house)
    walls = {(x, y) for y, row in enumerate(world.layout) for x, c in enumerate(row)
             if c == demo.CELL_WALL}

    def truth(robot):
        t = robot.get("/world/truth").json()
        objs = {(o["x"], o["y"]) for o in robot.get("/sim/objects").json()["objects"]}
        x, y = t["x_m"] / 0.3, t["y_m"] / 0.3
        A = demo.chassis(x, y, math.radians(t["heading_deg"]) - math.pi / 2)
        gap, inside = math.inf, False
        for c in walls | objs:
            if abs(c[0] + 0.5 - x) > 1.6 or abs(c[1] + 0.5 - y) > 1.6:
                continue
            gap = min(gap, demo.gap(A, demo.square(*c)) * demo.CELL_CM)
            inside = inside or demo.penetration(A, demo.square(*c)) > 0
        return round(gap, 2), inside
    return truth


def act(robot, body):
    t0 = time.monotonic()
    try:
        r = robot.post("/action", json=body, headers=HDR, timeout=60)
        out = r.json()
    except Exception as e:  # noqa: BLE001
        out = {"error": repr(e)}
    return round(time.monotonic() - t0, 3), out


def sample_wheels(robot, stop, rows):
    c = httpx.Client(base_url=str(robot.base_url), timeout=2)
    while not stop.is_set():
        try:
            w = c.get("/wheels").json()
            rows.append((time.monotonic(), w["left"]["position_rad"], w["right"]["position_rad"],
                         w["wheel_radius_m"]))
        except Exception:  # noqa: BLE001
            pass
        time.sleep(0.05)


def creep(robot, action, truth, emit, trial, room=None, piece=1):
    rows, stop = [], threading.Event()
    th = threading.Thread(target=sample_wheels, args=(robot, stop, rows), daemon=True)
    th.start()
    secs, out = act(robot, {"action": action, "speed": 5})
    stop.set()
    th.join(2)
    moving = None
    if len(rows) > 2:
        # the time the wheels were turning, and the speed they made meanwhile
        d = [abs(((b[1] - a[1]) + (b[2] - a[2])) / 2 * a[3]) for a, b in zip(rows, rows[1:])]
        ts = [b[0] - a[0] for a, b in zip(rows, rows[1:])]
        on = [(dd, tt) for dd, tt in zip(d, ts) if dd / tt > 0.002]
        if on:
            moving = {"moving_s": round(sum(t for _, t in on), 2),
                      "moved_m": round(sum(dd for dd, _ in on), 4),
                      "speed_m_s": round(sum(dd for dd, _ in on) / sum(t for _, t in on), 4),
                      "first_move_s": round(next(b[0] for (a, b), dd in zip(zip(rows, rows[1:]), d)
                                                 if dd > 0.0002) - rows[0][0], 2)}
    g, inside = truth(robot)
    emit({"trial": trial, "verb": action, "creep": True, "piece": piece, "room_cm": room,
          "seconds": secs,
          "wheels": moving,
          "gap_cm": g, "inside": inside, "out": out})
    return secs, out


def room_cm(robot, action, refusal):
    """The room the creep starts with: the forward veto off `/depth`, or the
    rear clearance the refusal named (None when neither says)."""
    if action == "FORWARD":
        return (robot.get("/depth").json().get("path") or {}).get("veto_cm")
    m = re.search(r"rear clearance=([0-9.]+)cm", refusal.get("detail") or "")
    return float(m.group(1)) if m else None


def to_bar(robot, action, truth, emit, trial):
    """Drive at speed 50 until refused: the robot then stands at the bar.
    Returns the refusal, or None if it never came."""
    for _ in range(25):
        secs, out = act(robot, {"action": action, "speed": 50, "duration": 0.5})
        g, inside = truth(robot)
        emit({"trial": trial, "verb": action, "creep": False, "seconds": secs,
              "gap_cm": g, "inside": inside, "out": out})
        if not out.get("executed", True) or "error" in out:
            return out
    return None


def main():
    mode, n = sys.argv[1], int(sys.argv[2])
    robot = demo.stack(HOUSE) if mode == "ros" else direct_stack(HOUSE)
    truth = make_truth(HOUSE)
    rng = random.Random(389)

    def emit(row):
        print(json.dumps(row), flush=True)
    creeps = []
    try:
        for trial in range(n):
            angle = rng.choice([30, 45, 60, 90, 120])
            # off the last creep's wall first, or the pivot guard refuses the turn
            act(robot, {"action": "FORWARD", "speed": 50, "duration": 0.5})
            act(robot, {"action": rng.choice(["LEFT", "RIGHT"]), "angle": angle})
            for drive, crawl in (("FORWARD", "FORWARD"), ("REVERSE", "REVERSE")):
                refusal = to_bar(robot, drive, truth, emit, trial)
                if refusal is not None:
                    room = room_cm(robot, crawl, refusal)
                    moved, pieces = 0.0, []
                    for piece in range(1, CREEP_PIECES + 1):
                        # as the escape does since 3.89: on, the same way, after a time cut
                        secs, out = creep(robot, crawl, truth, emit, trial, room, piece)
                        moved += abs((out.get("result") or {}).get("moved_m") or 0.0)
                        pieces.append(secs)
                        if (out.get("result") or {}).get("stopped_short") != "timeout":
                            break
                    if (room is not None and room >= 15) or moved >= 0.12:
                        creeps.extend(pieces)      # started at the bar
    finally:
        emit({"summary": True, "mode": mode, "house": HOUSE, "creeps": len(creeps),
              "max_s": max(creeps or [0]), "over_8": sum(s > 8 for s in creeps),
              "over_10": sum(s > 10 for s in creeps), "run_dir": demo.RUN_DIR})
        demo._kill_ports()
        os.system(f"docker rm -f {demo.CONTAINER} >/dev/null 2>&1")


if __name__ == "__main__":
    main()
