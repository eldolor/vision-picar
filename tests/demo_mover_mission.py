"""
python -m tests.demo_mover_mission [policy] [target] [max_steps]

PLAN-ros-alignment.md 3.30, criterion 5 -- the deployed path. Starts ONE
mission through the brain's HTTP API on a live stack whose robot server runs
with `SIM_MOVERS` set, and samples ground truth at 5 Hz (`/world/truth` for
the pose, `/sim/objects` for where the movers stand) until it ends. Reports
the outcome, the closest the chassis came to a mover, and whether it was
ever inside one -- judged with `tests/footprint_sweep.py`'s geometry, not the
sim's own collision.

    PICAR_ROBOT_URL=http://127.0.0.1:8000  PICAR_BRAIN_URL=http://127.0.0.1:8001
"""

import json
import math
import os
import sys
import time

import httpx

from tests.footprint_sweep import CELL_CM, chassis, gap, penetration, square

CELL_M = CELL_CM / 100.0


def _client(var, default):
    s = os.environ.get("APP_SHARED_SECRET") or os.environ.get("LOCAL_SECRET", "")
    return httpx.Client(base_url=os.environ.get(var, default),
                        headers={"x-app-secret": s}, timeout=30)


def main():
    policy = sys.argv[1] if len(sys.argv) > 1 else "frontier"
    target = sys.argv[2] if len(sys.argv) > 2 else "red backpack"
    max_steps = int(sys.argv[3]) if len(sys.argv) > 3 else 150
    robot = _client("PICAR_ROBOT_URL", "http://127.0.0.1:8000")
    brain = _client("PICAR_BRAIN_URL", "http://127.0.0.1:8001")
    if robot.get("/sim/objects").status_code != 200:
        sys.exit("the robot server has no /sim/objects -- not a sim, or predates 3.30")

    started = brain.post("/mission/start", json={
        "target_object": target, "policy": policy, "max_steps": max_steps})
    print("start", started.status_code, started.json() if started.status_code != 200 else "")
    t0 = time.time()
    samples, near_mover, inside = 0, math.inf, 0
    status = {}
    while time.time() - t0 < 900:
        truth = robot.get("/world/truth").json()
        movers = [(o["x"], o["y"]) for o in robot.get("/sim/objects").json()["objects"] if o["mover"]]
        x, y = truth["x_m"] / CELL_M, truth["y_m"] / CELL_M
        theta = math.radians(truth["heading_deg"]) - math.pi / 2
        A = chassis(x, y, theta)
        for cell in movers:
            B = square(*cell)
            near_mover = min(near_mover, gap(A, B) * CELL_CM)
            if penetration(A, B) > 0:
                inside += 1
        samples += 1
        status = brain.get("/mission/status").json()
        if not status.get("running") and status.get("outcome"):
            break
        time.sleep(0.2)
    print(json.dumps({
        "policy": policy, "target": target,
        "outcome": status.get("outcome"), "steps": status.get("step"),
        "seconds": round(time.time() - t0, 1), "samples": samples,
        "closest_to_a_mover_cm": round(near_mover, 1) if near_mover < math.inf else None,
        "samples_inside_a_mover": inside,
    }))


if __name__ == "__main__":
    main()
