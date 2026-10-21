"""3.89 criterion 3's fallback: a scripted wedge in the dining room
(docs/plans/ros-alignment/3.89-escape-creep-timeout.md), for when no live
explore mission's escape reaches its creep.

    eval "$(python tools/plan_section.py env)"
    ROS_IMAGE=vision-picar-ros:latest python evaluations/escape-creep-389/scripted_wedge.py 8 > wedge.jsonl

On the full ROS stack (`tests/demo_explore.stack()`), nav2 drives the robot
to a spot beside the dining table (`/world/goal`). The robot turns toward
the table, and FORWARD at speed 50 goes until the safety layer refuses
(the 20 cm bar). Then the brain's own escape runs: an `ExploreAgent` on
`RemoteRobot` (the brain's HTTP client, `DEFAULT_TIMEOUT_S` 10 s) with
`RemoteNavigator`, its full escape queued (`_queue_escape`: REVERSE, CREEP,
OPEN) when REVERSE is refused too, and else its CREEP alone, forward, as
3.88 A run 2's second escape crept. One JSON line per trial: every queued
verb's wall time, the creep events, any exception (a timeout is one), and
the ground-truth gap."""
import json
import math
import os
import random
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from brain.explore import ExploreAgent           # noqa: E402
from brain.memory import MissionMemory           # noqa: E402
from control.remote_navigator import RemoteNavigator  # noqa: E402
from control.remote_robot import RemoteRobot     # noqa: E402
from control.remote_world import RemoteWorld     # noqa: E402
from measure import act, make_truth, room_cm, to_bar   # noqa: E402
from tests import demo_explore as demo           # noqa: E402

HOUSE = "home_first_floor"


def spots(world, rng, n):
    """Free cells 0.5-1.2 m from the dining table's centre (between its
    four legs' cells)."""
    legs = [c for c, name in world.objects.items() if name == "dining table"]
    cx = (min(c[0] for c in legs) + max(c[0] for c in legs) + 1) / 2
    cy = (min(c[1] for c in legs) + max(c[1] for c in legs) + 1) / 2
    free = [(x + 0.5, y + 0.5) for x, y in demo.free_cells(world)
            if 0.5 / 0.3 <= math.hypot(x + 0.5 - cx, y + 0.5 - cy) <= 1.2 / 0.3]
    rng.shuffle(free)
    return free[:n], (cx, cy)


def go(nav, x_m, y_m, timeout_s=90):
    nav.set_goal(x_m, y_m)
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        g = (nav.get_goal() or {}).get("goal") or {}
        if g.get("state") not in ("pending", "active"):
            return g.get("state")
        time.sleep(1)
    nav.cancel_goal()
    return "timeout"


def face(robot, x_m, y_m):
    t = robot.get("/world/pose").json()
    # compass heading, 0 = -y, 90 = +x (world/interface.py); RIGHT adds to it
    want = math.degrees(math.atan2(x_m - t["x_m"], -(y_m - t["y_m"])))
    d = (want - t["heading_deg"] + 180) % 360 - 180
    if abs(d) > 5:
        act(robot, {"action": "RIGHT" if d > 0 else "LEFT", "angle": int(round(abs(d)))})
    return want


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    rng = random.Random(3890)
    robot = demo.stack(HOUSE)
    truth = make_truth(HOUSE)
    url = str(robot.base_url)
    brain_robot = RemoteRobot(url)                 # DEFAULT_TIMEOUT_S, as the brain
    nav = RemoteNavigator(url)
    cells, (cx, cy) = spots(demo.build_world(HOUSE), rng, n * 3)
    done = 0
    try:
        for x, y in cells:
            if done >= n:
                break
            state = go(nav, x * 0.3, y * 0.3)
            if state != "succeeded":
                print(f"skip {x, y}: goal {state}", file=sys.stderr, flush=True)
                continue
            # toward the table, a little to one side: its legs and chairs
            want = face(robot, cx * 0.3 + rng.uniform(-0.4, 0.4), cy * 0.3 + rng.uniform(-0.4, 0.4))
            rows = []
            refusal = to_bar(robot, "FORWARD", truth, rows.append, done)
            if refusal is None or "error" in refusal:
                print(f"skip {x, y}: no refusal {refusal}", file=sys.stderr, flush=True)
                continue
            room = room_cm(robot, "FORWARD", refusal)
            if room is None or room < 15:
                print(f"skip {x, y}: room {room} ({refusal.get('detail')})", file=sys.stderr, flush=True)
                continue
            agent = ExploreAgent(brain_robot, MissionMemory(mission="m", target_object="x"),
                                 navigator=nav, world=RemoteWorld(url))
            rev_cm, _ = agent.safety.reverse_clearance()
            wedged = rev_cm is not None and rev_cm < 20.0
            if wedged:
                agent._queue_escape("scripted")
            else:
                agent._pending = [("CREEP", ["FORWARD"]), ("OPEN",)]
            verbs, error = [], None
            while agent._pending:
                head = agent._pending[0]
                t0 = time.monotonic()
                try:
                    out = agent._do_pending()
                except Exception as e:  # noqa: BLE001 -- a timeout is the failure we look for
                    error = repr(e)
                    verbs.append({"item": str(head), "seconds": round(time.monotonic() - t0, 3),
                                  "error": error})
                    break
                verbs.append({"item": str(head), "seconds": round(time.monotonic() - t0, 3),
                              "out": out[2] if isinstance(out[2], dict) else str(out[2])})
            gap, inside = truth(robot)
            print(json.dumps({"trial": done, "start": [round(x * 0.3, 2), round(y * 0.3, 2)],
                              "room_cm": room, "rear_cm": rev_cm, "table_bearing_deg": round(want, 1), "full_escape": wedged,
                              "verbs": verbs,
                              "creeps": [e for e in agent.events if e["event"] == "creep"],
                              "error": error, "gap_cm": gap, "inside": inside,
                              "approach_min_gap_cm": min([r["gap_cm"] for r in rows] or [None])}),
                  flush=True)
            done += 1
            act(robot, {"action": "REVERSE", "speed": 50, "duration": 0.5})   # off the table
    finally:
        demo._kill_ports()
        os.system(f"docker rm -f {demo.CONTAINER} >/dev/null 2>&1")


if __name__ == "__main__":
    main()
