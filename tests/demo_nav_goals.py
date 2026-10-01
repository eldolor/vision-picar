"""
tests/demo_nav_goals.py -- R6's instrument (PLAN-ros-alignment.md 3.15).

Sends nav2 goals through the robot server (`POST /world/goal`, house frame)
and watches each one by GROUND TRUTH at 5 Hz: the outcome, where the robot
truly ended, and the closest its centre came to any wall or object surface --
the only way to see a side-swipe, since the sim's own collision check is one
ray straight ahead. Needs the full stack (ROBOT_DRIVE=ros, WORLD_MODE=ros,
a fresh container).

It drives R5's lap FIRST (tests/demo_slam_lap.py), because nav2 cannot plan
into a room SLAM has never seen: the first R6 run sent five of six goals
"off the global costmap". Mapping, then navigating, is the ordinary order.

    python -m tests.demo_nav_goals
"""

import json
import math
import os
import time

import httpx

from sim.maps import build_world

CELL = 0.30
HALF_WIDTH_M = 0.1155        # the chassis, picar_description: the UGV Rover's 0.231 m
TERMINAL = {"succeeded", "aborted", "canceled", "rejected"}

# Six places per house, in order, each reached from wherever the last one
# ended. Metres in the house frame: a cell centre is (col + 0.5) * 0.30.
STARTER_GOALS = [
    ("big room, through the start room's door", 2.40, 0.75),
    ("big room, far side",                      3.00, 1.05),
    ("lower room, middle",                      1.80, 2.25),
    ("lower room, west",                        0.60, 2.40),
    ("by the kitchen door",                     2.25, 2.25),
    ("back to the start",                       0.75, 0.75),
]
SCALED_GOALS = [
    ("hallway, north",                    4.35, 0.75),
    ("kitchen, through its door",         6.45, 1.35),
    ("hallway, south",                    4.35, 3.45),
    ("bedroom, through its door",         1.95, 3.15),
    ("study, across the hallway",         6.45, 3.15),
    ("back to the start",                 1.95, 1.35),
]

def _home_m(x_ft, y_ft):
    """A spot in the user's own house (sim/maps/home_first_floor.py), from
    feet on the appraisal sketch to metres in the house frame."""
    from sim.maps.home_first_floor import _cells
    return round(_cells(x_ft) * 0.30, 3), round(_cells(y_ft) * 0.30, 3)


# A tour of the user's house from the foyer, ordered so each goal lies in
# space SLAM has already seen on the way -- no scripted mapping lap.
HOME_GOALS = [(name, *_home_m(x, y)) for name, x, y in [
    ("family room",             20.0, 18.5),
    ("kitchen, by the island",  45.0, 13.5),
    ("garage hall",             41.7, 22.5),
    ("laundry",                 44.8, 22.0),
    ("garage, beside the car",  45.0, 36.0),
    ("dining room",             31.0, 30.0),
    ("living room",              6.0, 36.5),
    ("den",                      7.5, 12.5),
    ("back to the foyer",       16.0, 36.0),
]]

HOUSE = os.environ.get("SIM_MAP") or "starter_house"
GOALS = {"scaled_house": SCALED_GOALS, "home_first_floor": HOME_GOALS}.get(HOUSE, STARTER_GOALS)
# A goal inside a wall, per house (criterion 3).
UNREACHABLE = (0.15, 0.15) if HOUSE in ("scaled_house", "home_first_floor") else (0.75, 0.15)
# The mapping lap, per house: D-pad verbs through ROS so SLAM has seen every
# room before nav2 is asked to plan into it.
F, L90, R90 = ("FORWARD", {}), ("LEFT", {"angle": 90}), ("RIGHT", {"angle": 90})
SCALED_LAP = (
    [F] * 8 +            # (6.5, 4.5) E -> hallway (14.5, 4.5)
    [F] * 6 +            # -> kitchen (20.5, 4.5)
    [R90, R90] +         # face W
    [F] * 6 +            # -> hallway (14.5, 4.5)
    [L90] +              # face S
    [F] * 6 +            # -> hallway south (14.5, 10.5)
    [R90] +              # face W
    [F] * 6 +            # -> bedroom (8.5, 10.5)
    [R90, R90] +         # face E
    [F] * 10 +           # -> study (18.5, 10.5)
    [R90, R90] +         # face W
    [F] * 4 +            # -> hallway (14.5, 10.5)
    [R90] +              # face N
    [F] * 6 +            # -> (14.5, 4.5)
    [L90] +              # face W
    [F] * 8 +            # -> the start (6.5, 4.5)
    [R90, R90])          # face E again


def client():
    s = os.environ.get("APP_SHARED_SECRET") or os.environ.get("LOCAL_SECRET", "")
    return httpx.Client(base_url=os.environ.get("PICAR_ROBOT_URL", "http://127.0.0.1:8000"),
                        headers={"x-app-secret": s}, timeout=30)


def bridge():
    s = os.environ.get("APP_SHARED_SECRET") or os.environ.get("LOCAL_SECRET", "")
    return httpx.Client(base_url=os.environ.get("PICAR_BRIDGE_URL", "http://127.0.0.1:8090"),
                        headers={"x-app-secret": s}, timeout=30)


def _solids():
    g = build_world(os.environ.get("SIM_MAP") or "starter_house")
    solid = {(x, y) for y, row in enumerate(g.layout) for x, c in enumerate(row) if c == "#"}
    return solid | set(g.solid_cells)


SOLID = _solids()


def movers_now(robot):
    """3.30: the cells movers stand in right now, off the server's own
    ground truth (`GET /sim/objects`); empty when there are none, or on a
    server that predates the route."""
    r = robot.get("/sim/objects")
    if r.status_code != 200:
        return set()
    return {(o["x"], o["y"]) for o in r.json()["objects"] if o["mover"]}


def clearance_m(x, y, movers=(), static=True):
    """Centre to the nearest solid surface -- walls and furniture, plus,
    since 3.30, wherever a mover stands at this sample (`static=False`: the
    movers alone) -- in metres."""
    best = 9.9
    for cx, cy in (SOLID if static else set()) | set(movers):
        x0, y0 = cx * CELL, cy * CELL
        dx = max(x0 - x, 0.0, x - (x0 + CELL))
        dy = max(y0 - y, 0.0, y - (y0 + CELL))
        best = min(best, math.hypot(dx, dy))
    return best


def run_goal(robot, x, y, timeout_s=120.0):
    r = robot.post("/world/goal", json={"x_m": x, "y_m": y}).json()
    if not r.get("accepted"):
        return {"state": "not_sent", "reply": r}
    t0, state, min_clear, path_len = time.time(), "pending", 9.9, 0
    min_static, min_mover = 9.9, 9.9
    while time.time() - t0 < timeout_s:
        truth = robot.get("/world/truth").json()
        movers = movers_now(robot)
        min_static = min(min_static, clearance_m(truth["x_m"], truth["y_m"]))
        if movers:
            min_mover = min(min_mover, clearance_m(truth["x_m"], truth["y_m"], movers,
                                                   static=False))
        min_clear = min(min_static, min_mover)
        g = robot.get("/world/goal").json()
        state = (g.get("goal") or {}).get("state", "none")
        path_len = max(path_len, len(g.get("plan") or []))
        if state in TERMINAL:
            break
        time.sleep(0.2)
    time.sleep(0.5)
    truth = robot.get("/world/truth").json()
    wheels = robot.get("/wheels").json()
    return {"state": state, "seconds": round(time.time() - t0, 1),
            "end_error_m": round(math.dist((truth["x_m"], truth["y_m"]), (x, y)), 3),
            "min_clearance_m": round(min_clear, 3),
            "min_clearance_static_m": round(min_static, 3),
            "min_clearance_mover_m": round(min_mover, 3) if min_mover < 9.9 else None,
            "plan_points": path_len,
            "wheels_stopped": wheels["left"]["velocity_rad_s"] == 0 == wheels["right"]["velocity_rad_s"]}


def map_first(robot):
    from tests.demo_slam_lap import LAP
    if HOUSE == "home_first_floor":
        return                       # the tour maps as it goes
    for action, kw in (SCALED_LAP if HOUSE == "scaled_house" else LAP):
        # 3.30: a person crossing can stop a FORWARD short. The lap is a
        # script, so let them pass and send it again rather than run the rest
        # of the lap from the wrong place.
        for _attempt in range(4):
            r = robot.post("/action", json={"action": action, **kw},
                           headers={"x-driver": "twin-dpad"}).json()
            res = r.get("result") or {}
            short = (action == "FORWARD" and res.get("requested")
                     and res.get("moved", 1.0) < 0.5 * res["requested"])
            if r.get("executed") and not short:
                break
            time.sleep(1.5)
    time.sleep(1.5)                      # the D-pad's authority lapses


def unreachable(robot):
    """Criterion 3: a goal inside the start room's north wall."""
    return run_goal(robot, *UNREACHABLE, timeout_s=90.0)


def preempt(robot, br):
    """Criterion 4: a D-pad tap mid-goal cancels the goal within 1 s of the
    tap being SENT (a verb then takes its own ~1.5 s to finish, which is why
    the timing is taken from a poller started before the tap, not after)."""
    import threading
    robot.post("/world/goal", json={"x_m": GOALS[0][1], "y_m": GOALS[0][2]})
    deadline = time.time() + 20
    while time.time() < deadline and (robot.get("/world/goal").json().get("goal") or {}).get("state") != "active":
        time.sleep(0.1)
    time.sleep(1.5)
    seen = {}
    poller_client = client()

    def poll():
        while time.time() - t_tap < 6 and "canceled" not in seen:
            st = (poller_client.get("/world/goal").json().get("goal") or {}).get("state")
            seen.setdefault(st, round(time.time() - t_tap, 3))
            time.sleep(0.03)
    t_tap = time.time()
    th = threading.Thread(target=poll)
    th.start()
    tap = robot.post("/action", json={"action": "LEFT", "angle": 15},
                     headers={"x-driver": "twin-dpad"}).json()
    th.join()
    return {"tap_executed": tap.get("executed"),
            "seconds_from_tap_sent_to_canceled": seen.get("canceled"), "states_seen": seen}


if __name__ == "__main__":
    robot, br = client(), bridge()
    map_first(robot)
    br.post("/nav/stats/reset", json={})
    refusals_before = dict(robot.get("/health").json().get("refusal_counts") or {})
    print("refusals during the mapping lap", refusals_before)
    rows = []
    for name, x, y in GOALS:
        res = run_goal(robot, x, y)
        rows.append({"goal": name, **res})
        print(json.dumps(rows[-1]))
    stats = br.get("/nav/stats").json()
    print("nav stats", json.dumps(stats))
    print("unreachable", json.dumps(unreachable(robot)))
    print("preempt", json.dumps(preempt(robot, br)))
    print("robot/safety.py refusals by reason", robot.get("/health").json().get("refusal_counts"))
