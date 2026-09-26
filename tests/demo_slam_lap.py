"""
tests/demo_slam_lap.py -- R5's instrument (PLAN-ros-alignment.md 3.14).

Drives a fixed lap of D-pad verbs through the ROS chain (start room ->
hallway room -> kitchen door -> back), samples /world/error at 1 Hz AND at
rest after every verb, then scores SLAM's map against the house's true walls
and objects. Needs the full stack: robot server with ROBOT_DRIVE=ros and
WORLD_MODE=ros (and SIM_ODOM_DRIFT="1.0,1.03" for drift), and a FRESH
container -- start it after the robot server, never before.

    python -m tests.demo_slam_lap

**Read the at-rest numbers.** The 1 Hz samples taken while moving compare a
SLAM pose ~150 ms old with the live truth, and at 1.2 rad/s that alone is
~11 degrees: exact odometry reads 6-8 cm "wrong" in motion and 0.0 at rest.
"""
import os, sys, time, json, math, threading
import httpx
from sim.maps.starter_house import build_starter_world

CELL = 0.30
LAP = ([("FORWARD", {})] * 3 + [("RIGHT", {"angle": 90})] + [("FORWARD", {})] * 5 +
       [("LEFT", {"angle": 90})] + [("FORWARD", {})] * 2 + [("RIGHT", {"angle": 90})] * 2 +
       [("FORWARD", {})] * 2 + [("RIGHT", {"angle": 90})] + [("FORWARD", {})] * 5 +
       [("LEFT", {"angle": 90})] + [("FORWARD", {})] * 3 + [("RIGHT", {"angle": 90})] * 2)

def client():
    s = os.environ.get("APP_SHARED_SECRET") or os.environ.get("LOCAL_SECRET", "")
    return httpx.Client(base_url=os.environ.get("PICAR_ROBOT_URL", "http://127.0.0.1:8000"),
                        headers={"x-app-secret": s}, timeout=60)

def run_lap(R):
    samples, stop = [], threading.Event()
    def sampler():
        while not stop.is_set():
            e = R.get("/world/error").json()
            if e.get("usable"):
                samples.append((time.time(), e["position_error_m"], e["heading_error_deg"],
                                e["odom_position_error_m"], e["odom_heading_error_deg"]))
            stop.wait(1.0)
    th = threading.Thread(target=sampler); th.start()
    blocked = 0
    rest = []
    for a, kw in LAP:
        r = R.post("/action", json={"action": a, **kw}, headers={"x-driver": "twin-dpad"}).json()
        blocked += not r.get("executed")
        time.sleep(float(os.environ.get("SETTLE_S", "0.4")))  # at rest
        e = R.get("/world/error").json()
        rest.append((e["position_error_m"], e["heading_error_deg"],
                     e["odom_position_error_m"], e["odom_heading_error_deg"]))
    run_lap.rest = rest
    time.sleep(2.0)
    stop.set(); th.join()
    return samples, blocked

def score_map(R):
    m = R.get("/world/map").json()
    g = build_starter_world()
    solid = set()
    for yy, row in enumerate(g.layout):
        for xx, c in enumerate(row):
            if c == "#": solid.add((xx, yy))
    solid |= set(g.solid_cells)
    def dist_to_solid(px, py):
        best = 9.9
        for (cx, cy) in solid:
            x0, y0 = cx * CELL, cy * CELL
            dx = max(x0 - px, 0, px - (x0 + CELL)); dy = max(y0 - py, 0, py - (y0 + CELL))
            best = min(best, math.hypot(dx, dy))
        return best
    def inside_solid(px, py, margin=0.05):
        for (cx, cy) in solid:
            x0, y0 = cx * CELL, cy * CELL
            if x0 + margin < px < x0 + CELL - margin and y0 + margin < py < y0 + CELL - margin:
                return True
        return False
    occ = free = occ_ok = free_bad = 0
    res, w = m["resolution_m"], m["width"]
    for k, v in enumerate(m["cells"]):
        px = m["origin_x_m"] + (k % w + 0.5) * res; py = m["origin_y_m"] + (k // w + 0.5) * res
        if v == 1:
            occ += 1; occ_ok += dist_to_solid(px, py) <= 0.10
        elif v == 0:
            free += 1; free_bad += inside_solid(px, py)
    return {"occupied": occ, "occupied_within_10cm": occ_ok, "precision": occ_ok / max(occ, 1),
            "free": free, "free_inside_solid": free_bad, "free_bad_rate": free_bad / max(free, 1)}

if __name__ == "__main__":
    R = client()
    samples, blocked = run_lap(R)
    pe = [s[1] for s in samples]; he = [abs(s[2]) for s in samples]
    oe = [s[3] for s in samples if s[3] is not None]
    jumps = [abs(b[1] - a[1]) for a, b in zip(samples, samples[1:])]
    rest = run_lap.rest
    out = {"at_rest": {"slam_max_pos_m": max(r[0] for r in rest),
                       "slam_max_head_deg": max(abs(r[1]) for r in rest),
                       "slam_final": rest[-1][:2],
                       "odom_max_pos_m": max(r[2] for r in rest),
                       "odom_final": rest[-1][2:]},
           "samples": len(samples), "blocked_moves": blocked,
           "slam_max_pos_m": max(pe), "slam_final_pos_m": pe[-1], "slam_max_head_deg": max(he),
           "slam_final_head_deg": he[-1], "odom_final_pos_m": oe[-1] if oe else None,
           "odom_max_pos_m": max(oe) if oe else None, "odom_final_head_deg": samples[-1][4],
           "largest_step_in_slam_error_m": max(jumps) if jumps else None,
           "truth_end": R.get("/world/truth").json(), "map": score_map(R),
           "drive": R.get("/health").json().get("drive")}
    print(json.dumps(out, indent=1, default=str))
    for (a, kw), r in zip(LAP, rest):
        print(f"{a:8}{kw.get('angle',''):>4}  slam {r[0]:.3f} m {r[1]:+6.2f} deg   odom {r[2]:.3f} m {r[3]:+6.2f}")
