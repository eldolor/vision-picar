"""
ROS_IMAGE=vision-picar-ros:skew python evaluations/slam-340/skew_test.py [rate,rate ...]

PLAN-ros-alignment.md 3.40 step 1: the EFFECTIVE scan/odometry skew. Needs
the instrumented bridge (GET /skew: per scan, its capture time `stamp_unix`
and the odom yaw it was filed with) -- built from a scratch copy, never the
shipped image.

The robot turns in place at the start pose through the bridge's /cmd_vel,
half turns alternating direction; /world/truth is sampled as fast as the
robot server answers. For every scan: the heading it is FILED at (odom yaw
at its ROS stamp) minus the heading it was TAKEN at (truth interpolated at
its capture time). Odometry is exact in the sim, so at rest that is ~0; while
turning it is rate x skew. The slope of mismatch on rate IS the skew.
"""

import bisect
import json
import math
import os
import sys
import threading
import time

import httpx

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from tests import demo_explore as dx  # noqa: E402

HOUSE = "home_first_floor"


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def spin(bridge, rate, angle):
    end = time.time() + abs(angle) / rate
    while time.time() < end:
        bridge.post("/cmd_vel", json={"driver": "brain", "linear_m_s": 0.0,
                                      "angular_rad_s": math.copysign(rate, angle)})
        time.sleep(0.05)
    for _ in range(5):
        bridge.post("/cmd_vel", json={"driver": "brain", "linear_m_s": 0.0, "angular_rad_s": 0.0})
        time.sleep(0.05)


def main():
    rates = [float(x) for x in (sys.argv[1] if len(sys.argv) > 1 else "1.0,0.5").split(",")]
    robot = dx.stack(HOUSE)
    bridge = httpx.Client(base_url=f"http://127.0.0.1:{dx.BRIDGE}", timeout=10)
    time.sleep(5)
    truth, stop = [], threading.Event()

    def poll():
        c = httpx.Client(base_url=f"http://127.0.0.1:{dx.ROBOT}", timeout=5)
        while not stop.is_set():
            try:
                t0 = time.time()
                h = c.get("/world/truth").json()["heading_deg"]
                truth.append(((t0 + time.time()) / 2, math.radians(h)))
            except Exception:  # noqa: BLE001
                pass
    th = threading.Thread(target=poll, daemon=True)
    th.start()
    time.sleep(2)
    bridge.get("/skew")                                   # drop start-up scans
    for rate in rates:
        for k in range(6):
            spin(bridge, rate, math.pi if k % 2 == 0 else -math.pi)
            time.sleep(2.0)
    time.sleep(1)
    log = bridge.get("/skew").json()["log"]
    stop.set()
    th.join()
    dx._kill_ports()

    ts = [t for t, _ in truth]
    # unwrap truth so interpolation across +/-pi is sane
    hs, prev = [], None
    for _, h in truth:
        if prev is not None:
            h = prev + wrap(h - prev)
        hs.append(h)
        prev = h

    def at(t):
        i = bisect.bisect_left(ts, t)
        if i <= 0 or i >= len(ts):
            return None, None
        f = (t - ts[i - 1]) / (ts[i] - ts[i - 1])
        h = hs[i - 1] + f * (hs[i] - hs[i - 1])
        j0, j1 = max(0, i - 3), min(len(ts) - 1, i + 3)
        rate = (hs[j1] - hs[j0]) / (ts[j1] - ts[j0])
        return h, rate
    rows = []
    for cap, yaw, ros_stamp, recv in log:
        if cap is None:
            continue
        h, r = at(cap)
        if h is not None:
            rows.append((cap, yaw, h, r, ros_stamp, recv))
    # Odometry is exact and CCW; the truth compass is clockwise. Remove the
    # start-anchor offset with the at-rest median, and take the turn rate over
    # NEIGHBOURING SCANS (~0.4 s): the sim's truth moves in 50 ms steps, so a
    # rate over a few ms of it reads zero mid-turn (the first reading's -5 ms).
    hs, prev = [], None
    for r in rows:
        h = r[2] if prev is None else prev + wrap(r[2] - prev)
        hs.append(h)
        prev = h
    m = [wrap((r[1] - rows[0][1]) + (h - hs[0])) for r, h in zip(rows, hs)]
    pts = []
    for i in range(2, len(rows) - 2):
        dt = rows[i + 2][0] - rows[i - 2][0]
        if 0 < dt <= 0.6:
            pts.append((-(hs[i + 2] - hs[i - 2]) / dt, m[i]))
    rest_m = sorted(x for r, x in pts if abs(r) < 0.02)
    off = rest_m[len(rest_m) // 2]
    pts = [(r, x - off) for r, x in pts]
    turn = [(r, x) for r, x in pts if abs(r) > 0.3]
    rest = sorted(abs(x) for r, x in pts if abs(r) < 0.02)
    sxx = sum(r * r for r, _ in turn)
    slope = sum(r * x for r, x in turn) / sxx if sxx else float("nan")
    lag = sorted(math.degrees(x) * (1 if r > 0 else -1) for r, x in turn if 0.7 <= abs(r) < 1.3)
    raw = sorted(row[4] - row[0] for row in rows)          # ROS stamp - capture (two clocks)
    out = {"scans": len(rows), "turning_scans": len(turn),
           "effective_skew_ms": round(slope * 1000, 1),
           "rest_mismatch_deg_p50_p95": [round(math.degrees(rest[len(rest) // 2]), 3),
                                         round(math.degrees(rest[int(0.95 * len(rest))]), 3)],
           "at_1rad_s_signed_deg_p10_p50_p90": [round(lag[k], 2) for k in
                                                (len(lag) // 10, len(lag) // 2, 9 * len(lag) // 10)] if lag else None,
           "share_of_1rad_s_scans_off_by_over_1deg": round(sum(abs(x) > 1 for x in lag) / max(len(lag), 1), 3),
           "raw_stamp_minus_capture_ms_p5_p50_p95": [round(raw[int(q * (len(raw) - 1))] * 1000, 1)
                                                     for q in (0.05, 0.5, 0.95)],
           "truth_samples": len(truth)}
    print(json.dumps(out))
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.environ.get("SKEW_OUT", "skew-test.json")), "w") as f:
        json.dump({**out, "rows": rows}, f)


if __name__ == "__main__":
    main()
