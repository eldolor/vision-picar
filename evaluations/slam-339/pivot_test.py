"""
python -m evaluations.slam-339.pivot_test   (run as a file: python evaluations/slam-339/pivot_test.py [rates] [--slam yaml])

PLAN-ros-alignment.md 3.39: does a PIVOT alone put heading error into SLAM?
Run 2 of 2026-10-05 picked up 3.3 degrees in ~3 s while turning in the foyer
(odometry exact) and carried it 9 m east. This isolates that: a fresh stack
in the furnished home, the robot spun in place at the start pose through the
bridge's /cmd_vel at a fixed rate -- half turns, alternating direction -- and
SLAM's heading error (/world/error, against the sim's truth) read AT REST
after each. Nothing else moves the robot.

rates: comma-separated rad/s (default 1.0,0.5). Each rate gets a FRESH stack,
so one rate's error cannot carry into the next.
"""

import json
import math
import os
import sys
import time

import httpx

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from tests import demo_explore as dx  # noqa: E402

HOUSE = "home_first_floor"
HALF_TURNS = 8


def spin(bridge, rate, angle):
    """One pivot of `angle` rad at `rate` rad/s (sign = direction), 20 Hz."""
    end = time.time() + abs(angle) / rate
    while time.time() < end:
        bridge.post("/cmd_vel", json={"driver": "brain", "linear_m_s": 0.0,
                                      "angular_rad_s": math.copysign(rate, angle)})
        time.sleep(0.05)
    for _ in range(5):
        bridge.post("/cmd_vel", json={"driver": "brain", "linear_m_s": 0.0, "angular_rad_s": 0.0})
        time.sleep(0.05)


def one_rate(rate, slam_yaml=None):
    robot = dx.stack(HOUSE, slam_yaml=slam_yaml)
    bridge = httpx.Client(base_url=f"http://127.0.0.1:{dx.BRIDGE}", timeout=10)
    time.sleep(5)
    rows = []
    e = robot.get("/world/error").json()
    rows.append((0, e["heading_error_deg"], e["position_error_m"], e["truth"]["heading_deg"]))
    for k in range(HALF_TURNS):
        spin(bridge, rate, math.pi if k % 2 == 0 else -math.pi)
        time.sleep(2.0)                     # at rest: not latency
        e = robot.get("/world/error").json()
        rows.append((k + 1, e["heading_error_deg"], e["position_error_m"], e["truth"]["heading_deg"]))
    heads = [abs(r[1]) for r in rows]
    return {"rate_rad_s": rate, "slam": slam_yaml or "image",
            "max_abs_heading_err_deg": round(max(heads), 2),
            "final_heading_err_deg": rows[-1][1],
            "max_pos_err_m": round(max(r[2] for r in rows), 3),
            "rows": rows}


def main():
    args = sys.argv[1:]
    rates = [float(x) for x in (args[0] if args and not args[0].startswith("--")
                                else "1.0,0.5").split(",")]
    opt = dict(zip(args[1::2], args[2::2])) if args and not args[0].startswith("--") \
        else dict(zip(args[::2], args[1::2]))
    for rate in rates:
        print(json.dumps(one_rate(rate, opt.get("--slam"))), flush=True)
    dx._kill_ports()


if __name__ == "__main__":
    main()
