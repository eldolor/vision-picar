"""3.90 A, criteria 1 and 3, measured on the Jetson beside the stack.

Polls the robot server's GET /frame at --poll-hz for --seconds and records
each frame's age at the reply (`time.time() - captured_at`; robot and
bench on one machine, one clock). Reads the camera reader thread's CPU time
from /proc (its id is /health's camera.reader_tid) and the wheel loop's
late ticks from /health, before and after. Optionally starts a mission on
the brain so the wheels move (late ticks are counted only while moving) and
stops it at the end.

    python evaluations/camera-stream-390/measure_a.py --robot http://127.0.0.1:8000 \\
        --pid <robot server pid> --brain http://127.0.0.1:8001/brain --seconds 600 \\
        --out evaluations/camera-stream-390/A-jetson.json

Bars (written before measuring, 3.90 criteria 1 and 3): frame age p90 <=
100 ms; 0 late wheel-loop ticks; reader CPU <= 10% of one core.
"""

import argparse
import json
import os
import statistics
import time

import httpx


def _cpu_s(pid: int, tid: int) -> float:
    with open(f"/proc/{pid}/task/{tid}/stat") as f:
        fields = f.read().rsplit(")", 1)[1].split()
    # utime and stime are fields 14 and 15 of stat; after the ")" split
    # the state is index 0, so they are 11 and 12.
    return (int(fields[11]) + int(fields[12])) / os.sysconf("SC_CLK_TCK")


def _pct(xs, q):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(q * len(xs)))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", required=True)
    ap.add_argument("--brain")
    ap.add_argument("--pid", type=int, required=True)
    ap.add_argument("--seconds", type=float, default=600)
    ap.add_argument("--poll-hz", type=float, default=5)
    ap.add_argument("--policy", default="explore")
    ap.add_argument("--target", default="backpack")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    secret = os.environ.get("APP_SHARED_SECRET", "")
    headers = {"x-app-secret": secret} if secret else {}
    robot = httpx.Client(base_url=args.robot, headers=headers, timeout=10)
    brain = httpx.Client(base_url=args.brain, headers=headers, timeout=30) if args.brain else None

    h0 = robot.get("/health").json()
    tid = h0["camera"]["reader_tid"]
    if brain is not None:
        r = brain.post("/mission/start", json={"policy": args.policy, "target_object": args.target})
        r.raise_for_status()
    cpu0, wall0 = _cpu_s(args.pid, tid), time.monotonic()
    ages, stale, missions_ended = [], 0, []
    period = 1.0 / args.poll_hz
    next_t = time.monotonic()
    end = wall0 + args.seconds
    while time.monotonic() < end:
        r = robot.get("/frame")
        now = time.time()
        if r.status_code == 503:
            stale += 1
        else:
            r.raise_for_status()
            ages.append(now - r.json()["captured_at"])
        if brain is not None and len(ages) % 50 == 0:
            st = brain.get("/mission/status").json()
            if not st.get("running"):
                # An explored house ends the mission; the wheels must keep
                # moving for the late-tick count, so start the next one.
                missions_ended.append({"after_s": round(time.monotonic() - wall0, 1),
                                       "outcome": st.get("outcome")})
                brain.post("/mission/start", json={"policy": args.policy,
                                                   "target_object": args.target})
        next_t += period
        time.sleep(max(0.0, next_t - time.monotonic()))
    cpu1, wall1 = _cpu_s(args.pid, tid), time.monotonic()
    h1 = robot.get("/health").json()
    if brain is not None:
        brain.post("/mission/stop")
    wl0, wl1 = h0["wheel_loop"], h1["wheel_loop"]
    result = {
        "seconds": round(wall1 - wall0, 1),
        "polls": len(ages) + stale,
        "stale_503": stale,
        "age_ms": {"p50": round(statistics.median(ages) * 1000, 1),
                   "p90": round(_pct(ages, 0.9) * 1000, 1),
                   "max": round(max(ages) * 1000, 1)},
        "reader_cpu_pct_of_one_core": round(100 * (cpu1 - cpu0) / (wall1 - wall0), 2),
        "wheel_loop": {"moving_ticks": wl1["moving_ticks"] - wl0["moving_ticks"],
                       "late_ticks": wl1["late_ticks"] - wl0["late_ticks"],
                       "max_dt_s": wl1["max_dt_s"]},
        "camera": h1["camera"],
        "mission_ended": missions_ended,
        "bars": {"age_p90_ms": 100, "late_ticks": 0, "reader_cpu_pct": 10},
    }
    result["met"] = {
        "criterion_1": result["age_ms"]["p90"] <= 100,
        "criterion_3_late": result["wheel_loop"]["late_ticks"] == 0,
        "criterion_3_cpu": result["reader_cpu_pct_of_one_core"] <= 10,
    }
    with open(args.out, "w") as f:
        json.dump(result, f, indent=2)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
