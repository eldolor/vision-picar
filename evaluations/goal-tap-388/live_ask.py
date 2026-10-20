"""
python -m evaluations.goal-tap-388.live_ask [house]   (run as a file: see below)

3.88 A's live check: one explore mission in ask mode through the brain's
HTTP API on the full ROS stack (`tests/demo_explore.stack()`), a script in
the person's place. Each open choice is watched for ASK_HOLD_S with the
robot's ground truth sampled (it must not move), then answered through
`POST /mission/explore/pick` -- alternately the brain's own pick and the
last candidate offered, so person picks that differ from the rule are sent --
and one choice is left unanswered to time out (`rule_timeout`). Judged on:
the outcome, chassis contacts (demo_explore's measure), motion during
pauses, a stale pick refused 409, and the mission's choice file matching the
picks made. One JSON line.

    python evaluations/goal-tap-388/live_ask.py home_first_floor
"""

import json
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import httpx  # noqa: E402

import tests.demo_explore as demo  # noqa: E402
from brain.choice_log import read_choices  # noqa: E402

ASK_HOLD_S = 3.0
TIMEOUT_CHOICE = 3          # this choice is left to the 60 s timeout


def picker(brain, robot, out, stop):
    answered = set()
    while not stop.is_set():
        try:
            st = brain.get("/mission/status").json()
        except Exception:  # noqa: BLE001 -- the brain is busy; try again
            time.sleep(0.5)
            continue
        ask = (st.get("explore") or {}).get("ask")
        if not st.get("running"):
            if st.get("outcome") not in (None, "idle", "running"):
                return
        if ask and ask["choice"] not in answered:
            answered.add(ask["choice"])
            before = robot.get("/world/truth").json()
            moved, t0 = 0.0, time.time()
            hold = ASK_HOLD_S if ask["choice"] != TIMEOUT_CHOICE else 62.0
            while time.time() - t0 < hold:
                # Only while this choice is still open: the 2026-10-20 run
                # watched choice 3 for 62 s, past its 60 s pause, and counted
                # the rule's goal driving off (39.98 cm).
                cur = ((brain.get("/mission/status").json().get("explore") or {}).get("ask")
                       or {})
                if cur.get("choice") != ask["choice"]:
                    break
                t = robot.get("/world/truth").json()
                moved = max(moved, math.hypot(t["x_m"] - before["x_m"], t["y_m"] - before["y_m"]))
                time.sleep(0.25)
            row = {"choice": ask["choice"], "n": len(ask["candidates"]),
                   "moved_cm": round(moved * 100, 2)}
            if ask["choice"] != TIMEOUT_CHOICE:
                cid = ask["default"] if ask["choice"] % 2 else ask["candidates"][-1]["id"]
                r = brain.post("/mission/explore/pick", json={"choice": ask["choice"],
                                                              "candidate": cid})
                row.update(pick=cid, status=r.status_code)
                if ask["choice"] == 2:
                    # A stale pick: the previous choice's id.
                    s = brain.post("/mission/explore/pick",
                                   json={"choice": 1, "candidate": "1.0"})
                    row["stale_status"] = s.status_code
            else:
                late = brain.post("/mission/explore/pick", json={"choice": ask["choice"],
                                                                 "candidate": ask["default"]})
                row.update(pick=None, late_status=late.status_code)
            out.append(row)
        time.sleep(0.3)


def main():
    house = sys.argv[1] if len(sys.argv) > 1 else "home_first_floor"
    choice_dir = os.path.join(demo.LOGDIR, "choices")
    os.environ["EXPLORE_CHOICE_DIR"] = choice_dir
    robot = demo.stack(house)
    brain = httpx.Client(base_url=f"http://127.0.0.1:{demo.BRAIN}", timeout=60)
    # run_mission starts it; ask mode by the start's own field, so it is
    # patched in for this call only.
    real_post = httpx.Client.post

    def post(self, url, *a, **kw):
        if url == "/mission/start" and "json" in kw:
            kw["json"] = {**kw["json"], "explore_ask": True}
        return real_post(self, url, *a, **kw)
    httpx.Client.post = post
    rows, stop = [], threading.Event()
    th = threading.Thread(target=picker, args=(brain, robot, rows, stop), daemon=True)
    th.start()
    try:
        result = demo.run_mission(robot, house)
    finally:
        httpx.Client.post = real_post
        stop.set()
        th.join(5)
    files = sorted(os.listdir(choice_dir)) if os.path.isdir(choice_dir) else []
    recs = read_choices(os.path.join(choice_dir, files[-1])) if files else []
    status = json.load(open(os.path.join(demo.RUN_DIR, result["status_file"])))
    print(json.dumps({
        "house": house, **{k: result.get(k) for k in ("outcome", "seconds", "travel_m",
                                                       "min_gap_cm", "contact_samples",
                                                       "inside_samples")},
        "paused_s": (status.get("budget") or {}).get("paused_s"),
        "elapsed_s": (status.get("budget") or {}).get("elapsed_s"),
        "picks": rows,
        "max_moved_cm_in_pause": max([r["moved_cm"] for r in rows] or [None]),
        "records": len(recs), "by": [r["by"] for r in recs],
        "record_picks_match": [r["pick"] for r in recs if r["by"] == "person"]
        == [r["pick"] for r in rows if r.get("pick") and r.get("status") == 200][:sum(
            1 for r in recs if r["by"] == "person")],
        "choice_file": os.path.join(choice_dir, files[-1]) if files else None,
        "run_dir": demo.RUN_DIR,
    }))
    demo._kill_ports()
    os.system(f"docker rm -f {demo.CONTAINER} >/dev/null 2>&1")


if __name__ == "__main__":
    main()
