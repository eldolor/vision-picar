"""
3.105 criteria 4-5, live: a goal sent the moment a reloaded container reads
`ready`, and an explore mission from such a start.

    eval "$(python tools/plan_section.py env)"
    python evaluations/nav2-gate-3105/live.py [image] [starts] [--no-mission]
    python evaluations/nav2-gate-3105/live.py --score <result.json>   (re-judge a record)

`image` defaults to this branch's ($ROS_IMAGE); give `dev`'s image for the
red side. Uses the 3.50 harness (map_run, container) as it is. Records to
`evaluations/nav2-gate-3105/<image tag>.json` after every start; the saved
map stays in the run directory (a sim floor plan, never committed).

Known limits (review): `ready_at` is `d3.container`'s, once `slam.state` is
`ready` and the start truth is in, not the first `ready` read; a refusal is
recorded by the robot server's reason (`ros_unavailable` for every bridge
503), so the hold is told from other refusals only by when it happens; the
image's contents are not checked against a commit.
"""
import importlib.util
import json
import os
import sys
import threading
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, ROOT)
HERE = os.path.dirname(os.path.abspath(__file__))
IMAGE = next((a for a in sys.argv[1:] if ":" in a), os.environ.get("ROS_IMAGE", ""))
# The 3.50 harness checks it runs on 3.50's image at import; the image is
# set on it below instead.
os.environ["ROS_IMAGE"] = "vision-picar-ros:plan-3.50"
_spec = importlib.util.spec_from_file_location(
    "d4_runs", os.path.join(ROOT, "evaluations", "map-350", "d4_runs.py"))
d4 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(d4)
d3, de = d4.d3, d4.de
d3.IMAGE = IMAGE

HOUSE = "scaled_house"
GOAL_S = 120.0
HOLD_BAR_S = 60.0                     # criterion 4: the 503 window, each start


def goal_at_ready(robot, x_m, y_m, ready_at):
    """Post at once and on every refusal; one accepted goal, followed to its end."""
    out = {"refusals": [], "first_post_after_ready_s": round(time.time() - ready_at, 2)}
    t0 = time.time()
    while time.time() - t0 < HOLD_BAR_S + 30:
        r = robot.post("/world/goal", json={"x_m": x_m, "y_m": y_m})
        body = r.json() if r.headers.get("content-type", "").startswith("application/json") else {}
        if r.status_code == 200 and body.get("accepted"):
            out["accepted_after_ready_s"] = round(time.time() - ready_at, 2)
            break
        out["refusals"].append({"t": round(time.time() - ready_at, 2), "status": r.status_code,
                                "reason": body.get("reason") or r.text[:160]})
        time.sleep(0.25)
    else:
        out["state"] = "never_accepted"
        return out
    t1 = time.time()
    state = "pending"
    while time.time() - t1 < GOAL_S:
        state = (robot.get("/world/goal").json().get("goal") or {}).get("state", "none")
        if state in ("succeeded", "aborted", "canceled", "rejected"):
            break
        time.sleep(0.2)
    else:
        robot.delete("/world/goal")
    out["state"], out["goal_s"] = state, round(time.time() - t1, 1)
    return out


class GoalWatch:
    """Every goal the robot server reports during a mission, by its start."""

    def __init__(self, robot):
        self.robot, self.seen, self._stop = robot, {}, threading.Event()

    def __enter__(self):
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()
        return self

    def __exit__(self, *exc):
        self._stop.set()
        self._t.join(5)

    def _run(self):
        import httpx
        with httpx.Client(base_url=str(self.robot.base_url), timeout=5) as c:
            self._loop(c)

    def _loop(self, c):
        while not self._stop.is_set():
            try:
                g = c.get("/world/goal").json().get("goal") or {}
                if g.get("since") is not None:
                    self.seen[g["since"]] = g.get("state")
            except Exception:  # noqa: BLE001 -- a missed sample, not a result
                pass
            time.sleep(0.2)


def score(out):
    """Criteria 4-5 from a record (review: an instant accept, a fresh-map
    start and a mission that never ended were misjudged)."""
    ss = out["starts"]
    c = {"c4_no_rejected": all(s.get("state") != "rejected" for s in ss),
         "c4_all_succeeded": all(s.get("state") == "succeeded" for s in ss),
         "c4_hold_under_60s": all(s.get("accepted_after_ready_s") is not None
                                  and s["accepted_after_ready_s"] < HOLD_BAR_S for s in ss),
         "c4_on_saved_map": all(s.get("reload") == "matched" for s in ss),
         "rejected_starts": sum(s.get("state") == "rejected" for s in ss)}
    if "mission" in out:
        m = out["mission"]
        c["c5"] = (m.get("reload") == "matched"
                   and m.get("outcome") in ("found", "exhausted", "out_of_budget")
                   and ((m.get("explore") or {}).get("goals_failed")) == 0
                   and m.get("rejected") == 0 and m.get("contact_samples") == 0)
    return c


def dump(out):
    with open(os.path.join(HERE, f"{IMAGE.split(':')[-1]}.json"), "w") as f:
        json.dump(out, f, indent=1, default=str)


def main(argv):
    if "--score" in argv:
        out = json.load(open(argv[argv.index("--score") + 1]))
        print(json.dumps(score(out)))
        return
    starts = next((int(a) for a in argv if a.isdigit()), 5)
    try:
        run(argv, starts)
    finally:
        d4.subprocess_cleanup()


def run(argv, starts):
    de.heavy_lock()
    de._kill_ports()
    de._new_run_dir()
    print(f"run dir {de.RUN_DIR}, image {IMAGE}", flush=True)
    maps = os.path.join(de.RUN_DIR, "maps")
    out = {"image": IMAGE, "house": HOUSE, "starts": []}
    out["map_run"], robot, _ = d4.map_run(HOUSE, maps)
    print(f"map saved: {out['map_run']['saved']}", flush=True)
    # A goal 1-2 m from the dock, in the D4-a view spot's corridor: reached
    # in every D4 run.
    room, door = d4.CLOSED_ROOM[HOUSE]
    gx, gy = d4.cell_m(d4.outside(HOUSE, door, room))
    out["goal"] = [gx, gy]
    for i in range(starts):
        robot, brain, bridge, rel = d4.reload(HOUSE, maps)
        ready_at = rel["watch"].get("ready_at") or time.time()
        res = {"reload": rel["reload"].get("result"), "states": rel["watch"].get("states")}
        try:
            res.update(goal_at_ready(robot, gx, gy, ready_at))
            res["nav2_at_end"] = d4.health(bridge).get("nav2")
        except Exception as e:  # noqa: BLE001 -- recorded; the next start goes on
            res["error"] = f"{type(e).__name__}: {e}"
        out["starts"].append(res)
        dump(out)
        print(f"start {i + 1}: {res['reload']} refusals {len(res.get('refusals', []))} "
              f"accepted at {res.get('accepted_after_ready_s')} s -> {res.get('state')}", flush=True)
    if "--no-mission" not in argv:
        robot, brain, bridge, rel = d4.reload(HOUSE, maps)
        de.MISSION_LIMIT_S = 900
        with GoalWatch(robot) as w:
            mission = de.run_mission(robot, HOUSE, policy="explore")
        out["mission"] = {"reload": rel["reload"].get("result"), **mission,
                          "goal_states": sorted(w.seen.values()),
                          "rejected": sum(s == "rejected" for s in w.seen.values())}
        print(f"mission: {mission.get('outcome')} rejected {out['mission']['rejected']} "
              f"contacts {mission.get('contact_samples')}", flush=True)
    out["criteria"] = score(out)
    dump(out)
    print(json.dumps(out["criteria"]), flush=True)


if __name__ == "__main__":
    main(sys.argv[1:])
