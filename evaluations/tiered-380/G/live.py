"""
evaluations/tiered-380/G/live.py

3.80.G's re-run of criterion 2 (the plan's "3.80.G" criteria G1-G3): one live
tiered mission per house with `tier_search: explore`, the local tier on (VE
on the board, reached at LOCAL_VLM_URL), through the brain's HTTP API on a
fresh ROS stack each (`tests/demo_explore.py`'s `stack()`: robot server,
brain, nav2 + slam_toolbox, this section's ports and image), with
`pilot.py`'s counting proxy between the brain and /navigate.

Per mission it records: the end, cloud calls through the proxy (and their
cost), the arrival checks (`tier.stats.local_calls` + the cloud's
confirmations) and the TRUE distance from the robot to the target when the
first one was asked (G2: an arrival at the true target is one asked within
0.9 m), and the chassis' contacts with walls and furniture on ground truth
(G3), sampled every 0.5 s as `demo_explore.run_mission` does.

    eval "$(python tools/plan_section.py env)"
    LOCAL_VLM_URL=http://127.0.0.1:18005 \\
      .venv/bin/python evaluations/tiered-380/G/live.py --cloud standin --out ...
    ...                                             --cloud real --out ...   # Bedrock: the user's yes

`--cloud standin` answers every /navigate locally ("not visible, RIGHT", $0);
`real` forwards to the deployed service, capped at --cap-usd over the whole
run (each mission's budget is what is left at the measured $/call). Resumable
JSON lines; `--houses` limits the run (a smoke test).
"""

import argparse
import json
import math
import os
import sys
import threading
import time
from http.server import ThreadingHTTPServer
from pathlib import Path

import httpx

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "evaluations/tiered-380/E"))
import pilot  # noqa: E402
from pilot import SWEEP_HOUSES, Proxy, cost, secrets  # noqa: E402
from sim.grid_world import CELL_WALL  # noqa: E402
from sim.maps import build_world  # noqa: E402
from tests import demo_explore  # noqa: E402
from tests.demo_local_tier import free_port  # noqa: E402
from tests.footprint_sweep import CELL_CM, chassis, gap, penetration, square  # noqa: E402

TARGET = "red backpack"
CELL_M = 0.3
FOUND_RADIUS_M = 0.9
LIMIT_S = 1200
PER_CALL_USD = 0.0055    # measured in 3.80.E (633 tokens in, 92 out)


def target_m(truth, target_at):
    x, y = truth["x_m"] / CELL_M, truth["y_m"] / CELL_M
    return min(math.dist((x, y), (cx + 0.5, cy + 0.5)) for cx, cy in target_at) * CELL_M


def checks(status):
    """Arrival identity checks answered so far: the local tier's and the cloud's."""
    tier = status.get("tier") or {}
    stats = tier.get("stats") or {}
    return (stats.get("local_calls") or 0) + ((stats.get("triggers") or {})
                                              .get("arrival_confirmation") or 0)


def arrived(status):
    """The arrival rule held on this tick (the identity question is asked
    on it): the runner's readout is arrived, checking or refused -- read
    before any answer lands, as amendment 1 defines G2."""
    return ((status.get("arrival") or {}).get("state")
            in ("arrived", "checking", "refused")) or status.get("outcome") == "found"


def run_house(house, args, budget):
    Proxy.budget = budget
    Proxy.calls = []
    robot = demo_explore.stack(house)
    world = build_world(house)
    walls = {(x, y) for y, row in enumerate(world.layout) for x, c in enumerate(row)
             if c == CELL_WALL}
    brain = httpx.Client(base_url=f"http://127.0.0.1:{demo_explore.BRAIN}", timeout=60)
    r = brain.post("/mission/start", json={"target_object": TARGET, "policy": "tiered",
                                           "mission_time_limit_s": LIMIT_S})
    if r.status_code != 200:
        return {"house": house, "start": r.status_code, "detail": r.text[:300]}
    target_at = [(o["x"], o["y"]) for o in robot.get("/sim/objects").json()["objects"]
                 if o["name"] == TARGET]
    t0, status, first_check_m, min_gap = time.time(), {}, None, math.inf
    contacts = inside = samples = 0
    while time.time() - t0 < LIMIT_S + 120:
        truth = robot.get("/world/truth").json()
        objs = {(o["x"], o["y"]) for o in robot.get("/sim/objects").json()["objects"]}
        x, y = truth["x_m"] / CELL_M, truth["y_m"] / CELL_M
        a = chassis(x, y, math.radians(truth["heading_deg"]) - math.pi / 2)
        for c in walls | objs:
            if abs(c[0] + 0.5 - x) > 1.6 or abs(c[1] + 0.5 - y) > 1.6:
                continue
            g = gap(a, square(*c)) * CELL_CM
            min_gap = min(min_gap, g)
            contacts += g < 1.0
            inside += penetration(a, square(*c)) > 0
        samples += 1
        status = brain.get("/mission/status").json()
        if first_check_m is None and arrived(status):
            first_check_m = round(target_m(truth, target_at), 2)
        if not status.get("running") and status.get("outcome") not in (None, "idle", "running"):
            break
        time.sleep(0.5)
    else:
        brain.post("/mission/stop")
    stats = (status.get("tier") or {}).get("stats") or {}
    calls = [c for c in Proxy.calls if not c.get("refused")]
    usages = [c["usage"] for c in calls if c.get("usage")]
    tag = house.replace(":", "-")
    (Path(demo_explore.RUN_DIR) / f"status-{tag}.json").write_text(json.dumps(status))
    return {"house": house, "cloud": args.cloud, "outcome": status.get("outcome"),
            "seconds": round(time.time() - t0), "steps": status.get("step"),
            "cloud_calls_proxy": len(calls),
            "refused_over_budget": sum(1 for c in Proxy.calls if c.get("refused")),
            "cost_usd": round(sum(cost(u) for u in usages), 4),
            "arrival_checks": checks(status), "first_check_target_m": first_check_m,
            "arrived_at_true_target": (first_check_m is not None
                                       and first_check_m <= FOUND_RADIUS_M),
            "end_target_m": round(target_m(robot.get("/world/truth").json(), target_at), 2),
            "contact_samples": contacts, "inside_samples": inside, "samples": samples,
            "min_gap_cm": round(min_gap, 1),
            "explore": status.get("explore"), "tier_search": status.get("tier_search"),
            "local": stats.get("local"), "local_calls": stats.get("local_calls"),
            "local_triggers": stats.get("local_triggers"),
            "local_frame": stats.get("local_frame"),
            "frame_escalations": stats.get("frame_escalations"),
            "triggers": stats.get("triggers"), "frames": stats.get("frames"),
            "local_frame_ms": stats.get("local_frame_ms"), "local_ms": stats.get("local_ms"),
            "log_tail": (status.get("log_tail") or [])[-4:], "run_dir": demo_explore.RUN_DIR}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cloud", choices=("real", "standin"), default="standin")
    ap.add_argument("--out", required=True)
    ap.add_argument("--houses", type=int, default=len(SWEEP_HOUSES))
    ap.add_argument("--cap-usd", type=float, default=2.0)
    ap.add_argument("--only", action="append", default=[], help="these houses only (a smoke test)")
    args = ap.parse_args()
    if not os.environ.get("LOCAL_VLM_URL"):
        raise SystemExit("LOCAL_VLM_URL is not set: the local arm needs the board's VE")
    s = secrets()
    Proxy.upstream = (s.get("VISION_URL") or pilot.DEFAULT_VISION_URL).rstrip("/")
    Proxy.secret = s.get("VISION_SECRET", "")
    Proxy.mode = args.cloud
    proxy_p = free_port()
    httpd = ThreadingHTTPServer(("127.0.0.1", proxy_p), Proxy)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    # Read by demo_explore.stack()'s brain (it copies os.environ).
    os.environ.update(TIER_SEARCH="explore", TIER_LOCAL_DIRECTION="false",
                      VISION_URL=f"http://127.0.0.1:{proxy_p}", METRICS_URL="",
                      INVENTORY_BUCKET="")
    for k in ("VISION_SHARED_SECRET", "VISION_SECRET"):
        os.environ.pop(k, None)
    out = Path(args.out)
    done = [json.loads(line) for line in out.read_text().splitlines()] if out.exists() else []
    spent = sum(r.get("cost_usd") or 0 for r in done)
    try:
        for house in (args.only or SWEEP_HOUSES[:args.houses]):
            if any(r["house"] == house for r in done):
                continue
            budget = (10_000 if args.cloud == "standin"
                      else int((args.cap_usd - spent) / (PER_CALL_USD * 1.2)))
            if budget <= 0:
                print(f"stopping before {house}: ${spent:.2f} of ${args.cap_usd} spent", flush=True)
                break
            row = run_house(house, args, budget)
            spent += row.get("cost_usd") or 0
            with out.open("a") as f:
                f.write(json.dumps(row) + "\n")
            print(f"{house}: {row.get('outcome')} calls={row.get('cloud_calls_proxy')} "
                  f"checks={row.get('arrival_checks')} at={row.get('first_check_target_m')} "
                  f"contacts={row.get('contact_samples')} {row.get('seconds')} s "
                  f"total ${spent:.2f}", flush=True)
    finally:
        demo_explore._kill_ports()
        httpd.shutdown()


if __name__ == "__main__":
    main()
