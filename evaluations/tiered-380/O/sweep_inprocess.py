"""
evaluations/tiered-380/O/sweep_inprocess.py

3.80.O in process: explore's tiered search over houses with FakeNav for nav2,
the local VLM and the cloud stood in from the simulator's report
(`tests/test_local_arrival.py`'s TruthLocal / TruthCloud), perception blind
to the target or not. Per mission: the end, the first arrival (an identity
question asked, by the local tier or the cloud's guard) and the TRUE distance
to the target then, `found` far from the target (O2), cloud calls, guarded
arrivals, local questions by trigger.

    .venv/bin/python evaluations/tiered-380/O/sweep_inprocess.py --name blind-async \\
        [--sighted] [--sync] [--decoy 0.05] [--cloud truth|no|down] [--houses dev|criterion|<h,...>]
"""

import argparse
import json
import logging
import math
import random
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from brain.perceive import FrameReportedPipeline  # noqa: E402
from brain.tiered import TieredVision  # noqa: E402
from control.mission_runner import MissionRunner  # noqa: E402
from sim.grid_world import CELL_WALL  # noqa: E402
from sim.maps import build_world  # noqa: E402
from sim.mock_robot import MockRobot  # noqa: E402
from sim.mock_world import MockWorld  # noqa: E402
from tests.test_explore import FakeNav  # noqa: E402
from tests.test_local_arrival import HFOV, TARGET, TruthCloud, TruthLocal, _seen  # noqa: E402

sys.path.insert(0, str(ROOT / "evaluations/tiered-380/E"))
from pilot import SWEEP_HOUSES  # noqa: E402

class WalkingNav(FakeNav):
    """FakeNav that drives: each poll moves the robot one cell (0.3 m) along
    the shortest 4-connected floor path and passes 1 s, so the tier sees a
    frame per cell while a goal runs, as it does behind live nav2. FakeNav
    jumps to the goal in one poll (four frames a goal in all), which starves
    a blind search of questions (3.80.O amendment 1)."""

    def _path(self, a, b):
        from collections import deque
        g, prev, q = self.grid, {a: None}, deque([a])
        while q:
            c = q.popleft()
            if c == b:
                out = []
                while c is not None:
                    out.append(c)
                    c = prev[c]
                return out[::-1]
            for n in ((c[0] + 1, c[1]), (c[0] - 1, c[1]), (c[0], c[1] + 1), (c[0], c[1] - 1)):
                if n not in prev and g._cell(*n) != CELL_WALL and n not in g.objects:
                    prev[n] = c
                    q.append(n)
        return None

    def get_goal(self):
        self.calls.append(("get",))
        if self.goal and self.goal["state"] == "active":
            g = self.grid
            to = (math.floor(self.goal["x_m"] / CELL_M), math.floor(self.goal["y_m"] / CELL_M))
            path = None if to in g.objects else self._path((g.robot_x, g.robot_y), to)
            self.robot.pass_time(1.0)
            if path is None:
                self.goal["state"] = "aborted"
            elif len(path) <= 2:
                tx, ty = self.goal["x_m"] / CELL_M, self.goal["y_m"] / CELL_M
                if (tx, ty) != (g.x, g.y):
                    g.theta = math.atan2(ty - g.y, tx - g.x)
                g.x, g.y = tx, ty
                self.goal["state"] = "succeeded"
            else:
                nx, ny = path[1][0] + 0.5, path[1][1] + 0.5
                g.theta = math.atan2(ny - g.y, nx - g.x)
                g.x, g.y = nx, ny
        return {"goal": self.goal, "plan": []}


DEV_HOUSES = [f"procthor:val:{i}" for i in (6, 7, 8, 14, 16, 19, 20, 22, 23, 24)]
CELL_M = 0.3


class DecoyLocal(TruthLocal):
    """TruthLocal, plus a false yes: with probability `p` per question on a
    frame without the target, a yes with a box on another reported object."""

    def __init__(self, p, seed):
        super().__init__()
        self.p, self.rng, self.false_yes = p, random.Random(seed), 0

    def __call__(self, frame, box=False):
        got = super().__call__(frame, box)
        if got.get("box_frac") or self.p <= 0:
            return got
        others = [d for d in frame.get("detections") or ()
                  if d.get("label") != TARGET and d.get("bearing_deg") is not None]
        if others and self.rng.random() < self.p:
            self.false_yes += 1
            x = others[0]["bearing_deg"] / HFOV + 0.5
            return {"status": "answered", "score": 0.9, "box_frac": [x - 0.02, 0.4, x + 0.02, 0.6]}
        return got


def target_m(grid):
    cells = [c for c, n in grid.objects.items() if n == TARGET]
    return min(math.dist((grid.x, grid.y), (cx + 0.5, cy + 0.5)) for cx, cy in cells) * CELL_M


def run(house, a):
    grid = build_world(house)
    robot = MockRobot(grid, render=False)
    pipeline = FrameReportedPipeline(TARGET, blind=not a.sighted)
    pipeline.hfov_deg = HFOV
    sighting = DecoyLocal(a.decoy, hash(house) & 0xffff)
    identity = TruthLocal()
    cloud = {"truth": TruthCloud(), "no": TruthCloud(answer=False),
             "down": TruthCloud(raises=True)}[a.cloud]
    tier = TieredVision(pipeline, TruthCloud(answer=False), confirm_vision_fn=cloud,
                        local_identity_fn=identity, local_sighting_fn=sighting,
                        async_cloud=not a.sync, cold_search_after=a.cold_every)
    runner = MissionRunner(robot, target_object=TARGET, policy="tiered", vision_fn=tier,
                           tier_search="explore", navigator=(FakeNav(robot) if a.jump else WalkingNav(robot)),
                           world=MockWorld(grid), mission_time_limit_s=1200,
                           clock=lambda: grid.sim_time, idle=robot.pass_time)
    runner.start()
    ticks, first, arrivals = 0, None, []
    asked = 0
    while True:
        going = runner.tick()
        ticks += 1
        now_asked = identity.asked + tier.stats.guarded
        if now_asked > asked:
            arrivals.append(round(target_m(grid), 2))
            asked = now_asked
        if not going or ticks >= 20000:
            break
    first = arrivals[0] if arrivals else None
    s = runner.status()
    end_m = round(target_m(grid), 2)
    return {"house": house, "outcome": s["outcome"], "sim_s": round(grid.sim_time),
            "goals": s["explore"]["goals_sent"], "arrivals": arrivals,
            "first_arrival_target_m": first, "end_target_m": end_m,
            "found_far": s["outcome"] == "found" and end_m > 0.9,
            "local_identity_asked": identity.asked, "guarded": tier.stats.guarded,
            "cloud_confirms": cloud.calls, "close_ins": runner.agent.close_ins,
            "local_triggers": dict(tier.stats.local_triggers),
            "decoy_false_yes": sighting.false_yes, "tail": s["log_tail"][-1:]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", required=True)
    ap.add_argument("--sighted", action="store_true")
    ap.add_argument("--sync", action="store_true")
    ap.add_argument("--jump", action="store_true", help="FakeNav's jump to the goal")
    ap.add_argument("--decoy", type=float, default=0.0)
    ap.add_argument("--cold-every", type=int, default=6,
                    help="tier cold_search_after (the brain's default 6)")
    ap.add_argument("--cloud", choices=("truth", "no", "down"), default="truth")
    ap.add_argument("--houses", default="dev")
    a = ap.parse_args()
    houses = (DEV_HOUSES if a.houses == "dev" else SWEEP_HOUSES if a.houses == "criterion"
              else a.houses.split(","))
    logging.disable(logging.WARNING)
    t, rows = time.time(), []
    for h in houses:
        rows.append(run(h, a))
        print(json.dumps({k: v for k, v in rows[-1].items() if k != "tail"}), flush=True)
    at_target = sum(1 for r in rows if any(d <= 0.9 for d in r["arrivals"]))
    near = [min(r["arrivals"]) for r in rows if r["arrivals"] and min(r["arrivals"]) <= 0.9]
    out = {"args": vars(a), "missions": len(rows), "arrivals_at_true_target": at_target,
           "arrival_m_median": sorted(near)[len(near) // 2] if near else None,
           "found": sum(r["outcome"] == "found" for r in rows),
           "found_far": sum(r["found_far"] for r in rows),
           "local_identity_on_guarded": sum(r["local_identity_asked"] for r in rows
                                            if not a.sighted),
           "outcomes": {o: sum(r["outcome"] == o for r in rows)
                        for o in sorted({r["outcome"] for r in rows})},
           "rows": rows, "wall_s": round(time.time() - t)}
    out_dir = Path(__file__).parent
    (out_dir / f"{a.name}.json").write_text(json.dumps(out, indent=1) + "\n")
    print(json.dumps({k: v for k, v in out.items() if k != "rows"}))


if __name__ == "__main__":
    main()
