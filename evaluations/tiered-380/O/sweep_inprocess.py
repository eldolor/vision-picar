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
# --root <tree>: run against another checkout's code (the baseline arm runs
# today's code, before 3.80.O); the stand-ins below are this file's own, and
# WalkingNav comes from this tree when that checkout has none.
if "--root" in sys.argv:
    ROOT = Path(sys.argv[sys.argv.index("--root") + 1]).resolve()
sys.path.insert(0, str(ROOT))
from brain.perceive import FrameReportedPipeline  # noqa: E402
from brain.tiered import TieredVision  # noqa: E402
from control.mission_runner import MissionRunner  # noqa: E402
from sim.grid_world import CELL_WALL  # noqa: E402
from sim.maps import build_world  # noqa: E402
from sim.mock_robot import MockRobot  # noqa: E402
from sim.mock_world import MockWorld  # noqa: E402
from tests.test_explore import FakeNav  # noqa: E402
try:
    from tests.test_explore import WalkingNav  # noqa: E402
except ImportError:
    # --root at a checkout older than 3.80.O (the baseline arm): this tree's
    # WalkingNav, its test module loaded under another name (review).
    import importlib.util
    _spec = importlib.util.spec_from_file_location(
        "test_explore_380o", Path(__file__).resolve().parents[3] / "tests/test_explore.py")
    _mod = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(_mod)
    WalkingNav = _mod.WalkingNav

TARGET = "red backpack"
HFOV = 60.0


def _seen(frame):
    return [d for d in frame.get("detections") or () if d.get("label") == TARGET
            and d.get("bearing_deg") is not None]


class TruthLocal:
    """The local VLM from the simulator's report (as tests/test_local_arrival.py)."""

    def __init__(self):
        self.asked = 0

    def __call__(self, frame, box=False):
        self.asked += 1
        seen = _seen(frame)
        if not seen:
            return {"status": "answered", "score": 0.0}
        x = seen[0]["bearing_deg"] / HFOV + 0.5
        return {"status": "answered", "score": 0.95, "box_frac": [x - 0.02, 0.4, x + 0.02, 0.6]}


class TruthCloud:
    def __init__(self, answer=None, raises=False):
        self.calls, self.answer, self.raises = 0, answer, raises

    def __call__(self, frame):
        from brain.navigate import CloudUnavailable
        self.calls += 1
        if self.raises:
            raise CloudUnavailable("refused (sweep)")
        visible = bool(_seen(frame)) if self.answer is None else self.answer
        return {"obstacles_ahead": [], "free_space": "clear", "doorway_visible": False,
                "important_objects": [], "safest_direction": "LEFT",
                "_navigate": {"reasoning": "cloud", "target_visible": visible}}


class BlindPipeline:
    """Perception blind to the target, for any checkout: the simulator's
    report with the target taken out (what `blind=True` does since 3.80.O)."""

    def __init__(self, target):
        self.inner = FrameReportedPipeline(target)
        self.hfov_deg = HFOV

    def __getattr__(self, name):
        return getattr(self.inner, name)

    def perceive(self, frame):
        if frame.get("detections") is not None:
            frame = {**frame, "detections": [d for d in frame["detections"]
                                             if d.get("label") != TARGET]}
        return self.inner.perceive(frame)

sys.path.insert(0, str(ROOT / "evaluations/tiered-380/E"))
from pilot import SWEEP_HOUSES  # noqa: E402

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
    if a.sighted:
        pipeline = FrameReportedPipeline(TARGET)
        pipeline.hfov_deg = HFOV
    else:
        pipeline = BlindPipeline(TARGET)
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
        now_asked = identity.asked + getattr(tier.stats, "guarded", 0)
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
            "local_identity_asked": identity.asked, "guarded": getattr(tier.stats, "guarded", 0),
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
    ap.add_argument("--root", default=None, help="another checkout's code (read at import)")
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
