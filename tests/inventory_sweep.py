"""3.46's simulator sweep: does the inventory find, place and not invent?

    python -m tests.inventory_sweep            # all 20 missions, both halves
    python -m tests.inventory_sweep --half 1   # the tuning half only

Every mission runs the WHOLE path: `MissionRunner` -> the frontier policy
-> `robot/safety.py` -> `MockRobot` in `complex_house`, searching for a
thing that is not in the house so it explores until `max_steps`. The
inventory is fed by the runner's own frame hook, through
`brain.inventory.frame_detections()` (the sim's synthesised detections,
1.12) wrapped in a detector that errs:

* bearing noise, sigma `BEARING_SIGMA_DEG`;
* each detection dropped with probability `MISS_RATE`;
* on `RELABEL_RATE` of frames, one detection given another class's name.

Range comes from the scan (the inventory's own rule), never from the
sim's ground-truth distance. Ground truth is `complex_house.INSTANCES`:
one entry per piece of furniture or small thing, so a table's four legs
are one table.

What is scored (PLAN 3.46 criteria 3-5, thresholds written there first):

* SEEN: an instance a TRUE detection reported within 3 m from at least
  two distinct viewpoints (the inventory's own new-view rule).
* RECALL: seen instances with a REPORTED landmark of the right majority
  class within `MATCH_M` of the instance.
* PLACEMENT: reported landmarks matched to an instance of their class --
  distance to the nearest cell of it (0 if inside); duplicates per
  matched instance.
* PRECISION: reported landmarks within `MATCH_M` of an instance of their
  majority class; the same over ALL landmarks, candidates included.

Missions 1-10 are the tuning half (the `[PLACEHOLDER]`s in
`brain/inventory.py` are set there); 11-20 are judged.
"""
from __future__ import annotations

import argparse
import json
import math
import random
import statistics
import sys
import time

from brain import inventory as inv
from control.mission_runner import MissionRunner
from sim.grid_world import PAN_ANGLE_RAD
from sim.maps import build_world
from sim.maps import complex_house as house
from sim.mock_robot import MockRobot
from tests.conftest import mock_world_for

CELL_M = 0.30
BEARING_SIGMA_DEG = 3.0
MISS_RATE = 0.10
RELABEL_RATE = 0.05
MATCH_M = 1.0
SEEN_RANGE_M = 3.0
MAX_STEPS = 150
MISSIONS = 20
ABSENT_TARGET = "unicorn"

_CELL_TO_INSTANCE = {cell: k for k, (_, cells) in enumerate(house.INSTANCES) for cell in cells}
_NAMES = sorted({name for name, _ in house.INSTANCES})


def starts(n: int = MISSIONS, seed: int = 346) -> list:
    """Spread starts: rooms in turn, a free cell with free neighbours in
    each, a random heading. Deterministic for a seed."""
    rng = random.Random(seed)
    rooms = sorted(r for r in house.ROOMS if house.ROOMS[r])
    out = []
    for k in range(n):
        room = rooms[k % len(rooms)]
        cells = sorted(c for c in house.ROOMS[room]
                       if all((c[0] + dx, c[1] + dy) in house.ROOMS[room]
                              and (c[0] + dx, c[1] + dy) not in house.OBJECTS
                              for dx in (-1, 0, 1) for dy in (-1, 0, 1)))
        if not cells:
            continue
        out.append((rng.choice(cells), rng.uniform(-math.pi, math.pi), room))
    return out[:n]


class ErringDetector:
    """`frame_detections()` plus the errors a real detector makes. Also
    records the TRUE per-cell detections with the true pose, for scoring."""

    def __init__(self, grid, rng: random.Random, noisy: bool = True):
        self.grid, self.rng, self.noisy = grid, rng, noisy
        self.seen: dict = {}            # instance -> [true poses within range]

    def _truth(self, frame):
        g = self.grid
        view = g.theta + g.pan * PAN_ANGLE_RAD
        pose = {"x_m": g.x * CELL_M, "y_m": g.y * CELL_M, "heading_deg": g.heading_deg}
        for d in frame.get("detections") or []:
            if d["distance_m"] > SEEN_RANGE_M:
                continue
            a = view + math.radians(d["bearing_deg"])
            r = d["distance_m"] / CELL_M
            px, py = g.x + r * math.cos(a), g.y + r * math.sin(a)
            # Per-cell detections are measured to the cell centre.
            cell = (int(math.floor(px)), int(math.floor(py)))
            k = _CELL_TO_INSTANCE.get(cell)
            if k is None:
                near = [c for c in _CELL_TO_INSTANCE
                        if abs(c[0] + 0.5 - px) < 0.6 and abs(c[1] + 0.5 - py) < 0.6]
                k = _CELL_TO_INSTANCE[near[0]] if near else None
            if k is not None:
                self.seen.setdefault(k, []).append(pose)

    def __call__(self, frame):
        dets = inv.frame_detections(frame)
        if dets is None:
            return None
        self._truth(frame)
        if not self.noisy:
            return dets
        out = []
        for d in dets:
            if self.rng.random() < MISS_RATE:
                continue
            out.append({**d, "bearing_deg": d["bearing_deg"]
                        + self.rng.gauss(0.0, BEARING_SIGMA_DEG)})
        if out and self.rng.random() < RELABEL_RATE:
            k = self.rng.randrange(len(out))
            out[k] = {**out[k], "label": self.rng.choice(
                [n for n in _NAMES if n != out[k]["label"]])}
        return out


def _views(poses: list) -> int:
    kept = []
    for p in poses:
        if all(inv._new_view(q, p) for q in kept):
            kept.append(p)
    return len(kept)


def _dist_to_instance(x: float, y: float, k: int) -> float:
    best = float("inf")
    for i, j in house.INSTANCES[k][1]:
        dx = max(i * CELL_M - x, 0.0, x - (i + 1) * CELL_M)
        dy = max(j * CELL_M - y, 0.0, y - (j + 1) * CELL_M)
        best = min(best, math.hypot(dx, dy))
    return best


def _nearest_of_class(lm: dict) -> tuple:
    cands = [(k, _dist_to_instance(lm["x_m"], lm["y_m"], k))
             for k, (name, _) in enumerate(house.INSTANCES) if name == lm["label"]]
    return min(cands, key=lambda t: t[1]) if cands else (None, float("inf"))


def run_one(start, *, noisy=True, seed=0, max_steps=MAX_STEPS, inventory=True) -> dict:
    (cx, cy), theta, room = start
    grid = build_world("complex_house")
    grid.x, grid.y, grid.theta = cx + 0.5, cy + 0.5, theta
    robot = MockRobot(grid, render=False)
    det = ErringDetector(grid, random.Random(seed), noisy=noisy)
    runner = MissionRunner(robot, target_object=ABSENT_TARGET, max_steps=max_steps,
                           policy="frontier", world=mock_world_for(robot),
                           inventory=inventory, detections_fn=det)
    t0 = time.perf_counter()
    runner.start()
    while runner.tick():
        pass
    report = runner.inventory.report() if inventory else None
    return {"start_room": room, "report": report, "seen": det.seen,
            "actions": [a.action for a in runner.memory.actions],
            "outcome": runner.status()["outcome"],
            "wall_s": time.perf_counter() - t0}


def score(runs: list) -> dict:
    seen_n = found_n = small_seen = small_found = 0
    dists, dup_counts = [], []
    rep_ok = rep_n = all_ok = all_n = 0
    closures = 0
    for r in runs:
        rep = r["report"]
        reported, every = rep["reported"], rep["reported"] + rep["candidates"]
        seen = {k for k, poses in r["seen"].items() if _views(poses) >= 2}
        matched = {}
        for lm in reported:
            k, d = _nearest_of_class(lm)
            rep_n += 1
            if d <= MATCH_M:
                rep_ok += 1
                matched.setdefault(k, []).append(d)
        for lm in every:
            all_n += 1
            all_ok += _nearest_of_class(lm)[1] <= MATCH_M
        for k in seen:
            small = house.INSTANCES[k][0] in house.SMALL_NAMES
            seen_n += 1
            small_seen += small
            if k in matched:
                found_n += 1
                small_found += small
        for k, ds in matched.items():
            dists.extend(ds)
            dup_counts.append(len(ds))
    q = statistics.quantiles(dists, n=10) if len(dists) >= 2 else [float("nan")] * 9
    return {
        "missions": len(runs),
        "recall": found_n / seen_n if seen_n else float("nan"),
        "recall_small": small_found / small_seen if small_seen else float("nan"),
        "seen": seen_n, "seen_small": small_seen,
        "placement_median_m": statistics.median(dists) if dists else float("nan"),
        "placement_p90_m": q[8],
        "duplicates_per_instance": (sum(dup_counts) / len(dup_counts)) if dup_counts else float("nan"),
        "precision_reported": rep_ok / rep_n if rep_n else float("nan"),
        "precision_all": all_ok / all_n if all_n else float("nan"),
        "reported": rep_n, "landmarks": all_n,
        "loop_closures": closures,   # in-process pose is truth: none can occur
    }


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--half", type=int, choices=(1, 2))
    ap.add_argument("--steps", type=int, default=MAX_STEPS)
    ap.add_argument("--clean", action="store_true", help="no injected detector errors")
    ap.add_argument("--seed", type=int, default=346,
                    help="starts' seed: 346 is sweep 1; 3461 the fresh set for its amendment")
    args = ap.parse_args(argv)
    all_starts = starts(seed=args.seed)
    idx = range(MISSIONS)
    if args.half == 1:
        idx = range(MISSIONS // 2)
    elif args.half == 2:
        idx = range(MISSIONS // 2, MISSIONS)
    runs = []
    for k in idx:
        r = run_one(all_starts[k], noisy=not args.clean, seed=args.seed * 100 + k,
                    max_steps=args.steps)
        runs.append(r)
        print(f"mission {k + 1:2d} from {r['start_room']:<12} {r['outcome']:<10} "
              f"{len(r['report']['reported']):3d} reported {len(r['report']['candidates']):3d} "
              f"candidates  {r['wall_s']:.1f}s", file=sys.stderr)
    print(json.dumps(score(runs), indent=1))


if __name__ == "__main__":
    main()
