"""
evaluations/tiered-380/I/make_sets.py

3.80.I: the fixed frame sets, drawn before any model runs (seed 380).

* S1: 60 real frames labelled target-visible, uniform over the 535, at most
  8 a walk (labels are added by hand to `labels-s1.json`, by viewing).
* S2: 40 real frames labelled not visible, uniform, at most 4 a walk.
* S3: 2 renders per house of pilot.SWEEP_HOUSES, target out of view, at a
  seeded free pose 1.5-8 m by floor from the target; truth = the bearing of
  the point 1.0 m along the shortest 8-connected floor path, FORWARD within
  30 deg, else LEFT / RIGHT (theta grows to the RIGHT on the y-down grid).

    .venv/bin/python evaluations/tiered-380/I/make_sets.py
"""

import base64
import json
import math
import random
import sys
from collections import Counter, deque
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from sim.maps import build_world                     # noqa: E402
from sim.maps.procthor import TARGET                 # noqa: E402
from sim.mock_robot import MockRobot                 # noqa: E402

HERE = Path(__file__).parent
RECORDINGS = Path("/Users/anshugaind/vision-picar/recordings")
SEED = 380
HOUSES = ["home_first_floor"] + [f"procthor:val:{i}" for i in
                                 (0, 2, 3, 4, 5, 9, 10, 11, 12, 13, 15, 17, 18,
                                  21, 26, 27, 32, 34, 36, 37)]   # pilot.SWEEP_HOUSES
FORWARD_DEG, AHEAD_M = 30.0, 1.0


def draw(pool, n, cap, rng):
    rng.shuffle(pool)
    out, per = [], Counter()
    for walk, f, target in pool:
        if per[walk] < cap:
            out.append({"walk": walk, "frame": f, "target": target})
            per[walk] += 1
        if len(out) == n:
            break
    return sorted(out, key=lambda r: (r["walk"], r["frame"]))


def real_sets(rng):
    vis, nov = [], []
    for walk in sorted(p for p in RECORDINGS.iterdir() if (p / "labels.json").is_file()):
        L = json.loads((walk / "labels.json").read_text())
        for f, v in sorted(L["target_visible_labels"].items()):
            if (walk / f).is_file():
                (vis if v else nov).append((walk.name, f, L["description"]))
    return draw(vis, 60, 8, rng), draw(nov, 40, 4, rng), len(vis), len(nov)


def floor_path(grid, start, goal_cells):
    """Shortest 8-connected path over passable cells, start -> any goal cell."""
    prev = {start: None}
    q = deque([start])
    while q:
        c = q.popleft()
        if c in goal_cells:
            path = []
            while c is not None:
                path.append(c)
                c = prev[c]
            return path[::-1]
        x, y = c
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                n = (x + dx, y + dy)
                if n in prev or not (0 <= n[0] < grid.width and 0 <= n[1] < grid.height):
                    continue
                if not grid._is_passable(*n):
                    continue
                if dx and dy and not (grid._is_passable(x + dx, y) and grid._is_passable(x, y + dy)):
                    continue
                prev[n] = c
                q.append(n)
    return None


def renders(rng, per_house=2, out_dir=HERE / "s3"):
    out_dir.mkdir(exist_ok=True)
    rows = []
    for house in HOUSES:
        grid = build_world(house)
        robot = MockRobot(grid, render=True)
        cell_m = getattr(robot, "cell_m", 0.30)
        (tx, ty), = [c for c, label in grid.objects.items() if label == TARGET]
        goals = {(tx + dx, ty + dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                 if grid._is_passable(tx + dx, ty + dy)}
        free = [(x, y) for x in range(grid.width) for y in range(grid.height)
                if grid._is_passable(x, y)]
        took, tries = 0, 0
        while took < per_house and tries < 2000:
            tries += 1
            cx, cy = rng.choice(free)
            x, y = cx + 0.5, cy + 0.5
            if grid.footprint_overlaps(x, y):
                continue
            path = floor_path(grid, (cx, cy), goals)
            if not path:
                continue
            length_m = sum(math.dist(a, b) for a, b in zip(path, path[1:])) * cell_m
            if not 1.5 <= length_m <= 8.0:
                continue
            theta = rng.uniform(-math.pi, math.pi)
            grid.x, grid.y, grid.theta = x, y, theta
            if grid.footprint_overlaps(x, y):
                continue
            frame = robot.get_camera_frame()
            if TARGET in (frame.get("objects_visible") or []):
                continue
            # The point AHEAD_M along the path.
            acc, px, py = 0.0, None, None
            for a, b in zip(path, path[1:]):
                acc += math.dist(a, b) * cell_m
                if acc >= AHEAD_M:
                    px, py = b[0] + 0.5, b[1] + 0.5
                    break
            if px is None:
                px, py = path[-1][0] + 0.5, path[-1][1] + 0.5
            rel = math.degrees(math.atan2(py - y, px - x) - theta)
            rel = (rel + 180) % 360 - 180
            truth = "FORWARD" if abs(rel) <= FORWARD_DEG else ("RIGHT" if rel > 0 else "LEFT")
            name = f"{house.replace(':', '-')}-{took}.jpg"
            (out_dir / name).write_bytes(base64.b64decode(frame["image_base64"]))
            rows.append({"house": house, "image": f"s3/{name}", "target": TARGET,
                         "pose": [round(x, 2), round(y, 2), round(theta, 3)],
                         "path_m": round(length_m, 2), "rel_deg": round(rel, 1),
                         "truth": truth})
            took += 1
    return rows


def main():
    rng = random.Random(SEED)
    s1, s2, n_vis, n_nov = real_sets(rng)
    s3 = renders(rng)
    doc = {"seed": SEED, "pool": {"visible": n_vis, "not_visible": n_nov},
           "S1": s1, "S2": s2, "S3": s3,
           "S3_truth": dict(Counter(r["truth"] for r in s3))}
    (HERE / "sets.json").write_text(json.dumps(doc, indent=1) + "\n")
    print(json.dumps({"S1": len(s1), "S1_walks": len({r['walk'] for r in s1}),
                      "S2": len(s2), "S2_walks": len({r['walk'] for r in s2}),
                      "S3": len(s3), "S3_truth": doc["S3_truth"], "pool": doc["pool"]}))


if __name__ == "__main__":
    main()
