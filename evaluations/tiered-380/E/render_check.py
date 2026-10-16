"""
evaluations/tiered-380/E/render_check.py

3.80.E, after criterion 2's local arm: does the board's local VLM say yes to
the simulator's rendered target at all? Places the robot facing the target
from free cells 1-3 m away (a clear line of sight), renders the frame the
missions saw, and asks the local VLM the arrival question. A real recorded
frame of a backpack (a true yes in labels.json) is the control: the service
must say yes to it, or the check says nothing about the renderer.

    .venv/bin/python evaluations/tiered-380/E/render_check.py --url http://127.0.0.1:18005
"""

import argparse
import base64
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from brain.local_vlm import LocalVLMClient          # noqa: E402
from sim.maps import build_world                     # noqa: E402
from sim.maps.procthor import TARGET                 # noqa: E402
from sim.mock_robot import MockRobot                 # noqa: E402

HOUSES = ["procthor:val:5", "procthor:val:18", "home_first_floor"]
CONTROL = Path("/Users/anshugaind/vision-picar/recordings/"
               "grey-backpack-opus-4-5-20260921-210743/frame-0017.jpg")


def clear_line(grid, x0, y0, x1, y1) -> bool:
    n = int(max(abs(x1 - x0), abs(y1 - y0)) * 4) + 1
    for i in range(1, n):
        x = x0 + (x1 - x0) * i / n
        y = y0 + (y1 - y0) * i / n
        cx, cy = int(math.floor(x)), int(math.floor(y))
        if (cx, cy) == (int(x1), int(y1)):
            return True
        if not grid._is_passable(cx, cy):
            return False
    return True


def views(house, per_house=4):
    grid = build_world(house)
    robot = MockRobot(grid, render=True)
    cell_m = getattr(robot, "cell_m", 0.30)
    (tx, ty), = [c for c, label in grid.objects.items() if label == TARGET]
    tcx, tcy = tx + 0.5, ty + 0.5
    out = []
    for dist_m in (1.0, 1.5, 2.0, 3.0):
        r = dist_m / cell_m
        for k in range(16):
            a = 2 * math.pi * k / 16
            x, y = tcx + r * math.cos(a), tcy + r * math.sin(a)
            cx, cy = int(math.floor(x)), int(math.floor(y))
            if not (0 <= cx < grid.width and 0 <= cy < grid.height):
                continue
            if not grid._is_passable(cx, cy) or grid.footprint_overlaps(x, y):
                continue
            if not clear_line(grid, x, y, tcx, tcy):
                continue
            grid.x, grid.y = x, y
            grid.theta = math.atan2(tcy - y, tcx - x)
            out.append((dist_m, robot.get_camera_frame()))
            break
        if len(out) >= per_house:
            break
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", required=True)
    ap.add_argument("--out", default=str(Path(__file__).with_name("render-check.json")))
    args = ap.parse_args()
    vlm = LocalVLMClient(args.url, TARGET, deadline_s=6.0)
    frames_dir = Path(args.out).with_suffix("")
    frames_dir.mkdir(exist_ok=True)
    rows = []
    ctrl = {"image_base64": base64.b64encode(CONTROL.read_bytes()).decode(),
            "media_type": "image/jpeg"}
    got = LocalVLMClient(args.url, "Grey backpack", deadline_s=6.0).ask(ctrl)
    rows.append({"house": "control (real frame, labelled yes)", "target": "Grey backpack",
                 "status": got.get("status"), "score": got.get("score")})
    print(json.dumps(rows[-1]), flush=True)
    for house in HOUSES:
        for dist_m, frame in views(house):
            name = f"{house.replace(':', '-')}-{dist_m}m.jpg"
            (frames_dir / name).write_bytes(base64.b64decode(frame["image_base64"]))
            got = vlm.ask(frame)
            rows.append({"house": house, "dist_m": dist_m, "frame": name,
                         "target_in_detections": any(
                             TARGET in str(d) for d in frame.get("detections") or []),
                         "status": got.get("status"), "score": got.get("score")})
            print(json.dumps(rows[-1]), flush=True)
    Path(args.out).write_text(json.dumps(rows, indent=1))


if __name__ == "__main__":
    main()
