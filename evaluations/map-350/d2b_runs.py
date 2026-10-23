"""3.50.D2-b: does map_and_localization_slam_toolbox_node map as well as the
async node did? 3.40's own runs, on this branch's image:

  1. R5's starter-house lap, right encoder 3% long   (3.40 criterion 3)
  2. R6's scaled-house goals                           (3.40 criterion 3)
  3. two explore-then-tour runs in home_first_floor    (3.40 criterion 1)

Raw output in evaluations/map-350/D2b/; judged by evaluations/slam-340/judge.py.

    eval "$(python tools/plan_section.py env)"
    python evaluations/map-350/d2b_runs.py [lap|r6|home ...]
"""
import json
import os
import subprocess
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, ROOT)

from tests import demo_explore as de  # noqa: E402

# The control arm (ROS_IMAGE=vision-picar-ros:plan-3.50-async: this branch's
# image with only the SLAM executable put back to 3.40's async node) writes
# beside the main arm, so the two differ in that one thing.
IMAGE = os.environ.get("ROS_IMAGE", "")
assert IMAGE.endswith(("plan-3.50", "plan-3.50-async")), "run with this branch's env"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "D2b",
                   *(["control-async"] if IMAGE.endswith("-async") else []))


def run_module(module, house, name, drift="", args=()):
    de.stack(house, odom_drift=drift)
    env = dict(os.environ, SIM_MAP=house, PICAR_ROBOT_URL=f"http://127.0.0.1:{de.ROBOT}",
               PICAR_BRIDGE_URL=f"http://127.0.0.1:{de.BRIDGE}")
    env.pop("APP_SHARED_SECRET", None)
    env.pop("LOCAL_SECRET", None)
    with open(os.path.join(OUT, name), "w") as f:
        subprocess.run([de.PY, "-m", module, *args], cwd=ROOT, env=env, stdout=f,
                       stderr=subprocess.STDOUT, check=False)
    de._kill_ports()


def home(n=2):
    import glob
    import shutil
    import time
    t0 = time.time()
    # The child takes the heavy-run lock itself; holding it here (from lap or
    # r6 in the same process) left the child waiting on its parent for good.
    de.heavy_lock()
    r = subprocess.run([de.PY, "-m", "tests.demo_slam_home", str(n), "--tour", "1",
                        "--explore-first", "900"], cwd=ROOT,
                       env=dict(os.environ, PICAR_HEAVY_LOCK="0"),
                       capture_output=True, text=True)
    open(os.path.join(OUT, "home.out"), "w").write(r.stdout + r.stderr)
    # The full record (with `series`) is written beside each run's logs.
    found = sorted(p for p in glob.glob(os.path.join(de.LOGDIR, "run-*", "slam_home_*.json"))
                   if os.path.getmtime(p) >= t0)
    paths = []
    for i, src in enumerate(found, 1):
        p = os.path.join(OUT, f"home-run{i}.json")
        shutil.copy(src, p)
        paths.append(p)
    if paths:
        j = subprocess.run([de.PY, "evaluations/slam-340/judge.py", *paths], cwd=ROOT,
                           capture_output=True, text=True)
        open(os.path.join(OUT, "home-judge.out"), "w").write(j.stdout + j.stderr)


def main(which):
    os.makedirs(OUT, exist_ok=True)
    if "lap" in which:
        run_module("tests.demo_slam_lap", "starter_house", "r5-drift.out", drift="1.0,1.03")
    if "r6" in which:
        run_module("tests.demo_nav_goals", "scaled_house", "r6.out")
    if "home" in which:
        home()


if __name__ == "__main__":
    main(sys.argv[1:] or ["lap", "r6", "home"])
