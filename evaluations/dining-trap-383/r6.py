"""3.83 criterion 6: R6 on the scaled house, on a fresh stack of this branch,
with the keepout filter on (docs/plans/ros-alignment/3.83-dining-room-trap.md).

    eval "$(python tools/plan_section.py env)"   # run with the repo .venv python
    python evaluations/dining-trap-383/r6.py > evaluations/dining-trap-383/r6.out

Starts only when the laptop is quiet (3.79's gate), brings up
`tests/demo_explore.stack("scaled_house")`, runs `tests/demo_nav_goals.py`
against it, and prints how many keepout marks the robot server placed (a
mark in this house, which has no wedge, is a false alarm)."""
import contextlib
import io
import os
import runpy
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "evaluations" / "extra-goals-379"))

from run import wait_quiet                     # noqa: E402  3.79's quiet wait

if not os.environ.get("ROS_IMAGE") or not os.environ.get("PICAR_ROBOT_PORT"):
    raise SystemExit('run `eval "$(python tools/plan_section.py env)"` first')
print(f"load at start {wait_quiet():.2f}", flush=True)
os.environ["SIM_MAP"] = "scaled_house"
from tests import demo_explore                 # noqa: E402
try:
    err = io.StringIO()
    try:
        with contextlib.redirect_stderr(err):
            demo_explore.stack("scaled_house")
    finally:
        sys.stderr.write(err.getvalue())    # kept even when the stack fails to come up
    if "waiting for the heavy-run lock" in err.getvalue():
        # 3.79 amendment 1, as the room runner: another live run took the
        # lock after the quiet gate, so this one started under load.
        print("INVALID: waited for another live run after the quiet gate", flush=True)
    print(f"run dir {demo_explore.RUN_DIR}", flush=True)
    runpy.run_module("tests.demo_nav_goals", run_name="__main__")
finally:
    log = Path(demo_explore.RUN_DIR or "/nonexistent") / "explore_robot.log"
    marks = ([l for l in log.read_text().splitlines() if "keepout: marked" in l]
             if log.exists() else [])
    print(f"keepout marks {len(marks)}", flush=True)
    for l in marks:
        print("  " + l, flush=True)
    demo_explore._kill_ports()
