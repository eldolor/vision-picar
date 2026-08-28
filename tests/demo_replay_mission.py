"""
demo_replay_mission.py

Run a whole mission, with the vision model deciding every move, against a
walk you recorded in the twin's Robot view.

This is the first thing in the project that puts the *loop* on real
pixels. Stage 0's `manual_replay_navigate.py` asks the model about
photographs one at a time and prints what it said; this runs the actual
agent -- mission memory, the step budget, the vision-failure budget, the
arrival signal, the safety layer -- over the same frames, and reports what
a mission costs and how it ended.

Not an automated test: **every step is a real, paid call.** A 40-frame
walk is 40 calls.

Usage:
    export VISION_URL="https://<your vision service>"
    export APP_SHARED_SECRET="..."          # if the deployment sets one
    python -m tests.demo_replay_mission recordings/backpack-20260827-141233 "red backpack"

Recording a walk: open the twin's Guide tab, choose Robot view, connect
the brain service in Settings, switch on "Record this walk", and walk your
house holding the phone low -- about 10cm, the PiCar-X camera's height. The
frames land in the brain's `recording_dir`.

## What a run does and does not tell you

It measures the loop honestly: how many calls a mission takes, what it
costs, whether the arrival signal fires, and whether memory and the budgets
behave. It does **not** measure navigation -- the frames follow the path
*you* walked, so a LEFT at frame 12 does not change what frame 13 shows.
See `sim/replay_robot.py` for the full caveat, including why the safety
layer is inert here.
"""

import os
import sys
import time
from pathlib import Path

from brain.navigate import vision_fn_for
from control.mission_runner import MissionRunner
from sim.replay_robot import ReplayRobot

# Bedrock pricing for the model the vision service uses, per million
# tokens. Only ever an estimate -- the real number is in your bill -- but
# an order of magnitude is what the cost question actually needs.
COST_HINT = "check your Bedrock bill; a /navigate call is one image plus a short answer"


def main():
    if len(sys.argv) != 3:
        sys.exit("Usage: python -m tests.demo_replay_mission <walk-dir> <target object>")

    walk_dir, target = Path(sys.argv[1]), sys.argv[2]
    vision_url = os.environ.get("VISION_URL")
    if not vision_url:
        sys.exit("Set VISION_URL to the deployed vision service's base URL.")

    robot = ReplayRobot(walk_dir)
    runner = MissionRunner(
        robot,
        target_object=target,
        policy="vision",
        vision_fn=vision_fn_for(target, vision_url=vision_url),
        # One step per frame is the most a replay can honestly do; a few
        # spare covers the stuck-breaker turning in place.
        max_steps=len(robot.frames) + 5,
    )

    print(f"=== {len(robot.frames)} frames from {walk_dir}, target={target!r} ===")
    print("Every step is a paid call. Ctrl-C stops it.\n")

    runner.start()
    started = time.monotonic()
    try:
        # Looping on is_running() rather than on tick()'s return, so the
        # step that ends the mission still gets printed -- that is the one
        # you most want to read. The frame is read before the tick, because
        # afterwards the walk has already advanced past it.
        while runner.is_running():
            frame = robot.get_camera_frame()["metadata"]
            runner.tick()
            status = runner.status()
            print(f"  {status['step']:>3}  {str(status['last_action']):<8} "
                  f"[{frame['file']}]  {status['last_reasoning']}")
    except KeyboardInterrupt:
        runner.stop("interrupted")
        print("\n-- interrupted --")

    elapsed = time.monotonic() - started
    status = runner.status()

    print(f"\nOutcome:      {status['outcome']}")
    print(f"Steps/calls:  {status['step']}  ({COST_HINT})")
    print(f"Wall clock:   {elapsed:.1f}s  ({elapsed / max(status['step'], 1):.1f}s per step)")
    print(f"Frames used:  {robot.index + 1} of {len(robot.frames)}"
          f"{'  -- ran off the end' if robot.exhausted else ''}")
    if status["error"]:
        print(f"Error:        {status['error']}")
    if status["found"]:
        print(f"Arrived:      step {status['sighting']['step']} -- the model reported "
              f"target_reached")
    else:
        print("Arrived:      no -- the model never reported target_reached")

    print("\nJudge it yourself: read the reasoning column against the frames.")
    print("A run where every action is the same, or where the actions ignore")
    print("what is in shot, is a failure even if the mission 'completed'.")


if __name__ == "__main__":
    main()
