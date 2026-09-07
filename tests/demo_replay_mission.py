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

    # ...and the same walk under the tiered policy (P2), which runs YOLO
    # and CLIP locally over every frame and calls the model only on a
    # trigger. Needs `pip install -r requirements-perception.txt`.
    python -m tests.demo_replay_mission <walk> "red backpack" --policy tiered

Recording a walk: open the twin's Guide tab, choose Robot view, connect
the brain service in Settings, switch on "Record this walk", and walk your
house holding the phone low -- about 10cm, the PiCar-X camera's height. The
frames land in the brain's `recording_dir`.

## The tiered policy, and why a replay is a fair test of it

`--policy tiered` is the same mission with `brain/tiered.py` in front of
the vision call: perception runs locally on every frame and `/navigate` is
asked only on `mission_start`, `candidate_sighting` or `cold_search`. It
prints the trigger beside each step and the deliberation counter at the
end.

**A replay is open loop, and for the trigger discipline that does not
matter.** The usual caveat is that a LEFT at frame 12 does not change what
frame 13 shows -- which invalidates a *navigation* result. The thing being
measured here is how often a policy decides to ask, given a sequence of
perceptions, and that sequence is real either way. It is the same argument
4.10 makes for running the detector on recorded frames: *"a detector does
not care whether the next frame was caused by its own decision."*

What a replay still cannot tell you is throughput (2.9 -- a laptop is not
an 8L) and whether the saving holds on a walk the policy itself drove.

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
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    policy = "vision"
    if "--policy" in sys.argv:
        policy = sys.argv[sys.argv.index("--policy") + 1]
        args = [a for a in args if a != policy]
    # The CLIP threshold, for calibrating it on a rig walk -- which is the
    # only way the shippable number gets found. See brain/perceive.py's
    # DEFAULT_MATCH_MARGIN, which is provisional and measured too high.
    margin = None
    if "--margin" in sys.argv:
        raw = sys.argv[sys.argv.index("--margin") + 1]
        margin = float(raw)
        args = [a for a in args if a != raw]
    if len(args) != 2 or policy not in ("vision", "tiered"):
        sys.exit("Usage: python -m tests.demo_replay_mission <walk-dir> "
                 "<target object> [--policy vision|tiered]")

    walk_dir, target = Path(args[0]), args[1]
    vision_url = os.environ.get("VISION_URL")
    if not vision_url:
        sys.exit("Set VISION_URL to the deployed vision service's base URL.")

    robot = ReplayRobot(walk_dir)
    vision_fn = vision_fn_for(target, vision_url=vision_url)
    if policy == "tiered":
        # Built here rather than inside the runner for the same reason
        # control/brain_server.py builds it at mission start: the models
        # are an optional install, and finding that out mid-mission would
        # be counted as a vision failure and reported as the wrong cause.
        from brain.tiered import tiered_vision_fn_for

        from brain.perceive import pipeline_for

        pipeline = pipeline_for(target, **({"match_margin": margin} if margin is not None else {}))
        vision_fn = tiered_vision_fn_for(target, vision_fn, pipeline=pipeline)
        print(f"perception: {vision_fn.models['detector']} + "
              f"{vision_fn.models['scorer']}, crops via "
              f"{vision_fn.models['crop_source']}, "
              f"margin >= {vision_fn.pipeline.match_margin}")

    runner = MissionRunner(
        robot,
        target_object=target,
        policy=policy,
        vision_fn=vision_fn,
        # One step per frame is the most a replay can honestly do; a few
        # spare covers the stuck-breaker turning in place.
        max_steps=len(robot.frames) + 5,
    )

    print(f"=== {len(robot.frames)} frames from {walk_dir}, target={target!r}, "
          f"policy={policy} ===")
    print("Every step is a paid call.\n" if policy == "vision"
          else "Perception is free; only the triggered steps are paid.\n")

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
            # Under the tiered policy the interesting column is whether
            # this step cost anything -- that is the architecture's own
            # claim, one line at a time.
            tag = ""
            if status.get("perception"):
                tier = status.get("tier") or {}
                per = status["perception"]
                margin = per.get("match_margin")
                tag = (f"  <{per['status']}"
                       + (f" {margin:+.3f}" if margin is not None else "")
                       + (f" | CLOUD {tier.get('trigger')}"
                          if tier.get("cloud_called") else " | local")
                       + ">")
            print(f"  {status['step']:>3}  {str(status['last_action']):<8} "
                  f"[{frame['file']}]{tag}  {status['last_reasoning']}")
    except KeyboardInterrupt:
        runner.stop("interrupted")
        print("\n-- interrupted --")

    elapsed = time.monotonic() - started
    status = runner.status()

    print(f"\nOutcome:      {status['outcome']}")
    if status.get("tier"):
        st = status["tier"]["stats"]
        seen = status["tier"]["stats"]["perception"]
        print(f"Steps:        {status['step']}   "
              f"perception {dict(sorted(seen.items()))}")
        print(f"Paid calls:   {st['cloud_calls']} over {st['frames']} frames"
              + (f"  -- 1 per {st['frames_per_call']}" if st['frames_per_call'] else "")
              + f"   ({COST_HINT})")
        print(f"Triggers:     {dict(sorted(st['triggers'].items()))}")
    else:
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
