"""
demo_sim_mission.py

Run a whole mission in the grid-world simulator, with the vision model
deciding every move from rendered frames.

This is the run Stage 1's "Done when" asks for, and phase S2 is what made
it possible: until `sim/renderer.py` existed, `MockRobot` had no pixels
and the vision policy could not drive the sim at all.

Not an automated test: **every step is a real, paid call.** The step cap
below is the only thing bounding what a run costs, so it defaults low.

Usage:
    export VISION_URL="https://<your vision service>"
    export APP_SHARED_SECRET="..."          # if the deployment sets one
    export NAVIGATE_MODEL_ID="..."          # optional, from GET /navigate/models
    export NAVIGATE_PROMPT_VARIANT="..."    # optional, likewise (`prompts`)
    python -m tests.demo_sim_mission "red backpack" [max_steps]

The backend comes from `config/robot.yaml`, so `sim.sensor_noise.enabled:
true` applies here -- which is the point for M1
(`PLAN-microduck-transplants.md`): under the `bearing-only` wording the
distance sensor is the ONLY thing left refusing a move, so a run against a
noiseless one measures the easy case.

## Why this is worth running even though the sim is not a real room

Two things it can do that `demo_replay_mission.py` cannot, and they are
the two that matter most:

**It is closed loop.** A replay's frames follow the path *you* walked, so
a LEFT at frame 12 does not change what frame 13 shows -- which is why
that script's docstring says plainly that it does not measure navigation.
Here it does: the model's action moves the robot, and the next render is
what the robot would actually be looking at. This is the only closed-loop
measurement available before hardware, and the only free-to-repeat one
ever.

**The safety layer is live.** `MockRobot.get_distance()` returns a real
distance, so `robot/safety.py` can and does veto a FORWARD into a wall --
where a replay reports `NO_SENSOR_CM` and safety never fires. A run here
exercises the veto path a recorded walk structurally cannot.

**And the world knows the truth.** The summary below reports where the
robot actually ended up, from the grid, independently of whether the
model said `target_reached`. A mission that claims arrival two rooms away
is a failure this script can catch and a replay cannot.

## What it still does not tell you

The frames are flat-shaded raycaster geometry. `PLAN-sim-hardening.md`
3.5 is blunt about it: accuracy here says very little about accuracy on
photographs of a real living room. This measures the *loop* -- memory,
budgets, arrival, the veto, cost, wall clock. Stage 0's real-photo gate
measures the *seeing*. Neither substitutes for the other.
"""

import os
import sys
import time

from brain.navigate import vision_fn_for
from control.mission_runner import MissionRunner
from robot.factory import get_robot
from sim.mock_robot import MockRobot

# Each step is one paid /navigate call, so this is a budget, not a
# capability limit. The reference frontier walk to the backpack is 83
# steps; a vision policy that needs anything like that many is failing in
# a way worth stopping early to look at.
DEFAULT_MAX_STEPS = 40

COST_HINT = "check your Bedrock bill; a /navigate call is one image plus a short answer"


def main():
    if len(sys.argv) not in (2, 3):
        sys.exit('Usage: python -m tests.demo_sim_mission "<target object>" [max_steps]')

    target = sys.argv[1]
    max_steps = int(sys.argv[2]) if len(sys.argv) == 3 else DEFAULT_MAX_STEPS

    vision_url = os.environ.get("VISION_URL")
    if not vision_url:
        sys.exit("Set VISION_URL to the deployed vision service's base URL.")
    # Which model and which wording answer /navigate. Both optional, both
    # read the same way VISION_URL is, and both PRINTED below -- an
    # unattributable paid run is not a measurement, and this project has
    # already lost a week of walks to a value that was passed and ignored.
    model_id = os.environ.get("NAVIGATE_MODEL_ID") or None
    prompt_variant = os.environ.get("NAVIGATE_PROMPT_VARIANT") or None

    # Built from config/robot.yaml rather than constructed here, so
    # `sim.sensor_noise.enabled: true` and `sim.realtime` actually apply --
    # M1's whole argument is that the DISTANCE SENSOR owns clearance, and a
    # run against a noiseless sensor would be measuring the easy case. One
    # source of truth for the backend, the same one robot/server.py uses.
    # ROBOT_CONFIG_PATH, same env var robot/server.py honours, so a run can
    # point at a temp config (sensor noise on, say) without editing the
    # repo's own defaults.
    config_path = os.environ.get("ROBOT_CONFIG_PATH")
    robot = get_robot(config_path) if config_path else get_robot()
    if not isinstance(robot, MockRobot):
        sys.exit("config/robot.yaml is not in sim mode -- this script drives the "
                 "grid world.")
    world = robot.world
    runner = MissionRunner(
        robot,
        target_object=target,
        policy="vision",
        vision_fn=vision_fn_for(target, vision_url=vision_url,
                                model_id=model_id, prompt_variant=prompt_variant),
        max_steps=max_steps,
    )

    goal = [cell for cell, name in world.objects.items() if name == target]
    print(f"=== grid-world sim, target={target!r}, cap={max_steps} paid steps ===")
    print(f"Model:   {model_id or '(the vision service default)'}")
    print(f"Wording: {prompt_variant or '(the vision service default)'}")
    print(f"Sensor:  {'noisy (sim.sensor_noise on)' if robot.sensor else 'exact'}")
    print(f"Start: {(world.robot_x, world.robot_y)} facing {world.heading.name} "
          f"in the {world.room_at(world.robot_x, world.robot_y)}")
    if goal:
        print(f"Truth: {target!r} is at {goal[0]} in the "
              f"{world.room_at(*goal[0])} -- the model is not told this")
    print("Every step is a paid call. Ctrl-C stops it.\n")

    runner.start()
    started = time.monotonic()
    try:
        # Read the pose before the tick, since the tick is what moves it.
        # These are grid facts, which no policy may read -- this script is
        # the scorer, not the policy, which is the one role allowed to.
        while runner.is_running():
            pose = (world.robot_x, world.robot_y, world.heading.name)
            runner.tick()
            status = runner.status()
            print(f"  {status['step']:>3}  {str(status['last_action']):<8} "
                  f"[{pose[0]},{pose[1]} {pose[2]}]  {status['last_reasoning']}")
    except KeyboardInterrupt:
        runner.stop("interrupted")
        print("\n-- interrupted --")

    elapsed = time.monotonic() - started
    status = runner.status()
    ended = (world.robot_x, world.robot_y)

    print(f"\nOutcome:      {status['outcome']}")
    print(f"Steps/calls:  {status['step']}  ({COST_HINT})")
    print(f"Wall clock:   {elapsed:.1f}s  ({elapsed / max(status['step'], 1):.1f}s per step)")
    print(f"Ended at:     {ended} in the {world.room_at(*ended)}")
    if status["error"]:
        print(f"Error:        {status['error']}")

    # The model's own claim, and the world's answer to it. These agreeing
    # is the result; them disagreeing is the more interesting result.
    if status["found"]:
        print(f"Model said:   arrived at step {status['sighting']['step']}")
    else:
        print("Model said:   never reported target_reached")
    if goal:
        gx, gy = goal[0]
        distance = abs(gx - ended[0]) + abs(gy - ended[1])
        verdict = "yes" if distance <= 1 else f"NO -- {distance} cells away"
        print(f"Actually:     {verdict}")

    print("\nRead the reasoning column against the actions. The failure modes")
    print("worth naming (PLAN-sim-hardening.md's 3x3 matrix found all three):")
    print("  - the same action every step, whatever is in shot;")
    print("  - never FORWARD, turning on the spot with the target centred;")
    print("  - always FORWARD, into walls, with safety doing all the work.")


if __name__ == "__main__":
    main()
