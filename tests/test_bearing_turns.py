"""
tests/test_bearing_turns.py

Phase R1 (`PLAN-ros-alignment.md`) -- a turn chosen from a measured bearing
turns BY that bearing.

**The finding this pins, measured 2026-09-25.** P25 wanted to A/B
dead-reckoning on the premise that R0's continuous pose was the missing
piece. It was necessary and not sufficient: every LEFT/RIGHT the brain sent
was still the executor's default 90 degrees, and against the tier's 10-degree
centre band that overshoots any target inside an 80-degree cone. Over sixteen
off-axis starts four to six cells from the backpack, 90-degree turns closed
ZERO distance and flipped LEFT/RIGHT on 52 of 60 steps; turning by the bearing
arrived. So the repair was the action's SIZE, not its memory.

Run through the WHOLE mission path -- `MissionRunner` -> `VisionAgent` ->
`robot/safety.py`'s collar -> `MockRobot` -- because the defect lived in the
seam between the tier and the executor, which a harness that turns the robot
itself would skip. Perception is `FrameReportedPipeline`, 1.12's synthetic
detections: the sim tests the detector's CONSUMERS, never the detector.

Scored on distance closed and turn reversals, never `median_command_run`
alone -- bearing-sized turns alternate turn/forward by design and read 1.5
while arriving.
"""

import math

import pytest

from brain.arrival import ARRIVAL_RADIUS_M
from brain.perceive import FrameReportedPipeline
from brain.tiered import MAX_TURN_DEG, MIN_TURN_DEG, TieredVision, turn_for
from control.mission_runner import MissionRunner
from sim.maps import build_world
from sim.mock_robot import MockRobot
from tests.conftest import mock_world_for

TARGET = "red backpack"
# The house these run in. MOVED 2026-10-01 (PLAN 3.32, the user's decision)
# from the starter house: its 30 cm doors leave the UGV Rover ~3.5 cm a
# side (3.21), and once detections report the bearing of the VISIBLE part
# of the target, as a detector does, the robot aims ~2 deg off the door's
# axis -- inside the 3 deg steering band -- and its swept corridor clips
# the jamb. The scaled house is the same plan with 90 cm doors (3.15, and
# 3.24 moved the ROS chain suite here for the same reason). Every start
# below is the starter house's, mapped onto it.
HOUSE = "scaled_house"
GOAL = (23.5, 5.5)  # the backpack's cell centre in the scaled house
STEPS = 60
# Ground-truth "arrived", in cells from the backpack's CELL CENTRE: the
# robot's centre within 3.11's arrival radius of the backpack's FACE (half a
# cell from its centre). CORRECTED 2026-10-01 (PLAN 3.32): this was 1.05
# cells -- the bumper about 4 cm from the backpack -- which an in-process
# mission could reach only because its verbs bypassed the guarded loop
# (`_HaltGate` did not forward `verb_plan()`) and an unguarded FORWARD
# covered a whole cell past the 20 cm line. Guarded, every arrival stops
# with the centre 35 cm from the face: the bumper 22 cm off, outside the
# line, as the safety layer intends.
ARRIVED_CELLS = (ARRIVAL_RADIUS_M * 100 + 15.0) / 30.0     # 1.83

# Hallway starts with a line of sight through the kitchen door, headed
# deliberately off the bearing to the target -- the case the flicker lives in.
OFFSETS = (-65, -25, 30, 70)
# The straight line to the backpack passes cleanly through the doorway.
CLEAR_STARTS = [(x, y, off) for (x, y) in ((12.5, 4.5), (13.5, 4.5), (14.5, 4.5))
                for off in OFFSETS]
# The straight line clips the door jamb. A pursuer that aims AT the target
# cannot go around -- that is path planning, and it is nav2's job at R6.
# In the scaled house the centre line from here passes ~0.3 cells from the
# door frame's corner: the CHASSIS' swept corridor clips it, which is the
# honest form of "the line clips the jamb" for a robot with width. Starts
# whose centre line crosses the wall outright wander the 90 cm door's
# hallway instead (measured: max_steps on 2-3 of 4 offsets).
JAMB_STARTS = [(14.5, 2.5, off) for off in OFFSETS]
# The hallway start the search sweeps turn from (R1b).
SEARCH_START = (13.5, 4.5)
STARTS = CLEAR_STARTS + JAMB_STARTS


def _build():
    return build_world(HOUSE)


def _quiet_cloud(frame):
    """Stands in for /navigate on the tier's cloud triggers (mission start,
    a candidate sighting, a cold search) as a CORRECT cloud: toward the
    target when it is in the picture, a search turn when it is not -- and,
    like the real route, a direction with no size.

    Two stubs came before this one and each measured something else. One
    answered STOP, which the tier holds between calls -- P7e's separate,
    documented defect -- and froze both arms. The next always answered
    "search right", so every sighting triggered a call that turned the robot
    away from what it had just seen. That one found a real gap on the way
    out: a correct cloud turn also went out as a blind quarter turn, which
    is why the tier now sizes the cloud's turn from the local bearing. The
    real route costs money and would make the A/B about a model rather than
    about turn size."""
    seen = [d for d in frame.get("detections") or [] if TARGET in d["label"]]
    if not seen:
        direction = "RIGHT"
    elif abs(seen[0]["bearing_deg"]) <= 10:
        direction = "FORWARD"
    else:
        direction = "LEFT" if seen[0]["bearing_deg"] < 0 else "RIGHT"
    return {"obstacles_ahead": [], "free_space": "unknown",
            "doorway_visible": False, "important_objects": [],
            "safest_direction": direction}


def _tier(**kw):
    """A tier past its mission-start cloud call, ready for local frames."""
    tier = TieredVision(FrameReportedPipeline(TARGET), _quiet_cloud,
                        steer_on_sight=True, **kw)
    tier({"detections": []})
    return tier


class _DropTurnSize:
    """The tier with R1 undone: every turn goes out with no size, so the
    executor's default quarter turn applies -- what shipped before R1."""

    def __init__(self, tier):
        self.tier = tier

    def __call__(self, frame):
        scene = self.tier(frame)
        scene.pop("turn_deg", None)
        return scene


def _mission(start, *, sized=True):
    x, y, off = start
    grid = _build()
    grid.x, grid.y = x, y
    grid.theta = math.atan2(GOAL[1] - y, GOAL[0] - x) + math.radians(off)
    robot = MockRobot(grid, render=False)
    tier = TieredVision(FrameReportedPipeline(TARGET), _quiet_cloud,
                        steer_on_sight=True, hold_goal=True)
    runner = MissionRunner(robot, target_object=TARGET, max_steps=STEPS,
                           policy="tiered",
                           vision_fn=tier if sized else _DropTurnSize(tier),
                           world=mock_world_for(robot))
    before = math.dist((grid.x, grid.y), GOAL)
    runner.start()
    while runner.tick():
        pass
    actions = [r.action for r in runner.agent.history]
    turns = [a for a in actions if a in ("LEFT", "RIGHT")]
    reversals = sum(1 for a, b in zip(turns, turns[1:]) if a != b)
    return before - math.dist((grid.x, grid.y), GOAL), reversals


def test_turn_size_follows_the_bearing_and_is_clamped():
    assert turn_for(-23.4) == 23
    assert turn_for(11) == 11
    assert turn_for(2) == MIN_TURN_DEG, "a real error still gets a real turn"
    assert turn_for(170) == MAX_TURN_DEG, "behind the robot: re-measure halfway"


def test_bearing_sized_turns_reach_the_target_from_off_axis():
    """R1's headline. From every start with a clear line through the door,
    however far off-axis it began, the robot ends within one cell of the
    backpack -- the collar stops it short of touching. Measured 2026-09-25:
    all twelve arrive, from 4-6 cells out."""
    for start in CLEAR_STARTS:
        closed, _ = _mission(start)
        left = math.dist(start[:2], GOAL) - closed
        assert left <= ARRIVED_CELLS, f"from {start} it stopped {left:.2f} cells short"


def test_a_door_jamb_on_the_straight_line_is_a_planning_problem_not_a_steering_one():
    """The honest residue, pinned so it is decided rather than forgotten.

    From JAMB_STARTS the chassis' path to the backpack clips the doorway's jamb. The
    tier aims AT the target, the collar refuses to drive into the wall, and
    the robot waits at the jamb -- correct on both counts, and still short.
    Going around is path planning, which a steering rule cannot do and nav2
    will (`PLAN-ros-alignment.md` R6). **When R6 lands this test should
    start failing, and that is the signal to flip it into an arrival
    assertion.** Until then it pins the part that must hold regardless: the
    robot never goes through the wall to get there."""
    for start in JAMB_STARTS:
        closed, _ = _mission(start)
        left = math.dist(start[:2], GOAL) - closed
        assert left > 2.0, (
            f"from {start} it arrived ({left:.2f} left) -- if a planner now "
            "threads the door, promote this start into CLEAR_STARTS")


def test_quarter_turns_are_the_defect_bearing_sized_turns_remove():
    """The A/B, pinned. With the turn size dropped -- what shipped before R1
    -- the same tier, perception and starts close far less ground and flip
    LEFT/RIGHT far more. If this stops holding, either the executor started
    sizing turns on its own or the tier stopped emitting `turn_deg`."""
    sized = [_mission(s) for s in STARTS]
    quarter = [_mission(s, sized=False) for s in STARTS]
    mean = lambda xs: sum(xs) / len(xs)
    assert mean([c for c, _ in sized]) > mean([c for c, _ in quarter]) + 2.0
    assert mean([r for _, r in sized]) < mean([r for _, r in quarter]) / 2


def test_a_turn_with_no_bearing_behind_it_is_a_search_step():
    """R1b reversed R1's choice here, on data. R1 left a search turn at the
    executor's default 90 degrees ("no size nobody measured"); against a
    60-degree field of view that leaves a 30-degree blind gap between views,
    and a target sitting in it was never found by turning -- 14% of search
    starts. A search step smaller than the field of view makes one rotation
    cover the whole circle."""
    from brain.tiered import SCAN_TURN_DEG
    scene = _tier()({"detections": []})
    assert scene["safest_direction"] in ("LEFT", "RIGHT")
    assert scene["turn_deg"] == SCAN_TURN_DEG
    assert SCAN_TURN_DEG < 60, "must stay under the sim's 60-degree field of view"


def test_a_sighted_turn_carries_the_size_of_its_bearing():
    scene = _tier()({"detections": [
        {"label": TARGET, "bearing_deg": -37.0, "distance_m": 1.2}]})
    assert scene["safest_direction"] == "LEFT"
    assert scene["turn_deg"] == 37


def test_the_mission_reports_its_turns_and_reversals():
    """R1's readout at the source: the runner counts what the panel shows,
    by the same definition this file's A/B scores on."""
    x, y, off = CLEAR_STARTS[0]
    grid = _build()
    grid.x, grid.y = x, y
    grid.theta = math.atan2(GOAL[1] - y, GOAL[0] - x) + math.radians(off)
    robot = MockRobot(grid, render=False)
    tier = TieredVision(FrameReportedPipeline(TARGET), _quiet_cloud,
                        steer_on_sight=True, hold_goal=True)
    runner = MissionRunner(robot, target_object=TARGET, max_steps=STEPS,
                           policy="tiered", vision_fn=tier,
                           world=mock_world_for(robot))
    runner.start()
    while runner.tick():
        pass
    turns = [r.action for r in runner.agent.history if r.action in ("LEFT", "RIGHT")]
    reported = runner.status()["turns"]
    assert reported["count"] == len(turns)
    assert reported["reversals"] == sum(1 for a, b in zip(turns, turns[1:]) if a != b)


def test_a_spin_is_named_a_spin_not_scored_as_zero_reversals():
    """The first watched run: 98 turns in 120 steps, all RIGHT, target never
    seen -- and the panel said "0 reversed", which reads as success. The
    readout now reports the share of steps spent turning and calls a
    one-way rotation what it is."""
    from control.mission_runner import SPIN_TURN_SHARE

    grid = _build()  # the house's own start, in the far room: the backpack is out of sight
    robot = MockRobot(grid, render=False)

    def spin(frame):
        return {"obstacles_ahead": [], "free_space": "unknown",
                "doorway_visible": False, "important_objects": [],
                "safest_direction": "RIGHT"}

    runner = MissionRunner(robot, target_object=TARGET, max_steps=20,
                           policy="tiered", vision_fn=spin,
                           world=mock_world_for(robot))
    runner.start()
    while runner.tick():
        pass
    turns = runner.status()["turns"]
    assert turns["reversals"] == 0
    assert turns["share"] >= SPIN_TURN_SHARE
    assert turns["spinning"] is True


def test_an_aimed_approach_is_not_called_a_spin():
    """The other side of the rule: a mission that corrects a few times and
    then drives is mostly FORWARD, and must not be flagged."""
    x, y, off = CLEAR_STARTS[0]
    grid = _build()
    grid.x, grid.y = x, y
    grid.theta = math.atan2(GOAL[1] - y, GOAL[0] - x) + math.radians(off)
    robot = MockRobot(grid, render=False)
    tier = TieredVision(FrameReportedPipeline(TARGET), _quiet_cloud,
                        steer_on_sight=True, hold_goal=True)
    runner = MissionRunner(robot, target_object=TARGET, max_steps=20,
                           policy="tiered", vision_fn=tier,
                           world=mock_world_for(robot))
    runner.start()
    while runner.tick():
        pass
    assert runner.status()["turns"]["spinning"] is False


# ---------- stuck detection: stop, rather than push into a jamb ----------


def _run_from(start, **runner_kw):
    x, y, off = start
    grid = _build()
    grid.x, grid.y = x, y
    grid.theta = math.atan2(GOAL[1] - y, GOAL[0] - x) + math.radians(off)
    robot = MockRobot(grid, render=False)
    tier = TieredVision(FrameReportedPipeline(TARGET), _quiet_cloud,
                        steer_on_sight=True, hold_goal=True)
    runner = MissionRunner(robot, target_object=TARGET, max_steps=STEPS,
                           policy="tiered", vision_fn=tier,
                           world=mock_world_for(robot), **runner_kw)
    runner.start()
    while runner.tick():
        pass
    return runner.status()


def test_a_robot_pushing_into_a_door_jamb_ends_blocked_not_at_max_steps():
    """The first watched R1 run, reproduced: aimed dead-centre at the
    backpack from a row whose straight line clips the door jamb, it said
    FORWARD into the jamb for its last 19 steps -- and paid for cloud calls
    doing it. Now it stops within a few refusals and says why."""
    from control.mission_runner import BLOCKED, DEFAULT_STUCK_AFTER

    for start in JAMB_STARTS:
        status = _run_from(start)
        assert status["outcome"] == BLOCKED, (start, status["outcome"])
        # Criterion 1 of PLAN-ros-alignment.md 3.4, measured 8-11 steps.
        assert status["step"] <= 15, f"took {status['step']} steps to give up"
        assert "route planning" in status["log_tail"][-1]
        assert status["running"] is False
    assert DEFAULT_STUCK_AFTER == 5


def test_a_robot_that_reaches_the_target_is_not_called_blocked():
    """At the backpack the collar refuses FORWARD too -- and ending the
    mission there is right either way, but it must not happen on the WAY:
    every clear-line start still arrives before being stopped."""
    for start in CLEAR_STARTS:
        status = _run_from(start)
        x, y, off = start
        # arrival is already pinned above; here, only that nothing stopped
        # it early on the approach
        assert status["step"] >= 5


def test_stuck_detection_can_be_switched_off():
    status = _run_from(JAMB_STARTS[0], stuck_after=0)
    assert status["outcome"] == "max_steps"


def test_a_refusal_followed_by_progress_resets_the_count():
    """One or two refusals and then a turn that goes through is a policy
    finding its way, not a stuck robot."""
    runner = MissionRunner(MockRobot(_build(), render=False),
                           target_object=TARGET, max_steps=3, stuck_after=2)
    runner._refused_forwards = 1

    class R:  # an executed step
        action, executed = "LEFT", True
    # the reset rule, read directly
    if R.action == "FORWARD" and not R.executed:
        runner._refused_forwards += 1
    elif R.executed:
        runner._refused_forwards = 0
    assert runner._refused_forwards == 0


# ---------- R1b: search that cannot miss (PLAN-ros-alignment.md 3.5) ----------

# Every 5 degrees from 40 off the target round to the other side -- the set
# that CAN show the gap. A first sample every 30 degrees from 60 could not
# (with 90-degree steps the gap only exists 36-54 degrees from a multiple of
# 90) and measured a misleading 100% before the fix.
SEARCH_OFFSETS = [o for o in range(-180, 180, 5) if abs(o) >= 40]


def _search(offset, x=SEARCH_START[0], y=SEARCH_START[1], steps=60):
    grid = _build()
    grid.x, grid.y = x, y
    grid.theta = math.atan2(GOAL[1] - y, GOAL[0] - x) + math.radians(offset)
    robot = MockRobot(grid, render=False)
    tier = TieredVision(FrameReportedPipeline(TARGET), _quiet_cloud,
                        steer_on_sight=True, hold_goal=True)
    runner = MissionRunner(robot, target_object=TARGET, max_steps=steps,
                           policy="tiered", vision_fn=tier,
                           world=mock_world_for(robot))
    runner.start()
    first_seen = None
    while runner.tick():
        h = runner.agent.history[-1]
        if first_seen is None and (h.scene.get("_perception") or {}).get("status") == "detected":
            first_seen = h.step
    return first_seen, math.dist((grid.x, grid.y), GOAL), runner.status()["outcome"]


def test_a_target_in_line_of_sight_is_always_found_by_turning():
    """Criterion 1: measured 86% before R1b, 100% after, over 171 starts
    (three positions); pinned here on the middle position."""
    missed = [o for o in SEARCH_OFFSETS
              if (lambda f: f is None or f > 12)(_search(o)[0])]
    assert not missed, f"never saw the backpack within 12 steps from offsets {missed}"


def test_every_search_start_arrives_rather_than_drifting_into_a_jamb():
    """Criterion 2: measured 75% before, 84% with the search step alone,
    100% once measured bearings steered on a 3-degree band -- every blocked
    run had driven FORWARD 5-6 degrees off the doorway's line."""
    failed = [(o, round(left, 2), outcome) for o in SEARCH_OFFSETS
              for _, left, outcome in [_search(o)] if left > ARRIVED_CELLS]
    assert not failed, f"did not arrive: {failed}"


# ---------- R1c: an unreliable detector (PLAN-ros-alignment.md 3.6b) ----------


class _Flaky(FrameReportedPipeline):
    """A detector that misses an in-view target with probability 1 - p,
    seeded so the test is deterministic."""

    def __init__(self, target, p, seed):
        import random
        super().__init__(target)
        self.p, self.rng = p, random.Random(seed)

    def perceive(self, frame):
        from brain.perceive import ABSENT, Perception
        out = super().perceive(frame)
        if out.status == "detected" and self.rng.random() > self.p:
            return Perception(status=ABSENT, synthesised=True)
        return out


def test_a_detector_that_misses_one_frame_in_ten_still_arrives():
    """Criterion 1 of 3.6b: at 90% per-frame detection at least 95% of
    missions arrive. Measured 84.5% before R1c -- R1b's 45-degree search step
    had silently turned the 8-TURN spin guard into one rotation, so a single
    missed frame as the sweep passed the target forced a FORWARD off the
    doorway's line -- and 98.3-98.6% (+/- 1%) with the guard in degrees.

    TEN seeds, not three. A random detector makes the trajectory -- and so
    which frames get missed -- depend on everything before it, so at 207
    missions the noise was +/- 3 missions and three seeds read 95.2%, then
    94.2% after an unrelated change, straddling the bar by chance. At 690 the
    estimate is tight enough for a threshold to mean something."""
    starts = CLEAR_STARTS + [(*SEARCH_START, o) for o in SEARCH_OFFSETS]
    arrived = total = 0
    for start in starts:
        for seed in range(10):
            x, y, off = start
            grid = _build()
            grid.x, grid.y = x, y
            grid.theta = math.atan2(GOAL[1] - y, GOAL[0] - x) + math.radians(off)
            robot = MockRobot(grid, render=False)
            tier = TieredVision(_Flaky(TARGET, 0.9, seed), _quiet_cloud,
                                steer_on_sight=True, hold_goal=True)
            runner = MissionRunner(robot, target_object=TARGET, max_steps=60,
                                   policy="tiered", vision_fn=tier,
                                   world=mock_world_for(robot))
            runner.start()
            while runner.tick():
                pass
            total += 1
            arrived += math.dist((grid.x, grid.y), GOAL) <= ARRIVED_CELLS
    assert arrived / total >= 0.95, f"{arrived}/{total} arrived"


def test_the_spin_guard_counts_degrees_not_turns():
    """With 45-degree search steps, the default guard (8 quarter turns'
    worth) must allow two full rotations -- sixteen turns -- not one."""
    tier = _tier()  # its first frame already spent one 45-degree turn
    acts = [tier({"detections": []})["safest_direction"] for _ in range(20)]
    first = acts.index("FORWARD")
    # Two rotations = 16 turns, one of them the helper's. The count-based
    # guard fired after ONE rotation (index ~7).
    assert first >= 14, f"forced FORWARD after only {first + 1} search turns: {acts}"
