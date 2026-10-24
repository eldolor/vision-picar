"""
mission_runner.py

Phase B1 -- turn the mission loop inside out.

`ObjectSearchAgent.run_mission()` is a blocking `for` loop: it returns
only when the mission is over, so nothing outside it can start it, stop
it, or ask what it is doing. That is fine for a demo script and useless
for a service. `MissionRunner` owns the same mission -- target, step
budget, agent, memory -- but exposes it as `start()` / `tick()` /
`stop()` / `status()`, with the loop driven from outside.

**No decision logic moves.** The agent still decides; this file only
sequences it. That is deliberate: the frontier-exploration policy stays
exactly as validated in tests/demo_active_search.py, and
tests/test_mission_runner.py asserts a runner-driven mission takes the
identical number of steps as `run_mission()` does.

Phase B3.2 -- the AWS link failsafe -- also lives here, because this is
the only place that sees every vision call. The browser's autopilot logs
a /navigate error and schedules the next tick; a robot that keeps moving
while blind is the failure mode that matters on hardware. So a vision
call gets a timeout, and a run of consecutive failures ends the mission
with the robot stopped:

    vision raises/hangs  ->  STOP the car, count the failure
    N in a row           ->  STOP the car, mission ends "failed"
    any success          ->  counter resets

The timeout is enforced with a daemon thread rather than a signal or an
executor: a hung call must not be able to block interpreter exit, and
must not sit in a worker queue ahead of the next call.

One more thing the runner owes its caller: **once a mission is over, no
further movement command may reach the robot.** A tick already in flight
when stop() lands cannot be interrupted -- it is a blocking call on
another thread -- so instead the agent drives the robot through a gate
(`_HaltGate`) that refuses movement and pan commands as soon as the
mission ends. Without it, an operator's stop can be followed by the
motion command the last tick had already decided on. A command dispatched
in the microseconds *before* the stop is still possible; that residue is
what robot/server.py's watchdog (B3.1) exists to catch.
"""

import functools
import logging
import math
import re
import collections
import threading
from concurrent.futures import CancelledError
import time
from typing import Callable, Optional, Tuple

from brain.agent import ObjectSearchAgent, _takes_local_ok
from brain.arrival import SOURCE_LOCAL
from brain.explore import OWN_GOAL_TOLERANCE_M, ExploreAgent  # one tolerance for both (3.76)
from brain.choice_log import GoalAsk
from brain.goal_policy import goal_policy_for
from brain.frontier import RETRY_COOLDOWN_S, RETRY_LIMIT
from brain.inventory import MAX_RANGE_M as INVENTORY_RANGE_M
from brain.inventory import DEFAULT_FOV_DEG, Inventory, frame_detections
from brain.memory import MissionMemory
from brain.mission_report import map_coverage
from brain.navigate import CloudUnavailable
from brain.room_labels import RoomLabeller, frame_room
from brain.vision_agent import VisionAgent
from robot.interface import (DRIVER_MANUAL, PERSON_STOP, Preempted, RobotInterface,
                             StoppedByPerson, driver_priority)
from robot.safety import SafetyViolation
from world.interface import NullWorld, WorldInterface

logger = logging.getLogger("mission_runner")

# Deliberately NOT 30.0, which is exactly one grid cell.
#
# Against the noiseless sensor the two are indistinguishable: an exact
# get_distance() only ever returns a multiple of 30, so any threshold in
# (0, 30] blocks exactly the one case that matters -- a robot hard against a
# wall, reading 0.0. That is why 30.0 stood here unremarked for so long.
#
# Turn S5's sensor noise on and 30.0 becomes a coin flip. One cell of
# clearance reads 30 +/- 3cm, so the veto fires on roughly half of the
# perfectly legal moves; M1's closed-loop sim runs blocked FORWARDs at
# 28.0cm, 28.9cm and 27.7cm doing exactly this. 20.0 is 3.3 sigma clear of
# 30 and 6.7 sigma clear of 0, and it matches config/robot.yaml's
# safety.min_distance_cm, which robot/server.py has always used -- so the
# brain-side pre-check and the robot-side veto now agree on one number
# instead of two. tests/test_sensors.py pins the property.
DEFAULT_MIN_DISTANCE_CM = 20.0
# The reference backpack hunt (tests/demo_active_search.py) takes 83 steps
# with the frontier policy, so a budget of 80 would end just short of it.
# Under a vision policy every step is a paid call and this wants to come
# back down -- see PLAN-sim-hardening.md's cost note in S2b.
#
# 3.70 E4: only the fallback for a runner built with no time limit (the
# in-process harnesses). The brain always passes one, and then the step
# count is `derived_max_steps()` of it -- a runaway backstop, not a budget.
DEFAULT_MAX_STEPS = 120
# 3.70 E4 (decided by the user 2026-10-11, conflict 3): the backstop is the
# time limit at one decision per photo, a photo every 0.5 s, times two -- so
# it can fire only if the clock check itself has failed.
STEP_BACKSTOP_FRAME_S = 0.5
STEP_BACKSTOP_FACTOR = 2
# 3.70 E4: how often the battery is read (on the mission's clock), and how
# many readings in a row must sit under the floor. One low reading is a
# motor's inrush sagging the pack, not an empty battery; two, half a second
# apart, end the mission well inside E4's 2 s.
BATTERY_CHECK_S = 0.5
BATTERY_LOW_READINGS = 2
# 3.76 M2.7 (the user, 2026-10-21: "two distinct samples"): the Rover's
# board measures its battery only every 10 s (`oled_ctrl.h`
# `inaDataUpdate()`) and repeats that value in every frame between, so two
# reads half a second apart can be one sample -- one sag counted twice. A
# low reading counts again only when its value differs from the last one
# counted, or when the same value is still there past one board refresh (a
# pack that reads, say, 10.40 V twice running). 12 s: the 10 s refresh plus
# the firmware loop's slack, so a repeat is always a later sample.
BATTERY_SAMPLE_S = 12.0
DEFAULT_VISION_TIMEOUT_S = 20.0
# 3.56. The arrival confirmation's own deadline (the user's choice, "~8 s"):
# above the slowest cloud call ever recorded (6.78 s of 98), so a hanging
# cloud parks a robot at its target in about 25 s, not a minute. Never longer
# than `vision_timeout_s`.
DEFAULT_ARRIVAL_CONFIRM_TIMEOUT_S = 8.0
# The runner's guard sits this far above the confirmation's own HTTP deadline,
# so httpx ends a hang first (as CloudUnavailable) and an abandoned thread is
# the rare backstop: a quarter of the deadline, at most 2 s.
CONFIRM_GUARD_GRACE_MAX_S = 2.0
DEFAULT_MAX_VISION_FAILURES = 3

LOG_TAIL_LINES = 20

# Which policy decides the moves. "frontier" is the rule-based explorer
# (free, deterministic, not on the hardware path); "vision" hands each
# decision to the model through the caller's vision_fn; "tiered" is the
# same seam with brain/tiered.py's trigger discipline in front of it, so
# perception runs locally on every frame and the model is asked only on
# an event (PLAN-onboard-perception.md 2.4, phase P2).
POLICIES = ("frontier", "vision", "tiered", "explore")

# The two that hand the decision to a model. They differ only in what is
# bound into vision_fn -- which is the point of P2: `control/` never
# learns that perception grew a tier (2.6's first invariant), it only
# learns that this policy, like "vision", cannot run without one.
VISION_POLICIES = ("vision", "tiered")
# 3.80.G: who chooses where a `tiered` mission searches -- the cloud's
# direction (`cloud`, as before) or explore's nav2 goals (`explore`), with
# the tiers deciding identity and the last approach.
TIER_SEARCHES = ("cloud", "explore")

# 3.70 Part A: what a mission is for. `find` looks for a target (or goes to a
# room); `survey` (decided by the user 2026-10-11) has no target: explore's
# frontier loop maps the house and labels its rooms, and ends Exhausted --
# never Found, so it never asks the cloud to confirm a target.
JOBS = ("find", "survey")

# Mission outcomes. "running" is the only non-terminal one.
#
# 3.70 Part C (decided by the user 2026-10-11): a mission ENDS in one of four
# end states -- `END_STATES` -- or in a fault (or, for `room_reached`, goal)
# outcome kept beside them. docs/mission/ARCHITECTURE.md has the table.
RUNNING = "running"
IDLE = "idle"
FOUND = "found"
ROOM_REACHED = "room_reached"
FAILED = "failed"
# 3.70 E5. A person ended the mission from the phone: Stop (`cancelled_by:
# "stop"`), or a D-pad tap that took the robot from the brain (M4,
# `cancelled_by: "dpad"`). Deliberately NOT `failed`: nothing went wrong, and
# calling it a failure would put a normal human intervention in the same
# bucket as a dead AWS link. It is terminal all the same; a preempted brain
# does not get to argue. Replaces `stopped` and `preempted`.
CANCELLED = "cancelled"
# 3.70 E4. The mission's time limit or the battery floor was reached (or the
# step backstop derived from the time limit). Replaces `max_steps`.
OUT_OF_BUDGET = "out_of_budget"
# The way ahead is obstructed and the policy keeps asking to drive into it:
# `stuck_after` FORWARDs in a row refused by the safety layer. Found on the
# first watched R1 run, which aimed dead-centre at the backpack from a row
# whose straight line clips the kitchen door jamb, then spent its last 19
# steps -- and several paid cloud calls -- saying FORWARD into the jamb until
# the step budget ran out. Not `failed` (nothing broke; the collar did its
# job) and not `max_steps` (it was not still trying). Going AROUND is route
# planning, which is nav2's job at R6 -- and nav2 reports an unreachable goal
# the same way, which is why this is an outcome rather than a recovery.
BLOCKED = "blocked"
DEFAULT_STUCK_AFTER = 5
# 3.31. The `explore` policy ran out of places to look: no reachable frontier
# left, nothing waiting out a cooldown and (3.70 E3) no labelled room left
# unentered. Not `blocked` -- it is a finding about the house ("looked
# everywhere it could reach; not there"), and a target that is absent should
# end here, never in a refusal. Was `searched` until 3.70 E1.
EXHAUSTED = "exhausted"
# 3.47. The robot reached what the local tier took for the target (the lidar
# arrival rule held) but the cloud never ANSWERED the identity question --
# B3.2's budget ran out on an unreachable cloud. Not `found`: since handoff
# 1a only a cloud yes makes `found`, and a local false positive looks exactly
# like this. Not `failed`: the robot did its part and is parked at it. A
# cloud that answers "no" is a refusal and never ends here.
ARRIVED_UNCONFIRMED = "arrived_unconfirmed"

# 3.70: the four end states, in the order the phone lists them. Every other
# terminal outcome is a fault (`failed`, `blocked`, `arrived_unconfirmed`) or
# a goal reached (`room_reached`), shown in its own style; `end_state` is None
# for those. `arrived_unconfirmed` is never Found.
END_STATES = (FOUND, EXHAUSTED, OUT_OF_BUDGET, CANCELLED)
# The end states that carry a "where it looked" report (E3-E5).
REPORTED = (EXHAUSTED, OUT_OF_BUDGET, CANCELLED)

# 3.70 E1: the names written before 3.70, and what each is read as. Old
# records (mission logs, metrics rows, run JSON under `evaluations/`) are
# READ through `read_outcome()`, never rewritten.
_LEGACY_OUTCOMES = {
    "searched": (EXHAUSTED, None),
    "max_steps": (OUT_OF_BUDGET, None),
    "stopped": (CANCELLED, "stop"),
    "preempted": (CANCELLED, "dpad"),
}


def read_outcome(outcome: Optional[str],
                 cancelled_by: Optional[str] = None) -> tuple:
    """An outcome from any record, old or new, as `(outcome, cancelled_by)`
    in today's names. A record's own `cancelled_by` wins over the one a
    legacy name implies."""
    new, implied = _LEGACY_OUTCOMES.get(outcome, (outcome, None))
    return new, (cancelled_by or implied) if new == CANCELLED else None


# robot/server.py's refusal names the holder first: "<holder> is driving -- ...".
# "X is driving" (arbitration) or "X took over" (a verb cut off under
# drive: ros, 3.76 T1): X is who holds the robot now.
_PREEMPTED_BY = re.compile(r"^(\S+) (?:is driving|took over)\b")


def derived_max_steps(time_limit_s: Optional[float]) -> int:
    """3.70 E4's step backstop: `time_limit_s / 0.5 s x 2` (a 30 min limit
    gives 7,200). With no time limit, `DEFAULT_MAX_STEPS`."""
    if not time_limit_s or time_limit_s <= 0 or not math.isfinite(time_limit_s):
        return DEFAULT_MAX_STEPS
    return int(round(time_limit_s / STEP_BACKSTOP_FRAME_S * STEP_BACKSTOP_FACTOR))


def _caused_by(error: BaseException, kind: type) -> bool:
    """Is `kind` anywhere in the chain of EXPLICIT causes (`raise ... from`)?
    A wrapper added between the HTTP call and the runner changes nothing.
    `__context__` is not followed: a local error raised while a cloud error
    was being handled is not the cloud's."""
    seen = set()
    while error is not None and id(error) not in seen:
        if isinstance(error, kind):
            return True
        seen.add(id(error))
        error = error.__cause__
    return False


class VisionUnavailable(RuntimeError):
    """A vision call failed or timed out. Counted against the failure
    budget by tick(); see B3.2 in PLAN-brain-relocation.md."""


class LateCallInFlight(Exception):
    """`late_ask()` sent nothing: an earlier confirmation is still out
    (3.76 G1). Not a failed attempt -- control/reconfirm.py probes on and
    asks once it has ended."""


class LateCallFailed(Exception):
    """A late call that outlived its guard has since ended in an error
    (3.76 G1). Already counted as a failed attempt when it timed out; this
    carries the real error, as `__cause__`."""


class MissionHalted(RuntimeError):
    """Raised by _HaltGate when the agent tries to move a robot whose
    mission has already ended. Expected, not exceptional."""




class _GatedNavigator:
    """3.76 C1/C3/C4: the explore policy's nav2 goals, gated as `_HaltGate`
    gates its moves. No goal is sent once the mission has ended, and one
    accepted as it ended is cancelled at once. A goal refused because a
    PERSON holds the robot is a preemption, not a "wait and ask again".
    `cancel_own()` ends the mission's own goal at the mission's end -- the
    robot server spares a goal on the brain's stop (it reads one as a
    loser's teardown), and under explore the goal is the brain's -- but not
    a goal someone else has set since."""

    def __init__(self, navigator, is_running: Callable[[], bool]):
        self._nav, self._is_running = navigator, is_running
        self._own: Optional[Tuple[float, float]] = None
        # The client is closed by whichever finishes last: `end()`, or a
        # goal still on its way when the mission ended -- which must still
        # be able to cancel itself.
        self._lock = threading.Lock()
        self._in_flight = 0
        self._ending = False
        self._closed = False

    def set_goal(self, x_m: float, y_m: float) -> dict:
        with self._lock:
            if not self._is_running() or self._ending:
                return {"accepted": False, "reason": "mission_ended"}
            self._in_flight += 1
        try:
            reply = self._nav.set_goal(x_m, y_m) or {}
            if reply.get("accepted"):
                self._own = (x_m, y_m)
                if not self._is_running():
                    self.cancel_own()           # the mission ended while it was on its way
        finally:
            with self._lock:
                self._in_flight -= 1
            self._close_if_done()
        if reply.get("reason") == PERSON_STOP and not reply.get("accepted"):
            # 3.84: a person's Stop latched autonomy off; the mission ends.
            raise StoppedByPerson(reply.get("detail") or "a person pressed Stop")
        if reply.get("reason") == "preempted" and not reply.get("accepted"):
            holder = _PREEMPTED_BY.match(reply.get("detail") or "")
            if holder and driver_priority(holder.group(1)) == DRIVER_MANUAL:
                raise Preempted(reply["detail"])
        return reply

    def get_goal(self) -> dict:
        return self._nav.get_goal()

    def cancel_goal(self) -> dict:
        if not self._is_running():
            # The goal on the server may be a person's by now; the
            # mission's own is ended by cancel_own().
            return {"canceled": False, "reason": "mission_ended"}
        return self._nav.cancel_goal()

    def cancel_own(self) -> None:
        own = self._own
        if own is None:
            return
        try:
            goal = (self._nav.get_goal() or {}).get("goal") or {}
            if (goal.get("state") in ("pending", "active")
                    and math.hypot(goal.get("x_m", math.inf) - own[0],
                                   goal.get("y_m", math.inf) - own[1]) < OWN_GOAL_TOLERANCE_M):
                self._nav.cancel_goal()
        except Exception as e:  # noqa: BLE001 -- nothing on the way out may raise (3.55)
            logger.warning("could not end the mission's nav2 goal: %s", e)

    def end(self) -> None:
        """The mission ended: its own goal cancelled, then the client closed
        once no goal is on its way. Run off the brain's event loop (a Stop
        runs `_finish` there) -- both are calls through to the ROS bridge."""
        self.cancel_own()
        with self._lock:
            self._ending = True
        self._close_if_done()

    def _close_if_done(self) -> None:
        with self._lock:
            if not self._ending or self._in_flight or self._closed:
                return
            self._closed = True
        # 3.76 K5.3 (the user, 2026-10-21): the keepout marks the mission's
        # wedges left on nav2's map go with it. Here, by whichever finishes
        # last -- `end()` or a goal still on its way -- so it follows every
        # goal the mission sent, and no mark made under one is left behind.
        clear = getattr(self._nav, "clear_keepout", None)
        if clear is not None:
            try:
                clear()
            except Exception as e:  # noqa: BLE001 -- nothing on the way out may raise (3.55)
                logger.warning("could not clear the mission's keepout marks: %s", e)
        closer = getattr(self._nav, "close", None)
        if closer is not None:
            try:
                closer()
            except Exception as e:  # noqa: BLE001
                logger.warning("closing the navigator failed: %s", e)


class _HaltGate(RobotInterface):
    """Wraps the robot the agent drives, and cuts movement off the moment
    the mission ends.

    Sensing and stop() always pass through: reading a sensor after the
    mission ends is harmless, and a gate that could block a stop command
    would be worse than no gate at all.
    """

    MOVEMENT = (
        "drive_forward", "reverse", "turn_left", "turn_right",
        "look_left", "look_right", "look_center", "set_wheel_velocity",
    )

    def __init__(self, robot: RobotInterface, is_running: Callable[[], bool],
                 on_frame: Optional[Callable[[dict], None]] = None):
        self._robot = robot
        self._is_running = is_running
        # 3.46: called with every frame the agent takes, before the agent
        # sees it -- so the inventory reads the pose and scan of the moment
        # the frame was captured, not after the step's move.
        self._on_frame = on_frame

    def _guard(self, name: str):
        if not self._is_running():
            raise MissionHalted(f"mission is over -- refusing {name}")

    def drive_forward(self, speed: int = 50, duration: float = 0.5) -> dict:
        self._guard("drive_forward")
        return self._robot.drive_forward(speed, duration)

    def reverse(self, speed: int = 50, duration: float = 0.5) -> dict:
        self._guard("reverse")
        return self._robot.reverse(speed, duration)

    def turn_left(self, angle: int = 90) -> dict:
        self._guard("turn_left")
        return self._robot.turn_left(angle)

    def turn_right(self, angle: int = 90) -> dict:
        self._guard("turn_right")
        return self._robot.turn_right(angle)

    # 3.22's guarded verbs, through the gate (found 2026-10-01, PLAN 3.32):
    # without these the safety layer saw the gate's default `verb_plan()`
    # (None) and called the raw verb, so every IN-PROCESS mission ran turns
    # with no pivot vetting (3.19) and forwards checked once, not every
    # period. The deployed path was unaffected -- RemoteRobot has no plan
    # and the robot server guards its own verbs -- but the sim's mission
    # sweeps measured a weaker path than the car's. The loop's motion still
    # goes through `set_wheel_velocity()` below, so a mission that ends
    # mid-verb is still cut off.
    def verb_plan(self, action: str, speed: int = 50, duration: float = 0.5,
                  angle: int = 90):
        self._guard(action)
        return self._robot.verb_plan(action, speed=speed, duration=duration, angle=angle)

    def verb_done(self, action: str, plan: dict, outcome: dict, **kwargs) -> dict:
        return self._robot.verb_done(action, plan, outcome, **kwargs)

    @property
    def stop_count(self) -> int:
        """The body's stop count, so a verb in progress sees a stop() made
        through the gate (3.22's criterion 3)."""
        return getattr(self._robot, "stop_count", 0)

    def look_left(self) -> dict:
        self._guard("look_left")
        return self._robot.look_left()

    def look_right(self) -> dict:
        self._guard("look_right")
        return self._robot.look_right()

    def look_center(self) -> dict:
        self._guard("look_center")
        return self._robot.look_center()

    def stop(self) -> dict:
        return self._robot.stop()

    def get_camera_frame(self) -> dict:
        frame = self._robot.get_camera_frame()
        if self._on_frame is not None:
            self._on_frame(frame)
        return frame

    def get_distance(self) -> float:
        return self._robot.get_distance()

    def get_depth_grid(self) -> dict:
        # Sensing passes through, same as the two above. Inheriting
        # RobotInterface's all-unusable default here instead would make
        # every mission report that its robot had no depth sensor,
        # whatever it was actually wrapping -- which is why the gate is
        # now one of the backends in tests/test_robot_contract.py.
        return self._robot.get_depth_grid()

    def get_odometry(self) -> dict:
        # Phase B, and the third sensing method to pass through here for
        # the same reason. A gate that inherited the honest no-op would
        # report "this robot cannot measure its own motion" while wrapping
        # one that can -- and a distance-based cold-search interval would
        # then simply never fire, on every mission, silently.
        return self._robot.get_odometry()

    def get_wheel_state(self) -> dict:
        # Phase R2 -- sensing passes through, for the reason every other
        # read above does: a gate that inherited the honest no-op would
        # report "no encoders" while wrapping a robot that has them.
        return self._robot.get_wheel_state()

    def get_scan(self, max_range_m=None) -> dict:
        return self._robot.get_scan(max_range_m=max_range_m)

    def get_battery(self) -> dict:
        return self._robot.get_battery()

    def set_wheel_velocity(self, left_rad_s: float, right_rad_s: float) -> dict:
        # A MOVE, so gated like every other one: a mission that has ended
        # must not be able to start the wheels.
        self._guard("set_wheel_velocity")
        return self._robot.set_wheel_velocity(left_rad_s, right_rad_s)

    def advance(self, dt: float) -> None:
        return self._robot.advance(dt)


def call_with_timeout(fn: Callable, *args, timeout_s: Optional[float] = None,
                      on_start: Optional[Callable[[threading.Thread], None]] = None):
    """Run `fn(*args)`, raising TimeoutError if it outlasts `timeout_s`.

    The call runs on a daemon thread that is abandoned on timeout -- a
    blocking socket read cannot be interrupted from outside, and the
    alternative (leaving it in a pooled executor) would make the *next*
    call queue behind the hung one. Abandoned threads are bounded by the
    consecutive-failure budget, which ends the mission after a few.
    """
    if not timeout_s or timeout_s <= 0:
        return fn(*args)

    box: dict = {}

    def run():
        try:
            box["value"] = fn(*args)
        except BaseException as e:  # noqa: BLE001 -- re-raised on the caller's thread
            box["error"] = e

    thread = threading.Thread(target=run, daemon=True, name="vision-call")
    thread.start()
    if on_start is not None:
        on_start(thread)  # 3.56: a caller that must not start a second call beside it
    thread.join(timeout_s)
    if thread.is_alive():
        raise TimeoutError(f"call exceeded {timeout_s}s")
    if "error" in box:
        raise box["error"]
    return box["value"]


# When the Turns readout calls a mission a spin: at least this many steps in,
# at least this share of them turns, and almost none reversing the one before
# -- i.e. rotating in one direction rather than correcting. Descriptive; it
# changes no decision. The first watched R1 run was 98 turns in 120 steps
# with 0 reversals.
SPIN_MIN_STEPS = 10
SPIN_TURN_SHARE = 0.75
SPIN_MAX_REVERSAL_SHARE = 0.1


class MissionRunner:
    """One mission's lifecycle, driven a tick at a time from outside."""

    def __init__(
        self,
        robot: RobotInterface,
        target_object: Optional[str] = None,
        target_room: Optional[str] = None,
        mission: Optional[str] = None,
        max_steps: Optional[int] = None,
        min_distance_cm: float = DEFAULT_MIN_DISTANCE_CM,
        policy: str = "frontier",
        vision_fn: Optional[Callable[[dict], dict]] = None,
        vision_timeout_s: float = DEFAULT_VISION_TIMEOUT_S,
        arrival_confirm_timeout_s: float = DEFAULT_ARRIVAL_CONFIRM_TIMEOUT_S,
        max_vision_failures: int = DEFAULT_MAX_VISION_FAILURES,
        world: Optional[WorldInterface] = None,
        stuck_after: Optional[int] = DEFAULT_STUCK_AFTER,
        navigator=None,
        explore_goal_policy: str = "rule",
        tier_search: str = "cloud",
        clock: Optional[Callable[[], float]] = None,
        idle: Optional[Callable[[float], None]] = None,
        retry_cooldown_s: float = RETRY_COOLDOWN_S,
        retry_limit: int = RETRY_LIMIT,
        inventory: bool = True,
        detections_fn: Optional[Callable[[dict], Optional[list]]] = None,
        mission_time_limit_s: Optional[float] = None,
        battery_floor_v: Optional[float] = None,
        vision_max_calls: Optional[int] = None,
        job: str = "find",
        room_label_fn: Optional[Callable[[dict], Optional[str]]] = None,
        room_label_max_calls: Optional[int] = None,
        room_label_mode: str = "spacing",
        room_label_timeout_s: Optional[float] = None,
        explore_ask_timeout_s: Optional[float] = None,
        house: Optional[str] = None,
    ):
        if job not in JOBS:
            raise ValueError(f"Unknown job: {job!r}. Known: {', '.join(JOBS)}")
        if tier_search not in TIER_SEARCHES:
            raise ValueError(f"Unknown tier_search: {tier_search!r}. "
                             f"Known: {', '.join(TIER_SEARCHES)}")
        if policy not in POLICIES:
            raise ValueError(f"Unknown policy: {policy!r}. Known: {', '.join(POLICIES)}")
        if policy in VISION_POLICIES and vision_fn is None:
            # The default vision_fn is the free sensor-built scene. Running
            # the vision policy on top of it would produce a mission that
            # looks like it used the model and did not.
            raise ValueError(
                f"The {policy} policy needs a vision_fn -- see brain/navigate.py's "
                "vision_fn_for(), which binds a target and the vision service URL"
                " (and brain/tiered.py's tiered_vision_fn_for(), which wraps one)."
            )
        if job == "survey":
            if policy != "explore":
                raise ValueError("A survey runs explore's frontier loop: policy must be "
                                 f"'explore', not {policy!r}")
            if target_object or target_room:
                raise ValueError("A survey has no target: it maps the house and labels "
                                 "its rooms, and never ends Found")
        elif not target_object and not target_room:
            raise ValueError(
                "A mission needs a target_object or a target_room -- otherwise "
                "MissionMemory.is_complete() is never true and it can only end "
                "out of budget."
            )

        self.robot = robot
        self.job = job
        # 3.70 S2: a survey names rooms as it goes -- an ask every 2 m and
        # through every doorway (`spacing`), or one per room (`rooms`), at
        # most `room_label_max_calls` (brain/room_labels.py). The label
        # source defaults to the frame's own `room`, which only a simulated
        # frame carries; on the car it is the cloud (a paid call each).
        self.room_labeller = (RoomLabeller(room_label_fn or frame_room,
                                           max_calls=room_label_max_calls,
                                           mode=room_label_mode,
                                           clock=lambda: self.clock())
                              if job == "survey" else None)
        self._label_error_logged = False
        # A paid source's own timeout: an ask that could not finish inside
        # what is left of the tick's deadline waits for a later tick (review).
        self.room_label_timeout_s = room_label_timeout_s
        self._tick_began = time.monotonic()
        # WORLD state, and the first consumer `control/` has ever had for it
        # (`PLAN-mapping.md` N1 built the routes; nothing here read them).
        # Optional and defaulting to `NullWorld()` rather than None, so every
        # consumer gets the same honest `usable: False` shape instead of
        # having to branch on a missing object -- the same choice
        # `unusable_odometry()` makes one interface over.
        #
        # The body/world line is why this is a separate argument rather than
        # something read off `robot`: odometry is what the body says about
        # itself, pose is what the world says about the body, and at R5 the
        # two answers come from two different processes.
        self.world = world if world is not None else NullWorld()
        # See BLOCKED. 0 or None switches it off.
        self.stuck_after = stuck_after or 0
        self._refused_forwards = 0
        # 3.31: being stuck is retried before it is believed. A wall stays a
        # wall; a person in a doorway moves on. The clock a wait is measured
        # on comes from the robot when it has one -- a simulated body offers
        # `sim_clock()` and `pass_time()`, so an in-process mission waits on
        # sim time at no wall cost -- and is wall time otherwise, with a
        # short sleep per waiting tick (never longer than a tick interval,
        # well inside the brain server's hung-tick deadline). Duck-typed,
        # because `control/` may not import a backend.
        sim_clock = getattr(robot, "sim_clock", None)
        sim_idle = getattr(robot, "pass_time", None)
        simulated = sim_clock is not None and sim_idle is not None
        self.clock = clock or (sim_clock if simulated else time.monotonic)
        self.idle = idle or (sim_idle if simulated else time.sleep)
        self.retry_cooldown_s = retry_cooldown_s
        self.retry_limit = retry_limit
        self._stuck_episodes = 0
        self._cooldown_until: Optional[float] = None
        self._last_wait: Optional[str] = None
        # Metrics shipping. Off unless a URL is configured, which is what
        # keeps tests and laptop runs from POSTing anywhere.
        self.metrics_url = ""
        self.metrics_secret = ""
        self.metrics_config: dict = {}
        self.git_revision = ""
        # 3.46: every object the mission saw. Recorded and reported only --
        # no policy is handed it. `detections_fn` turns a frame into
        # `[{label, bearing_deg}]` or None (no detector ran); the default
        # reads what the frame carries, which today only a sim frame does.
        # `inventory_sink` is set by the brain server: called once with the
        # report when the mission ends, never allowed to fail it.
        self.inventory = Inventory() if inventory else None
        self.detections_fn = detections_fn or frame_detections
        self.inventory_sink: Optional[Callable[[dict], None]] = None
        self._inventory_error_logged = False
        self.policy = policy
        # 3.70 E4: the budget is a time limit (on `self.clock`, so sim time in
        # process) and a battery floor. The step count is only a backstop
        # derived from the time limit; an explicit `max_steps` (a harness,
        # a sweep) still sets it, and still ends Out of budget.
        if mission_time_limit_s is not None and not math.isfinite(mission_time_limit_s):
            raise ValueError("mission_time_limit_s must be a finite number of seconds")
        self.mission_time_limit_s = (mission_time_limit_s
                                     if mission_time_limit_s and mission_time_limit_s > 0
                                     else None)
        self.battery_floor_v = battery_floor_v if battery_floor_v else None
        self.max_steps = max_steps or derived_max_steps(self.mission_time_limit_s)
        # Under `vision` every step is a paid /navigate call, and a time
        # limit is not a call limit: before 3.70 the 120-step cap bounded
        # both. Counted per call SENT (a failed one may still be billed).
        # `tiered` rations its own calls (`tier_max_calls`); no other policy
        # makes a paid call per step.
        self.vision_max_calls = (vision_max_calls
                                 if policy == "vision" and vision_max_calls else None)
        self._cloud_calls = 0
        self._started_clock: Optional[float] = None
        self._ended_clock: Optional[float] = None    # elapsed stops at the end
        self._battery_checked_at: Optional[float] = None
        self._battery_v: Optional[float] = None
        self._battery_low = 0
        self._battery_low_v: Optional[float] = None     # the last low reading counted
        self._battery_low_at: Optional[float] = None    # ... and when
        self._battery_error_logged = False
        # None means "the agent's own sensor-built scene" -- resolved after
        # the agent exists, below, because that scene reads the agent's
        # SafetyController (`ConstrainedAgent.sensed_scene()`).
        self.vision_fn = vision_fn
        self.vision_timeout_s = vision_timeout_s
        self.arrival_confirm_timeout_s = arrival_confirm_timeout_s
        # 3.56: the confirmation thread last started, so a backstop timeout
        # never puts a second call to the cloud beside one still in flight.
        self._confirm_thread: Optional[threading.Thread] = None
        # 3.76 G1: the box a late call that outlived its guard answers into.
        # Its answer was paid for; the next late_ask() takes it rather than
        # paying again.
        self._late_box: Optional[dict] = None
        self.max_vision_failures = max_vision_failures
        # 3.88 A: who and where, on every recorded choice; the file is set by
        # the brain server (`choice_log`), and the records kept here as well.
        self.mission_id: Optional[str] = None
        self.house = house
        self.choice_log = None
        self.choices: list = []
        self._choice_error_logged = False

        self.memory = MissionMemory(
            mission=mission or ("Survey the house." if job == "survey"
                                else self._default_mission(target_object, target_room)),
            target_object=target_object,
            target_room=target_room,
        )
        # The agent drives a gated robot; self.robot stays the raw one so
        # stop() is never gated. The policy decides which agent class runs;
        # everything else about the mission is identical either way.
        # 3.80.G: explore's goal loop also drives a tiered mission whose
        # search is explore's; its sightings then come from the tiers'
        # perception, never from the frame's simulator detections.
        self.tier_search = tier_search if policy == "tiered" else None
        self.explores = policy == "explore" or self.tier_search == "explore"
        agent_class = (ExploreAgent if self.explores
                       else VisionAgent if policy in VISION_POLICIES else ObjectSearchAgent)
        extra = {}
        self._navigator: Optional[_GatedNavigator] = None
        if self.explores:
            if navigator is None:
                raise ValueError(f"The {policy} policy's explore search needs a navigator "
                                 "-- nav2's goals (control/remote_navigator.py), "
                                 "PLAN-ros-alignment.md 3.31")
            self._navigator = _GatedNavigator(navigator, self.is_running)
            extra = {"navigator": self._navigator, "clock": self.clock,
                     # 3.65 phase 2: "rule" (None) or the learned goal choice.
                     "goal_policy": goal_policy_for(explore_goal_policy)}
            if explore_ask_timeout_s:
                if policy != "explore":
                    # A tiered search's questions run on through a pause, and
                    # skipping them breaks its arrival (3.88 A review).
                    raise ValueError("explore_ask_timeout_s (ask mode) is for policy "
                                     f"'explore' only, not {policy!r}")
                # 3.88 A: ask mode -- each goal choice waits for a person's
                # pick (POST /mission/explore/pick), and is recorded.
                extra["ask"] = GoalAsk(clock=lambda: self.clock(),
                                       timeout_s=explore_ask_timeout_s)
                extra["choice_sink"] = self._record_choice
            if policy == "tiered":
                extra["sight_from"] = "perception"
                # 3.80.O: the agent's vision_fn is the guarded wrapper below,
                # so the tier's tracking switch is handed over by name.
                extra["local_track_fn"] = getattr(vision_fn, "set_local_track", None)
        self._gated = _HaltGate(robot, self.is_running,
                                on_frame=self._observe if self.inventory else None)
        self.agent = agent_class(
            self._gated,
            self.memory,
            **extra,
            min_distance_cm=min_distance_cm,
            vision_fn=self._guarded_vision,
            # Handoff 2026-10-02 1a: identity at arrival, under the same
            # B3.2 timeout and failure budget as every other cloud call.
            arrival_confirm_fn=self._guarded_confirm,
            world=self.world,
        )
        if self.vision_fn is None:
            # The UNGATED agent method is fine here: a scene only reads
            # sensors, and `_HaltGate` exists to stop moves, not reads.
            self.vision_fn = self.agent.sensed_scene

        self._lock = threading.RLock()
        self._running = False
        self._outcome = IDLE
        # 3.70: why it ended, who cancelled it, and where it looked.
        self._end_reason: Optional[str] = None
        self._cancelled_by: Optional[str] = None
        self._report: Optional[dict] = None
        self._last_tick_at = None
        self._ticks = 0
        self._started_at = None
        self._error: Optional[str] = None
        self._vision_failures = 0
        self._failed_arrival: Optional[dict] = None
        # 3.53: the frame that arrival was judged on, held with it, and kept
        # by _finish for a late confirmation (control/reconfirm.py).
        self._failed_arrival_frame: Optional[dict] = None
        self._arrival_frame: Optional[dict] = None
        self._late: Optional[dict] = None
        self._metrics_row: Optional[dict] = None
        self._metrics_thread: Optional[threading.Thread] = None
        self._late_ship_thread: Optional[threading.Thread] = None
        # Set as _finish's LAST act -- after the policy is closed and the
        # metrics row built -- so a late check never starts beside them.
        self.finished = threading.Event()
        # 3.70: set once `status.report` is final (built, or none is due).
        self.report_done = threading.Event()
        self._last_action: Optional[str] = None
        # R1's readout (PLAN-ros-alignment.md). How many turns the mission
        # has made, how many REVERSED the one before (LEFT straight after
        # RIGHT or the reverse -- the flicker P25 measured by hand), and how
        # far the last one was sized to. Reversals and distance closed are
        # the honest pair; `median_command_run` alone rewards a spin.
        self._turns = 0
        self._reversals = 0
        self._last_turn: Optional[str] = None
        self._last_turn_deg: Optional[int] = None
        self._last_reasoning: Optional[str] = None
        # Phase P2's readouts, and the only thing in this file that knows
        # a tier exists at all. Both are whatever the last scene carried
        # under `_tier` / `_perception` -- copied, never computed here, so
        # `control/` stays free of perception logic (2.6) and a policy
        # that publishes neither simply leaves them null. `vision` does
        # exactly that, which is what makes an absent readout mean
        # "this policy has no perception tier" rather than "it failed".
        self._tier: Optional[dict] = None
        self._perception: Optional[dict] = None
        # 3.11's arrival readout -- the range at the target's bearing and the
        # streak -- copied from the scene like the two above.
        self._arrival: Optional[dict] = None
        self._last_frame_seq: Optional[int] = None
        # 3.90 E: the decision taken on each recent frame, by the robot's
        # frame seq, for the stream recorder (control/stream_recorder.py) to
        # file beside the frame it was taken on. Bounded: a recorder more
        # than this many decisions behind files the rest as not perceived.
        self._decisions: "collections.OrderedDict[int, dict]" = collections.OrderedDict()
        self._decisions_lock = threading.Lock()
        self._last_decided = None          # (boot_id, seq) of the newest decision
        # 3.91 A: cloud answers taken at the end that no decision carried.
        self.unfiled_cloud_answers: list = []
        self._log: list = []
        self._camera_centred = False

    # ---------- lifecycle ----------

    def start(self) -> None:
        with self._lock:
            if self._running:
                raise RuntimeError("Mission already running")
            if self._outcome != IDLE:
                raise RuntimeError("A MissionRunner runs one mission; build a new one")
            self._running = True
            self._outcome = RUNNING
            self._started_at = time.monotonic()
            self._started_clock = self.clock()
            self._last_tick_at = time.monotonic()
            limits = [f"step backstop {self.max_steps}"]
            if self.mission_time_limit_s:
                limits.insert(0, f"time limit {self.mission_time_limit_s:g} s")
            if self.battery_floor_v:
                limits.append(f"battery floor {self.battery_floor_v:g} V")
            if self.vision_max_calls:
                limits.append(f"cloud call cap {self.vision_max_calls}")
            self._log_line(f"mission started: {self.memory.mission} ({', '.join(limits)})")

    def tick(self) -> bool:
        """Advance the mission by one agent step. Returns True while the
        mission is still running, so a caller can `while runner.tick():`."""
        if not self._running:
            return False
        self._tick_began = time.monotonic()

        # The step backstop is checked at the END of a tick, below, where
        # _finish() then sets _running False -- so a duplicate check here
        # could never fire, and a second copy of the rule is one that can
        # drift from the real one. Removed after coverage showed it
        # unreachable rather than untested.
        #
        # 3.70 E4: the time limit and the battery floor are checked here,
        # BEFORE a step, so a tick that starts past the limit moves nothing
        # -- and at the end as well, so the mission ends within one tick of it.
        if self._out_of_budget():
            return False
        if self._cooldown_until is not None:
            # 3.31: waiting out a stuck retry. Nothing moves; time passes.
            remaining = self._cooldown_until - self.clock()
            if remaining > 0:
                with self._lock:
                    self._last_tick_at = time.monotonic()
                self.idle(min(0.25, remaining))
                return self._running
            self._cooldown_until = None
            self._log_line(f"retrying after cooldown (episode {self._stuck_episodes} "
                           f"of {self.retry_limit})")

        try:
            if self._ticks == 0 and not self._camera_centred:
                # 3.20. A mission that ended mid-peek leaves the camera
                # panned, and this policy's first frame, depth grid and scene
                # would be cast 90 degrees off its heading. Centred here, on
                # the first tick and through the gate -- not at the end of the
                # last mission, which by then may no longer hold authority --
                # and not counted as a step: it is not a decision.
                self._camera_centred = True
                self._gated.look_center()
            result = self.agent.step()
        except MissionHalted:
            # stop()/abort() landed while this tick was in flight. The
            # gate refused the action, which is the point.
            return False
        except VisionUnavailable as e:
            return self._handle_vision_failure(e)
        except Preempted as e:
            # Phase M4. Not counted against any budget and not retried:
            # a preemption is not a transient fault, it is the answer to
            # "who is driving", and it will keep being the answer for as
            # long as the person keeps tapping. _finish() stops the car,
            # which is correct even though the robot is already doing what
            # the other driver said -- a stop from the loser of an
            # arbitration is one more command from a driver that has been
            # outranked, and the server refuses nobody's stop.
            self._finish_preempted(e)
            return False
        except Exception as e:  # noqa: BLE001
            # Anything else -- a dead robot link, a backend fault -- is a
            # reason to stop the car, not to keep looping blind.
            logger.exception("mission step failed")
            self._finish(FAILED, f"step failed: {e}")
            return False

        with self._lock:
            if not self._running:
                # stop()/abort() landed while the step was in flight.
                return False
            # Phase M5. When a tick last COMPLETED, which is what a health
            # verdict needs and what no other field here reports. `step`
            # advancing is the same fact, but only if you sample it twice
            # and know how long you waited; this is one number a health
            # check can compare against a deadline it already has.
            self._last_tick_at = time.monotonic()
            self._ticks += 1
            self._vision_failures = 0
            self._failed_arrival = None
            self._failed_arrival_frame = None
            self._last_action = result.action
            self._last_reasoning = self._describe(result)
            if self.explores:
                # The explore policy's 'why' is its goal bookkeeping (sent,
                # reached, aborted and set aside, waiting on a cooldown), not
                # the scene -- 3.31's first house runs could not be read
                # without it.
                self._last_reasoning = f"{result.detail} -- {self._last_reasoning}"
            scene = result.scene or {}
            if result.action in ("LEFT", "RIGHT"):
                self._turns += 1
                if self._last_turn and self._last_turn != result.action:
                    self._reversals += 1
                self._last_turn = result.action
                # What was actually asked for: a bearing-sized turn, or None
                # for the executor's default quarter turn (a scan, a cloud
                # answer with nothing local to size it by).
                size = scene.get("turn_deg")
                # 3.80.G: explore's own turns are not the tiers' scene's.
                self._last_turn_deg = int(size) if size and not self.explores else None
            # Held rather than overwritten with None: a tiered mission's
            # counters must survive a frame whose scene arrived from
            # somewhere else, or 6.3's "single number" would blink out
            # exactly when something unusual happened.
            self._tier = scene.get("_tier") or self._tier
            self._perception = scene.get("_perception") or self._perception
            self._arrival = scene.get("_arrival") or self._arrival
            # WHICH frame this decision was made on.
            #
            # Under teleop the twin pushes frames on its own timer and polls
            # /mission/status separately, while the mission ticks on a third
            # clock -- measured at a median of 2.5 pushed frames per step,
            # range 1-10. So the status a recorder saves beside frame N
            # routinely describes a decision taken on some earlier frame, and
            # a walk scored per-frame from walk.jsonl is scored against the
            # wrong pixels. That nearly published a false finding on
            # 2026-09-08: two corroborated sightings looked like both tiers
            # confabulating, and were in fact correct answers filed against
            # frames captured seconds later.
            #
            # sim/teleop_robot.py already stamps every pushed frame with a
            # sequence number and echoes it to the pusher, so the id exists
            # on both ends and only had to be carried through. Absent for
            # every backend that does not stamp one, which is honest: a
            # walk that cannot say which frame a decision saw should say so
            # rather than imply an alignment it does not have.
            # The decision first, then the seq that says it exists (review:
            # a recorder reading between the two filed it as unperceived).
            self._note_decision(result)
            self._last_frame_seq = (
                (result.frame or {}).get("metadata", {}).get("seq")
                if isinstance(result.frame, dict) else None
            ) or self._last_frame_seq
            if result.action != "WAIT":
                self._log_line(
                    f"step {result.step}: {result.action} "
                    f"({'ok' if result.executed else 'blocked'}) -- {self._last_reasoning}"
                )
                self._last_wait = None
            elif self._last_reasoning != self._last_wait:
                # The explore policy waits on nav2 several times a second;
                # log a wait only when what it is waiting on changes.
                self._log_line(f"waiting -- {self._last_reasoning}")
                self._last_wait = self._last_reasoning
            # Consecutive FORWARDs the safety layer refused. Any move that
            # went through resets it, so a robot that turns away and makes
            # progress is never called stuck.
            if result.action == "FORWARD" and not result.executed:
                self._refused_forwards += 1
            elif result.executed:
                self._refused_forwards = 0

        # _finish() stops the car, which is an HTTP call when the robot is
        # remote -- so decide outside the lock rather than holding it across
        # the network.
        if self.memory.is_complete():
            self._finish(FOUND if self.memory.found else ROOM_REACHED, self.memory.summary())
            return False
        if self.room_labeller is not None and self.is_running() and self._label_fits():
            # The pose now, and -- only if it asks -- a frame taken now: the
            # step's own frame was taken before its move. A label is never
            # worth a survey: any error here (the pose, the frame, the label
            # source, the map cut) skips this step's label and is logged once.
            try:
                name = self.room_labeller.observe(getattr(self.agent, "_last_map", None),
                                                  self.world.get_pose(), self._label_frame,
                                                  alive=self.is_running)
                if name is not None:
                    self._log_line(f"room label {self.room_labeller.calls}: {name}")
            except Exception:  # noqa: BLE001 -- see above
                if not self._label_error_logged:
                    logger.warning("room label skipped", exc_info=True)
                    self._label_error_logged = True
        if getattr(self.agent, "exhausted", False):
            self._finish(EXHAUSTED, self.agent.last_event or "no reachable frontier left")
            return False
        if self.stuck_after and self._refused_forwards >= self.stuck_after:
            if (self._arrival or {}).get("state") == "refused":
                # Not an obstacle: the robot reached what it was steering at
                # and the cloud said it is not the target (spec review 3,
                # fix 9). Backing off to approach the same object again would
                # only buy another paid confirmation of the same "no", so this
                # ends at once rather than through 3.31's retries -- and
                # blaming the path would point the operator at nav2.
                note = ((self._arrival.get("identity") or {}).get("reason")
                        or "the cloud did not confirm it")
                self._finish(BLOCKED, (
                    "stopped at an object whose identity the cloud did not confirm "
                    f"as the target ({note}); FORWARD then refused "
                    f"{self._refused_forwards} times by the safety layer"))
                return False
            self._stuck_episodes += 1
            if self._stuck_episodes >= self.retry_limit:
                self._finish(BLOCKED, (
                    f"FORWARD refused {self._refused_forwards} times in a row by the "
                    f"safety layer, {self._stuck_episodes} times over -- the way ahead "
                    "is obstructed. Going around it is route planning (nav2, "
                    "PLAN-ros-alignment.md R6)"))
                return False
            self._back_off()
            if not self._running:
                return False
        if result.action == "WAIT":
            self.idle(0.25)
        if self._out_of_budget():
            return False
        if len(self.agent.history) >= self.max_steps:
            self._finish(OUT_OF_BUDGET, f"step backstop of {self.max_steps} reached"
                         + (f" (derived from the {self.mission_time_limit_s:g} s time limit)"
                            if self.mission_time_limit_s else ""))
            return False
        return True

    def _elapsed_s(self) -> Optional[float]:
        if self._started_clock is None:
            return None
        end = self._ended_clock if self._ended_clock is not None else self.clock()
        return end - self._started_clock

    def _out_of_budget(self) -> bool:
        """3.70 E4: end the mission Out of budget if its time limit has
        passed or the battery has read under its floor twice in a row. True
        when it has ended (by this or anything before it)."""
        if not self._running:
            return True
        elapsed = self._elapsed_s()
        # 3.88 A: time paused on a person's pick is not the search's.
        paused = self._paused_s()
        if (self.mission_time_limit_s and elapsed is not None
                and elapsed - paused >= self.mission_time_limit_s):
            self._finish(OUT_OF_BUDGET, f"time limit of {self.mission_time_limit_s:g} s "
                         f"reached ({elapsed - paused:.1f} s"
                         + (f", plus {paused:.1f} s paused on picks)" if paused else ")"))
            return True
        if self.vision_max_calls and self._cloud_calls >= self.vision_max_calls:
            self._finish(OUT_OF_BUDGET, f"cloud call cap of {self.vision_max_calls} "
                         f"reached ({self._cloud_calls} calls)")
            return True
        if self.battery_floor_v and self._battery_spent():
            self._finish(OUT_OF_BUDGET, f"battery at {self._battery_v:.2f} V, under the "
                         f"{self.battery_floor_v:g} V floor "
                         f"{BATTERY_LOW_READINGS} readings in a row")
            return True
        return False

    def _ask(self) -> Optional[GoalAsk]:
        return getattr(self.agent, "ask", None) if self.explores else None

    def _paused_s(self) -> float:
        ask = self._ask()
        return ask.paused_s() if ask is not None else 0.0

    def pick_goal(self, choice, candidate_id) -> tuple:
        """3.88 A: a person's pick among the open choice's candidates, from
        the brain's HTTP thread. (True, "") once taken; (False, why) when no
        mission runs, it does not ask, or the id is stale or unknown -- and
        then nothing is sent. The tick thread sends the pick."""
        with self._lock:
            if not self._running:
                return False, "no mission is running"
            ask = self._ask()
            if ask is None:
                return False, "this mission does not ask for goal picks"
            return ask.pick(choice, candidate_id)

    def _record_choice(self, rec: dict) -> None:
        """One decided choice: kept, and appended to the mission's file. A
        write that fails is logged once and never fails the mission."""
        full = {"mission_id": self.mission_id, "house": self.house, **rec}
        self.choices.append(full)
        if self.choice_log is None:
            return
        try:
            self.choice_log.append(full)
        except Exception as e:  # noqa: BLE001 -- a label lost, never the mission
            if not self._choice_error_logged:
                logger.warning("goal choice not recorded: %s", e)
                self._choice_error_logged = True

    def _battery_spent(self) -> bool:
        """Read the battery at most every `BATTERY_CHECK_S` of the mission's
        clock. A body with no reading, or a read that fails, never ends a
        mission: the floor guards a battery the brain can see. Low readings
        count only as distinct samples (`BATTERY_SAMPLE_S`, M2.7)."""
        now = self.clock()
        if (self._battery_checked_at is not None
                and now - self._battery_checked_at < BATTERY_CHECK_S):
            return False
        self._battery_checked_at = now
        try:
            reading = self.robot.get_battery() or {}
        except Exception as e:  # noqa: BLE001 -- see docstring
            if not self._battery_error_logged:
                self._battery_error_logged = True
                self._log_line(f"battery unreadable: {e}")
            return False
        v = reading.get("v") if reading.get("usable") else None
        if not isinstance(v, (int, float)):
            self._battery_low = 0
            return False
        self._battery_v = float(v)
        if v >= self.battery_floor_v:
            self._battery_low = 0
            return False
        # Under the floor: a new sample, or the board's last one read again
        # (M2.7)? Only a new one adds to the count.
        if (self._battery_low == 0 or v != self._battery_low_v
                or now - self._battery_low_at >= BATTERY_SAMPLE_S):
            self._battery_low += 1
            self._battery_low_v, self._battery_low_at = float(v), now
        return self._battery_low >= BATTERY_LOW_READINGS

    def _back_off(self) -> None:
        """3.31: one vetted REVERSE to give whatever is in the way room, then
        a cooldown before the policy is asked again. A refused REVERSE is
        fine -- the cooldown is the part that lets a person move on."""
        self._log_line(
            f"FORWARD refused {self._refused_forwards} times in a row; backing off "
            f"and retrying in {self.retry_cooldown_s:.0f} s (episode "
            f"{self._stuck_episodes} of {self.retry_limit})")
        self._refused_forwards = 0
        try:
            self.agent.safety.check_and_execute("REVERSE")
        except SafetyViolation as e:
            self._log_line(f"back-off refused: {e}")
        except Preempted as e:
            self._finish_preempted(e)
            return
        except Exception as e:  # noqa: BLE001 -- same rule as a step that fails
            logger.exception("back-off failed")
            self._finish(FAILED, f"back-off failed: {e}")
            return
        self._cooldown_until = self.clock() + self.retry_cooldown_s

    def _finish_preempted(self, error: Preempted) -> None:
        """3.70 E5: a refusal for authority ends the mission Cancelled by the
        operator -- a person at the D-pad, or the phone's tap-to-goal (a nav2
        goal in progress). One that names an AUTONOMOUS holder (another
        brain, the ROS stack) is no person's doing: an arbitration fault,
        `failed`. A refusal that names no holder reads as a person, as every
        preemption did before 3.70."""
        detail = str(error)
        if isinstance(error, StoppedByPerson):
            # 3.84: the brain was refused because a person pressed Stop (the
            # D-pad's, not only the brain panel's): ended by that Stop.
            self._finish(CANCELLED, f"stopped by a person: {detail}", cancelled_by="stop")
            return
        holder = _PREEMPTED_BY.match(detail)
        if holder and driver_priority(holder.group(1)) != DRIVER_MANUAL:
            self._finish(FAILED, f"preempted by an autonomous driver: {detail}")
            return
        self._finish(CANCELLED, detail, cancelled_by="dpad")

    def stop(self, reason: str = "stopped by operator") -> None:
        """Operator stop: ends the mission Cancelled, `cancelled_by: "stop"`
        (3.70 E5). Always stops the car as well as the loop -- stopping the
        thinking is not stopping the robot."""
        self._finish(CANCELLED, reason, cancelled_by="stop")

    def abort(self, reason: str) -> None:
        """Failsafe stop (B3.3's hung-tick guard calls this). Same as
        stop(), but the mission ends failed rather than merely halted."""
        self._finish(FAILED, reason)

    def is_running(self) -> bool:
        return self._running

    # ---------- reporting ----------

    DECISIONS_KEPT = 256

    def _take_cloud_answers(self, final: bool = False) -> list:
        """3.91 A: the cloud answers the tiered policy holds, taken once.
        Any other policy has none."""
        take = getattr(getattr(self, "vision_fn", None), "take_cloud_answers", None)
        if take is None:
            return []
        try:
            return list((take(final=True) if final else take()) or [])
        except (Exception, CancelledError) as e:  # noqa: BLE001 -- a record never
            # costs a tick, nor the stop `_finish` runs it before (3.55); a
            # cancelled future's CancelledError is a BaseException (round 3)
            logger.warning("reading the cloud answers failed: %s", e)
            return []

    def _note_decision(self, result) -> None:
        """3.90 E: what was decided on this tick's frame, kept by its seq."""
        # 3.91 A: every cloud answer that landed by this decision, including
        # one collected by the arrival's wait inside this tick, and any left
        # by a tick that ended before its decision was noted. Taken here, the
        # one place rows are filed from, so a scene thrown away loses none.
        meta = ((result.frame or {}).get("metadata") or {}) if isinstance(result.frame, dict) else {}
        seq = meta.get("seq")
        if not isinstance(seq, int):
            # No frame number, no row. A body that never numbers its frames
            # has no stream recorder: taken and dropped, so the queue never
            # grows for a record nobody files (review round 3). Once a
            # numbered frame was decided, they wait for the next (round 4).
            if self._last_decided is None:
                self._take_cloud_answers()
            return
        boot = meta.get("boot_id")
        perception = self._perception or {}
        decision = {"action": result.action,
                    "reasoning": self._last_reasoning,
                    "target_visible": (perception.get("status") == "detected"
                                       if perception else None),
                    "_missionStatus": {"last_frame_seq": seq,
                                       "last_action": result.action,
                                       "perception": self._perception,
                                       "tier_trigger": (self._tier or {}).get("trigger")}}
        # A frame decided on again (the newest frame is taken, so a phone that
        # stalls repeats it) may already be filed by the recorder: it keeps
        # its first answers, and later ones wait for the next new frame
        # (review round 2). Taken once the row is built, and outside
        # `_decisions_lock`, which the recorder reads under (round 4). One
        # tick thread notes decisions, so the two reads cannot interleave.
        with self._decisions_lock:
            earlier = self._decisions.get((boot, seq))
        answers = (earlier.get("cloud_answers") if earlier is not None
                   else self._take_cloud_answers())
        with self._decisions_lock:
            if answers:
                decision["cloud_answers"] = answers
            self._decisions[(boot, seq)] = decision
            self._last_decided = (boot, seq)
            while len(self._decisions) > self.DECISIONS_KEPT:
                self._decisions.popitem(last=False)

    def decision_for(self, seq: int, boot_id=None) -> Optional[dict]:
        """3.90 E: the decision taken on the robot's frame `seq` of the
        body's life `boot_id`, or None if no tick took that frame (or it has
        aged out). Keyed by both, so a restarted body's seq 1 is never
        another life's seq 1 (review)."""
        with self._decisions_lock:
            return self._decisions.get((boot_id, seq))

    def decided_past(self, seq: int, boot_id=None) -> bool:
        """3.90 E: has a tick decided on this frame, or a later one of the
        same body's life? A frame never taken is then final."""
        with self._decisions_lock:
            last = self._last_decided
        return last is not None and last[0] == boot_id and last[1] >= seq

    @property
    def last_frame_seq(self) -> Optional[int]:
        return self._last_frame_seq

    def status(self) -> dict:
        with self._lock:
            elapsed = self._elapsed_s()
            return {
                "running": self._running,
                "outcome": self._outcome,
                # 3.70: one of END_STATES, or None (running, idle, or a
                # fault outcome); why it ended; who, if a person did.
                "end_state": self._outcome if self._outcome in END_STATES else None,
                "end_reason": self._end_reason,
                "cancelled_by": self._cancelled_by,
                # Where it looked (E3-E5): set once the robot has stopped, so
                # None for a moment after `running` turns False.
                "report": dict(self._report) if self._report else None,
                "budget": {
                    "time_limit_s": self.mission_time_limit_s,
                    "elapsed_s": (round(elapsed, 1) if elapsed is not None else None),
                    # 3.88 A: of which paused on a person's pick (not counted).
                    "paused_s": round(self._paused_s(), 1),
                    "battery_floor_v": self.battery_floor_v,
                    "battery_v": self._battery_v,
                    "cloud_calls": self._cloud_calls if self.policy == "vision" else None,
                    "cloud_call_limit": self.vision_max_calls,
                },
                "error": self._error,
                "policy": self.policy,
                "job": self.job,
                # 3.70 S2: a survey's room labels -- how many asks, the cap,
                # whether it was reached, and the names so far.
                "survey": (self.room_labeller.status() if self.room_labeller else None),
                "mission": self.memory.mission,
                "target_object": self.memory.target_object,
                "target_room": self.memory.target_room,
                "step": len(self.agent.history),
                "max_steps": self.max_steps,
                "found": self.memory.found,
                "room_reached": self.memory.room_reached,
                "complete": self.memory.is_complete(),
                "last_action": self._last_action,
                "turns": self._turns_readout(),
                "last_reasoning": self._last_reasoning,
                "rooms_visited": sorted(self.memory.visited_rooms),
                "rooms_searched": sorted(self.memory.searched_rooms),
                "vision_failures": self._vision_failures,
                # Phase P2 / 6.3. `tier` carries the deliberation-call
                # counter -- *"that single number makes the whole
                # architecture watchable"* -- and the model names beside
                # it; `perception` is the tri-state and the CLIP margin
                # for the most recent frame. Null under every policy that
                # has no perception tier, which is all of them but one.
                "tier": self._tier,
                "perception": self._perception,
                "arrival": self._arrival,
                # 3.53: the cloud's answer to the identity question asked
                # AFTER an `arrived_unconfirmed` ending, once it was back.
                # Beside the outcome, never instead of it. None otherwise.
                "late_confirmation": dict(self._late) if self._late else None,
                # The teleop frame id the last decision was made on, so a
                # recorded walk can align decisions to pixels exactly instead
                # of by wall-clock coincidence. None for a backend that does
                # not stamp frames.
                "last_frame_seq": self._last_frame_seq,
                # Phase M5, all three description-only in themselves. The
                # verdict is built from them by control/health.py, which is
                # the only place that decides what "unhealthy" means.
                "ticks": self._ticks,
                "seconds_since_last_tick": (
                    round(time.monotonic() - self._last_tick_at, 2)
                    if self._last_tick_at is not None else None
                ),
                "tick_rate_hz": self._tick_rate_hz(),
                "sighting": self._sighting_dict(),
                # 3.31. The retry rule's state, and the explore policy's
                # goal counts -- absent (None) for a policy with no goals.
                "stuck_episodes": self._stuck_episodes,
                "cooling_down_s": (round(max(0.0, self._cooldown_until - self.clock()), 1)
                                   if self._cooldown_until is not None else None),
                "explore": ({"goals_sent": self.agent.goals_sent,
                             "goals_failed": self.agent.goals_failed,
                             "last_event": self.agent.last_event,
                             "camera_seen_m2": round(len(self.agent.seen)
                                                     * (self.agent.seen_res or 0) ** 2, 2),
                             # 3.77: goal-choice wall time, every choice.
                             "choice_ms": self.agent.choice_stats(),
                             # 3.88 A: ask mode, and the choice open now.
                             "asks": self._ask() is not None,
                             "ask": (self._ask().snapshot()
                                     if self._ask() is not None else None),
                             "choices": len(self.choices)}
                            if self.explores else None),
                "tier_search": self.tier_search,
                # 3.46: counts only; the list is GET /mission/inventory.
                "inventory": (self.inventory.counts()
                              if self.inventory is not None else None),
                "log_tail": self._log[-LOG_TAIL_LINES:],
            }

    # ---------- internal ----------

    def _turns_readout(self) -> dict:
        """R1's readout, with the reading that makes it honest.

        **Reversals alone reward a spin.** A robot turning RIGHT for ever
        never reverses, so "0 reversed" read as success on the first run
        anyone watched -- 98 turns in 120 steps, all one way, target never
        seen. That is `median_command_run`'s failure in new clothes, which
        this readout was introduced to avoid. So it also reports what SHARE
        of the mission's steps were turns, and names a spin when most steps
        were turns in one direction: the pair a person needs to tell "aimed
        and drove" from "stood still and rotated".

        The rule lives here, not in the page, so there is one definition and
        a test for it (M5's rule: the twin renders a verdict, never invents
        one).
        """
        steps = len(self.agent.history)
        share = (self._turns / steps) if steps else 0.0
        spinning = (steps >= SPIN_MIN_STEPS and share >= SPIN_TURN_SHARE
                    and self._reversals <= SPIN_MAX_REVERSAL_SHARE * self._turns)
        return {"count": self._turns, "reversals": self._reversals,
                "last_turn_deg": self._last_turn_deg,
                "share": round(share, 3), "spinning": spinning}

    def _guarded_vision(self, frame: dict) -> dict:
        """The agent's vision_fn, wrapped in B3.2's timeout. Raises
        VisionUnavailable, which tick() turns into the failure budget.

        Also where room-level step memory (docs/guides/AGENT-HARNESS.md section 10)
        reaches the vision call: if self.vision_fn carries a
        set_searched_rooms attribute (brain/navigate.py's vision_fn_for()
        does; the rule-based default does not), refresh it from
        MissionMemory right before every call. This lives here rather than
        in the agent because self.memory only exists on this side of the
        seam at call time -- vision_fn is bound before the mission's
        MissionMemory is constructed."""
        set_searched_rooms = getattr(self.vision_fn, "set_searched_rooms", None)
        if set_searched_rooms is not None:
            set_searched_rooms(sorted(self.memory.searched_rooms))

        # Phase C's seam. Odometry rides IN THE FRAME, the way pan/tilt
        # already do (1.15.3), rather than being handed to the policy as a
        # second channel -- so a policy that wants it reads one dict and a
        # policy that does not is unchanged. It is attached here rather
        # than where the frame is built because this is the only place
        # that holds both the robot and the vision call.
        #
        # A backend with no encoders returns the honest no-op and the
        # frame carries `usable: False`, which the tiered policy reads as
        # "fall back to counting frames" -- never as "has not moved".
        # Failure to read it at all is non-fatal on purpose: odometry is
        # an input to a *pacing* decision, and a mission that died because
        # an encoder route 500'd would be a worse robot than one that
        # paced itself on frame count for a while.
        try:
            frame = {**frame, "odometry": self.robot.get_odometry()}
        except Exception as e:  # noqa: BLE001
            logger.debug("odometry unavailable this frame: %s", e)
        if self.policy == "vision":
            with self._lock:
                self._cloud_calls += 1
        try:
            return call_with_timeout(self.vision_fn, frame, timeout_s=self.vision_timeout_s)
        except TimeoutError as e:
            raise VisionUnavailable(f"vision timed out after {self.vision_timeout_s}s") from e
        except Exception as e:  # noqa: BLE001
            raise VisionUnavailable(f"vision call failed: {e}") from e

    def confirm_timeout_s(self) -> float:
        """3.56: the arrival confirmation's own deadline, never longer than
        B3.2's (a drill's shortened `vision_timeout_s` shortens it too). The
        brain builds the confirmation's HTTP client with this deadline."""
        return min(self.arrival_confirm_timeout_s, self.vision_timeout_s)

    def confirm_guard_s(self) -> float:
        """The runner's backstop above `confirm_timeout_s()`."""
        t = self.confirm_timeout_s()
        return t + min(CONFIRM_GUARD_GRACE_MAX_S, 0.25 * t)

    def late_pending(self) -> Optional[str]:
        """3.76 G1: "out" while a confirmation (the mission's own, or a late
        one) is still in flight; "value" or "error" once a late call that
        outlived its guard has ended -- the next late_ask() returns that
        answer or raises LateCallFailed, sending nothing; else None."""
        if self._confirm_alive():
            return "out"
        box = self._late_box
        if box is None:
            return None
        return "value" if "value" in box else ("error" if "error" in box else None)

    def _confirm_alive(self) -> bool:
        t = self._confirm_thread
        return t is not None and t.is_alive()

    def _refresh_tier_stats(self) -> None:
        stats = self._policy_stats()
        with self._lock:
            if stats is not None and self._tier is not None:
                self._tier = {**self._tier, "stats": stats}

    def _set_confirm_thread(self, thread: threading.Thread) -> None:
        self._confirm_thread = thread

    def _cloud_only_confirmer(self, confirm):
        """3.80.O: the policy's cloud-only confirmer, else its
        `confirm_arrival` told to skip the local tier, else None (a confirmer
        that cannot skip it is not asked)."""
        only = getattr(self.vision_fn, "cloud_only_confirmer", None)
        if only is not None:
            return only()
        return functools.partial(confirm, local_ok=False) if _takes_local_ok(confirm) else None

    def _guarded_confirm(self, frame: dict, local_ok: bool = True) -> dict:
        """The policy's `confirm_arrival` (brain/tiered.py), wrapped in
        B3.2's timeout like `_guarded_vision`. A policy without one cannot
        confirm identity, so its arrivals are never confirmed (1a)."""
        confirm = getattr(self.vision_fn, "confirm_arrival", None)
        if confirm is None:
            return {"confirmed": False, "cloud_called": False,
                    "reason": "this policy has no cloud to confirm identity"}
        # 3.56: the whole step -- waiting for calls still out, then the call
        # -- shares ONE deadline, B3.2's `vision_timeout_s`, as it did before
        # the confirmation had a deadline of its own. The arrival tick stays
        # bounded, so B3.3's tick deadline never aborts a mission that B3.2
        # should have counted.
        deadline = time.monotonic() + self.vision_timeout_s

        def left(what: str) -> float:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                # call_with_timeout(0) would run unbounded; never pass it.
                msg = f"{what}: no time left of {self.vision_timeout_s:g}s"
                raise VisionUnavailable(msg) from CloudUnavailable(msg)
            return remaining

        # 3.80.C: the local VLM first. A verdict -- yes, no or undecided --
        # ends the tick with no cloud call, and nothing waits on a cloud
        # trigger still out, which the local answer does not need. It may use
        # what is left of the step minus a cloud confirmation's guard (an
        # absent VLM hands the SAME frame to the cloud), or half of what is
        # left when B3.2's deadline is too short for both (a drill). A local
        # call that outlives that is an unable answer, never a cloud outage.
        local = getattr(self.vision_fn, "confirm_local", None)
        cloud = getattr(self.vision_fn, "confirm_cloud", None)
        if not local_ok:
            # 3.80.O: an arrival the local VLM's box led to; the cloud decides
            # (never the local tier, which would confirm its own sighting).
            confirm = self._cloud_only_confirmer(confirm)
            if confirm is None:
                return {"confirmed": False, "undecided": True, "cloud_called": False,
                        "reason": "no cloud-only confirmer for a local-led arrival"}
            free = getattr(self.vision_fn, "wait_local_free", None)
            remaining = left("the local question in flight")
            if free is not None and not free(max(remaining - self.confirm_guard_s(),
                                                  remaining / 2)):
                # A frame's question still holds the local VLM (review): not a
                # cloud outage -- undecided, asked again next frame.
                return {"confirmed": False, "undecided": True, "cloud_called": False,
                        "reason": "a local question is still in flight; asking again"}
            with self._lock:
                running = self._running
            if not running:
                # A Stop during that wait: nothing paid for an ended mission,
                # and nothing refused either (review).
                return {"confirmed": False, "undecided": True, "cloud_called": False,
                        "reason": "the mission ended during the identity check"}
        elif (local is not None and cloud is not None
                and getattr(self.vision_fn, "local_identity_fn", None) is not None):
            # Only with a local tier configured: without one the step is
            # exactly what it was before 3.80 (`confirm_arrival`).
            remaining = left("the local identity check")
            budget = max(remaining - self.confirm_guard_s(), remaining / 2)
            begin = getattr(self.vision_fn, "begin_local", None)
            call_id = begin() if begin is not None else None
            try:
                verdict = (call_with_timeout(local, frame, call_id, timeout_s=budget)
                           if call_id is not None
                           else call_with_timeout(local, frame, timeout_s=budget))
            except TimeoutError:
                abandon = getattr(self.vision_fn, "abandon_local", None)
                verdict = abandon(call_id) if abandon is not None else None
                logger.warning("the local identity check outlived %.1f s; counted unable",
                               budget)
            if verdict is not None:
                return verdict
            with self._lock:
                running = self._running
            if not running:
                # A Stop landed during the local check (close() drops the
                # client, which reads as absent): the cloud is not asked for
                # a mission that has ended (bug pass, 2026-10-14; 3.76 E's
                # class). Undecided, as the same returns around it (3.76 L2:
                # without it the agent recorded a refusal for a Stop).
                return {"confirmed": False, "undecided": True, "cloud_called": False,
                        "reason": "the mission ended during the identity check"}
            confirm = cloud

        # An async trigger call still out is waited for first (its deadline
        # is the triggers'); only then does the confirmation's own, shorter
        # clock start, so a healthy but slow trigger is never charged to it.
        settle = getattr(self.vision_fn, "wait_inflight", None)
        if settle is not None:
            try:
                call_with_timeout(settle, timeout_s=left("waiting for an earlier cloud call"))
            except TimeoutError as e:
                # Charged to B3.2 here, so the policy forgets that call:
                # its late failure must not be counted a second time.
                abandon = getattr(self.vision_fn, "abandon_inflight", None)
                if abandon is not None:
                    abandon()
                msg = f"an earlier cloud call outlived {self.vision_timeout_s:g}s"
                raise VisionUnavailable(msg) from CloudUnavailable(msg)
        prev = self._confirm_thread
        if prev is not None and prev.is_alive():
            # The last confirmation outlived its guard and is still out.
            # Another call beside it would break "one call in flight". Wait
            # for it (up to one guard) so the budget is spent at the cloud's
            # pace, not the tick's; still out, it is the same unreachable cloud.
            prev.join(min(self.confirm_guard_s(), left("waiting for a confirmation still out")))
            if prev.is_alive():
                msg = "the previous arrival confirmation is still in flight"
                raise VisionUnavailable(msg) from CloudUnavailable(msg)
        with self._lock:
            running = self._running
        if not running:
            # A Stop during the waits above pays nothing (3.80.O review; on
            # every path since 3.76 K2 -- the plain one bought a call).
            return {"confirmed": False, "undecided": True, "cloud_called": False,
                    "reason": "the mission ended during the identity check"}
        guard = min(self.confirm_guard_s(), left("the arrival confirmation"))
        try:
            return call_with_timeout(confirm, frame, timeout_s=guard,
                                     on_start=self._set_confirm_thread)
        except TimeoutError as e:
            # The confirmation waits only on cloud calls (brain/tiered.py
            # confirm_arrival), so its hang is the cloud's (3.47).
            msg = f"arrival confirmation timed out after {guard:g}s"
            raise VisionUnavailable(msg) from CloudUnavailable(msg)
        except Exception as e:  # noqa: BLE001
            raise VisionUnavailable(f"arrival confirmation failed: {e}") from e

    def _handle_vision_failure(self, error: Exception) -> bool:
        cloud = _caused_by(error, CloudUnavailable)
        with self._lock:
            running = self._running
            if running:
                self._vision_failures += 1
                failures, budget = self._vision_failures, self.max_vision_failures
                # 3.47. An arrival confirmation that raised carries THIS
                # tick's arrival readout (brain/agent.py); kept only when it
                # failed because the cloud was unreachable, and only for the
                # current run of failures -- a tick that succeeds clears it
                # with the count. No move executes on a failed tick, so it is
                # where the robot is.
                readout = getattr(error, "arrival_readout", None)
                if readout is not None:
                    # A confirmation that failed on OUR side (a 401) drops
                    # any readout held from earlier in the run.
                    self._failed_arrival = readout if cloud else None
                    self._failed_arrival_frame = (
                        getattr(error, "arrival_frame", None) if cloud else None)
                pending = self._failed_arrival
                pending_frame = self._failed_arrival_frame
                # Logged under the lock (the RLock _finish takes), so a
                # stop() can never put its end line before this one.
                self._log_line(f"vision failure {failures}/{budget}: {error}")
        if not running:
            # stop()/abort() already ended it and stopped the car. A late
            # failure does nothing at all: not counted, not logged, and no
            # stop -- this runner no longer owns the robot, and the server
            # refuses nobody's stop, so one now would halt a person at the
            # D-pad or the next mission (3.47, third review).
            return False
        # Stop the car on every blind step, not only on the last one.
        self._safe_stop()
        if failures < budget:
            return self._running
        # `arrived_unconfirmed` only when this run of failures includes an
        # arrival whose confirmation raised AND the failure that spends the
        # budget is the cloud being unreachable (brain/navigate.py raises
        # CloudUnavailable where the HTTP call is made). A local fault, or a
        # timeout nothing attributes to the cloud, still ends `failed`.
        if pending is not None and cloud:
            self._finish(ARRIVED_UNCONFIRMED, (
                f"arrived (lidar {pending.get('range_m')} m, streak "
                f"{pending.get('streak')}) but identity unconfirmed: vision "
                f"unavailable {failures} times in a row: {error}"),
                arrival={**pending, "state": "unconfirmed",
                         "reason": f"arrived, but the cloud could not be asked: {error}"},
                tier_stats=self._policy_stats(), arrival_frame=pending_frame)
            return False
        self._finish(FAILED, f"vision unavailable {failures} times in a row: {error}")
        return False

    def _policy_stats(self) -> Optional[dict]:
        """The policy's own counters, or None. The held `_tier` is the last
        SUCCESSFUL tick's snapshot and predates the calls that raised. Never
        allowed to fail a finish. Safe under the runner's lock (an RLock;
        the policy's `as_dict()` takes none), where `_ship_late_metrics()`
        calls it so a row and its counters are read in one step."""
        as_dict = getattr(getattr(self.vision_fn, "stats", None), "as_dict", None)
        if as_dict is None:
            return None
        try:
            return as_dict()
        except Exception as e:  # noqa: BLE001
            logger.warning("reading the policy's stats failed: %s", e)
            return None

    def _finish(self, outcome: str, note: str, arrival: Optional[dict] = None,
                tier_stats: Optional[dict] = None,
                arrival_frame: Optional[dict] = None,
                cancelled_by: Optional[str] = None) -> None:
        with self._lock:
            if self._ended():
                return
            # 3.91 A: answers that landed after the last decision was noted
            # (an arrival's wait, the last frame's call). Taken before the
            # end is visible, so the stream recorder's finish can read them.
            self.unfiled_cloud_answers = self._take_cloud_answers(final=True)
            self._running = False
            self._outcome = outcome
            self._end_reason = note
            self._cancelled_by = cancelled_by if outcome == CANCELLED else None
            if self._started_clock is not None:
                try:
                    self._ended_clock = self.clock()
                except Exception:  # noqa: BLE001 -- nothing before the stop may raise (3.55)
                    pass
            ask = self._ask()
            if ask is not None:
                # 3.88 A: the pause ends with the mission; no pick lands after.
                try:
                    ask.end()
                except Exception:  # noqa: BLE001 -- nothing before the stop may raise
                    pass
            # 3.47: set with the outcome, in the same locked step, so a
            # stop() that wins the race never finds them rewritten.
            if arrival is not None:
                self._arrival = arrival
            if tier_stats is not None and self._tier is not None:
                self._tier = {**self._tier, "stats": tier_stats}
            if arrival_frame is not None:
                self._arrival_frame = arrival_frame
            if outcome == FAILED:
                self._error = note
            # 3.46: before the end line, which stays the log's last word --
            # the twin and the tests read the end reason off log_tail[-1].
            if self.inventory is not None:
                # 3.55: nothing on the way to `_safe_stop()` may raise. A
                # summary that failed here once skipped the stop, the
                # policy's close and the end line.
                try:
                    self._log_line(self.inventory.summary())
                except Exception as e:  # noqa: BLE001
                    logger.warning("inventory summary failed: %s", e)
            self._log_line(f"mission ended ({outcome}): {note}")
        if self._navigator is not None:
            # 3.76 C1: the mission's own nav2 goal, which the brain's stop
            # spares, is cancelled too -- started first, beside the stop, and
            # on a thread, because a Stop runs this on the brain's event loop
            # and these are calls through to the ROS bridge.
            threading.Thread(target=self._navigator.end, daemon=True,
                             name="mission-goal-end").start()
        # Outside the lock: this is an HTTP call when the robot is remote.
        self._safe_stop()
        # 3.70 E3-E5: where it looked -- after the stop, because it reads the
        # map (an HTTP call when the world is remote, up to its timeout) and
        # walks it. On a thread of its own: `_finish` also runs on the brain's
        # event loop (the loop's cancellation handler on a Stop), which must
        # never wait on a map. `report_done` is set when it lands.
        if outcome in REPORTED:
            threading.Thread(target=self._report_in_background, daemon=True,
                             name="mission-report").start()
        else:
            self.report_done.set()
        # Phase A. A policy that dispatches its cloud calls holds a worker
        # thread and may have one answer outstanding right now. Closing it
        # here bumps its epoch, so a late answer is dropped rather than
        # applied to a robot that has stopped -- the same orphaned-call
        # rule `guidanceEpoch` enforces in the twin. `getattr` because a
        # vision_fn is only a callable by contract: the rule-based default
        # is a plain function and must stay usable.
        closer = getattr(self.vision_fn, "close", None)
        if closer is not None:
            try:
                closer()
            except Exception as e:  # noqa: BLE001
                logger.warning("closing the vision policy failed: %s", e)
        self._finish_inventory()
        # One metrics row per mission, on a daemon thread. Last, and
        # after the policy is closed, so the latency samples are complete.
        # It can never fail a mission -- control/metrics_client.py holds
        # that rule, the same one the odometry read above follows.
        self._ship_metrics()
        self.finished.set()

    def _report_in_background(self) -> None:
        try:
            report = self._build_report()
            with self._lock:
                self._report = report
        finally:
            self.report_done.set()

    def _build_report(self) -> dict:
        """3.70 E4/E5's "where it looked": the rooms the robot's centre
        entered, how much of the floor the map holds, the goals it reached,
        and the map it was on. Never allowed to fail a finish."""
        rooms = set(self.memory.visited_rooms)
        rooms.update(getattr(self.agent, "rooms_entered", ()) or ())
        report = {
            "rooms_entered": sorted(rooms),
            "goals_reached": [list(g) for g in getattr(self.agent, "goals_reached", ())],
            "coverage": None, "mapped_free_m2": None,
            # 3.50 saves maps; until it is built there is no saved version,
            # so this is the map the mission ran on, as the world names it.
            "map_id": None, "map_version": None,
        }
        try:
            m = self.world.get_map()
            report.update(map_coverage(m))
            report["map_id"], report["map_version"] = m.get("map_id"), m.get("map_version")
        except Exception as e:  # noqa: BLE001 -- a report never fails a finish
            logger.warning("report: map unreadable: %s", e)
        return report

    def _ship_metrics(self) -> None:
        if not self.metrics_url:
            return
        try:
            from control.metrics_client import row_for, ship_run_async
            row = row_for(self.status(), git_revision=self.git_revision,
                          config=self.metrics_config)
            # Kept so a late confirmation (3.53) can re-send THIS row: the
            # same run_id and finished_at, so the same stored object.
            self._metrics_row = row
            # Kept so the late row can wait for this one to land first.
            self._metrics_thread = ship_run_async(self.metrics_url, row,
                                                  secret=self.metrics_secret)
        except Exception as e:  # noqa: BLE001
            logger.warning("metrics row not built: %s", e)

    # ---------- 3.53: asking again once the cloud is back ----------
    #
    # Driven by control/reconfirm.py after an `arrived_unconfirmed` ending.
    # Nothing here touches the robot: the mission is over and _finish has
    # stopped the car; a stop from here would halt whoever drives next.

    def late_confirmation_pending(self) -> bool:
        """An `arrived_unconfirmed` ending, fully finished, with a frame to
        ask about and no final late answer yet."""
        with self._lock:
            return (self.finished.is_set()
                    and self._outcome == ARRIVED_UNCONFIRMED
                    and self._arrival_frame is not None
                    and (self._late is None or self._late["state"] == "waiting"))

    def late_update(self, **fields) -> Optional[dict]:
        """Merge `fields` into `late_confirmation`. A final state is never
        overwritten -- the first answer wins, as the outcome does. A runner
        that did not end `arrived_unconfirmed` with a frame has nothing to
        record and returns None."""
        with self._lock:
            if self._outcome != ARRIVED_UNCONFIRMED or self._arrival_frame is None:
                return None
            late = self._late or {"state": "waiting", "probes": 0, "paid_calls": 0}
            if late["state"] != "waiting":
                return dict(late)
            self._late = {**late, **fields, "at": time.time()}
            snapshot = dict(self._late)
            if snapshot["state"] != "waiting":
                self._log_line(f"late confirmation ({snapshot['state']}): "
                               f"{snapshot.get('reason', '')}")
        if snapshot["state"] != "waiting":
            self._ship_late_metrics(snapshot)
        return snapshot

    def late_ask(self) -> Optional[dict]:
        """At most ONE paid call: the policy's `confirm_arrival` on the
        stored arrival frame, under B3.2's timeout. Returns None without
        calling once the record is final (a cancel landed first). `paid_calls`
        counts a call that went out -- one that raised, or a verdict with
        `cloud_called` -- never a cap refusal that made none. A TimeoutError
        is raised ("the late confirmation hung") while the call is still in
        flight on its abandoned thread; its answer, if it comes, is returned
        by the next late_ask() without a new call. `LateCallInFlight` means
        nothing was sent: an earlier call is still out.

        Not `_guarded_confirm()`: that maps a timeout to VisionUnavailable
        for B3.2's budget, and the late path has no budget -- it needs to
        know a call is still outstanding, so it never sends a second one
        beside it (tiered.py: one call in flight at a time)."""
        # 3.76 G1: an earlier late call that outlived its guard may have
        # ended since. Its answer was paid for and IS the answer; its error
        # was already counted as a failed attempt when it timed out.
        landed = self._late_box
        if landed is not None and not self._confirm_alive():
            self._late_box = None
            if "value" in landed:
                with self._lock:
                    if (self._late or {"state": "waiting"})["state"] != "waiting":
                        return None
                return landed["value"]
            if "error" in landed:
                raise LateCallFailed(str(landed["error"])) from landed["error"]
        # 3.56: the mission's last arrival confirmation (or an earlier late
        # call) may have outlived its guard and still be out. Nothing is
        # sent beside it, and nothing waits for it here (3.76 G1: a join of
        # B3.2's timeout on every probe held the loop 35 s a cycle and a
        # cancel up to 20 s): the caller asks again on a later probe.
        if self._confirm_alive():
            raise LateCallInFlight("an earlier confirmation is still in flight")
        with self._lock:
            late = self._late or {"state": "waiting"}
            if late["state"] != "waiting":
                return None
            frame = self._arrival_frame
            confirm = getattr(self.vision_fn, "confirm_arrival", None)
            if confirm is not None and (self._arrival or {}).get("source") == SOURCE_LOCAL:
                # 3.80.O: an arrival the local VLM's box led to is confirmed by
                # the cloud alone, late as on time -- never by the local VLM.
                confirm = self._cloud_only_confirmer(confirm)
            if confirm is None or frame is None:
                return {"confirmed": False, "cloud_called": False,
                        "reason": "no confirmer or no arrival frame"}
            # Counted HERE, in the same locked step as the check: from now
            # on the call is committed, so a cancel that lands while it is
            # out ships a record (and metrics row) that already includes it.
            self._late = {**late, "paid_calls": late.get("paid_calls", 0) + 1}
        box: dict = {}

        def ask(f):
            try:
                box["value"] = confirm(f)
                return box["value"]
            except BaseException as e:  # noqa: BLE001 -- kept, and re-raised
                box["error"] = e
                raise
            finally:
                # Ended after its guard gave up on it: its counters (cloud_ms,
                # cloud_calls) are only in the policy now. A record a cancel
                # already made final is re-sent with them (3.76 G1), as the
                # call path's own `finally` does for a call that returned.
                self._refresh_tier_stats()
                with self._lock:
                    reship = (dict(self._late) if box.get("abandoned") and self._late
                              and self._late["state"] != "waiting" else None)
                if reship is not None:
                    self._ship_late_metrics(reship)
        self._late_box = box
        uncount = False
        try:
            try:
                verdict = call_with_timeout(ask, frame, timeout_s=self.vision_timeout_s,
                                            on_start=self._set_confirm_thread)
            except TimeoutError as e:
                with self._lock:
                    hung = self._confirm_alive()
                    box["abandoned"] = hung
                if hung:
                    # Out on its abandoned thread; its answer or error, when
                    # it comes, waits in `box` for the next late_ask().
                    raise TimeoutError(f"the late confirmation hung: {e}") from e
                # Ended between the guard's look and this one: its answer is
                # paid for and in the box -- used, never asked again.
                self._late_box = None
                if "value" not in box:
                    raise box.get("error") or e
                verdict = box["value"]
            except BaseException:
                self._late_box = None   # ended here; nothing left to settle
                raise
            self._late_box = None
            # A cap refusal made no call; take back the count (the policy
            # decides that locally, before any I/O).
            uncount = not verdict.get("cloud_called")
            return verdict
        finally:
            stats = self._policy_stats()
            reship = None
            with self._lock:
                final = self._late is not None and self._late["state"] != "waiting"
                if uncount and self._late is not None:
                    self._late = {**self._late,
                                  "paid_calls": max(0, self._late.get("paid_calls", 1) - 1)}
                if stats is not None and self._tier is not None:
                    self._tier = {**self._tier, "stats": stats}
                if final:
                    # A cancel shipped the record while this call was out:
                    # send it again, with the call counted (or taken back)
                    # and counters that include it.
                    reship = dict(self._late)
            if reship is not None:
                self._ship_late_metrics(reship)

    def _ship_late_metrics(self, late: dict) -> None:
        """Re-send the mission's own row (same run_id, so the same stored
        object) with the late verdict. Its counters come from the policy NOW,
        so the late call's cloud_calls and cloud_ms are in them; and it is
        sent only after the original row's ship has finished -- both write
        one key, and a late row landing first would be overwritten."""
        row = self._metrics_row
        if not self.metrics_url or row is None:
            return
        try:
            from control.metrics_client import ship_run

            def send(before, late_row):
                for t in before:
                    t.join(timeout=60)
                ship_run(self.metrics_url, late_row, secret=self.metrics_secret)
            # Built, chained and queued in ONE locked step, from the record
            # and counters as they are at that moment: a row queued later is
            # always at least as fresh as every row queued before it, and
            # lands after them (the original row, then earlier late rows).
            with self._lock:
                current = self._late or late
                stats = {**(row.get("stats") or {}), **(self._policy_stats() or {})}
                stats["late_confirmation"] = {k: current.get(k) for k in
                                              ("state", "probes", "paid_calls", "reason")}
                before = [t for t in (self._metrics_thread, self._late_ship_thread)
                          if t is not None]
                t = threading.Thread(target=send, args=(before, {**row, "stats": stats}),
                                     name="metrics-late", daemon=True)
                self._late_ship_thread = t
            t.start()
        except Exception as e:  # noqa: BLE001
            logger.warning("late metrics row not built: %s", e)

    def _tick_rate_hz(self):
        """Ticks per second since the mission started -- phase M5.

        **Description, never verdict**, and the reason is worth stating
        because the plan asked for a rate check. There is no knowable
        target to compare this against: a frontier tick is milliseconds of
        work plus `tick_interval_s`, and a vision tick is several seconds
        of a paid API call. The same number is healthy in one policy and
        alarming in the other, so a threshold on it would fire on the
        wrong thing. `seconds_since_last_tick` against the mission's own
        `tick_timeout_s` is the comparison that has a target, and that is
        what control/health.py uses. This is here to be read by a person.
        """
        if self._started_at is None or not self._ticks:
            return None
        elapsed = time.monotonic() - self._started_at
        return round(self._ticks / elapsed, 3) if elapsed > 0 else None

    def _ended(self) -> bool:
        """The mission has finished: stopped, with an outcome recorded. Not
        true before `start()` (outcome still IDLE). `_finish` and `_observe`
        share this one rule, so a frame is dropped exactly when the report
        it would change has already been saved."""
        return not self._running and self._outcome != IDLE

    def discard(self) -> None:
        """A runner built but never started (a Stop landed on its start,
        3.76 C2): let go of what it holds."""
        if self._navigator is not None:
            self._navigator.end()
        # The policy too: its perception models and the local VLM client, so
        # the next start's build is not beside them (review of the 3.80
        # rebase, 2026-10-17; 3.81.E's rule).
        close = getattr(self.vision_fn, "close", None)
        if callable(close):
            try:
                close()
            except Exception as e:  # noqa: BLE001 -- a discard never raises
                logger.warning(f"could not close a discarded runner's policy: {e}")

    def _safe_stop(self) -> None:
        try:
            self.robot.stop()
        except Exception as e:  # noqa: BLE001
            # A failsafe that raises is not a failsafe.
            logger.error(f"could not stop the robot: {e}")
            self._log_line(f"WARNING: stop command failed: {e}")

    def _observe(self, frame: dict) -> None:
        """3.46: one frame into the inventory, with the pose and scan read
        now. Two extra reads per frame, and only when the frame carries
        detections. Any failure is logged once and never reaches the agent:
        an inventory is a by-product of a search, never a reason it stops.

        3.55: the update runs under `self._lock`, the lock every reader
        (`status()`, `inventory_report()`, `_finish`) takes -- this runs on
        the tick thread, and a read in the middle of `observe()` saw a
        landmark with no points yet. The two reads stay outside the lock
        (HTTP calls on a remote robot). A frame that lands after the
        mission ended is dropped: the report has already been saved."""
        try:
            detections = self.detections_fn(frame)
            if detections is None or self._ended():
                return                        # no reads once the mission has ended
            pose = self.world.get_pose()
            scan = self.robot.get_scan(max_range_m=INVENTORY_RANGE_M + 0.5)
            with self._lock:
                if self._ended():
                    return                    # ended meanwhile: the report is saved
                self.inventory.observe(
                    detections, pose, scan,
                    pan_deg=float(frame.get("pan_deg") or 0.0),
                    fov_deg=float(frame.get("fov_deg") or DEFAULT_FOV_DEG),
                    room=frame.get("room", "unknown"), step=self._ticks)
        except Exception as e:  # noqa: BLE001
            if not self._inventory_error_logged:
                self._inventory_error_logged = True
                logger.warning("inventory: frame not recorded: %s", e)

    def inventory_report(self) -> Optional[dict]:
        if self.inventory is None:
            return None
        with self._lock:      # 3.55: the list and the outcome from one moment
            report = self.inventory.report()
            report["mission"] = self.memory.mission
            report["outcome"] = self._outcome
        return report

    def _finish_inventory(self) -> None:
        if self.inventory is None or self.inventory_sink is None:
            return
        try:
            self.inventory_sink(self.inventory_report())
        except Exception as e:  # noqa: BLE001
            logger.warning("inventory not saved: %s", e)

    def _log_line(self, line: str) -> None:
        self._log.append(line)
        logger.info(line)

    def _sighting_dict(self) -> Optional[dict]:
        s = self.memory.found_sighting
        if not s:
            return None
        return {
            "step": s.step,
            "object_name": s.object_name,
            "room": s.room,
            # A map-frame pose dict, or None -- see brain/memory.py's
            # Sighting. It was a grid cell until the ROS alignment.
            "position": s.position,
        }

    @staticmethod
    def _describe(result) -> str:
        """One line of 'why', for the status panel and the twin.

        The vision policy has an actual rationale, so it wins when present.
        The rule-based policy has none -- what it gets instead is the
        observation it acted on, which is the honest substitute."""
        frame, scene = result.frame, result.scene

        nav = scene.get("_navigate") or {}
        if nav.get("reasoning"):
            seen = (
                f"target {nav.get('target_direction')}"
                if nav.get("target_visible") else "target not visible"
            )
            if nav.get("target_reached"):
                seen = "target reached"
            # Under the tiered policy, whether this step cost money is the
            # single most useful thing the line can say -- the local
            # scene's own reasoning already announces itself, so this only
            # has to label the frames that DID call out. Absent for every
            # other policy, where every step calls out and a label saying
            # so would be noise.
            tier = scene.get("_tier") or {}
            if tier.get("identity_tier") == "local":
                paid = " + a cloud call" if tier.get("cloud_called") else ""
                return f"[local: {tier.get('trigger')}{paid}] {seen} -- {nav['reasoning']}"
            if tier.get("cloud_called"):
                return f"[cloud: {tier.get('trigger')}] {seen} -- {nav['reasoning']}"
            return f"{seen} -- {nav['reasoning']}"

        objects = ", ".join(scene.get("important_objects", [])) or "nothing of note"
        return (
            f"{frame.get('room', 'unknown')}, facing {frame.get('facing', '?')}, "
            f"free space {scene.get('free_space', '?')}, sees {objects}"
        )

    # Left of the tick's deadline beyond the label call's own timeout, for
    # the rest of the tick and the hung-tick guard's own polling.
    LABEL_TICK_MARGIN_S = 2.0

    def _label_fits(self) -> bool:
        """Whether a label call with its full timeout still ends inside
        brain_server's hung-tick deadline (B3.3, `tick_timeout_s`, set on the
        runner by brain_server -- a drill's shorter one too). The ask runs
        after the step in the same tick, so a long step (an escape's creeps)
        leaves less room. No timeout or no deadline: always."""
        deadline = getattr(self, "tick_timeout_s", None)
        if not self.room_label_timeout_s or not deadline:
            return True
        spent = time.monotonic() - self._tick_began
        return spent + self.room_label_timeout_s + self.LABEL_TICK_MARGIN_S <= deadline

    def _label_frame(self) -> Optional[dict]:
        """The frame a survey's room label is asked about, taken now -- or
        None once the mission has stopped, so a stop landing during the map
        cut asks nobody."""
        return self.robot.get_camera_frame() if self.is_running() else None

    def room_labels(self) -> Optional[dict]:
        """3.70 S2: a survey's room labels as the map's `room_labels` block
        (`{"names", "cells"}`, what brain/explore.py reads), on the world's
        current map; None for a find mission or with no usable map. Saving
        it with the map is S4's (3.50's map store)."""
        if self.room_labeller is None:
            return None
        try:
            return self.room_labeller.layer(self.world.get_map())
        except Exception:  # noqa: BLE001 -- a read, never fatal
            logger.warning("room labels unavailable", exc_info=True)
            return None

    @staticmethod
    def _default_mission(target_object: Optional[str], target_room: Optional[str]) -> str:
        if target_object and target_room:
            return f"Find the {target_object} in the {target_room}."
        if target_object:
            return f"Find the {target_object}."
        return f"Go to the {target_room}."
