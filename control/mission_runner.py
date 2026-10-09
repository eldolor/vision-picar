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

import logging
import threading
import time
from typing import Callable, Optional

from brain.agent import ObjectSearchAgent
from brain.explore import ExploreAgent
from brain.frontier import RETRY_COOLDOWN_S, RETRY_LIMIT
from brain.inventory import MAX_RANGE_M as INVENTORY_RANGE_M
from brain.inventory import DEFAULT_FOV_DEG, Inventory, frame_detections
from brain.memory import MissionMemory
from brain.navigate import CloudUnavailable
from brain.vision_agent import VisionAgent
from robot.interface import Preempted, RobotInterface
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
DEFAULT_MAX_STEPS = 120
DEFAULT_VISION_TIMEOUT_S = 20.0
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

# Mission outcomes. "running" is the only non-terminal one.
RUNNING = "running"
IDLE = "idle"
FOUND = "found"
ROOM_REACHED = "room_reached"
STOPPED = "stopped"
MAX_STEPS = "max_steps"
FAILED = "failed"
# Phase M4. Someone with more authority took the robot -- a person at the
# D-pad, by the order in AGENT-HARNESS.md. Deliberately NOT `failed`:
# nothing went wrong, the mission was outranked, and calling it a failure
# would put a normal human intervention in the same bucket as a dead AWS
# link. It is terminal all the same; a preempted brain does not get to
# argue.
PREEMPTED = "preempted"
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
# left and nothing waiting out a cooldown. Not `blocked` -- it is a finding
# about the house ("looked everywhere it could reach; not there"), and a
# target that is absent should end here, never in a refusal.
SEARCHED = "searched"
# 3.47. The robot reached what the local tier took for the target (the lidar
# arrival rule held) but the cloud never ANSWERED the identity question --
# B3.2's budget ran out on an unreachable cloud. Not `found`: since handoff
# 1a only a cloud yes makes `found`, and a local false positive looks exactly
# like this. Not `failed`: the robot did its part and is parked at it. A
# cloud that answers "no" is a refusal and never ends here.
ARRIVED_UNCONFIRMED = "arrived_unconfirmed"


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


class MissionHalted(RuntimeError):
    """Raised by _HaltGate when the agent tries to move a robot whose
    mission has already ended. Expected, not exceptional."""


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

    def set_wheel_velocity(self, left_rad_s: float, right_rad_s: float) -> dict:
        # A MOVE, so gated like every other one: a mission that has ended
        # must not be able to start the wheels.
        self._guard("set_wheel_velocity")
        return self._robot.set_wheel_velocity(left_rad_s, right_rad_s)

    def advance(self, dt: float) -> None:
        return self._robot.advance(dt)


def call_with_timeout(fn: Callable, *args, timeout_s: Optional[float] = None):
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
        max_steps: int = DEFAULT_MAX_STEPS,
        min_distance_cm: float = DEFAULT_MIN_DISTANCE_CM,
        vision_proximity_veto: bool = False,
        policy: str = "frontier",
        vision_fn: Optional[Callable[[dict], dict]] = None,
        vision_timeout_s: float = DEFAULT_VISION_TIMEOUT_S,
        max_vision_failures: int = DEFAULT_MAX_VISION_FAILURES,
        world: Optional[WorldInterface] = None,
        stuck_after: Optional[int] = DEFAULT_STUCK_AFTER,
        navigator=None,
        clock: Optional[Callable[[], float]] = None,
        idle: Optional[Callable[[float], None]] = None,
        retry_cooldown_s: float = RETRY_COOLDOWN_S,
        retry_limit: int = RETRY_LIMIT,
        inventory: bool = True,
        detections_fn: Optional[Callable[[dict], Optional[list]]] = None,
    ):
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
        if not target_object and not target_room:
            raise ValueError(
                "A mission needs a target_object or a target_room -- otherwise "
                "MissionMemory.is_complete() is never true and it can only end "
                "at max_steps."
            )

        self.robot = robot
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
        self.max_steps = max_steps
        # None means "the agent's own sensor-built scene" -- resolved after
        # the agent exists, below, because that scene reads the agent's
        # SafetyController (`ConstrainedAgent.sensed_scene()`).
        self.vision_fn = vision_fn
        self.vision_timeout_s = vision_timeout_s
        self.max_vision_failures = max_vision_failures

        self.memory = MissionMemory(
            mission=mission or self._default_mission(target_object, target_room),
            target_object=target_object,
            target_room=target_room,
        )
        # The agent drives a gated robot; self.robot stays the raw one so
        # stop() is never gated. The policy decides which agent class runs;
        # everything else about the mission is identical either way.
        agent_class = (VisionAgent if policy in VISION_POLICIES
                       else ExploreAgent if policy == "explore" else ObjectSearchAgent)
        extra = {}
        if policy == "explore":
            if navigator is None:
                raise ValueError("The explore policy needs a navigator -- nav2's goals "
                                 "(control/remote_navigator.py), PLAN-ros-alignment.md 3.31")
            extra = {"navigator": navigator, "clock": self.clock}
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
            # Only reaches the vision policy: the rule-based one runs
            # against MockRobot, which has a real distance reading, so the
            # veto would return immediately anyway. Passing it either way
            # would just be a flag that cannot fire.
            vision_proximity_veto=vision_proximity_veto and policy in VISION_POLICIES,
            world=self.world,
        )
        if self.vision_fn is None:
            # The UNGATED agent method is fine here: a scene only reads
            # sensors, and `_HaltGate` exists to stop moves, not reads.
            self.vision_fn = self.agent.sensed_scene

        self._lock = threading.RLock()
        self._running = False
        self._outcome = IDLE
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
        # Set as _finish's LAST act -- after the policy is closed and the
        # metrics row built -- so a late check never starts beside them.
        self.finished = threading.Event()
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
            self._last_tick_at = time.monotonic()
            self._log_line(f"mission started: {self.memory.mission} (max_steps={self.max_steps})")

    def tick(self) -> bool:
        """Advance the mission by one agent step. Returns True while the
        mission is still running, so a caller can `while runner.tick():`."""
        if not self._running:
            return False

        # The step budget is checked at the END of a tick, below, where
        # _finish() then sets _running False -- so a duplicate check here
        # could never fire, and a second copy of the rule is one that can
        # drift from the real one. Removed after coverage showed it
        # unreachable rather than untested.
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
            self._finish(PREEMPTED, str(e))
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
            if self.policy == "explore":
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
                self._last_turn_deg = int(size) if size else None
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
        if getattr(self.agent, "searched", False):
            self._finish(SEARCHED, self.agent.last_event or "no reachable frontier left")
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
        if len(self.agent.history) >= self.max_steps:
            self._finish(MAX_STEPS, f"step budget of {self.max_steps} exhausted")
            return False
        return True

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
            self._finish(PREEMPTED, str(e))
            return
        except Exception as e:  # noqa: BLE001 -- same rule as a step that fails
            logger.exception("back-off failed")
            self._finish(FAILED, f"back-off failed: {e}")
            return
        self._cooldown_until = self.clock() + self.retry_cooldown_s

    def stop(self, reason: str = "stopped by operator") -> None:
        """Operator stop. Always stops the car as well as the loop --
        stopping the thinking is not stopping the robot."""
        self._finish(STOPPED, reason)

    def abort(self, reason: str) -> None:
        """Failsafe stop (B3.3's hung-tick guard calls this). Same as
        stop(), but the mission ends failed rather than merely halted."""
        self._finish(FAILED, reason)

    def is_running(self) -> bool:
        return self._running

    # ---------- reporting ----------

    def status(self) -> dict:
        with self._lock:
            return {
                "running": self._running,
                "outcome": self._outcome,
                "error": self._error,
                "policy": self.policy,
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
                                                     * (self.agent.seen_res or 0) ** 2, 2)}
                            if self.policy == "explore" else None),
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
        try:
            return call_with_timeout(self.vision_fn, frame, timeout_s=self.vision_timeout_s)
        except TimeoutError as e:
            raise VisionUnavailable(f"vision timed out after {self.vision_timeout_s}s") from e
        except Exception as e:  # noqa: BLE001
            raise VisionUnavailable(f"vision call failed: {e}") from e

    def _guarded_confirm(self, frame: dict) -> dict:
        """The policy's `confirm_arrival` (brain/tiered.py), wrapped in
        B3.2's timeout like `_guarded_vision`. A policy without one cannot
        confirm identity, so its arrivals are never confirmed (1a)."""
        confirm = getattr(self.vision_fn, "confirm_arrival", None)
        if confirm is None:
            return {"confirmed": False, "cloud_called": False,
                    "reason": "this policy has no cloud to confirm identity"}
        try:
            return call_with_timeout(confirm, frame, timeout_s=self.vision_timeout_s)
        except TimeoutError as e:
            # The confirmation waits only on cloud calls (brain/tiered.py
            # confirm_arrival), so its hang is the cloud's (3.47).
            msg = f"arrival confirmation timed out after {self.vision_timeout_s}s"
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
        SUCCESSFUL tick's snapshot and predates the calls that raised. Read
        outside the runner's lock, and never allowed to fail a finish."""
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
                arrival_frame: Optional[dict] = None) -> None:
        with self._lock:
            already_done = not self._running and self._outcome != IDLE
            if already_done:
                return
            self._running = False
            self._outcome = outcome
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
                self._log_line(self.inventory.summary())
            self._log_line(f"mission ended ({outcome}): {note}")
        # Outside the lock: this is an HTTP call when the robot is remote.
        self._safe_stop()
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
        is raised as is: the call is still in flight on its abandoned thread.

        Not `_guarded_confirm()`: that maps a timeout to VisionUnavailable
        for B3.2's budget, and the late path has no budget -- it needs to
        know a call is still outstanding, so it never sends a second one
        beside it (tiered.py: one call in flight at a time)."""
        with self._lock:
            late = self._late or {"state": "waiting"}
            if late["state"] != "waiting":
                return None
            frame = self._arrival_frame
            confirm = getattr(self.vision_fn, "confirm_arrival", None)
            if confirm is None or frame is None:
                return {"confirmed": False, "cloud_called": False,
                        "reason": "no confirmer or no arrival frame"}
            # Counted HERE, in the same locked step as the check: from now
            # on the call is committed, so a cancel that lands while it is
            # out ships a record (and metrics row) that already includes it.
            self._late = {**late, "paid_calls": late.get("paid_calls", 0) + 1}
        uncount = False
        try:
            verdict = call_with_timeout(confirm, frame, timeout_s=self.vision_timeout_s)
            # A cap refusal made no call; take back the count (the policy
            # decides that locally, before any I/O).
            uncount = not verdict.get("cloud_called")
            return verdict
        finally:
            stats = self._policy_stats()
            with self._lock:
                if uncount and self._late is not None:
                    self._late = {**self._late,
                                  "paid_calls": max(0, self._late.get("paid_calls", 1) - 1)}
                if stats is not None and self._tier is not None:
                    self._tier = {**self._tier, "stats": stats}

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
            stats = {**(row.get("stats") or {}), **(self._policy_stats() or {})}
            stats["late_confirmation"] = {k: late.get(k) for k in
                                          ("state", "probes", "paid_calls", "reason")}
            late_row = {**row, "stats": stats}
            first = self._metrics_thread

            def send():
                if first is not None:
                    first.join(timeout=60)
                ship_run(self.metrics_url, late_row, secret=self.metrics_secret)
            threading.Thread(target=send, name="metrics-late", daemon=True).start()
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
        an inventory is a by-product of a search, never a reason it stops."""
        try:
            detections = self.detections_fn(frame)
            if detections is None:
                return
            pose = self.world.get_pose()
            scan = self.robot.get_scan(max_range_m=INVENTORY_RANGE_M + 0.5)
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
        with self._lock:
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
            if tier.get("cloud_called"):
                return f"[cloud: {tier.get('trigger')}] {seen} -- {nav['reasoning']}"
            return f"{seen} -- {nav['reasoning']}"

        objects = ", ".join(scene.get("important_objects", [])) or "nothing of note"
        return (
            f"{frame.get('room', 'unknown')}, facing {frame.get('facing', '?')}, "
            f"free space {scene.get('free_space', '?')}, sees {objects}"
        )

    @staticmethod
    def _default_mission(target_object: Optional[str], target_room: Optional[str]) -> str:
        if target_object and target_room:
            return f"Find the {target_object} in the {target_room}."
        if target_object:
            return f"Find the {target_object}."
        return f"Go to the {target_room}."
