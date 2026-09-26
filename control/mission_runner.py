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
from brain.memory import MissionMemory
from brain.vision_agent import VisionAgent
from robot.interface import Preempted, RobotInterface
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
POLICIES = ("frontier", "vision", "tiered")

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
        "look_left", "look_right", "look_center",
    )

    def __init__(self, robot: RobotInterface, is_running: Callable[[], bool]):
        self._robot = robot
        self._is_running = is_running

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
        return self._robot.get_camera_frame()

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
        # Metrics shipping. Off unless a URL is configured, which is what
        # keeps tests and laptop runs from POSTing anywhere.
        self.metrics_url = ""
        self.metrics_secret = ""
        self.metrics_config: dict = {}
        self.git_revision = ""
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
        agent_class = VisionAgent if policy in VISION_POLICIES else ObjectSearchAgent
        self.agent = agent_class(
            _HaltGate(robot, self.is_running),
            self.memory,
            min_distance_cm=min_distance_cm,
            vision_fn=self._guarded_vision,
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
        self._last_frame_seq: Optional[int] = None
        self._log: list = []

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
        try:
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
            self._last_action = result.action
            self._last_reasoning = self._describe(result)
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
            self._log_line(
                f"step {result.step}: {result.action} "
                f"({'ok' if result.executed else 'blocked'}) -- {self._last_reasoning}"
            )

        # _finish() stops the car, which is an HTTP call when the robot is
        # remote -- so decide outside the lock rather than holding it across
        # the network.
        if self.memory.is_complete():
            self._finish(FOUND if self.memory.found else ROOM_REACHED, self.memory.summary())
            return False
        if len(self.agent.history) >= self.max_steps:
            self._finish(MAX_STEPS, f"step budget of {self.max_steps} exhausted")
            return False
        return True

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

        Also where room-level step memory (AGENT-HARNESS.md section 12)
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

    def _handle_vision_failure(self, error: Exception) -> bool:
        with self._lock:
            self._vision_failures += 1
            failures, budget = self._vision_failures, self.max_vision_failures
        # Stop the car on every blind step, not only on the last one.
        self._safe_stop()
        self._log_line(f"vision failure {failures}/{budget}: {error}")
        if failures >= budget:
            self._finish(FAILED, f"vision unavailable {failures} times in a row: {error}")
            return False
        return self._running

    def _finish(self, outcome: str, note: str) -> None:
        with self._lock:
            already_done = not self._running and self._outcome != IDLE
            if already_done:
                return
            self._running = False
            self._outcome = outcome
            if outcome == FAILED:
                self._error = note
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
        # One metrics row per mission, on a daemon thread. Last, and
        # after the policy is closed, so the latency samples are complete.
        # It can never fail a mission -- control/metrics_client.py holds
        # that rule, the same one the odometry read above follows.
        self._ship_metrics()

    def _ship_metrics(self) -> None:
        if not self.metrics_url:
            return
        try:
            from control.metrics_client import row_for, ship_run_async
            ship_run_async(self.metrics_url,
                           row_for(self.status(),
                                   git_revision=self.git_revision,
                                   config=self.metrics_config),
                           secret=self.metrics_secret)
        except Exception as e:  # noqa: BLE001
            logger.warning("metrics row not built: %s", e)

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
