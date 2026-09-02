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
from typing import Callable, Optional

from brain.agent import ObjectSearchAgent
from brain.memory import MissionMemory
from brain.vision import describe_grid_frame
from brain.vision_agent import VisionAgent
from robot.interface import RobotInterface

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
# decision to the model through the caller's vision_fn.
POLICIES = ("frontier", "vision")

# Mission outcomes. "running" is the only non-terminal one.
RUNNING = "running"
IDLE = "idle"
FOUND = "found"
ROOM_REACHED = "room_reached"
STOPPED = "stopped"
MAX_STEPS = "max_steps"
FAILED = "failed"


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
    ):
        if policy not in POLICIES:
            raise ValueError(f"Unknown policy: {policy!r}. Known: {', '.join(POLICIES)}")
        if policy == "vision" and vision_fn is None:
            # The default vision_fn is the offline grid converter. Running
            # the vision policy on top of it would produce a mission that
            # looks like it used the model and did not.
            raise ValueError(
                "The vision policy needs a vision_fn -- see brain/navigate.py's "
                "vision_fn_for(), which binds a target and the vision service URL."
            )
        if not target_object and not target_room:
            raise ValueError(
                "A mission needs a target_object or a target_room -- otherwise "
                "MissionMemory.is_complete() is never true and it can only end "
                "at max_steps."
            )

        self.robot = robot
        self.policy = policy
        self.max_steps = max_steps
        self.vision_fn = vision_fn or describe_grid_frame
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
        agent_class = VisionAgent if policy == "vision" else ObjectSearchAgent
        self.agent = agent_class(
            _HaltGate(robot, self.is_running),
            self.memory,
            min_distance_cm=min_distance_cm,
            vision_fn=self._guarded_vision,
            # Only reaches the vision policy: the rule-based one runs
            # against MockRobot, which has a real distance reading, so the
            # veto would return immediately anyway. Passing it either way
            # would just be a flag that cannot fire.
            vision_proximity_veto=vision_proximity_veto and policy == "vision",
        )

        self._lock = threading.RLock()
        self._running = False
        self._outcome = IDLE
        self._error: Optional[str] = None
        self._vision_failures = 0
        self._last_action: Optional[str] = None
        self._last_reasoning: Optional[str] = None
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
            self._vision_failures = 0
            self._last_action = result.action
            self._last_reasoning = self._describe(result)
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
                "last_reasoning": self._last_reasoning,
                "rooms_visited": sorted(self.memory.visited_rooms),
                "rooms_searched": sorted(self.memory.searched_rooms),
                "vision_failures": self._vision_failures,
                "sighting": self._sighting_dict(),
                "log_tail": self._log[-LOG_TAIL_LINES:],
            }

    # ---------- internal ----------

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
            "position": list(s.position) if s.position else None,
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
