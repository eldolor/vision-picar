"""
agent.py

Phase 2 (sim) -- constrained action loop.

capture frame -> send to vision -> receive action -> safety check ->
execute in simulator -> stop -> capture next frame

The agent only ever chooses from a small action set (FORWARD, LEFT,
RIGHT, REVERSE, STOP, LOOK_LEFT, LOOK_RIGHT) -- see build plan Phase 2.
Each move is short and re-evaluated every step, same discipline this
will need on real hardware where a wrong multi-second command is much
more costly to reverse.

Note this is decision logic only. Every action still passes through
robot/safety.py before it reaches the robot, so even a bad decision here
(or a bad VLM read) can't drive the robot into something.
"""

import logging
import math
from dataclasses import dataclass
from typing import Callable, Optional

from robot.interface import RobotInterface
from robot.interface import NO_SENSOR_CM
from robot.safety import FORWARD_ACTIONS, VERB_MIN_MOVE_M, SafetyController, SafetyViolation
from brain.arrival import ARRIVED, NOT_JUDGED, REFUSED, ArrivalCheck, arrived_scene
from brain.memory import MissionMemory
from world.interface import NullWorld, WorldInterface, unusable_pose

logger = logging.getLogger("agent")

ALLOWED_ACTIONS = {"FORWARD", "LEFT", "RIGHT", "REVERSE", "STOP", "LOOK_LEFT", "LOOK_RIGHT"}

# The three cardinal lookup tables that used to live here -- heading
# vectors, and which compass point is to the right and left of which --
# are GONE (`PLAN-ros-alignment.md`). The frontier preference below works
# on a continuous pose: a bearing in degrees, and the metre offsets that
# bearing implies. A robot at 47 degrees had no entry in any of them.

# How far ahead the frontier check looks, in metres. One nominal move:
# `MockRobot`'s default `drive_forward()` covers one 30cm cell, and on
# hardware `C2`'s continuous motion makes a "step" a time slice rather than
# a cell -- so this is a distance, not a count, and it is deliberately the
# same order as the map resolution it gets bucketed at.
FRONTIER_LOOKAHEAD_M = 0.30

# How coarsely a pose is remembered as "been there". Visited-ness has to be
# quantised or a continuous pose never repeats and the frontier preference
# degenerates into "always go forward"; the map's own `resolution_m` is the
# non-arbitrary choice, and it is what nav2's frontier search buckets at too.
# Falls back to this when the map does not say.
DEFAULT_VISIT_BUCKET_M = 0.30

# What a pivot is worth, in degrees, when predicting where LEFT or RIGHT
# would point. Ninety because that is what `turn_left()`/`turn_right()`
# command by default -- the *prediction* has to match the action actually
# issued, not the finest turn the robot is now capable of (R0).
_PIVOT_DEG = 90.0

# Where `sensed_scene()` stops calling the path "some" and starts calling it
# "clear". Descriptive only -- both answers mean FORWARD, and the boundary
# that changes a decision is `min_distance_cm`. Three of the sim's 30cm
# cells, which is where the grid-fact converter this replaced drew the same
# line, so a logged scene reads the same across the change.
SCENE_CLEAR_CM = 90.0


# 3.34: how a verb can end without having made its move (robot/interface.py
# carry_out_verb). `clamped` is deliberately absent -- see ConstrainedAgent.step.
SHORT_MOVES = ("timeout", "stalled")


@dataclass
class StepResult:
    step: int
    frame: dict
    scene: dict
    action: str
    executed: bool
    detail: object


class ConstrainedAgent:
    """
    Runs the capture -> vision -> decide -> safety-check -> execute loop.

    `vision_fn` defaults to `sensed_scene()`: free, offline, no API call,
    and built from the robot's SENSORS rather than from anything the
    simulator knows. It used to default to a converter that read a grid-cell
    count off the sim's frame; that could only ever run against the grid
    world. This runs against any backend with a depth grid -- the sim today,
    the lidar on hardware day -- which is the property the rest of this
    project is organised around. Swap in a vision policy (`brain/navigate.py`)
    and decide() and everything downstream is unaffected: every vision_fn
    returns the same scene schema.
    """

    def __init__(
        self,
        robot: RobotInterface,
        min_distance_cm: float = 20.0,
        vision_fn: Optional[Callable[[dict], dict]] = None,
        max_consecutive_stops: int = 3,
        vision_proximity_veto: bool = False,
    ):
        self.robot = robot
        self.safety = SafetyController(robot, min_distance_cm=min_distance_cm)
        self.vision_fn = vision_fn or self.sensed_scene
        self.max_consecutive_stops = max_consecutive_stops
        # Off by default, and see _vision_proximity_veto() for the three
        # conditions that still have to hold before it can fire.
        self.vision_proximity_veto = vision_proximity_veto
        self.history: list[StepResult] = []

    def sensed_scene(self, frame: dict) -> dict:
        """A scene built from the robot's own sensors -- the free policy's eyes.

        Two sources, and neither is the simulator's map:

        * **Clearance from `robot/safety.py`'s `forward_clearance()`** -- the
          depth grid's path cone (M3) and the chassis' swept corridor off the
          scan (3.18), in series: exactly what the collar vets a FORWARD
          against. Reusing it rather than re-reducing anything here is what
          guarantees this scene says STOP exactly when the collar would veto
          a FORWARD, and at no other time. That is the one
          boundary in this scene that changes a decision; "clear" versus
          "some" is descriptive and both map to FORWARD.
        * **Objects from perception** -- `frame["objects_visible"]`. On the
          sim that is the simulator standing in for a detector
          (`GridWorld.frame_description()`); a backend with no perception
          omits it and this reports nothing, which is the truth.

        `doorway_visible` is always False: nothing measures doorways, and
        both vision policies already report it that way.
        """
        clearance_cm, _source = self.safety.forward_clearance()
        if clearance_cm is None:
            # "Nothing within range" -- M3's second outcome, never a veto.
            free_space = "clear"
        elif clearance_cm < self.safety.min_distance_cm + VERB_MIN_MOVE_M * 100:
            # Since 3.22 a guarded FORWARD stops AT the line and a move that
            # would cover under VERB_MIN_MOVE_M is refused -- so a robot
            # parked on the line has no forward, though clearance == min.
            free_space = "none"
        elif clearance_cm < SCENE_CLEAR_CM:
            free_space = "some"
        else:
            free_space = "clear"
        return {
            "obstacles_ahead": ["obstacle"] if free_space == "none" else [],
            "free_space": free_space,
            "doorway_visible": False,
            "important_objects": list(frame.get("objects_visible") or []),
            "safest_direction": "STOP" if free_space == "none" else "FORWARD",
        }

    def decide(self, scene: dict, frame: Optional[dict] = None) -> str:
        """
        Maps a vision scene description to one of the allowed actions.
        Deliberately simple for Phase 2: trust vision's safest_direction,
        with a stuck-breaker so the agent doesn't freeze forever against
        the same wall. `frame` is accepted but unused here -- it exists
        so subclasses (MissionAgent) can use richer observations without
        changing the call signature.
        """
        action = scene.get("safest_direction", "STOP")
        if action not in ALLOWED_ACTIONS:
            action = "STOP"

        recent_stops = self._consecutive_stops()
        if action == "STOP" and recent_stops >= self.max_consecutive_stops:
            action = "RIGHT"
            logger.info(f"Stuck for {recent_stops} steps in a row -- forcing RIGHT turn")

        return action

    def _vision_proximity_veto(self, action: str, scene: dict) -> None:
        """Stop a FORWARD the model says would hit something, on a backend
        that has no distance sensor to say otherwise.

        **This is not a safety layer and must never be mistaken for one.**
        `robot/safety.py` is, and its docstring is explicit that it "never
        trusts the AI's own claims about distance" -- which is why this
        lives in brain/ instead, alongside the brain-side `min_distance_cm`
        pre-check that config/robot.yaml already documents. The real
        ultrasonic still re-checks every move it can.

        What it is for: on ReplayRobot and TeleopRobot, `get_distance()`
        returns NO_SENSOR_CM, so the veto path in robot/safety.py is dead
        code and a whole Robot-view walk says nothing about collision
        avoidance. This makes that path execute against real pixels, which
        nothing else does before hardware exists.

        Three conditions, all required:

        1. Explicitly enabled. Off by default.
        2. The backend genuinely has no sensor. If a real reading exists,
           it wins -- a model's guess must never override or pre-empt a
           measurement.
        3. The model actually said "within_one_step". "unknown" is what a
           prompt variant that never asked returns, and it is never acted
           on.

        Do NOT copy this into robot/hardware_robot.py. On the PiCar the
        ultrasonic is the obstacle sensor, and CLAUDE.md's own measurements
        say why this signal cannot be one: on identical frames Opus reports
        obstacle_ahead ~100% of the time and Qwen ~0%.
        """
        if not self.vision_proximity_veto:
            return
        if action not in FORWARD_ACTIONS:
            return
        if self.robot.get_distance() < NO_SENSOR_CM:
            return  # a real sensor is present; it decides, not the model
        estimate = (scene.get("_navigate") or {}).get("distance_estimate")
        if estimate != "within_one_step":
            return
        self.robot.stop()
        msg = (
            f"Blocked {action}: vision estimate says within_one_step and this "
            "backend has no distance sensor (brain-side estimate, not a "
            "measurement)"
        )
        logger.warning(msg)
        raise SafetyViolation(msg)

    def step(self) -> StepResult:
        frame = self.robot.get_camera_frame()
        scene = self._review_scene(self.vision_fn(frame), frame)
        action = self.decide(scene, frame)

        try:
            # Before the real check, never instead of it: on any backend
            # with a sensor this returns immediately and robot/safety.py
            # remains the only thing that can veto a move.
            self._vision_proximity_veto(action, scene)
            # R1: a turn chosen from a measured bearing carries its SIZE.
            # Without it every LEFT/RIGHT was the executor's default 90
            # degrees, which overshoots any target inside an 80-degree cone
            # and is what made the tier flip on 52 of 60 steps. A scene with
            # no `turn_deg` (a scan, a cloud answer, the rule-based policy)
            # gets exactly the call it always did.
            kwargs = {}
            if action in ("LEFT", "RIGHT") and scene.get("turn_deg"):
                kwargs["angle"] = int(scene["turn_deg"])
            result = self.safety.check_and_execute(action, **kwargs)
            executed = True
            # 3.34: a verb that ran out of time, or whose body did not move,
            # fell short of the move it was asked for -- a wheel snagged on a
            # rug, on the car. It did not happen, and five in a row is a
            # robot that is stuck (MissionRunner's `stuck_after`). A verb
            # `clamped` by the safety layer at the line is not this: it moved
            # as far as was safe, which is what a guarded move means.
            short = result.get("stopped_short") if isinstance(result, dict) else None
            if short in SHORT_MOVES:
                executed = False
                result = (f"{action} fell short ({short}): "
                          f"{result.get('reason') or 'the wheels did not reach the target'}")
        except SafetyViolation as e:
            result = str(e)
            executed = False

        step_result = StepResult(
            step=len(self.history),
            frame=frame,
            scene=scene,
            action=action,
            executed=executed,
            detail=result,
        )
        self.history.append(step_result)
        logger.info(
            f"step={step_result.step} room={frame.get('room')} "
            f"action={action} executed={executed}"
        )
        return step_result

    def _review_scene(self, scene: dict, frame: dict) -> dict:
        """A seam between perception and decision. Nothing here: this
        agent has no mission, so there is nothing to arrive at."""
        return scene

    def run(self, max_steps: int = 20) -> list[StepResult]:
        for _ in range(max_steps):
            self.step()
        return self.history

    def _consecutive_stops(self) -> int:
        count = 0
        for r in reversed(self.history):
            if r.action == "STOP":
                count += 1
            else:
                break
        return count


class MissionAgent(ConstrainedAgent):
    """
    Phase 4 (sim) -- adds mission awareness on top of the constrained
    action loop from Phase 2: every step is recorded into MissionMemory
    (rooms visited/searched, object sightings, action history), and the
    agent stops with success once the mission's target object is sighted.

    Navigation uses frontier-preference exploration: at each decision
    point it peeks forward/right/left via camera pan, and among the
    clear directions prefers whichever leads somewhere it hasn't been
    yet -- judged from the WORLD's pose in metres, bucketed at the map's
    own resolution (`PLAN-ros-alignment.md`; it used to be a grid cell read
    off the camera frame). Plain "trust vision's safest_direction" (Phase 2) just loops the
    outer boundary of the starting room forever; plain right-hand wall
    following spins in tight circles inside small rooms since a turn is
    almost always "clear" there. Preferring unvisited cells fixes both.

    This still isn't Phase 5's semantic navigation ("go to the kitchen")
    -- it's blind full-coverage exploration, not goal-directed movement.
    It needs a world that can localise; given `NullWorld` (or a mapper that
    is down) it degrades to the right-hand rule, so it stays usable on a
    backend that cannot say where it is.

    Room-level "searching" is also simplified -- a visited room's frame
    already reveals any objects present, so no deliberate look-around
    sweep happens yet. That's Phase 6's job (active object search).
    """

    def __init__(
        self,
        robot: RobotInterface,
        memory: MissionMemory,
        side_clearance_cm: float = 30.0,
        world: Optional[WorldInterface] = None,
        arrival_confirm_fn: Optional[Callable[[dict], dict]] = None,
        **kwargs,
    ):
        super().__init__(robot, **kwargs)
        self.memory = memory
        self.side_clearance_cm = side_clearance_cm
        # WORLD state (`PLAN-mapping.md` N1), and what makes the frontier
        # preference below metric rather than cell-shaped -- see `decide()`.
        # `NullWorld` rather than None so there is one shape to read: a
        # backend that cannot localise answers `usable: False` and the
        # frontier degrades to the right-hand rule, which is exactly what it
        # already did for a camera-only frame.
        self.world = world if world is not None else NullWorld()
        # Places the robot has already been, as (col, row) buckets of the
        # map's own resolution. NOT grid cells: a bucket is a quantisation
        # of a continuous pose, computed here, and the same arithmetic works
        # against a SLAM map at 5cm as against the sim's 30cm.
        self.visited_buckets: set = set()
        # Log an unreachable world once per mission, not once per step.
        self._world_warned = False
        # P7e's first half (PLAN-ros-alignment.md 3.11): the target detected,
        # centred and within reach ON THE RANGE SENSOR, two frames running,
        # ends the mission found. Judged only for scenes carrying local
        # perception -- see brain/arrival.py for what it refuses to judge.
        self.arrival = ArrivalCheck()
        # Handoff 2026-10-02 1a: the lidar decides DISTANCE, the cloud
        # IDENTITY. An arrival ends `found` only once a cloud call on the
        # arrival frame says the target is in it. MissionRunner passes a
        # guarded call; otherwise the vision_fn's own `confirm_arrival`
        # (brain/tiered.py) is used, and with neither nothing is confirmed.
        self.arrival_confirm_fn = arrival_confirm_fn
        # One question per arrival: set when the cloud says no, cleared when
        # the arrival rule stops holding -- so a robot parked in front of the
        # wrong object pays once, not every frame.
        self._identity_refused = False
        # The refusing verdict, carried on every later refused frame so the
        # mission's final status still says why (spec review 3, fix 9).
        self._refusal: Optional[dict] = None
        self._confirmed_this_frame = False

    def _confirm_identity(self, readout: dict, frame: dict) -> dict:
        """Returns the readout; sets `self._confirmed_this_frame` when this
        frame asked the cloud."""
        if self._identity_refused:
            return {**readout, "state": REFUSED, "identity": self._refusal,
                    "reason": "the cloud did not confirm this arrival; not asking again "
                              "until the arrival ends"}
        confirm = self.arrival_confirm_fn or getattr(self.vision_fn, "confirm_arrival", None)
        verdict = (confirm(frame) if confirm is not None
                   else {"confirmed": False, "cloud_called": False,
                         "reason": "no cloud on this policy to confirm identity"})
        self._confirmed_this_frame = bool(verdict.get("cloud_called"))
        readout = {**readout, "identity": verdict}
        if verdict.get("confirmed"):
            return readout
        self._identity_refused = True
        self._refusal = verdict
        return {**readout, "state": REFUSED,
                "reason": f"arrived, but identity not confirmed: {verdict.get('reason')}"}

    def _review_scene(self, scene: dict, frame: dict) -> dict:
        if not self.memory.target_object or self.memory.is_complete():
            return scene
        readout = self.arrival.observe(scene, self.robot)
        if readout["state"] == NOT_JUDGED and not scene.get("_perception"):
            return scene  # a policy with no local perception: nothing to say
        self._confirmed_this_frame = False
        if readout["state"] == ARRIVED:
            readout = self._confirm_identity(readout, frame)
        else:
            self._identity_refused = False
            self._refusal = None
        if readout["state"] == ARRIVED:
            scene = arrived_scene(scene, self.memory.target_object, readout)
        else:
            scene = dict(scene)
        if self._confirmed_this_frame:
            scene = self._label_confirmation(scene, readout, readout["identity"])
        scene["_arrival"] = readout
        return scene

    @staticmethod
    def _label_confirmation(scene: dict, readout: dict, verdict: dict) -> dict:
        """The step that paid for the confirmation says so (spec review 3,
        fixes 9-10): `_tier` is marked as a cloud step with its trigger, so
        the log reads `[cloud: arrival_confirmation]`, and its stats are the
        tier's AFTER the call -- the snapshot the vision step took predates
        it, and a mission that ends `found` takes no later one."""
        tier = dict(scene.get("_tier") or {})
        tier.update(cloud_called=True, trigger="arrival_confirmation")
        if verdict.get("stats"):
            tier["stats"] = verdict["stats"]
        scene["_tier"] = tier
        if readout.get("state") == REFUSED:
            nav = dict(scene.get("_navigate") or {})
            nav["reasoning"] = (f"arrival refused -- {verdict.get('reason')}; "
                                + nav.get("reasoning", ""))
            scene["_navigate"] = nav
        return scene

    def decide(self, scene: dict, frame: Optional[dict] = None) -> str:
        if self.memory.is_complete():
            return "STOP"

        # Right after a turn, commit to entering the cell we just
        # confirmed was clear, instead of re-peeking from the same spot
        # (which would just spin in place forever in open rooms).
        if (
            self.history
            and self.history[-1].action in ("LEFT", "RIGHT")
            and self.history[-1].executed
        ):
            return "FORWARD"

        pose = self._pose()
        bucket_m = self._visit_bucket_m()
        if pose.get("usable"):
            self.visited_buckets.add(self._bucket(pose["x_m"], pose["y_m"], bucket_m))

        # "Clear" means a move that way would get somewhere: since 3.22 a
        # guarded FORWARD stops AT the collar's line, so a reading of exactly
        # the threshold is where the last move ended, not room for the next
        # -- the same rule `sensed_scene()` applies.
        room_cm = self.side_clearance_cm + VERB_MIN_MOVE_M * 100
        self.robot.look_right()
        right_clear = self.robot.get_distance() >= room_cm
        self.robot.look_left()
        left_clear = self.robot.get_distance() >= room_cm
        self.robot.look_center()
        forward_clear = self.robot.get_distance() >= room_cm

        if pose.get("usable"):
            heading_deg = pose["heading_deg"]
            options = []
            if forward_clear:
                options.append(("FORWARD", heading_deg))
            if right_clear:
                options.append(("RIGHT", heading_deg + _PIVOT_DEG))
            if left_clear:
                options.append(("LEFT", heading_deg - _PIVOT_DEG))

            frontier = [
                (action, bearing)
                for action, bearing in options
                if self._ahead_bucket(pose, bearing, bucket_m) not in self.visited_buckets
            ]
            if frontier:
                return frontier[0][0]
            if options:
                return options[0][0]  # everything nearby already visited -- backtrack
        else:
            # The world cannot localise -- a photograph-driven backend, a
            # teleop phone, or any robot with no mapper. Fall back to the
            # plain right-hand rule, exactly as this did for a frame with no
            # grid coordinates before the pose replaced them.
            if right_clear:
                return "RIGHT"
            if forward_clear:
                return "FORWARD"
            if left_clear:
                return "LEFT"

        # Boxed in on all three sides -- reuse Phase 2's stuck-breaker.
        return super().decide(scene, frame)

    def _pose(self) -> dict:
        """Where the world says we are, or an honest "it cannot say".

        **A world that is down must not end a mission**, and that is worth
        being explicit about rather than leaving to a stack trace. The map
        is ADVISORY to this policy: it chooses between three directions the
        distance sensor has already called clear, so losing it costs
        exploration efficiency and nothing else -- the robot falls back to
        the right-hand rule and keeps going. The body is what is
        load-bearing, and `MissionRunner`'s failure budget is about the
        VISION call for the same reason.

        The contract (`world/interface.py`) says a backend that cannot
        localise answers `usable: False`. One that raises instead -- an
        unreachable SLAM bridge, a 401, a socket timeout -- is breaking that
        contract, and the agent's job when a contract is broken is to
        degrade the way the contract would have. `Exception` broadly and on
        purpose: `brain/` may not import a backend, so it cannot name that
        backend's transport error, and guessing at the list would let a new
        one through.
        """
        for source in (self.world.get_pose,):
            try:
                return source()
            except Exception as exc:  # noqa: BLE001 -- see docstring
                if not self._world_warned:
                    logger.warning(
                        "world model unreachable (%s) -- exploring by the "
                        "right-hand rule instead of the map", exc)
                    self._world_warned = True
                return unusable_pose()

    def _visit_bucket_m(self) -> float:
        """The map's own resolution, or a sane default if it has none.

        Asked of the map rather than fixed, because the whole point of
        bucketing at the resolution is that it follows the map: the sim's
        30cm cells and a `slam_toolbox` map's 5cm ones want different
        coarseness, and hardcoding either would make the explorer either
        forgetful or blind depending on which it met.
        """
        try:
            resolution = self.world.get_map().get("resolution_m")
        except Exception:  # noqa: BLE001 -- same reasoning as _pose()
            resolution = None
        return float(resolution) if resolution else DEFAULT_VISIT_BUCKET_M

    @staticmethod
    def _bucket(x_m: float, y_m: float, bucket_m: float) -> tuple:
        return (math.floor(x_m / bucket_m), math.floor(y_m / bucket_m))

    @classmethod
    def _ahead_bucket(cls, pose: dict, bearing_deg: float, bucket_m: float) -> tuple:
        """Which bucket one nominal move along `bearing_deg` lands in.

        The compass convention every angle in this project uses -- clockwise
        from north, positive to the robot's right -- so north is -y and east
        is +x, matching `world/interface.py`'s map origin. This is the one
        place that conversion happens on this side of the wall.
        """
        radians = math.radians(bearing_deg)
        x = pose["x_m"] + math.sin(radians) * FRONTIER_LOOKAHEAD_M
        y = pose["y_m"] - math.cos(radians) * FRONTIER_LOOKAHEAD_M
        return cls._bucket(x, y, bucket_m)

    def step(self) -> StepResult:
        result = super().step()
        room = result.frame.get("room", "unknown")
        if room == "unknown":
            # A real-camera frame carries no room label of its own (sim
            # frames always do). The vision policy's scene may carry a
            # room guess instead (brain/navigate.py's to_scene(), sourced
            # from /navigate's room_guess) -- backfill it here so
            # MissionMemory tracks rooms the same way regardless of which
            # policy produced the frame. Rule-based scenes have no
            # "_navigate" key, so this is a no-op for them.
            guessed = (result.scene.get("_navigate") or {}).get("room_guess")
            if guessed and guessed != "unclear":
                room = guessed
                result.frame["room"] = room
        # A sighting is a statement about the HOUSE, so it is anchored to
        # where the world says the robot was -- not to anything the camera
        # frame carries. Attached here because this is the one place that
        # holds the world, the frame and the memory at once.
        pose = self._pose()
        if pose.get("usable"):
            result.frame["pose"] = {
                k: pose[k] for k in ("x_m", "y_m", "heading_deg", "map_id")}
        self.memory.record_observation(result.step, result.frame, result.scene)
        self.memory.mark_room_searched(room)
        self.memory.record_action(result.step, room, result.action, result.executed)
        return result

    def run_mission(self, max_steps: int = 40) -> dict:
        """Runs the loop until the mission's goal is met (object found
        and/or target room reached) or max_steps is hit. Returns a
        mission report -- this is what the checkpoint demos check."""
        for _ in range(max_steps):
            self.step()
            if self.memory.is_complete():
                break

        return {
            "found": self.memory.found,
            "room_reached": self.memory.room_reached,
            "mission_complete": self.memory.is_complete(),
            "steps_taken": len(self.history),
            "rooms_visited": sorted(self.memory.visited_rooms),
            "rooms_searched": sorted(self.memory.searched_rooms),
            "sighting": self.memory.found_sighting,
        }


class ObjectSearchAgent(MissionAgent):
    """
    Phase 6 (sim) -- active object search.

    MissionAgent (Phase 4) only sees whatever's in view when it happens
    to pass through a room -- a single forward-facing frame at center
    pan. This agent performs a deliberate look_left -> look_right ->
    look_center scan on first entering any room, matching the build
    plan's "use the pan/tilt camera aggressively" guidance -- each pan
    is a real dispatched action (through the same safety table as any
    other move), and each one's resulting frame gets recorded into
    memory on the following step, the same as normal movement frames.

    Only triggers when the mission actually has an object target
    (target_object set) -- a pure "go to room" mission (Phase 5) has
    nothing to scan for, so this degrades to MissionAgent's frontier
    exploration unchanged.
    """

    SCAN_SEQUENCE = ["LOOK_LEFT", "LOOK_RIGHT", "LOOK_CENTER"]

    def __init__(self, robot: RobotInterface, memory: MissionMemory, **kwargs):
        super().__init__(robot, memory, **kwargs)
        self._pending_scan: list = []

    def decide(self, scene: dict, frame: Optional[dict] = None) -> str:
        if self.memory.is_complete():
            self._pending_scan = []
            return "STOP"

        if self._pending_scan:
            return self._pending_scan.pop(0)

        room = frame.get("room") if frame else None
        if (
            self.memory.target_object
            and room
            and room != "unknown"
            and room not in self.memory.searched_rooms
        ):
            logger.info(f"First visit to {room} -- starting look-around scan")
            self._pending_scan = list(self.SCAN_SEQUENCE)
            return self._pending_scan.pop(0)

        return super().decide(scene, frame)
