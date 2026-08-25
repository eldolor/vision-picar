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
from dataclasses import dataclass
from typing import Callable, Optional

from robot.interface import RobotInterface
from robot.safety import SafetyController, SafetyViolation
from brain.vision import describe_grid_frame
from brain.memory import MissionMemory

logger = logging.getLogger("agent")

ALLOWED_ACTIONS = {"FORWARD", "LEFT", "RIGHT", "REVERSE", "STOP", "LOOK_LEFT", "LOOK_RIGHT"}

# Used only by MissionAgent's frontier-preference navigation, and only
# when a frame happens to expose grid-style position/facing (sim only --
# see MissionAgent docstring for how this degrades gracefully without it).
_HEADING_VECTORS = {"N": (0, -1), "E": (1, 0), "S": (0, 1), "W": (-1, 0)}
_RIGHT_OF = {"N": "E", "E": "S", "S": "W", "W": "N"}
_LEFT_OF = {"N": "W", "W": "S", "S": "E", "E": "N"}


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

    `vision_fn` defaults to the free/offline grid-world converter so this
    runs entirely in simulation with no API calls and no cost. Swap in a
    wrapper around brain.vision.describe_image once real/stock photos
    replace grid-world frames -- decide() and everything downstream is
    unaffected, since both vision functions return the same schema.
    """

    def __init__(
        self,
        robot: RobotInterface,
        min_distance_cm: float = 20.0,
        vision_fn: Callable[[dict], dict] = describe_grid_frame,
        max_consecutive_stops: int = 3,
    ):
        self.robot = robot
        self.safety = SafetyController(robot, min_distance_cm=min_distance_cm)
        self.vision_fn = vision_fn
        self.max_consecutive_stops = max_consecutive_stops
        self.history: list[StepResult] = []

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

    def step(self) -> StepResult:
        frame = self.robot.get_camera_frame()
        scene = self.vision_fn(frame)
        action = self.decide(scene, frame)

        try:
            result = self.safety.check_and_execute(action)
            executed = True
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
    clear directions prefers whichever leads to a cell it hasn't visited
    yet. Plain "trust vision's safest_direction" (Phase 2) just loops the
    outer boundary of the starting room forever; plain right-hand wall
    following spins in tight circles inside small rooms since a turn is
    almost always "clear" there. Preferring unvisited cells fixes both.

    This still isn't Phase 5's semantic navigation ("go to the kitchen")
    -- it's blind full-coverage exploration, not goal-directed movement.
    It only uses frame position/facing when available (sim grid-world);
    without them it degrades to Phase 2's simpler policy, so it stays
    compatible with a real/stock-photo vision pipeline that has no notion
    of grid coordinates.

    Room-level "searching" is also simplified -- a visited room's frame
    already reveals any objects present, so no deliberate look-around
    sweep happens yet. That's Phase 6's job (active object search).
    """

    def __init__(
        self,
        robot: RobotInterface,
        memory: MissionMemory,
        side_clearance_cm: float = 30.0,
        **kwargs,
    ):
        super().__init__(robot, **kwargs)
        self.memory = memory
        self.side_clearance_cm = side_clearance_cm
        self.visited_positions: set = set()

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

        position = frame.get("position") if frame else None
        facing = frame.get("facing") if frame else None
        if position:
            self.visited_positions.add(position)

        self.robot.look_right()
        right_clear = self.robot.get_distance() >= self.side_clearance_cm
        self.robot.look_left()
        left_clear = self.robot.get_distance() >= self.side_clearance_cm
        self.robot.look_center()
        forward_clear = self.robot.get_distance() >= self.side_clearance_cm

        if position and facing in _HEADING_VECTORS:
            options = []
            if forward_clear:
                options.append(("FORWARD", facing))
            if right_clear:
                options.append(("RIGHT", _RIGHT_OF[facing]))
            if left_clear:
                options.append(("LEFT", _LEFT_OF[facing]))

            frontier = [
                (action, heading)
                for action, heading in options
                if self._next_cell(position, heading) not in self.visited_positions
            ]
            if frontier:
                return frontier[0][0]
            if options:
                return options[0][0]  # everything nearby already visited -- backtrack
        else:
            # No grid position/facing exposed (e.g. a real-camera frame)
            # -- fall back to plain right-hand-rule.
            if right_clear:
                return "RIGHT"
            if forward_clear:
                return "FORWARD"
            if left_clear:
                return "LEFT"

        # Boxed in on all three sides -- reuse Phase 2's stuck-breaker.
        return super().decide(scene, frame)

    @staticmethod
    def _next_cell(position: tuple, heading: str) -> tuple:
        dx, dy = _HEADING_VECTORS[heading]
        return (position[0] + dx, position[1] + dy)

    def step(self) -> StepResult:
        result = super().step()
        room = result.frame.get("room", "unknown")
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
