"""
memory.py

Phase 4 (sim) -- agent harness / mission memory.

Maintains everything the agent needs across steps: the mission, rooms
visited (and searched), object sightings, and the executed-action
history. This is what turns "wander safely" (Phase 2) into "search
purposefully" (Phase 4+).

Also produces a harness-formatted context string matching the shape from
the build plan:

    MISSION: Find a red backpack.
    MEMORY: Living room searched. No backpack found.
    CURRENT OBSERVATION: Hallway with doorway on right.
    AVAILABLE TOOLS: look_left(), look_right(), forward(), turn_right(), stop()

Nothing consumes as_context() yet -- Phase 4's own decision-making stays
rule-based (see brain/agent.py:MissionAgent). This becomes the actual
prompt for brain/planner.py once an LLM makes the room-level decisions
in Phase 5+.
"""

from dataclasses import dataclass
from typing import Optional


@dataclass
class Sighting:
    step: int
    object_name: str
    room: str
    # Where the robot was when it saw this, as the WORLD reported it --
    # `{x_m, y_m, heading_deg, map_id}` -- or None when nothing could
    # localise. It used to be a grid cell read off the camera frame; a
    # sighting is a statement about the house, so it carries a map-frame
    # pose, and `map_id` says which map. The ROS-native form of this is a
    # `geometry_msgs/PoseStamped` in `map`, which is what R3 makes it.
    position: Optional[dict]


@dataclass
class ActionRecord:
    step: int
    room: str
    action: str
    executed: bool


class MissionMemory:
    def __init__(
        self,
        mission: str,
        target_object: Optional[str] = None,
        target_room: Optional[str] = None,
    ):
        self.mission = mission
        self.target_object = target_object.lower() if target_object else None
        self.target_room = target_room.lower() if target_room else None

        self.visited_rooms: set = set()
        self.searched_rooms: set = set()
        self.sightings: list = []
        self.actions: list = []

        self.found: bool = False
        self.found_sighting: Optional[Sighting] = None
        self.room_reached: bool = False

    # ---------- recording ----------

    def record_observation(self, step: int, frame: dict, scene: dict) -> None:
        room = frame.get("room", "unknown")
        if room != "unknown":
            self.visited_rooms.add(room)
            if (
                self.target_room
                and not self.room_reached
                and room.lower() == self.target_room
            ):
                self.room_reached = True

        for obj in scene.get("important_objects", []):
            sighting = Sighting(
                step=step, object_name=obj, room=room, position=frame.get("pose")
            )
            self.sightings.append(sighting)
            if self.target_object and not self.found and self.target_object in obj.lower():
                self.found = True
                self.found_sighting = sighting

    def record_action(self, step: int, room: str, action: str, executed: bool) -> None:
        self.actions.append(
            ActionRecord(step=step, room=room, action=action, executed=executed)
        )

    def mark_room_searched(self, room: str) -> None:
        if room != "unknown":
            self.searched_rooms.add(room)

    # ---------- querying ----------

    def is_room_searched(self, room: str) -> bool:
        return room in self.searched_rooms

    def is_complete(self) -> bool:
        """True once any goal the mission cares about has been met --
        an object sighting (target_object) or a room reached
        (target_room). Missions that set neither never auto-complete."""
        return self.found or self.room_reached

    def summary(self) -> str:
        """Free-text memory summary, matching the build plan's harness
        example (e.g. 'Living room searched. No backpack found.')."""
        if not self.searched_rooms:
            return "No rooms searched yet."
        rooms = ", ".join(sorted(self.searched_rooms)).capitalize()

        status_parts = []
        if self.target_object:
            status_parts.append("Found." if self.found else f"No {self.target_object} found.")
        if self.target_room:
            status_parts.append(
                f"Reached {self.target_room}." if self.room_reached
                else f"{self.target_room.capitalize()} not reached yet."
            )
        status = " ".join(status_parts) if status_parts else ""

        return f"{rooms} searched. {status}".rstrip()

    def as_context(self, current_observation: str, available_tools: list) -> str:
        """Formats memory as the harness context block from the build plan."""
        tools = ", ".join(f"{t}()" for t in available_tools)
        return (
            f"MISSION: {self.mission}\n"
            f"MEMORY: {self.summary()}\n"
            f"CURRENT OBSERVATION: {current_observation}\n"
            f"AVAILABLE TOOLS: {tools}"
        )
