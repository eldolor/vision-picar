"""
grid_world.py

A minimal 2D grid-world simulator standing in for the physical house.
This is intentionally simple: fast to build, fast to iterate, good enough
for testing agent decision-making, mission memory, and room recognition
logic before any hardware exists (see build plan, Phase 0.5).

Grid convention:
    - (0, 0) is top-left.
    - x increases to the right (East), y increases downward (South).
    - Headings: N, E, S, W (compass-style, matches turn_left/turn_right).
"""

from dataclasses import dataclass, field
from enum import Enum
import logging

logger = logging.getLogger("grid_world")


class Heading(Enum):
    N = (0, -1)
    E = (1, 0)
    S = (0, 1)
    W = (-1, 0)

    def turn_left(self) -> "Heading":
        order = [Heading.N, Heading.W, Heading.S, Heading.E]
        return order[(order.index(self) + 1) % 4]

    def turn_right(self) -> "Heading":
        order = [Heading.N, Heading.E, Heading.S, Heading.W]
        return order[(order.index(self) + 1) % 4]

    def compass_deg(self) -> int:
        """This heading as a compass bearing -- clockwise from north,
        positive to the robot's right.

        The convention every angle in this project uses: `get_odometry()`'s
        `heading_deg`, `get_pose()`'s, 1.15.3's pan and the depth grid's
        columns. Lives on `Heading` rather than in a backend because both
        `sim/mock_robot.py` (body) and `sim/mock_world.py` (world) need it
        and two copies of a mapping is how they would start disagreeing.

        NOT `sim/renderer.py`'s `HEADING_ANGLE`, which is a maths-convention
        angle in radians (+x is 0, y grows downward). That one stays where
        it is: it is the renderer's own internal geometry, and converting
        at that boundary is exactly what a backend is for.
        """
        return {"N": 0, "E": 90, "S": 180, "W": 270}[self.name]


CELL_WALL = "#"
CELL_FLOOR = "."
CELL_DOOR = "D"


@dataclass
class GridWorld:
    """
    A small labeled house. `layout` is a list of strings, one per row.
    `rooms` maps a room name to the set of (x, y) floor cells that belong
    to it. `objects` maps (x, y) -> object label (e.g. "red backpack").
    """

    layout: list
    rooms: dict
    objects: dict = field(default_factory=dict)

    robot_x: int = 1
    robot_y: int = 1
    heading: Heading = Heading.N
    pan: int = 0  # -1 = looking left, 0 = center, 1 = looking right

    log: list = field(default_factory=list)

    def __post_init__(self):
        self.height = len(self.layout)
        self.width = len(self.layout[0])

    # ---------- internal helpers ----------

    def _cell(self, x: int, y: int) -> str:
        if 0 <= y < self.height and 0 <= x < self.width:
            return self.layout[y][x]
        return CELL_WALL

    def _is_passable(self, x: int, y: int) -> bool:
        return self._cell(x, y) in (CELL_FLOOR, CELL_DOOR)

    def _view_heading(self) -> Heading:
        """Effective heading accounting for camera pan (look_left/right)."""
        if self.pan < 0:
            return self.heading.turn_left()
        if self.pan > 0:
            return self.heading.turn_right()
        return self.heading

    def room_at(self, x: int, y: int) -> str:
        for room, cells in self.rooms.items():
            if (x, y) in cells:
                return room
        return "unknown"

    def _record(self, event: str):
        self.log.append(event)
        logger.info(event)

    # ---------- movement ----------

    def move(self, cells: int) -> dict:
        """Move forward (positive) or backward (negative) up to `cells`
        steps in the current heading, stopping early if blocked."""
        dx, dy = self.heading.value
        moved = 0
        step = 1 if cells >= 0 else -1
        for _ in range(abs(cells)):
            nx, ny = self.robot_x + dx * step, self.robot_y + dy * step
            if not self._is_passable(nx, ny):
                self._record(
                    f"BLOCKED at ({nx},{ny}) while moving {'forward' if step > 0 else 'backward'}"
                )
                break
            self.robot_x, self.robot_y = nx, ny
            moved += step
        self._record(
            f"MOVE requested={cells} moved={moved} pos=({self.robot_x},{self.robot_y}) heading={self.heading.name}"
        )
        return {"requested": cells, "moved": moved, "position": (self.robot_x, self.robot_y)}

    def turn_left(self) -> dict:
        self.heading = self.heading.turn_left()
        self._record(f"TURN_LEFT heading={self.heading.name}")
        return {"heading": self.heading.name}

    def turn_right(self) -> dict:
        self.heading = self.heading.turn_right()
        self._record(f"TURN_RIGHT heading={self.heading.name}")
        return {"heading": self.heading.name}

    # ---------- camera pan ----------

    def look_left(self) -> dict:
        self.pan = -1
        self._record("LOOK_LEFT")
        return {"pan": self.pan}

    def look_right(self) -> dict:
        self.pan = 1
        self._record("LOOK_RIGHT")
        return {"pan": self.pan}

    def look_center(self) -> dict:
        self.pan = 0
        self._record("LOOK_CENTER")
        return {"pan": self.pan}

    # ---------- sensing ----------

    def distance_ahead(self, max_range: int = 10) -> int:
        """Cells of free space in the direction the camera/sensor is
        currently facing (heading + pan), capped at max_range."""
        dx, dy = self._view_heading().value
        x, y = self.robot_x, self.robot_y
        dist = 0
        while dist < max_range:
            x, y = x + dx, y + dy
            if not self._is_passable(x, y):
                break
            dist += 1
        self._record(f"DISTANCE view_heading={self._view_heading().name} dist={dist}")
        return dist

    def frame_description(self) -> dict:
        """Stand-in for get_camera_frame(). In grid-world this is a
        structured text description rather than pixels; brain/vision.py
        treats this the same way it would treat a VLM caption of a real
        or stock photo."""
        view = self._view_heading()
        dx, dy = view.value
        ahead_x, ahead_y = self.robot_x + dx, self.robot_y + dy

        current_room = self.room_at(self.robot_x, self.robot_y)
        ahead_cell = self._cell(ahead_x, ahead_y)
        is_doorway = ahead_cell == CELL_DOOR
        free_cells = self.distance_ahead()

        visible_objects = []
        # Objects in the 3 cells directly ahead in the view direction.
        vx, vy = self.robot_x, self.robot_y
        for _ in range(3):
            vx, vy = vx + dx, vy + dy
            if (vx, vy) in self.objects:
                visible_objects.append(self.objects[(vx, vy)])

        frame = {
            "room": current_room,
            "facing": view.name,
            "free_space_cells": free_cells,
            "doorway_ahead": is_doorway,
            "objects_visible": visible_objects,
            "position": (self.robot_x, self.robot_y),
        }
        self._record(f"FRAME {frame}")
        return frame
