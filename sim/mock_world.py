"""
mock_world.py

`WorldInterface` against the grid world -- the map half of the simulator,
`sim/mock_robot.py`'s sibling. Phase N1 of `PLAN-mapping.md`.

The two wrap the SAME `GridWorld` and answer different questions about
it. `MockRobot` answers body questions ("how far is the wall in front of
me"); this answers world questions ("what does the house look like, and
where am I in it"). Neither imports the other.

**The map is OBSERVED, not handed over.** `GridWorld` knows the whole
layout, and returning it would have been three lines -- and would have
made the twin draw a complete house the instant a mission starts, which
is a false picture of what a mapper does and would leave `CELL_UNKNOWN`
untested in the only place it can be exercised without hardware. So
instead the robot *discovers* the house: every read casts a 360-degree
ring from wherever it now stands, marks what the ring crosses as floor
and what stops it as wall, and leaves everything it has not seen
unknown. Drive into a room and watch that room appear.

This is the same shape as `MockRobot.get_depth_grid()` synthesising zones
from `renderer.cast_ray()` -- the simulator earning its sensor readings
rather than reading the answer off the map -- one abstraction up.

**What this deliberately does NOT simulate**, because N1 does not need it
and pretending would be worse than omitting: drift, loop closure, or any
disagreement between where the robot thinks it is and where it is. The
pose here is exact. A real mapper's is not, and the whole reason
`get_pose()` is separate from `get_odometry()` is to leave room for that
difference -- see `world/interface.py`. When `world/ros_world.py` lands
in N6, the jump on loop closure becomes real without any consumer
changing, which is the entire point of doing this in the sim first.

**The pose was quantised to cell centres until R0, and no longer is.** N1
shipped against a `GridWorld` of integer cells and four cardinal headings,
noting that this was "a limitation of the *simulator*, not of the
contract: `x_m` is already a float and `heading_deg` is already degrees, so
C2 makes the pose smooth without changing one line on either side of the
wall". `PLAN-ros-alignment.md` R0 made it continuous and that prediction
held exactly: the three lines below read `world.x` where they read
`world.robot_x + 0.5`, and nothing else here -- and nothing at all in
`world/interface.py`, `control/remote_world.py` or the twin -- changed.
Worth recording, because it is the evidence that the wall was drawn in the
right place before there was anything behind it.
"""

import math

from sim import renderer
from sim.grid_world import CELL_WALL, GridWorld
from world.interface import (
    CELL_FREE,
    CELL_OCCUPIED,
    CELL_UNKNOWN,
    WorldInterface,
)

# One grid cell is 30cm, the same number `sim/mock_robot.py` and
# `sim/sensors.py` already agree on. Imported rather than restated would
# be better, but that constant lives on the BODY side and this module may
# not reach across -- so it is named here with the reason, and a test
# pins the two together.
CELL_M = 0.30

# One ray per degree, because the part chosen in 1.1 is a 360-degree
# scanner and a forward-facing cone would flatter the simulator: a robot
# that maps only what its camera sees never discovers the corridor behind
# it, and the frontier logic in N3 would be measuring the wrong thing.
DEFAULT_RAYS = 360

# How far the ring reaches, in cells. `FPV_MAX_DIST` is the renderer's own
# horizon (14 cells = 4.2m), reused so the sim has ONE distance beyond
# which nothing is known rather than two that can drift apart. The real
# RPLidar C1 sees considerably further; that difference belongs in
# `sim/renderer.py`'s fidelity note with every other one.
DEFAULT_RANGE_CELLS = renderer.FPV_MAX_DIST


class MockWorld(WorldInterface):
    """The grid world's house, as it gets discovered."""

    def __init__(
        self,
        world: GridWorld,
        map_id: str = "sim-grid",
        rays: int = DEFAULT_RAYS,
        range_cells: float = DEFAULT_RANGE_CELLS,
    ):
        self.world = world
        self.map_id = map_id
        self.rays = rays
        self.range_cells = range_cells

        # Row-major, one entry per grid cell, all unknown until seen.
        self._cells = [CELL_UNKNOWN] * (world.width * world.height)
        self._version = 0

    # ---------- observation ----------

    def observe(self) -> int:
        """Integrate one 360-degree scan from the robot's current cell.

        Returns the number of cells whose state CHANGED, which is what
        `map_version` is bumped on: a robot sitting still re-observes the
        same cells every tick, and a version that moved every time would
        make `map_version` useless for exactly the job it exists for
        (letting a client skip re-downloading 10^5 cells).

        Public rather than private because `get_map()` calls it and a test
        wants to drive it directly -- and because a consumer that wants an
        explicitly-timed scan should not have to fake a read to get one.
        """
        px = self.world.x
        py = self.world.y
        changed = 0

        for i in range(self.rays):
            angle = 2 * math.pi * i / self.rays
            dx, dy = math.cos(angle), math.sin(angle)
            dist = 0.0
            while dist < self.range_cells:
                dist += renderer.FPV_STEP
                cx = math.floor(px + dx * dist)
                cy = math.floor(py + dy * dist)
                if not (0 <= cx < self.world.width and 0 <= cy < self.world.height):
                    break
                # `GridWorld._cell()` is the world's own truth about a
                # cell and returns CELL_WALL out of bounds, so the ring
                # cannot mark anything beyond the layout as floor.
                # Objects are solid (3.9): a lidar beam returns off a
                # backpack, and a SLAM map shows it as an obstacle.
                wall = (self.world._cell(cx, cy) == CELL_WALL
                        or (cx, cy) in self.world.objects)
                changed += self._mark(cx, cy, CELL_OCCUPIED if wall else CELL_FREE)
                if wall:
                    # A ray stops at the first wall. Everything behind it
                    # stays unknown, which is the whole difference between
                    # a map and a copy of the layout.
                    break

        # The cell the robot is standing in is floor by construction --
        # it drove there. The ring alone can miss it, because every ray
        # leaves it immediately.
        changed += self._mark(self.world.robot_x, self.world.robot_y, CELL_FREE)

        if changed:
            self._version += 1
        return changed

    def _mark(self, x: int, y: int, state: int) -> int:
        idx = y * self.world.width + x
        if self._cells[idx] == state:
            return 0
        self._cells[idx] = state
        return 1

    # ---------- WorldInterface ----------

    def get_pose(self) -> dict:
        """Exact, and quantised to cell centres -- see the module note.

        `heading_deg` is the BODY heading, never the view heading, for the
        same reason `get_odometry()` reports the body one: a camera pan
        moves where the robot is looking and not where it is. A map that
        swung 90 degrees every time someone tapped look-left would be
        describing the camera, not the house.
        """
        return {
            "usable": True,
            "map_id": self.map_id,
            "x_m": self.world.x * CELL_M,
            "y_m": self.world.y * CELL_M,
            "heading_deg": round(self.world.heading_deg, 4),
        }

    def get_truth(self) -> dict:
        """Ground truth, straight off the grid -- R2.

        Identical to `get_pose()` today, because this backend's pose IS the
        truth. That identity is the point, not a redundancy: R5 replaces the
        pose with `slam_toolbox`'s estimate and leaves this where it is, and
        the difference between the two becomes the error readout.
        """
        return {
            "usable": True,
            "source": "sim",
            "x_m": self.world.x * CELL_M,
            "y_m": self.world.y * CELL_M,
            "heading_deg": round(self.world.heading_deg, 4),
        }

    def get_map(self) -> dict:
        """The house as discovered so far.

        **This observes first, and that is deliberate even though it makes
        a getter mutate.** Nothing in the loop knows to tell a world model
        when to look: `MissionRunner` drives the body, and the body may not
        reach across to the world. A real mapper integrates continuously
        and needs no such prompt. The alternative -- a map that only
        updates when someone remembers to call `observe()` -- would show an
        empty house for a whole mission and look exactly like a broken
        mapper.
        """
        self.observe()
        return {
            "usable": True,
            "map_id": self.map_id,
            "map_version": self._version,
            "resolution_m": CELL_M,
            "width": self.world.width,
            "height": self.world.height,
            # Origin is the grid's own (0, 0) corner, in metres. Not the
            # robot's start: a map that recentred on wherever the robot
            # began would invalidate every stored pose the moment a
            # mission restarted somewhere else.
            "origin_x_m": 0.0,
            "origin_y_m": 0.0,
            "cells": list(self._cells),
        }
