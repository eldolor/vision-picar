"""
interface.py

The abstract contract for **world state** -- what is true about the house
the robot is driving around in, as opposed to what is true about the
robot's own body. Phase N1 of `PLAN-mapping.md`.

`robot/interface.py` is the sibling of this file and the older of the two.
The line between them, which decides where any future method goes:

    Is the answer expressed in the ROBOT's frame, or in the WORLD's?

Egocentric is body, allocentric is world. `get_distance()` ("how far is
the nearest thing from ME"), `get_depth_grid()` ("zones fanned out AHEAD
of me") and `get_odometry()` ("how far have I driven since I started")
are all body state and stay on `RobotInterface`. A position on a map is
not, and neither is the map.

The shortest form of the distinction, and the one worth remembering:

    **Odometry is what the body says about itself.
    Pose is what the world says about the body.**

Those two disagree permanently and on purpose. `get_odometry()`'s
`distance_m` is monotonically non-decreasing by contract -- dead
reckoning that drifts and never admits it. A pose from a mapper JUMPS:
when SLAM recognises a room it has already seen, it corrects the estimate
by however much the drift had accumulated. Keeping them as separate
methods on separate interfaces is what makes that disagreement visible
instead of averaged away.

**No backend here is required to know anything.** Both methods carry an
honest "I cannot tell you" default -- the same choice `unusable_grid()`
and `unusable_odometry()` make one interface over -- so adding this file
breaks no existing backend and changes no existing behaviour. A robot
with no mapper says so; it does not invent an origin and call itself the
centre of the universe.

**What must never appear in this file** (`PLAN-onboard-perception.md`
3.3, and `PLAN-mapping.md` 0): TF frames, covariance matrices,
quaternions, ROS message types, or anything else whose shape is decided
by ROS. Under (b+) a ROS service will implement one backend of this
contract, and the whole value of (b+) over (c) is that the contract is
ours and ROS converts on its own side of the wall. The moment a
quaternion appears below, ROS has leaked and the wall is decorative.
"""

from abc import ABC


# ---------- the occupancy grid's tri-state ----------
#
# The same three outcomes `robot/interface.py`'s depth zones have, for the
# same reason, one abstraction up:
#
#   CELL_OCCUPIED  a fact -- something is there
#   CELL_FREE      also a fact -- floor was observed and was empty
#   CELL_UNKNOWN   the ABSENCE of a fact -- never observed
#
# M3's argument applies here unchanged and is why this is not a boolean:
# *unmapped* must not look like *empty floor*. A planner that treats
# unknown as free will route straight through a wall it simply has not
# seen yet, and it will look confident while doing it. A planner that
# treats unknown as occupied will refuse to explore, which is the whole
# job. So neither -- the third state exists so a consumer has to decide
# what to do about it explicitly.
#
# The integer values are chosen to be compact (a house is ~10^5 cells and
# these travel over HTTP), not to match anyone. That -1 is also what ROS's
# OccupancyGrid uses for unknown is convergence on the same problem, not
# a borrowing: a ROS-backed implementation converts on ITS side of the
# wall, like every other ROS type.
CELL_UNKNOWN = -1
CELL_FREE = 0
CELL_OCCUPIED = 1

CELL_STATES = (CELL_UNKNOWN, CELL_FREE, CELL_OCCUPIED)


def unusable_pose() -> dict:
    """The honest answer from a backend that does not know where it is.

    Same choice `unusable_odometry()` makes: say so rather than fabricate.
    `usable: False` and every other field `None`, which no consumer may
    feed to a planner -- so a backend with no mapper cannot make a mission
    look as though it had navigated by coordinates it never had.

    **`(0, 0)` would have been the tempting default and is the dangerous
    one.** It is a perfectly valid pose, indistinguishable from a real
    robot genuinely at its map's origin, and it would put every
    map-less backend at the same believable-looking spot. `None` cannot
    be plotted, compared or planned over by accident.
    """
    return {
        "usable": False,
        "map_id": None,
        "x_m": None,
        "y_m": None,
        "heading_deg": None,
    }


def unusable_map() -> dict:
    """The honest answer from a backend that has no map.

    Note this differs from `unusable_grid()` in a way worth explaining,
    because the two are otherwise the same idea. A depth backend with no
    sensor still publishes eight `unusable` zones, so the twin draws a
    grey strip of the right width and "no sensor" and "sensor reading
    nothing" look different rather than one being a blank space.

    **A map has no equivalent right shape.** An unmapped house has no
    known extent, so there is no honest number of cells to hatch. Width
    and height are therefore 0 and `cells` is empty -- and the twin's
    empty-map state is a *label* ("map is empty"), not a drawing. That is
    1.5's bootstrap rule arriving from the rendering side: an empty graph
    is not a special mode, but the context says so explicitly.
    """
    return {
        "usable": False,
        "map_id": None,
        "map_version": 0,
        "resolution_m": None,
        "width": 0,
        "height": 0,
        "origin_x_m": None,
        "origin_y_m": None,
        "cells": [],
    }


class WorldInterface(ABC):
    """What is true about the house. The allocentric half of the robot's
    picture of itself; `RobotInterface` is the egocentric half.

    Neither method is abstract, for the same reason `get_depth_grid()` is
    not: most backends cannot answer, the honest default says so, and a
    backend that really has a mapper overrides it. A new backend is
    therefore never *broken* by this file -- only silent, which is the
    correct thing for a robot with no map to be.
    """

    def get_pose(self) -> dict:
        """Where this robot is on the map.

            {"usable": bool,           # False means every other field is None
             "map_id": str|None,       # WHICH map these coordinates mean
             "x_m": float|None,        # metres, map frame
             "y_m": float|None,
             "heading_deg": float|None}  # 0 = +x, counter-clockwise positive

        **`map_id` is not bookkeeping.** A pose without a map is two
        numbers that mean nothing, and the failure is silent: coordinates
        from a map built yesterday are byte-identical to coordinates from
        the one being built now, so a stale pose plots somewhere plausible
        and wrong. Every consumer must check that the `map_id` it is
        holding is the one its coordinates came from. This is the same
        instinct as 1.5's `schema_version` from the first write.

        **Metres, like `get_odometry()`, and not the centimetres
        `get_distance()` uses.** The split there was deliberate -- a
        proximity reading off a centimetre-resolution sensor against an
        accumulating estimate that will be metres before a mission ends --
        and a map is even more firmly on the metres side. Mixing them in
        one unit guarantees an off-by-100 somewhere.

        **Degrees, to match `get_odometry()`'s `heading_deg`.** Radians
        are what `sim/renderer.py` takes and what a mapper will produce
        internally; the conversion belongs in the backend, so that two
        headings on the same interface are never in two units.
        """
        return unusable_pose()

    def get_map(self) -> dict:
        """The house, as an occupancy grid.

            {"usable": bool,
             "map_id": str|None,
             "map_version": int,        # bumps whenever `cells` changes
             "resolution_m": float|None,  # metres per cell
             "width": int, "height": int,
             "origin_x_m": float|None,  # map-frame position of cell (0, 0)
             "origin_y_m": float|None,
             "cells": list[int]}        # row-major, one CELL_* per cell

        **`map_version` is what makes this pollable.** A pose changes
        every tick; a house changes slowly and is ~10^5 cells. A consumer
        polls `get_pose()` freely and re-reads `cells` only when the
        version moves. Without it the twin would re-download the house
        twice a second to watch a dot move.

        **Row-major, and the grid carries its own shape**, the same rule
        `get_depth_grid()` follows: `len(cells) == width * height`, and a
        consumer reads the shape off the answer rather than assuming one.

        **`origin_*` exist because a map grows in every direction.** Cell
        (0, 0) is not the robot's starting point and is not (0, 0) in
        metres -- a mapper that discovers a room to the west has to extend
        the grid that way, and the origin is what keeps previously-stored
        poses meaning the same place afterwards.
        """
        return unusable_map()


class NullWorld(WorldInterface):
    """A world model that knows nothing, and is honest about it.

    Every method inherited unchanged. This exists so `world/factory.py`
    has something concrete to return, and so "this deployment has no
    mapper" is a NAMED configuration rather than the absence of one --
    the same reason `robot/factory.py` refuses to fall back to sim when
    hardware mode is requested.
    """
