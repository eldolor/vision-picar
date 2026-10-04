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

**The pose is CONTINUOUS as of phase R0** (`PLAN-ros-alignment.md`, which
merges C2 of `PLAN-onboard-perception.md` with 1.14's continuous motion).
The state of record is `x`, `y` (cell units, fractional) and `theta`
(radians), and the integer cell / cardinal `Heading` view of it is a
derived convenience that still works everywhere it always did.

Three reasons this had to happen before any ROS package is installed:

  * **nav2 cannot drive integer cells.** Its local controller emits
    `cmd_vel` continuously at ~20Hz; a pose that can only ever be a cell
    centre and a multiple of 90 degrees has nothing to receive that.
  * **A SLAM pose is not quantisable.** `PLAN-mapping.md` N1 shipped
    `MockWorld.get_pose()` against a quantised pose deliberately, noting
    that `x_m` was already a float and C2 would make the motion smooth
    "with nothing changing on either side of the wall". This is that, and
    that prediction held: `sim/mock_world.py` gained no new concept.
  * **P25 could not run.** `brain/goal_pose.py` ships default OFF because
    the sim turned in 90-degree quanta against a 10-degree centre band, so
    a target off a cardinal direction could never be centred and both arms
    of the A/B just alternated LEFT/RIGHT. A discrete action space cannot
    track a continuous bearing; this removes the last discrete thing in
    the stack.

**Angle convention, and it is the one bug worth being explicit about.**
`theta` is the RAY angle the renderer casts on: 0 is +x (East) and it
increases toward +y (South). Because y grows downward, that is CLOCKWISE
on screen, which on a north-up map is a turn to the robot's RIGHT. So
`theta` increases when the robot turns right, while REP-103's body yaw
rate (`omega`, used by `sim/mock_robot.py`'s wheel kinematics) is
counter-clockwise-positive and therefore carries the opposite sign. The
conversion happens in exactly one place -- `MockRobot.step()` -- and
`tests/test_world_contract.py` already warns why: "two headings in two
rotational senses is the harder bug, because it looks right at 0 and 180".

`compass_deg`/`heading_deg` remain what every other angle in this project
uses: clockwise from north, positive to the robot's right. That is
`theta` plus 90 degrees, and `Heading.angle_rad()` is the inverse.
"""

from enum import Enum
import logging
import math

# The raycaster the twin draws with, reused here so that the geometry a
# move is checked against is the SAME geometry the picture is drawn from.
# Two collision models -- one for the mover, one for the renderer -- is
# how a robot starts driving through a wall it can see.
from sim import renderer
from robot.safety import FOOTPRINT_LENGTH_M, FOOTPRINT_WIDTH_M
from sim.movers import MOVER_KEEPOUT_M, point_to_cell_cells as movers_point_to_cell

logger = logging.getLogger("grid_world")


class Heading(Enum):
    N = (0, -1)
    E = (1, 0)
    S = (0, 1)
    W = (-1, 0)

    # `turn_left()` / `turn_right()` used to live here, rotating one cardinal
    # to the next. **Deleted at R0**, because turning is an operation on an
    # angle now (`GridWorld.rotate()`) and `_nearest_cardinal()` is the only
    # place an angle becomes a cardinal. Keeping them would have left a second
    # way to turn that quietly re-quantised whatever it was given -- which is
    # exactly the bug R0 exists to remove.

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

    def angle_rad(self) -> float:
        """This heading as a `theta` -- the ray angle the renderer casts on.

        The inverse of `compass_deg()`, and deliberately derived from it
        rather than written out as a second table: `sim/renderer.py`'s
        `HEADING_ANGLE` is the third statement of this mapping already, and
        it is kept only because it is a line-for-line port of the
        JavaScript. `tests/test_continuous_pose.py` pins this against it, so
        a fourth copy cannot quietly disagree with the picture.
        """
        return math.radians(self.compass_deg() - 90)


CELL_WALL = "#"
CELL_FLOOR = "."
CELL_DOOR = "D"

# How far the robot's own body reaches ahead of its centre, in cells. The
# grid world's robot occupies its cell, so a move is capped where the
# FRONT of the robot meets the wall rather than where its centre does --
# the same correction `MockRobot.get_depth_grid()` applies to a reported
# clearance, which is why the constant lives here (the footprint is a fact
# about the thing in the world) and is imported there rather than restated.
ROBOT_HALF_CELL = 0.5

# The chassis as the thing that COLLIDES, since 3.18 (PLAN-ros-alignment):
# the URDF rectangle `robot/safety.py` vets with, in cells. Before this a
# translation was checked by one ray from the centre, so a corner could pass
# into furniture the centre-line missed -- the sim said nothing about the
# very escape 3.17 found, and a clamp-less standing twist drove the chassis
# up to 2.7cm deep into the dining chairs. The ray above still caps travel
# head-on (its half cell, 15cm, is further forward than the chassis' front
# edge -- 11.4cm on the 2WD build, 12.65cm on the UGV Rover since 3.21 -- so
# every existing head-on stop is unchanged); the rectangle adds the corners
# and sides.
#
# Less a 0.2cm skin. It was sized for the 2WD chassis, whose corner radius
# (15.1cm) grazed both walls of a 30cm gap by 0.1cm when pivoting at a cell
# centre -- and a pose already in contact is exempt below, which would have
# switched the check off there. The UGV Rover's corner radius is 17.1cm, so
# in a 30cm gap it cannot pivot at all and no skin pretends otherwise: the
# pivot stops where the corner touches, which is the truth.
FOOTPRINT_CELL_M = 0.30
FOOTPRINT_SKIN_CM = 0.2
FOOTPRINT_HALF_LENGTH = (FOOTPRINT_LENGTH_M * 50.0 - FOOTPRINT_SKIN_CM) / (FOOTPRINT_CELL_M * 100)
FOOTPRINT_HALF_WIDTH = (FOOTPRINT_WIDTH_M * 50.0 - FOOTPRINT_SKIN_CM) / (FOOTPRINT_CELL_M * 100)
# Longest stretch checked at its end only. A rectangle translating along its
# own axis sweeps exactly the union of where it starts and ends, so the end
# pose is the whole test -- as long as the stretch is shorter than the
# rectangle. Half a cell is well inside its ~0.75.
FOOTPRINT_CHECK_CELLS = 0.5

# What `look_left()` / `look_right()` swing the camera by. Ninety degrees
# because that is what the pan has always been worth here -- `pan` was a
# tri-state that used to turn the whole cardinal heading. R3
# replaces this with a revolute joint in the URDF and a real ST3215 range;
# until then it is one constant with one meaning rather than a `turn_left()`
# call that pretends the body moved.
PAN_ANGLE_RAD = math.pi / 2

# Sub-step limits for continuous integration. A translation is checked
# against the raycaster once per sub-step, so these bound how far the robot
# may advance, and how far it may turn, between two collision checks. Both
# are far below one cell / one FOV slice, which is what keeps an arc from
# cutting a corner it should have hit.
MAX_SUBSTEP_CELLS = 0.1
MAX_SUBSTEP_RAD = math.radians(5)

# A move short of what was asked by less than this is not "blocked", it is
# floating point. `renderer.cast_ray()` marches in `FPV_STEP` increments
# and returns the first step already inside the wall, so the clearance it
# reports is up to one step LONG; the slack here is what stops an ordinary
# one-cell step into the last free cell from being logged as a collision.
TRAVEL_EPS = renderer.FPV_STEP

# How close an object must be to count as FOUND by the rule-based policy
# (`objects_visible`), in cells. Three because that is what the cell walk
# this replaced looked along, so that policy's step counts stay comparable.
# It is an arrival radius, NOT a detection range: `detections` reports
# everything the picture shows, out to the renderer's horizon, because that
# is what a detector does (R1 learned this the hard way -- see below). Field
# of view and occlusion come from the renderer either way, so neither can
# report something the picture hides.
SIM_PERCEPTION_RANGE_CELLS = 3.0


def _nearest_cardinal(angle_rad: float) -> Heading:
    """The cardinal `Heading` closest to a `theta`.

    Only the `heading` convenience property reads it now -- nothing that
    DECIDES anything rounds an angle to a compass point any more.
    """
    compass = math.degrees(angle_rad + math.pi / 2) % 360.0
    return [Heading.N, Heading.E, Heading.S, Heading.W][int(round(compass / 90.0)) % 4]


class GridWorld:
    """
    A small labeled house. `layout` is a list of strings, one per row.
    `rooms` maps a room name to the set of (x, y) floor cells that belong
    to it. `objects` maps (x, y) -> object label (e.g. "red backpack").

    **No longer a dataclass, as of R0**, because `robot_x`, `robot_y` and
    `heading` are now *views* of the continuous pose rather than the state
    itself -- and a field and a property cannot share a name. The
    constructor signature is unchanged, so every caller and every test
    that builds a world by cell and cardinal heading still does.
    """

    def __init__(
        self,
        layout: list,
        rooms: dict,
        objects: dict = None,
        robot_x: int = 1,
        robot_y: int = 1,
        heading: Heading = Heading.N,
        pan: int = 0,  # -1 = looking left, 0 = center, 1 = looking right
        log: list = None,
        x: float = None,
        y: float = None,
        theta: float = None,
        movers: list = None,
    ):
        self.layout = layout
        self.rooms = rooms
        self.objects = {} if objects is None else objects
        # 3.30: the sim's own clock, in seconds, advanced by whoever lets
        # time pass (`MockRobot.step()`, the robot server's idle ticks).
        # Nothing reads it but the movers, so a world without movers is
        # unchanged by it to the last bit.
        self.sim_time = 0.0
        self.movers = []
        self.log = [] if log is None else log
        self.pan = pan
        self.height = len(self.layout)
        self.width = len(self.layout[0])

        # The state of record. Cell units and radians, both continuous.
        # `robot_x`/`robot_y`/`heading` seed them through the compatibility
        # properties below, so "cell 2" means "the centre of cell 2" and a
        # cardinal heading means its exact angle -- which is what keeps a
        # world built the old way bit-identical to the one this replaces.
        self.robot_x = robot_x
        self.robot_y = robot_y
        self.heading = heading

        # ...and the continuous overrides win, for a caller that has a real
        # pose and should not have to round it to say so.
        if x is not None:
            self.x = x
        if y is not None:
            self.y = y
        if theta is not None:
            self.theta = renderer.normalize_angle(theta)

        for mover in movers or ():
            self.add_mover(mover)

    def __repr__(self):  # pragma: no cover - debugging convenience
        return (f"GridWorld({self.width}x{self.height}, x={self.x:.3f}, "
                f"y={self.y:.3f}, heading_deg={self.heading_deg:.1f}, "
                f"pan={self.pan})")

    # ---------- the pose, both ways of reading it ----------
    #
    # `x`/`y`/`theta` are the truth. The three properties below are the
    # cell-and-cardinal view of it, kept because a great deal of this
    # project legitimately thinks in cells: `sim/maps/starter_house.py`
    # places the robot in one, `brain/agent.py`'s frontier preference reads
    # `position`/`facing` off the frame (and `PLAN-sim-hardening.md` 2.2
    # says to leave that alone), `room_at()` is keyed by cell, and
    # `MockWorld`'s occupancy grid has cells for its whole reason to exist.
    #
    # The getters FLOOR and the setters snap to the centre of the named
    # cell, which is lossy on purpose: `world.robot_x += 1` is a statement
    # about cells and has no opinion about where in the cell to land, so
    # the only non-arbitrary answer is the middle of it.

    @property
    def robot_x(self) -> int:
        return int(math.floor(self.x))

    @robot_x.setter
    def robot_x(self, cell: int):
        self.x = cell + 0.5

    @property
    def robot_y(self) -> int:
        return int(math.floor(self.y))

    @robot_y.setter
    def robot_y(self, cell: int):
        self.y = cell + 0.5

    @property
    def heading(self) -> Heading:
        """The nearest cardinal heading to `theta`.

        Lossy, and every consumer of it is a consumer that was already
        cardinal-only. Anything that wants the real answer reads
        `heading_deg` or `theta` -- `MockWorld.get_pose()` and
        `MockRobot.get_odometry()` both now do.
        """
        return _nearest_cardinal(self.theta)

    @heading.setter
    def heading(self, heading: Heading):
        self.theta = heading.angle_rad()

    @property
    def heading_deg(self) -> float:
        """The body heading as a compass bearing in [0, 360) -- the
        convention every other angle in this project uses."""
        return math.degrees(self.theta + math.pi / 2) % 360.0

    @property
    def solid_cells(self) -> frozenset:
        """Cells an object stands in -- obstacles to anything that senses or
        moves, since 2026-09-26 (`PLAN-ros-alignment.md` 3.9: "objects must
        be treated as solid to emulate the real world"). A real backpack
        stops a robot and returns a lidar beam; until then the robot drove
        onto it and lost it under its own footprint."""
        return frozenset(self.objects)

    # ---------- things that move (PLAN-ros-alignment.md 3.30) ----------
    #
    # `objects` is REPLACED, never mutated in place: the robot server reads
    # the scan on its threadpool while the wheel loop steps the world, and
    # iterating a dict another thread is resizing raises. Rebinding an
    # attribute is atomic, so a reader holds either the old dict or the new.

    def _turning_circle_cells(self) -> float:
        return math.hypot(FOOTPRINT_HALF_LENGTH, FOOTPRINT_HALF_WIDTH)

    def describe_objects(self) -> dict:
        """Every object, movers marked, and the sim clock -- what
        `GET /sim/objects` serves. Here rather than in the route so the
        body served from another process (`sim/body_server.py`, 3.36)
        answers with the same code."""
        objects, movers = self.objects, {m.cell for m in self.movers}
        return {
            "sim_time_s": round(self.sim_time, 3),
            "objects": [{"x": x, "y": y, "name": name, "mover": (x, y) in movers}
                        for (x, y), name in sorted(objects.items())],
        }

    def move_object(self, src, dst) -> None:
        """Move the object at cell `src` to cell `dst` -- furniture someone
        rearranged. Seen by everything at once: there is nothing to refresh.

        Refused (ValueError) onto a wall, onto another object, onto the
        robot's turning circle (that would put the robot inside it), or for
        a mover, which walks its own path."""
        src, dst = tuple(src), tuple(dst)
        if src not in self.objects:
            raise ValueError(f"no object at {src}")
        if any(m.cell == src for m in self.movers):
            raise ValueError(f"{self.objects[src]!r} at {src} is a mover; it walks its own path")
        if not self._is_passable(*dst):
            raise ValueError(f"{dst} is not floor")
        if dst in self.objects:
            raise ValueError(f"{dst} already holds {self.objects[dst]!r}")
        if movers_point_to_cell(self.x, self.y, dst) < self._turning_circle_cells():
            raise ValueError(f"{dst} is inside the robot's turning circle")
        objects = dict(self.objects)
        objects[dst] = objects.pop(src)
        self.objects = objects
        self._record(f"MOVED {objects[dst]} {src} -> {dst}")

    def add_mover(self, mover) -> None:
        """Put a `sim.movers.Mover` at the start of its path."""
        for cell in mover.path:
            if not self._is_passable(*cell):
                raise ValueError(f"mover {mover.name!r}: {cell} is not floor")
        if mover.cell in self.objects:
            raise ValueError(f"mover {mover.name!r}: {mover.cell} already holds "
                             f"{self.objects[mover.cell]!r}")
        # The keep-out holds from the first instant, not only from the first
        # hop: a mover placed on the robot would leave it inside an obstacle
        # (found by the 3.30 sweep, which started one there).
        keepout = self._turning_circle_cells() + MOVER_KEEPOUT_M / FOOTPRINT_CELL_M
        if movers_point_to_cell(self.x, self.y, mover.cell) < keepout:
            raise ValueError(f"mover {mover.name!r}: {mover.cell} is inside the keep-out "
                             "around the robot")
        objects = dict(self.objects)
        objects[mover.cell] = mover.name
        self.objects = objects
        mover.next_hop_at = self.sim_time + mover.hop_s
        self.movers.append(mover)

    def advance_time(self, dt: float) -> None:
        """Let `dt` seconds of sim time pass: every mover takes the hops
        that fell due. A mover whose next cell is blocked, or inside the
        keep-out around the robot, waits and tries again at its next hop."""
        self.sim_time += dt
        for mover in self.movers:
            while mover.next_hop_at <= self.sim_time + 1e-9:
                mover.next_hop_at += mover.hop_s
                self._hop(mover)

    def _hop(self, mover) -> None:
        dst = mover.next_cell
        keepout = self._turning_circle_cells() + MOVER_KEEPOUT_M / FOOTPRINT_CELL_M
        # Never CLOSER to the robot than the keep-out -- or than it already
        # is, when the robot came nearer itself. Stepping past at the same
        # distance, or away, is allowed: the first version waited whenever
        # the next cell was inside the keep-out at all, and a person beside a
        # doorway then froze for good with nav2 waiting on them (3.30's
        # second live run, 5/6). A person keeps walking.
        floor = min(keepout, movers_point_to_cell(self.x, self.y, mover.cell))
        if (dst in self.objects or not self._is_passable(*dst)
                or movers_point_to_cell(self.x, self.y, dst) < floor - 1e-9):
            mover.waits += 1
            return
        objects = dict(self.objects)
        del objects[mover.cell]
        objects[dst] = mover.name
        self.objects = objects
        mover.index = (mover.index + 1) % len(mover.path)
        mover.hops += 1

    def view_angle(self) -> float:
        """The angle the CAMERA points along, in `theta`'s convention.

        Body heading plus the pan. The render, the depth grid, the distance
        reading and simulated perception are all cast from this one angle,
        so none of them can disagree about which way the camera points.
        """
        return renderer.normalize_angle(self.theta + self.pan * PAN_ANGLE_RAD)

    # ---------- internal helpers ----------

    def _cell(self, x: int, y: int) -> str:
        if 0 <= y < self.height and 0 <= x < self.width:
            return self.layout[y][x]
        return CELL_WALL

    def _is_passable(self, x: int, y: int) -> bool:
        return self._cell(x, y) in (CELL_FLOOR, CELL_DOOR)

    def room_at(self, x: int, y: int) -> str:
        for room, cells in self.rooms.items():
            if (x, y) in cells:
                return room
        return "unknown"

    def _record(self, event: str):
        self.log.append(event)
        logger.info(event)

    # ---------- movement ----------

    def translate(self, cells: float) -> float:
        """Drive `cells` along the body heading -- negative is backwards --
        and return how far it actually got, signed, in cells.

        **The primitive the whole continuous stack stands on.** `move()`
        below is a thin wrapper for the discrete verb layer, and
        `MockRobot.step()` calls this once per integration sub-step.

        Collision is a single ray in the direction of travel, capped where
        the robot's own front meets the wall:

            allowed = cast_ray(direction) - ROBOT_HALF_CELL

        which is exact enough to be worth stating precisely, because the
        discrete version it replaces had a different failure mode. The old
        one asked "is the next CELL passable", so it could not represent
        being 4cm from a wall at all -- the robot was either in a cell or
        not in it. This one is continuous, and it is deliberately NOT
        `get_depth_grid()`'s conservative reduction (which subtracts a
        further `FPV_STEP` because it is feeding a safety veto and must
        never overstate clearance). Here the same subtraction would leave
        every ordinary one-cell step 1.5cm short of the cell it was aiming
        for and log a collision that did not happen, so instead the
        `TRAVEL_EPS` slack absorbs the ray's overshoot. The robot's CENTRE
        still stops half a cell from the wall face either way, which is the
        invariant that matters: `floor(x), floor(y)` is never a wall.

        **Since 3.18 the chassis rectangle is checked too**
        (`_footprint_limit()`): the ray caps travel head-on, the rectangle
        catches a corner or a side the centre-line misses.
        """
        if cells == 0:
            return 0.0
        sign = 1.0 if cells > 0 else -1.0
        angle = self.theta if sign > 0 else renderer.normalize_angle(self.theta + math.pi)
        want = abs(cells)
        room = renderer.cast_ray(self.layout, self.x, self.y, angle,
                                 solid=self.solid_cells) - ROBOT_HALF_CELL
        allowed = max(0.0, min(want, room))
        allowed = self._footprint_limit(angle, allowed)
        self.x += math.cos(angle) * allowed
        self.y += math.sin(angle) * allowed
        if allowed < want - TRAVEL_EPS:
            self._record(
                f"BLOCKED at ({self.x:.2f},{self.y:.2f}) after {allowed:.2f} of "
                f"{want:.2f} cells while moving "
                f"{'forward' if sign > 0 else 'backward'}"
            )
        return sign * allowed

    def footprint_overlaps(self, x: float, y: float) -> bool:
        """Whether the chassis rectangle, at (x, y) and the current heading,
        overlaps a wall or a solid object -- separating axes, touching
        allowed."""
        ux, uy = math.cos(self.theta), math.sin(self.theta)
        nx, ny = -uy, ux
        hl, hw = FOOTPRINT_HALF_LENGTH, FOOTPRINT_HALF_WIDTH
        corners = [(x + ux * a + nx * b, y + uy * a + ny * b)
                   for a, b in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))]
        xs = [c[0] for c in corners]
        ys = [c[1] for c in corners]
        c_u, c_n = x * ux + y * uy, x * nx + y * ny
        solid = self.solid_cells
        for cy in range(math.floor(min(ys)), math.floor(max(ys)) + 1):
            for cx in range(math.floor(min(xs)), math.floor(max(xs)) + 1):
                if renderer._cell_at(self.layout, cx, cy) != CELL_WALL and (cx, cy) not in solid:
                    continue
                if max(xs) <= cx or min(xs) >= cx + 1 or max(ys) <= cy or min(ys) >= cy + 1:
                    continue
                square = ((cx, cy), (cx + 1, cy), (cx + 1, cy + 1), (cx, cy + 1))
                pu = [px * ux + py * uy for px, py in square]
                pn = [px * nx + py * ny for px, py in square]
                if max(pu) <= c_u - hl or min(pu) >= c_u + hl:
                    continue
                if max(pn) <= c_n - hw or min(pn) >= c_n + hw:
                    continue
                return True
        return False

    def _footprint_limit(self, angle: float, allowed: float) -> float:
        """Cap a translation of `allowed` cells along `angle` where the
        chassis rectangle would first touch something.

        **A pose already overlapping is exempt** -- a map that starts the
        robot inside furniture, or a pivot into a corner (rotation is never
        blocked), must not freeze it for good. It can always move; the
        rectangle only refuses motion from a clean pose into contact.
        """
        if allowed <= 0 or self.footprint_overlaps(self.x, self.y):
            return allowed
        dx, dy = math.cos(angle), math.sin(angle)
        done = 0.0
        while done < allowed:
            step_to = min(allowed, done + FOOTPRINT_CHECK_CELLS)
            if self.footprint_overlaps(self.x + dx * step_to, self.y + dy * step_to):
                lo, hi = done, step_to
                for _ in range(16):
                    mid = (lo + hi) / 2
                    if self.footprint_overlaps(self.x + dx * mid, self.y + dy * mid):
                        hi = mid
                    else:
                        lo = mid
                return lo
            done = step_to
        return allowed

    def rotate(self, delta_rad: float) -> float:
        """Pivot in place by `delta_rad`, and return it.

        Positive is a turn to the robot's RIGHT -- see the module docstring
        on why that is `theta` increasing on a y-down grid. Returns what
        was actually turned: since 3.19 a pivot stops where the chassis
        rectangle would first touch something (its corners sweep beyond its
        sides -- "pivots within its own footprint" was true of a circle).
        """
        if delta_rad == 0:
            return 0.0
        # 3.19: the rectangle's corners reach past its sides, so a pivot
        # CAN hit something. From a clean pose, stop where it would first
        # touch; a pose already in contact is exempt, as in translate().
        if not self.footprint_overlaps(self.x, self.y):
            start = self.theta
            self.theta = renderer.normalize_angle(start + delta_rad)
            if self.footprint_overlaps(self.x, self.y):
                lo, hi = 0.0, 1.0
                for _ in range(16):
                    mid = (lo + hi) / 2
                    self.theta = renderer.normalize_angle(start + delta_rad * mid)
                    if self.footprint_overlaps(self.x, self.y):
                        hi = mid
                    else:
                        lo = mid
                self.theta = renderer.normalize_angle(start + delta_rad * lo)
                self._record(f"BLOCKED turning at ({self.x:.2f},{self.y:.2f}) after "
                             f"{math.degrees(delta_rad * lo):.1f} of {math.degrees(delta_rad):.1f} deg")
                return delta_rad * lo
            return delta_rad
        self.theta = renderer.normalize_angle(self.theta + delta_rad)
        return delta_rad

    # `move(cells)` used to live here and is **deleted at R0** for the same
    # reason as `Heading.turn_left()` above: `MockRobot` reaches the pose
    # through `translate()` one integration sub-step at a time, so a
    # whole-cells wrapper was a second path from a command to a position with
    # nobody calling it. The MOVE log line it wrote is now written by
    # `MockRobot.verb_done()`, which is where the request it reports
    # originates.

    def turn_left(self, degrees: float = 90.0) -> dict:
        """Pivot left. Defaults to the cardinal 90 every caller used before
        R0, so a world driven a quarter-turn at a time still lands exactly
        on a cardinal heading and `heading` stays lossless there."""
        self.rotate(-math.radians(degrees))
        self._record(f"TURN_LEFT degrees={degrees} heading={self.heading.name} "
                     f"heading_deg={self.heading_deg:.1f}")
        return {"heading": self.heading.name, "heading_deg": self.heading_deg}

    def turn_right(self, degrees: float = 90.0) -> dict:
        self.rotate(math.radians(degrees))
        self._record(f"TURN_RIGHT degrees={degrees} heading={self.heading.name} "
                     f"heading_deg={self.heading_deg:.1f}")
        return {"heading": self.heading.name, "heading_deg": self.heading_deg}

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
        currently facing (`view_angle()`), capped at max_range.

        A ray now, not a cell walk -- otherwise a robot standing at 47
        degrees would be answered about a cardinal direction it is not
        facing, which is exactly the mismatch that made `brain/goal_pose.py`
        untestable (P25). The answer is still an integer number of cells,
        because `get_distance()` has always spoken in cells x 30cm and the
        safety collar's thresholds were measured against that.

        The reduction is `get_depth_grid()`'s, for the reason that matters
        most about these two numbers: they are the two candidates
        `robot/safety.py`'s veto chooses between (M3), so they must not
        drift apart. On an axis-aligned wall from a cell centre this
        returns exactly the free-cell count the old walk did.
        """
        raw = renderer.cast_ray(self.layout, self.x, self.y, self.view_angle(),
                                solid=self.solid_cells)
        free = raw - renderer.FPV_STEP - ROBOT_HALF_CELL
        dist = max(0, min(int(max_range), int(round(free))))
        view_deg = math.degrees(self.view_angle() + math.pi / 2) % 360.0
        self._record(f"DISTANCE view_deg={view_deg:.1f} dist={dist}")
        return dist

    def frame_description(self) -> dict:
        """What the simulated camera PERCEIVES, as opposed to what it shows.

        Two fields, and both are the simulator standing in for perception
        that a real robot gets from a model:

        * `room` -- the room the robot is standing in. `MissionMemory` keys
          its room bookkeeping off this and the contract requires it
          (`tests/test_robot_contract.py`). A camera-only backend answers
          "unknown" and a vision policy backfills a guess; the sim knows.
        * `objects_visible` -- what a detector would report in this frame.
          `PLAN-onboard-perception.md` 1.12 forbids running the real
          detector on a raycaster render, so the sim reports ground truth
          instead -- the same move `MockWorld` makes for the lidar.

        **What used to be here and is gone** (`PLAN-ros-alignment.md`):
        `position` (a grid cell), `facing` (a cardinal letter),
        `free_space_cells` and `doorway_ahead` (a cell walk along that
        cardinal). All four were answers a real robot cannot give in that
        form. The pose now comes from `WorldInterface.get_pose()` in metres
        and degrees, and clearance from `get_depth_grid()` in centimetres --
        the two places a real robot will get them from.

        **Visibility is continuous and range-limited.** The old version
        looked exactly three cells straight down a cardinal direction, so an
        object at 45 degrees was invisible from every pose. This asks the
        renderer which objects are inside the field of view and not behind
        a wall -- the same test that decides whether they are DRAWN -- and
        keeps those within `SIM_PERCEPTION_RANGE_CELLS`. A picture that
        shows the backpack and a perception field that denies it would be
        exactly the disagreement the renderer was ported to Python to end.
        """
        # Everything in the picture, beyond the robot's own footprint (an
        # object sharing its cell is underneath it, and the renderer's own
        # test -- angle to a point at distance zero -- calls it dead ahead).
        in_view = [
            obj
            for obj in reversed(renderer._visible_objects(
                self.layout, self.objects, self.x, self.y, self.view_angle()))
            if ROBOT_HALF_CELL < obj["dist"] <= renderer.FPV_MAX_DIST
        ]
        frame = {
            "room": self.room_at(self.robot_x, self.robot_y),
            # Close enough to count as FOUND -- the rule-based policy's
            # arrival test, see SIM_PERCEPTION_RANGE_CELLS.
            "objects_visible": [obj["name"] for obj in in_view
                                if obj["dist"] <= SIM_PERCEPTION_RANGE_CELLS],
            # Every object IN THE PICTURE with where it is (R1): bearing off
            # the camera axis, positive to the right, and range in metres.
            # Out to the renderer's own horizon rather than the 3-cell
            # "found" radius above -- a detector reports what the frame
            # shows, and the corpus walks detect targets from across a room,
            # not from 90cm. The first cut used the 3-cell cap and the tier
            # never saw a target it had to approach. This
            # is 1.12's "the sim synthesises detections from grid truth",
            # built at last -- what a detector's box would give the tier on
            # a real frame, read off the geometry the picture was drawn from
            # so the two cannot disagree. `brain/perceive.py`'s
            # `FrameReportedPipeline` is the only consumer; nothing on the
            # hardware path reads it, because nothing there has it.
            # 3.32: the bearing is the centre of what the camera can SEE of
            # the object (`renderer.visible_bearing`), as a detector's box
            # would be, and an object no ray reaches is not reported.
            "detections": self._detections(),
        }
        self._record(f"FRAME {frame}")
        return frame

    def _detections(self) -> list:
        """What a detector would report on this frame -- PLAN 3.32. Every
        object in range whose face some ray reaches past the walls, at the
        bearing of the centre of its VISIBLE part (`renderer.visible_bearing`),
        nearest first. Unlike the picture's billboard rule (one ray to the
        object's centre), an object half behind a door jamb is reported, at
        the bearing of the half that shows; one wholly behind it is not."""
        out = []
        view = self.view_angle()
        for cell, name in self.objects.items():
            dist = math.hypot(cell[0] + 0.5 - self.x, cell[1] + 0.5 - self.y)
            if not ROBOT_HALF_CELL < dist <= renderer.FPV_MAX_DIST:
                continue
            others = {c for c in self.objects if c != cell}
            b = renderer.visible_bearing(self.layout, self.x, self.y, view, cell, solid=others)
            if b is not None:
                out.append((dist, {"label": name, "bearing_deg": round(math.degrees(b), 2),
                                   "distance_m": round(dist * 0.30, 3)}))
        return [d for _, d in sorted(out, key=lambda t: t[0])]
