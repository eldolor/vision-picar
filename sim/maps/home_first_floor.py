"""
home_first_floor.py -- the user's own house, first floor, as a simulation.

**Where the numbers come from.** The EXTERIOR is the measured floor-plan
sketch in the home's 2012 appraisal (page 29): every wall segment below is a
dimension printed on that sketch, taken by physical measurement. It closes
to ~1,513 ft^2 of living area against the appraisal's 1,483 ft^2 (2%), and
the built-in garage to 20.2 x 21.2 ft exactly.

**What is NOT measured -- and must be corrected by the person who lives
there.** The sketch draws only the outside walls and prints room NAMES where
the rooms are. Every interior wall, doorway and opening below is INFERRED
from where those names sit, and is marked `PROVISIONAL`. So is the target
object. There is no furniture yet: it is the next thing to add, as solid
objects, because furniture is most of what a floor robot actually meets.

Coordinates are FEET, x to the right and y DOWN the sketch (front of the
house at the bottom, y = 39.6 ft), so every number can be checked against
the drawing by eye. `build_home_world()` rasterises them into the
simulator's 0.30 m grid. Only the first floor: the robot does not climb
stairs. Selected with SIM_MAP=home_first_floor.
"""

import math

from sim.grid_world import GridWorld, Heading

CELL_M = 0.30
FT_M = 0.3048
MARGIN_FT = 1.0        # a band of wall round the outside of the house

# ---------------- MEASURED (appraisal sketch, page 29) ----------------
# The living area, walked clockwise from the den's outside corner. y = 6 is
# the main rear wall; the family room bumps out 6 ft beyond it; the foyer
# bumps out 1.5 ft at the front.
HOUSE_FT = [
    (0.0, 6.0),     # 12 ft rear wall (den) ...
    (12.0, 6.0),    # ... then 6 ft up the family room bump-out
    (12.0, 0.0),    # 16.2 ft family room rear wall
    (28.2, 0.0),    # 6 ft back down
    (28.2, 6.0),    # 20.5 ft rear wall (breakfast, kitchen)
    (48.7, 6.0),    # 19.5 ft right side (kitchen, laundry)
    (48.7, 25.5),   # 16.7 ft shared with the garage
    (32.0, 25.5),   # 14.1 ft shared with the garage (dining)
    (32.0, 39.6),   # 10 ft front wall (dining)
    (22.0, 39.6),   # foyer bump-out, 1.5 ft
    (22.0, 41.1),   # 10 ft foyer front
    (12.0, 41.1),
    (12.0, 39.6),   # 12 ft front wall (living)
    (0.0, 39.6),    # 33.6 ft left side, back to the start
]
# The built-in two-car garage: 20.2 x 21.2 ft, 3.5 ft proud of the right side.
GARAGE_FT = (32.0, 25.5, 52.2, 46.7)       # x0, y0, x1, y1

# ---------------- PROVISIONAL (inferred, to be corrected) ----------------
# Interior walls as (x0, y0, x1, y1) centre lines. A doorway is a gap
# between two segments; every doorway here is 3 ft (0.9 m) unless noted.
WALLS_FT = [
    # the house / garage walls (measured positions, but a wall of the
    # garage, so they stay walls): the laundry-to-garage door is the gap
    (32.0, 25.5, 43.5, 25.5), (46.5, 25.5, 48.7, 25.5),   # PROVISIONAL door 43.5-46.5
    (32.0, 25.5, 32.0, 39.6),
    # den: closed to the family room, door to the hall below
    (12.0, 6.0, 12.0, 16.0),                              # PROVISIONAL
    (0.0, 16.0, 8.5, 16.0), (11.5, 16.0, 12.0, 16.0),     # PROVISIONAL door 8.5-11.5
    # half bath, below the den
    (8.0, 16.0, 8.0, 19.0), (8.0, 22.0, 8.0, 23.0),       # PROVISIONAL door 19-22
    (0.0, 23.0, 13.0, 23.0),                              # PROVISIONAL
    # living: open to the foyer through a wide opening
    (13.0, 23.0, 13.0, 28.0), (13.0, 36.0, 13.0, 39.6),   # PROVISIONAL opening 28-36
    # dining: opening to the foyer, and one towards the kitchen side
    (22.0, 28.0, 22.0, 30.0), (22.0, 37.0, 22.0, 39.6),   # PROVISIONAL opening 30-37
    (22.0, 28.0, 24.0, 28.0), (28.0, 28.0, 32.0, 28.0),   # PROVISIONAL opening 24-28
    # laundry, between the kitchen and the garage
    (36.0, 19.0, 38.0, 19.0), (41.0, 19.0, 48.7, 19.0),   # PROVISIONAL door 38-41
    (36.0, 19.0, 36.0, 25.5),                             # PROVISIONAL
    # family, breakfast and kitchen: left open to one another -- PROVISIONAL
]

# Rooms as rectangles (x0, y0, x1, y1), in the order they claim cells.
# Positions follow the room names on the sketch; boundaries are PROVISIONAL.
ROOMS_FT = [
    ("den", (0.0, 6.0, 12.0, 16.0)),
    ("half bath", (0.0, 16.0, 8.0, 23.0)),
    ("living room", (0.0, 23.0, 13.0, 39.6)),
    ("foyer", (13.0, 22.0, 22.0, 41.1)),
    ("dining room", (22.0, 28.0, 32.0, 39.6)),
    ("family room", (12.0, 0.0, 28.2, 22.0)),
    ("breakfast", (28.2, 6.0, 38.0, 19.0)),
    ("kitchen", (38.0, 6.0, 48.7, 19.0)),
    ("laundry", (36.0, 19.0, 48.7, 25.5)),
    ("garage", (32.0, 25.5, 52.2, 46.7)),
]
DEFAULT_ROOM = "hall"

# The robot starts in the foyer, facing into the house. PROVISIONAL, and so
# is the only object: a mission target, not a piece of furniture.
START_FT = (17.0, 34.0)
START_HEADING = Heading.N
OBJECTS_FT = {(44.0, 10.0): "red backpack"}     # PROVISIONAL: in the kitchen


# ---------------- rasterising ----------------

def _cells(ft: float) -> float:
    """Feet (from the house's corner) to grid cells, margin included."""
    return (ft + MARGIN_FT) * FT_M / CELL_M


def _inside(poly, x, y) -> bool:
    inside = False
    for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            inside = not inside
    return inside


def _near_segment(px, py, seg, half_width) -> bool:
    x0, y0, x1, y1 = seg
    dx, dy = x1 - x0, y1 - y0
    t = 0.0 if dx == dy == 0 else max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - (x0 + t * dx), py - (y0 + t * dy)) <= half_width


def _layout():
    width = int(math.ceil(_cells(GARAGE_FT[2] + MARGIN_FT)))
    height = int(math.ceil(_cells(GARAGE_FT[3] + MARGIN_FT)))
    house = [(_cells(x), _cells(y)) for x, y in HOUSE_FT]
    gx0, gy0, gx1, gy1 = (_cells(v) for v in GARAGE_FT)
    walls = [tuple(_cells(v) for v in w) for w in WALLS_FT]
    rows = []
    for j in range(height):
        row = []
        for i in range(width):
            cx, cy = i + 0.5, j + 0.5
            floor = _inside(house, cx, cy) or (gx0 < cx < gx1 and gy0 < cy < gy1)
            # A wall is one cell thick: a cell whose centre lies within half
            # a cell of a wall's centre line.
            if floor and any(_near_segment(cx, cy, w, 0.5) for w in walls):
                floor = False
            row.append("." if floor else "#")
        rows.append("".join(row))
    return rows


def _rooms(layout):
    rooms = {name: set() for name, _ in ROOMS_FT}
    rooms[DEFAULT_ROOM] = set()
    boxes = [(name, tuple(_cells(v) for v in box)) for name, box in ROOMS_FT]
    for j, row in enumerate(layout):
        for i, c in enumerate(row):
            if c != ".":
                continue
            for name, (x0, y0, x1, y1) in boxes:
                if x0 <= i + 0.5 < x1 and y0 <= j + 0.5 < y1:
                    rooms[name].add((i, j))
                    break
            else:
                rooms[DEFAULT_ROOM].add((i, j))
    return rooms


LAYOUT = _layout()
ROOMS = _rooms(LAYOUT)
OBJECTS = {(int(_cells(x)), int(_cells(y))): name for (x, y), name in OBJECTS_FT.items()}


def build_home_world() -> GridWorld:
    return GridWorld(
        layout=LAYOUT,
        rooms=ROOMS,
        objects=dict(OBJECTS),
        robot_x=int(_cells(START_FT[0])),
        robot_y=int(_cells(START_FT[1])),
        heading=START_HEADING,
    )
