"""
complex_house.py -- a larger, harder house with small things on the floor
(PLAN-ros-alignment.md 3.46). Selected with SIM_MAP=complex_house.

**Why it exists.** 3.46's object inventory is judged on objects the size
of real search targets, and `home_first_floor` has one (the backpack); the
rest is furniture. The user asked (2026-10-07) for the new house to be
complex as well as cluttered. It is a test house, not anyone's home:
realistic sizes, typical furniture, nothing surveyed.

What makes it harder than `home_first_floor`, on purpose:

* **A loop.** Living room -> kitchen (a wide opening) -> the hall -> back
  to the living room. A robot that goes round it gives SLAM a loop closure,
  which 3.46's criterion 4 needs to measure.
* **An L-shaped hall**: a 4 ft corridor that turns into the entry.
* **Dead ends**: a study reached only through the dining room, a closet
  inside bedroom 1, an ensuite inside the main bedroom.
* **A room that is not a rectangle**: bedroom 2 wraps round the bathroom.
* **Small objects on the floor**: one cell each (0.30 m, the grid's
  resolution, so a mug is as big as a shoe here), solid like everything
  else since 3.9, placed where things get dropped and never in a doorway.

Doors are 3 ft, the same convention as `home_first_floor` (a wall is drawn
one cell thick round its centre line, so the clear gap is ~0.6 m, and the
Rover's 0.231 m fits; `tests/test_complex_house.py` checks every room is
reachable by the real chassis).

Coordinates are FEET, x to the right and y DOWN (the front door at the
bottom), rasterised into the 0.30 m grid exactly as `home_first_floor` does.
"""

import math

from sim.grid_world import GridWorld, Heading
from sim.maps.home_first_floor import _inside, _near_segment

CELL_M = 0.30
FT_M = 0.3048
MARGIN_FT = 1.0

# The outline: 60 x 40 ft, plus the entry's 4 ft bump-out at the front left.
HOUSE_FT = [
    (0.0, 0.0), (60.0, 0.0), (60.0, 40.0), (10.0, 40.0),
    (10.0, 44.0), (0.0, 44.0),
]

# Interior walls as (x0, y0, x1, y1) centre lines; a doorway is a gap.
WALLS_FT = [
    # north rooms / the hall: doors to the living room (14-17) and the
    # kitchen (30-33); the study and the main bedroom close it above x 33
    (0.0, 20.0, 14.0, 20.0), (17.0, 20.0, 30.0, 20.0), (33.0, 20.0, 60.0, 20.0),
    # living | kitchen: a wide opening, y 4-14 -- one side of the loop
    (22.0, 0.0, 22.0, 4.0), (22.0, 14.0, 22.0, 20.0),
    # kitchen | dining: an opening, y 3-9; kitchen | study: a wall
    (40.0, 0.0, 40.0, 3.0), (40.0, 9.0, 40.0, 20.0),
    # dining | study: the study's only door, x 50-53
    (40.0, 12.0, 50.0, 12.0), (53.0, 12.0, 60.0, 12.0),
    # the hall's south wall: bedroom 1 (18-21), bath (26-29), bedroom 2 (37-40)
    (10.0, 24.0, 18.0, 24.0), (21.0, 24.0, 26.0, 24.0),
    (29.0, 24.0, 37.0, 24.0), (40.0, 24.0, 46.0, 24.0),
    # entry | bedroom 1 -- the hall turns south into the entry at x < 10
    (10.0, 24.0, 10.0, 40.0),
    # bedroom 1's closet, door y 35-38
    (10.0, 34.0, 14.0, 34.0), (14.0, 34.0, 14.0, 35.0), (14.0, 38.0, 14.0, 40.0),
    # bedroom 1 | bath and bedroom 2; the bath's two inner walls
    (24.0, 24.0, 24.0, 40.0),
    (31.0, 24.0, 31.0, 34.0), (24.0, 34.0, 31.0, 34.0),
    # the main bedroom: its door ends the hall (y 20.5-23.5)
    (46.0, 20.0, 46.0, 20.5), (46.0, 23.5, 46.0, 40.0),
    # the ensuite, door x 53-56
    (52.0, 32.0, 52.0, 40.0), (52.0, 32.0, 53.0, 32.0), (56.0, 32.0, 60.0, 32.0),
]

# Rooms in the order they claim cells: a room listed earlier wins a cell,
# which is how bedroom 2 becomes an L round the bathroom.
ROOMS_FT = [
    ("living room", (0.0, 0.0, 22.0, 20.0)),
    ("kitchen", (22.0, 0.0, 40.0, 20.0)),
    ("dining room", (40.0, 0.0, 60.0, 12.0)),
    ("study", (40.0, 12.0, 60.0, 20.0)),
    ("hall", (10.0, 20.0, 46.0, 24.0)),
    ("entry", (0.0, 20.0, 10.0, 44.0)),
    ("closet", (10.0, 34.0, 14.0, 40.0)),
    ("bedroom 1", (10.0, 24.0, 24.0, 40.0)),
    ("bathroom", (24.0, 24.0, 31.0, 34.0)),
    ("bedroom 2", (24.0, 24.0, 46.0, 40.0)),
    ("ensuite", (52.0, 32.0, 60.0, 40.0)),
    ("main bedroom", (46.0, 20.0, 60.0, 40.0)),
]
DEFAULT_ROOM = "hall"

# The robot starts in the entry, facing into the house.
START_FT = (5.0, 36.0)
START_HEADING = Heading.N

# (name, x0, y0, x1, y1, kind): "solid" fills the rectangle, "legs" puts a
# leg at each corner and leaves the middle drivable.
FURNITURE_FT = [
    # living room
    ("sofa", 0.0, 6.0, 3.0, 15.0, "solid"),
    ("coffee table", 5.0, 8.0, 8.0, 13.0, "solid"),
    ("armchair", 10.0, 2.0, 12.5, 4.5, "solid"),
    ("armchair", 14.0, 9.0, 16.5, 11.5, "solid"),
    ("tv console", 6.0, 0.0, 14.0, 1.5, "solid"),
    ("bookcase", 0.0, 18.5, 6.0, 20.0, "solid"),
    # kitchen
    ("kitchen counter", 24.0, 0.0, 38.0, 2.0, "solid"),
    ("refrigerator", 38.0, 0.0, 40.0, 2.5, "solid"),
    ("kitchen island", 28.0, 8.0, 34.0, 11.0, "solid"),
    # dining room: a table on legs and six chairs, a sideboard
    ("dining table", 46.0, 3.0, 54.0, 8.0, "legs"),
    ("chair", 47.0, 1.0, 48.5, 2.5, "solid"),
    ("chair", 51.5, 1.0, 53.0, 2.5, "solid"),
    ("chair", 47.0, 8.5, 48.5, 10.0, "solid"),
    ("chair", 51.5, 8.5, 53.0, 10.0, "solid"),
    ("chair", 44.0, 4.75, 45.5, 6.25, "solid"),
    ("chair", 54.5, 4.75, 56.0, 6.25, "solid"),
    ("sideboard", 58.5, 2.0, 60.0, 10.0, "solid"),
    # study
    ("desk", 41.0, 16.5, 46.0, 19.0, "solid"),
    ("desk chair", 43.0, 14.5, 44.5, 16.0, "solid"),
    ("bookcase", 58.5, 13.0, 60.0, 19.0, "solid"),
    # entry
    ("shoe bench", 0.0, 30.0, 1.5, 36.0, "solid"),
    # bedroom 1
    ("bed", 18.5, 30.0, 23.5, 37.0, "solid"),
    ("dresser", 11.0, 24.5, 16.0, 26.0, "solid"),
    # bathroom
    ("toilet", 24.5, 31.0, 26.0, 33.0, "solid"),
    ("vanity", 29.5, 24.5, 31.0, 27.5, "solid"),
    # bedroom 2
    ("bed", 40.0, 31.0, 45.5, 38.0, "solid"),
    ("desk", 25.0, 38.0, 30.0, 40.0, "solid"),
    ("dresser", 32.0, 38.5, 37.0, 40.0, "solid"),
    # main bedroom
    ("bed", 53.5, 20.5, 60.0, 27.5, "solid"),
    ("dresser", 46.5, 26.0, 48.0, 32.0, "solid"),
    ("armchair", 47.0, 36.0, 49.5, 38.5, "solid"),
    # ensuite
    ("toilet", 58.5, 33.0, 60.0, 35.0, "solid"),
    ("bathtub", 56.5, 36.0, 60.0, 40.0, "solid"),
]

# Small things on the floor, one cell each: (name, x, y) in feet. Placed
# where things get dropped, never in a doorway or the hall.
SMALL_FT = [
    ("shoes", 2.5, 25.5),
    ("umbrella", 8.0, 27.0),
    ("keys", 6.0, 42.0),
    ("tv remote", 8.5, 6.0),
    ("toy car", 12.0, 14.5),
    ("mug", 18.0, 16.5),
    ("water bottle", 35.0, 14.5),
    ("trash can", 38.5, 10.5),
    ("pet bowl", 25.0, 17.5),
    ("laptop bag", 47.5, 15.0),
    ("guitar", 56.5, 17.5),
    ("slippers", 12.5, 30.0),
    ("laundry basket", 16.5, 38.5),
    ("towel", 27.5, 30.0),
    ("headphones", 35.0, 28.0),
    ("book", 50.5, 30.0),
]
# The mission target -- on the floor of bedroom 2, round its L.
TARGET_FT = ((28.0, 36.5), "red backpack")


def _cells(ft: float) -> float:
    return (ft + MARGIN_FT) * FT_M / CELL_M


def _layout():
    width = int(math.ceil(_cells(60.0 + MARGIN_FT)))
    height = int(math.ceil(_cells(44.0 + MARGIN_FT)))
    house = [(_cells(x), _cells(y)) for x, y in HOUSE_FT]
    walls = [tuple(_cells(v) for v in w) for w in WALLS_FT]
    rows = []
    for j in range(height):
        row = []
        for i in range(width):
            cx, cy = i + 0.5, j + 0.5
            floor = _inside(house, cx, cy)
            if floor and any(_near_segment(cx, cy, w, 0.5) for w in walls):
                floor = False
            row.append("." if floor else "#")
        rows.append("".join(row))
    return rows


def _rooms(layout):
    rooms = {name: set() for name, _ in ROOMS_FT}
    rooms[DEFAULT_ROOM] = rooms.get(DEFAULT_ROOM, set())
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


def _objects(layout):
    """Furniture, small things and the target, as GridWorld's solid cells,
    and the same cells grouped by the piece they belong to (`INSTANCES`):
    a scorer needs "this table" where the grid only has cells named "dining
    table" -- a table's four legs are not four tables. A piece never claims
    a wall cell; a small thing never lands on a piece."""
    objects = {}
    instances = []
    for name, x0, y0, x1, y1, kind in FURNITURE_FT:
        cx0, cy0, cx1, cy1 = (_cells(v) for v in (x0, y0, x1, y1))
        if kind == "legs":
            eps = 0.3
            cells = {(int(cx0 + eps), int(cy0 + eps)), (int(cx1 - eps), int(cy0 + eps)),
                     (int(cx0 + eps), int(cy1 - eps)), (int(cx1 - eps), int(cy1 - eps))}
        else:
            cells = {(i, j) for j in range(int(cy0), int(math.ceil(cy1)))
                     for i in range(int(cx0), int(math.ceil(cx1)))
                     if cx0 <= i + 0.5 < cx1 and cy0 <= j + 0.5 < cy1}
        cells = {(i, j) for i, j in cells if layout[j][i] == "."}
        for cell in cells:
            objects[cell] = name
        if cells:
            instances.append((name, frozenset(cells)))
    (tx, ty), target = TARGET_FT
    for name, x, y in SMALL_FT + [(target, tx, ty)]:
        cell = (int(_cells(x)), int(_cells(y)))
        if layout[cell[1]][cell[0]] != "." or cell in objects:
            raise ValueError(f"{name} at {x}, {y} ft lands on {objects.get(cell, 'a wall')}")
        objects[cell] = name
        instances.append((name, frozenset({cell})))
    return objects, instances


LAYOUT = _layout()
ROOMS = _rooms(LAYOUT)
OBJECTS, INSTANCES = _objects(LAYOUT)
SMALL_NAMES = frozenset(name for name, _, _ in SMALL_FT) | {TARGET_FT[1]}


def build_complex_world() -> GridWorld:
    return GridWorld(
        layout=LAYOUT,
        rooms=ROOMS,
        objects=dict(OBJECTS),
        robot_x=int(_cells(START_FT[0])),
        robot_y=int(_cells(START_FT[1])),
        heading=START_HEADING,
    )
