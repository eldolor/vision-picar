"""
home_first_floor.py -- the user's own house, first floor, as a simulation.

**Where the numbers come from.** The EXTERIOR is the measured floor-plan
sketch in the home's 2012 appraisal (page 29): every wall segment below is a
dimension printed on that sketch, taken by physical measurement. It closes
to ~1,513 ft^2 of living area against the appraisal's 1,483 ft^2 (2%), and
the built-in garage to 20.2 x 21.2 ft exactly.

**What is NOT measured.** The sketch draws only the outside walls and prints
room NAMES where the rooms are. The interior was inferred from those names,
and the user CONFIRMED the layout on 2026-09-26 ("You are accurate, except
the kitchen leads straight to the garage with a pantry to the left and the
laundry room to the right" -- now built). What stays PROVISIONAL is exact:
where each interior wall and doorway sits to the foot, and the staircase,
whose position the user has not given.

**The furniture is typical, not surveyed** -- the user asked for "typical
furniture", placed by room at common sizes. How each piece meets a 20 cm
robot is the modelling choice that matters: anything that reaches the floor
(sofas, cabinets, counters, the island, appliances, a car) is SOLID; a table
is its four LEGS, because a floor robot can drive under a dining table and
the lidar sees only the legs; chairs are solid, as they are pushed in round
the table.

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
    # garage, so they stay walls): the only door is at the end of the hall
    # from the kitchen, below
    (32.0, 25.5, 40.2, 25.5), (43.2, 25.5, 48.7, 25.5),   # CONFIRMED by the user: kitchen -> garage
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
    # CONFIRMED by the user 2026-09-26: "the kitchen leads straight to the
    # garage with a pantry to the left and the laundry room to the right".
    # A short hall (x 40-43.5) runs from the kitchen to the garage door;
    # positions and door widths are still PROVISIONAL.
    (36.0, 19.0, 40.0, 19.0), (43.5, 19.0, 48.7, 19.0),   # kitchen side: the hall is open to it
    (36.0, 19.0, 36.0, 25.5),                             # pantry's west wall
    (40.0, 19.0, 40.0, 20.5), (40.0, 23.5, 40.0, 25.5),   # pantry door 20.5-23.5, off the hall
    (43.5, 19.0, 43.5, 20.5), (43.5, 23.5, 43.5, 25.5),   # laundry door 20.5-23.5, off the hall
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
    ("pantry", (36.0, 19.0, 40.0, 25.5)),
    ("garage hall", (40.0, 19.0, 43.5, 25.5)),
    ("laundry", (43.5, 19.0, 48.7, 25.5)),
    ("garage", (32.0, 25.5, 52.2, 46.7)),
]
DEFAULT_ROOM = "hall"

# The robot starts in the foyer, facing into the house (PROVISIONAL).
START_FT = (16.0, 36.0)
START_HEADING = Heading.N

# ---------------- TYPICAL FURNITURE (placed, not surveyed) ----------------
# (name, x0, y0, x1, y1, kind) in feet. kind "solid" fills the rectangle;
# "legs" puts one leg at each corner and leaves the middle drivable.
FURNITURE_FT = [
    # stairs up to the second floor -- a wall to this robot. PROVISIONAL:
    # the user has not said where they are; a foyer staircase is typical.
    ("staircase", 18.5, 22.0, 22.0, 32.5, "solid"),
    # den: a desk and its chair, a bookcase on the wall it shares with the family room
    ("desk", 1.0, 7.0, 6.0, 9.5, "solid"),
    ("desk chair", 3.0, 10.0, 4.5, 11.5, "solid"),
    ("bookcase", 10.5, 7.5, 12.0, 14.0, "solid"),
    # half bath
    ("toilet", 1.0, 17.0, 2.5, 19.5, "solid"),
    ("vanity", 0.0, 20.5, 2.0, 22.5, "solid"),
    # living room
    ("sofa", 0.0, 27.0, 3.0, 34.5, "solid"),
    ("coffee table", 4.5, 28.5, 7.0, 33.0, "solid"),
    ("armchair", 8.0, 25.0, 10.5, 27.5, "solid"),
    ("armchair", 8.0, 35.0, 10.5, 37.5, "solid"),
    ("side table", 0.5, 35.5, 2.0, 37.0, "solid"),
    # family room: an L sectional, coffee table, TV console, an armchair
    ("sectional sofa", 13.0, 3.0, 16.0, 13.0, "solid"),
    ("sectional sofa", 13.0, 13.0, 20.0, 16.0, "solid"),
    ("coffee table", 17.5, 7.0, 21.5, 10.0, "solid"),
    ("tv console", 17.0, 0.0, 25.0, 1.5, "solid"),
    ("armchair", 23.0, 12.0, 25.5, 14.5, "solid"),
    # breakfast nook: a table on four legs, four chairs
    ("breakfast table", 31.25, 10.75, 34.75, 14.25, "legs"),
    ("chair", 32.25, 9.0, 33.75, 10.5, "solid"),
    ("chair", 32.25, 14.5, 33.75, 16.0, "solid"),
    ("chair", 29.5, 11.75, 31.0, 13.25, "solid"),
    ("chair", 35.0, 11.75, 36.5, 13.25, "solid"),
    # kitchen: counters on the back and right walls, the fridge in the run, an island
    ("kitchen counter", 38.0, 6.0, 48.7, 8.0, "solid"),
    ("refrigerator", 45.7, 8.0, 48.7, 10.5, "solid"),
    ("kitchen counter", 46.7, 10.5, 48.7, 17.0, "solid"),
    ("kitchen island", 40.0, 11.0, 43.0, 16.0, "solid"),
    # dining room: a table for six on four legs, six chairs
    ("dining table", 25.25, 30.8, 28.75, 36.8, "legs"),
    ("chair", 23.5, 31.3, 25.0, 32.8, "solid"),
    ("chair", 23.5, 34.8, 25.0, 36.3, "solid"),
    ("chair", 29.0, 31.3, 30.5, 32.8, "solid"),
    ("chair", 29.0, 34.8, 30.5, 36.3, "solid"),
    ("chair", 26.25, 29.0, 27.75, 30.5, "solid"),
    ("chair", 26.25, 37.1, 27.75, 38.6, "solid"),
    # pantry shelving; washer and dryer
    ("pantry shelves", 36.0, 19.0, 37.5, 25.5, "solid"),
    ("washer", 46.0, 19.5, 48.7, 22.0, "solid"),
    ("dryer", 46.0, 22.3, 48.7, 24.8, "solid"),
    # garage: one car in the left bay, shelving on the right wall
    ("car", 33.5, 29.0, 40.0, 45.0, "solid"),
    ("garage shelves", 50.5, 27.0, 52.2, 40.0, "solid"),
]
# The mission target -- a thing to find, on the floor. PROVISIONAL.
TARGET_FT = ((44.5, 17.8), "red backpack")     # kitchen floor, by the hall to the garage

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


def _objects(layout):
    """Furniture and the target, as the solid cells GridWorld wants.

    A cell belongs to a piece when its centre falls inside the piece's
    rectangle ("solid"), or when it holds one of the four corners ("legs").
    A piece never claims a wall cell.
    """
    objects = {}
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
        for i, j in cells:
            if layout[j][i] == ".":
                objects[(i, j)] = name
    (tx, ty), target = TARGET_FT
    objects[(int(_cells(tx)), int(_cells(ty)))] = target
    return objects


LAYOUT = _layout()
ROOMS = _rooms(LAYOUT)
OBJECTS = _objects(LAYOUT)


def build_home_world() -> GridWorld:
    return GridWorld(
        layout=LAYOUT,
        rooms=ROOMS,
        objects=dict(OBJECTS),
        robot_x=int(_cells(START_FT[0])),
        robot_y=int(_cells(START_FT[1])),
        heading=START_HEADING,
    )
