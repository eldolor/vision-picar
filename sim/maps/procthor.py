"""
procthor.py -- ProcTHOR-10K houses as simulator houses
(PLAN-ros-alignment.md 3.66). Selected with SIM_MAP=procthor:val:<index>, or
procthor:test:<index> for the `test` split (3.65 amendment 3: houses kept
out of every tuning run).

**Why.** Four hand-drawn houses are four chances to tune a policy to a
house. ProcTHOR-10K (AI2, Apache-2.0, `procthor_data/LICENSE`) is ten
thousand procedurally generated houses; this module takes the `val`
split's floor plans -- rooms, walls, doors and what stands on the floor --
and rasterises them into the same 0.30 m `GridWorld` every other house
uses. Only the floor plan crosses over: no ProcTHOR image, renderer or
physics, so there is still one simulator.

The data is `procthor_data/val.json.gz`, written by
`tools/procthor_import.py` from pinned sources; the suite never fetches.

Conventions, so a number here can be checked against the source:
* ProcTHOR's x -> grid x, its z -> grid y, one cell of wall round the
  outside. (A mirror image of AI2-THOR's top view; distances and topology
  are unchanged.)
* A wall is drawn one cell thick round its centre line, as in
  `home_first_floor`; a door is a gap in it. Open-plan (`empty`) walls are
  not drawn; the front door stays shut.
* Tables are their four legs (a floor robot drives under them); everything
  else on the floor is solid. A piece smaller than a cell takes the cell
  under its centre; one the wall covers takes the nearest floor cell.
* The target, `red backpack`, is placed by a seed from the house index in
  a room other than the start's, away from walls and door gaps.
"""

import gzip
import json
import math
import os
import random
import re
from collections import deque
from dataclasses import dataclass, field

from sim.grid_world import GridWorld, Heading

CELL_M = 0.30
SPLITS = ("val", "test")
_DIR = os.path.join(os.path.dirname(__file__), "procthor_data")
DATA = os.path.join(_DIR, "val.json.gz")
# The usable houses (three or more rooms; the Rover, with the safety margin,
# starts free and reaches every room), as measured by
# evaluations/procthor-366/measure.py on tests/chassis_fit.py.
USABLE = os.path.join(_DIR, "usable_val.json")


def data_path(split: str = "val") -> str:
    return os.path.join(_DIR, f"{split}.json.gz")


def usable_path(split: str = "val") -> str:
    return os.path.join(_DIR, f"usable_{split}.json")
TARGET = "red backpack"
LEGGED = {"DiningTable", "SideTable", "CoffeeTable", "Desk"}
# A leg sits this far in from the footprint's corner, metres.
LEG_INSET_M = 0.09
_NAMES = {"BasketBall": "basketball", "TVStand": "tv stand"}

_houses = {}


def houses(split: str = "val") -> list:
    """Every reduced house of one split, loaded once."""
    if split not in _houses:
        with gzip.open(data_path(split), "rt") as f:
            _houses[split] = json.load(f)["houses"]
    return _houses[split]


def usable(split: str = "val") -> list:
    """Indices of the usable houses, ascending: a sweep takes the first N."""
    with open(usable_path(split)) as f:
        return json.load(f)


def parse_name(name: str):
    """'procthor:val:12' -> 12, or None for a name that is not one of these."""
    m = re.fullmatch(r"procthor:val:(\d+)", name)
    return int(m.group(1)) if m else None


def parse_split_name(name: str):
    """'procthor:test:12' -> ('test', 12); 'procthor:val:12' -> ('val', 12);
    None for a name that is not one of these."""
    m = re.fullmatch(r"procthor:(val|test):(\d+)", name)
    return (m.group(1), int(m.group(2))) if m else None


def object_name(kind: str) -> str:
    if kind in _NAMES:
        return _NAMES[kind]
    return re.sub(r"(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])", " ", kind).lower()


def _inside(poly, x, y) -> bool:
    inside = False
    for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            inside = not inside
    return inside


def _segment_cells(seg, half_width, width, height):
    """Cells whose centres lie within `half_width` (cells) of a segment."""
    x0, y0, x1, y1 = seg
    dx, dy = x1 - x0, y1 - y0
    L2 = dx * dx + dy * dy
    out = []
    for j in range(max(0, int(min(y0, y1) - 1)), min(height, int(max(y0, y1) + 2))):
        for i in range(max(0, int(min(x0, x1) - 1)), min(width, int(max(x0, x1) + 2))):
            px, py = i + 0.5, j + 0.5
            t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / L2))
            if math.hypot(px - (x0 + t * dx), py - (y0 + t * dy)) <= half_width:
                out.append((i, j))
    return out


@dataclass
class Conversion:
    """One house, converted, and what a judge needs to check the conversion
    against its source (3.66's criteria 1-3)."""
    index: int
    world: GridWorld
    room_ids: dict                      # grid room name -> ProcTHOR room id
    joins: set                          # ProcTHOR's own room-id pairs
    floor_objects: int                  # source objects on the floor
    placed_objects: int                 # ... of which claimed a cell
    source_area_m2: float
    outline_area_m2: float              # cells (floor or wall) inside the polygons
    door_cells: set = field(default_factory=set)

    def grid_joins(self) -> set:
        """Room-id pairs whose floor cells touch (4-adjacent), objects
        ignored: the grid's side of criterion 1, to compare with `joins`."""
        owner = {}
        for name, cells in self.world.rooms.items():
            for cell in cells:
                owner[cell] = self.room_ids[name]
        joins = set()
        for (i, j), a in owner.items():
            for n in ((i + 1, j), (i, j + 1)):
                b = owner.get(n)
                if b is not None and b != a:
                    joins.add(tuple(sorted((a, b))))
        return joins


def convert(index: int, split: str = "val") -> Conversion:
    h = houses(split)[index]
    xs = [p[0] for r in h["rooms"] for p in r["poly"]]
    zs = [p[1] for r in h["rooms"] for p in r["poly"]]
    ox, oz = min(xs), min(zs)

    def c(x, z):
        return (x - ox) / CELL_M + 1.0, (z - oz) / CELL_M + 1.0

    width = int(math.ceil((max(xs) - ox) / CELL_M)) + 2
    height = int(math.ceil((max(zs) - oz) / CELL_M)) + 2
    polys = [[c(*p) for p in r["poly"]] for r in h["rooms"]]

    inside = [[any(_inside(p, i + 0.5, j + 0.5) for p in polys) for i in range(width)]
              for j in range(height)]
    wall = set()
    door_cells = set()
    for ax, az, bx, bz, holes in h["walls"]:
        (x0, y0), (x1, y1) = c(ax, az), c(bx, bz)
        length = math.hypot(bx - ax, bz - az)
        cuts, t = [], 0.0
        for t0, t1 in holes:
            cuts.append((t, t0))
            t = max(t, t1)
            hx0, hy0 = x0 + (x1 - x0) * t0 / length, y0 + (y1 - y0) * t0 / length
            hx1, hy1 = x0 + (x1 - x0) * t1 / length, y0 + (y1 - y0) * t1 / length
            door_cells.update(_segment_cells((hx0, hy0, hx1, hy1), 1.0, width, height))
        cuts.append((t, length))
        for s0, s1 in cuts:
            if s1 <= s0:
                continue
            seg = (x0 + (x1 - x0) * s0 / length, y0 + (y1 - y0) * s0 / length,
                   x0 + (x1 - x0) * s1 / length, y0 + (y1 - y0) * s1 / length)
            wall.update(_segment_cells(seg, 0.5, width, height))

    layout = ["".join("." if inside[j][i] and (i, j) not in wall else "#" for i in range(width))
              for j in range(height)]

    def floor(i, j):
        return 0 <= i < width and 0 <= j < height and layout[j][i] == "."

    # Rooms, named by type; a second of a type is "bedroom 2".
    rooms, room_ids, seen = {}, {}, {}
    for r, poly in zip(h["rooms"], polys):
        base = object_name(r["type"])
        seen[base] = seen.get(base, 0) + 1
        name = base if seen[base] == 1 else f"{base} {seen[base]}"
        rooms[name] = {(i, j) for j in range(height) for i in range(width)
                       if floor(i, j) and _inside(poly, i + 0.5, j + 0.5)}
        room_ids[name] = r["id"]

    objects, placed = {}, 0
    for o in h["objects"]:
        cx, cy = c(o["x"], o["z"])
        a = math.radians(o["yaw"])
        ca, sa = math.cos(a), math.sin(a)
        hx, hz = o["sx"] / 2 / CELL_M, o["sz"] / 2 / CELL_M
        name = object_name(o["type"])
        if o["type"] in LEGGED:
            ix, iz = hx - LEG_INSET_M / CELL_M, hz - LEG_INSET_M / CELL_M
            # local x -> (cos, -sin), local z -> (sin, cos): Unity's yaw, mirrored
            cells = {(int(cx + lx * ca + lz * sa), int(cy - lx * sa + lz * ca))
                     for lx in (-ix, ix) for lz in (-iz, iz)}
        else:
            r = math.hypot(hx, hz) + 1
            cells = set()
            for j in range(int(cy - r), int(cy + r) + 1):
                for i in range(int(cx - r), int(cx + r) + 1):
                    dx, dy = i + 0.5 - cx, j + 0.5 - cy
                    if abs(dx * ca - dy * sa) <= hx and abs(dx * sa + dy * ca) <= hz:
                        cells.add((i, j))
            if not cells:
                cells = {(int(cx), int(cy))}
        cells = {cell for cell in cells if floor(*cell)}
        if not cells:
            # ProcTHOR's walls have no thickness and its furniture stands
            # flush against them; drawn a cell thick here, a wall can cover
            # a shallow piece entirely. It takes the nearest floor cell.
            near = [(math.hypot(i + 0.5 - cx, j + 0.5 - cy), (i, j))
                    for j in range(int(cy) - 2, int(cy) + 3) for i in range(int(cx) - 2, int(cx) + 3)
                    if floor(i, j)]
            near = [cell for d, cell in sorted(near) if d <= 1.5]
            cells = set(near[:1])
        if cells:
            placed += 1
        for cell in cells:
            objects.setdefault(cell, name)

    free = {(i, j) for j in range(height) for i in range(width)
            if floor(i, j) and (i, j) not in objects}

    def spacious(cell):
        i, j = cell
        return all((i + di, j + dj) in free for di in (-1, 0, 1) for dj in (-1, 0, 1))

    sx, sy = c(*h["start"])
    start = min((cell for cell in free if spacious(cell)),
                key=lambda cell: (math.hypot(cell[0] + 0.5 - sx, cell[1] + 0.5 - sy), cell),
                default=None)
    if start is None:
        raise ValueError(f"procthor:{split}:{index} has no free cell to start on")

    reach, queue = {start}, deque([start])
    while queue:
        i, j = queue.popleft()
        for n in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
            if n in free and n not in reach:
                reach.add(n)
                queue.append(n)
    start_room = next((n for n, cells in rooms.items() if start in cells), None)
    rng = random.Random(index)
    options = {n: sorted(cell for cell in cells & reach
                         if spacious(cell) and cell not in door_cells and cell != start)
               for n, cells in rooms.items() if n != start_room}
    options = {n: v for n, v in options.items() if v}
    if not options and start_room:
        options = {start_room: sorted(cell for cell in rooms[start_room] & reach
                                      if spacious(cell) and cell != start)}
        options = {n: v for n, v in options.items() if v}
    if options:
        room = rng.choice(sorted(options))
        objects[rng.choice(options[room])] = TARGET

    source_area = sum(_area(r["poly"]) for r in h["rooms"])
    outline = sum(1 for j in range(height) for i in range(width) if inside[j][i]) * CELL_M ** 2
    world = GridWorld(layout=layout, rooms=rooms, objects=objects,
                      robot_x=start[0], robot_y=start[1], heading=Heading.N)
    return Conversion(index=index, world=world, room_ids=room_ids,
                      joins={tuple(j) for j in h["joins"]},
                      floor_objects=len(h["objects"]), placed_objects=placed,
                      source_area_m2=source_area, outline_area_m2=outline,
                      door_cells=door_cells)


def _area(poly) -> float:
    return abs(sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]))) / 2


def build_procthor_world(index: int, split: str = "val") -> GridWorld:
    return convert(index, split).world
