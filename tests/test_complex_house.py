"""
tests/test_complex_house.py -- sim/maps/complex_house.py, 3.46's test
house. Pins what the inventory sweep and the user's "complex as well"
depend on: every room reachable by the real chassis, the loop, the dead
ends, and the small things on the floor where the robot can see them.
"""
from collections import deque

import pytest

from sim.maps import build_world
from sim.maps.complex_house import (INSTANCES, LAYOUT, OBJECTS, ROOMS, SMALL_NAMES,
                                    build_complex_world)


def _free():
    return {(i, j) for j, row in enumerate(LAYOUT) for i, c in enumerate(row)
            if c == "." and (i, j) not in OBJECTS}


def _reach(free, start):
    seen, queue = {start}, deque([start])
    while queue:
        i, j = queue.popleft()
        for n in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
            if n in free and n not in seen:
                seen.add(n)
                queue.append(n)
    return seen


def test_it_is_registered():
    assert build_world("complex_house").map_name == "complex_house"


def test_the_outside_is_closed():
    assert set(LAYOUT[0]) == {"#"} and set(LAYOUT[-1]) == {"#"}
    assert all(row[0] == "#" and row[-1] == "#" for row in LAYOUT)


def test_every_room_is_reachable_around_the_furniture():
    world = build_complex_world()
    seen = _reach(_free(), (world.robot_x, world.robot_y))
    assert not {name for name, cells in ROOMS.items() if cells and not cells & seen}


def test_the_rover_reaches_every_room():
    """The real chassis, on ground truth (`tests/chassis_fit.py`, 3.21)."""
    pytest.importorskip("scipy")
    from tests.chassis_fit import SAFETY_MARGIN_M, UGV_ROVER, fit
    got = fit("complex_house", *UGV_ROVER, SAFETY_MARGIN_M)
    assert got["start_free"]
    assert not [name for name, r in got["rooms"].items() if not r["reached"]]


def test_there_is_a_loop():
    """Living room -> kitchen -> hall -> living room: closing the living
    room's door to the hall leaves both still joined, through the kitchen."""
    free = _free()
    living = next(iter(ROOMS["living room"] & free))
    hall = next(iter(ROOMS["hall"] & free))
    door = {c for c in ROOMS["living room"] | ROOMS["hall"]
            if c in free and c[1] in (min(j for _, j in ROOMS["hall"]) - 1,
                                      min(j for _, j in ROOMS["hall"]))
            and c[0] < min(i for i, _ in ROOMS["kitchen"])}
    assert hall in _reach(free - door, living)


@pytest.mark.parametrize("room", ["study", "closet", "ensuite"])
def test_the_dead_ends_have_one_way_in(room):
    """Each is entered by one doorway: the floor cells touching the rest of
    the house form a single run."""
    cells = ROOMS[room]
    free = _free() | {c for c in cells}
    edge = {c for c in cells for n in ((c[0] + 1, c[1]), (c[0] - 1, c[1]),
                                        (c[0], c[1] + 1), (c[0], c[1] - 1))
            if n in free and n not in cells}
    assert edge and len(_reach(edge, next(iter(edge)))) == len(edge)


def test_small_things_are_one_cell_each_and_off_the_walls():
    small = [(name, cells) for name, cells in INSTANCES if name in SMALL_NAMES]
    assert len(small) >= 15
    assert all(len(cells) == 1 for _, cells in small)
    assert all(LAYOUT[j][i] == "." for _, cells in small for i, j in cells)


def test_a_table_is_one_instance_on_four_legs():
    tables = [cells for name, cells in INSTANCES if name == "dining table"]
    assert len(tables) == 1 and len(tables[0]) == 4
