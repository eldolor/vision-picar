"""
tests/test_home_map.py -- the user's own house (sim/maps/home_first_floor.py).

What is pinned is what was MEASURED: the outline and the garage come from the
home's appraisal sketch, so they must keep agreeing with the appraisal's own
figures. The interior is provisional and pinned only for what any correction
must preserve -- every room reachable from the start.
"""

from collections import deque

from sim.maps.home_first_floor import (
    GARAGE_FT, HOUSE_FT, LAYOUT, OBJECTS, ROOMS, build_home_world)

APPRAISAL_LIVING_FT2 = 1483.05      # first floor, the sketch's own area table
APPRAISAL_GARAGE_FT2 = 428.24


def _shoelace(poly):
    return abs(sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]))) / 2


def test_the_outline_matches_the_appraisal_within_three_percent():
    """The drawn segments close to 1,513 ft^2 against the table's 1,483 --
    the sketch's small corner notches are not in the outline."""
    assert abs(_shoelace(HOUSE_FT) - APPRAISAL_LIVING_FT2) / APPRAISAL_LIVING_FT2 < 0.03


def test_the_garage_is_the_measured_twenty_by_twenty_one():
    x0, y0, x1, y1 = GARAGE_FT
    assert round(x1 - x0, 1) == 20.2 and round(y1 - y0, 1) == 21.2
    assert abs((x1 - x0) * (y1 - y0) - APPRAISAL_GARAGE_FT2) < 1.0


def test_every_room_is_reachable_from_the_start():
    world = build_home_world()
    floor = {(i, j) for j, row in enumerate(LAYOUT) for i, c in enumerate(row) if c == "."}
    seen, queue = {(world.robot_x, world.robot_y)}, deque([(world.robot_x, world.robot_y)])
    while queue:
        i, j = queue.popleft()
        for n in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
            if n in floor and n not in seen:
                seen.add(n)
                queue.append(n)
    assert seen == floor
    assert all(cells and cells <= seen for cells in ROOMS.values())


def test_the_outside_is_closed():
    assert set(LAYOUT[0]) == {"#"} and set(LAYOUT[-1]) == {"#"}
    assert all(row[0] == "#" and row[-1] == "#" for row in LAYOUT)


def test_objects_sit_on_floor():
    assert all(LAYOUT[j][i] == "." for (i, j) in OBJECTS)
