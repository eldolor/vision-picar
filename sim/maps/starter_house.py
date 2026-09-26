"""
starter_house.py

A small hand-built house for early testing:

    #############
    #...#.......#
    #...D.......#
    #...#.......#
    #####.......#
    #.......#####
    #.......#...#
    #.......D...#
    #.......#...#
    #############

Living room (left top), hallway (right, large open area), kitchen
(bottom-right small room). One red backpack hidden in the kitchen so
Phase 6 (object search) has something to find.
"""

from sim.grid_world import GridWorld, Heading

LAYOUT = [
    "#############",
    "#...#.......#",
    "#...D.......#",
    "#...#.......#",
    "#####.......#",
    "#.......#####",
    "#.......#...#",
    "#.......D...#",
    "#.......#...#",
    "#############",
]

LIVING_ROOM = {(x, y) for x in range(1, 4) for y in range(1, 4)}
HALLWAY = {(x, y) for x in range(5, 12) for y in range(1, 8)} | {
    (x, 4) for x in range(1, 5)
} | {(4, 2), (8, 7)}  # doorway cells count as the hallway side of the door
KITCHEN = {(x, y) for x in range(9, 12) for y in range(6, 9)}

ROOMS = {
    "living room": LIVING_ROOM,
    "hallway": HALLWAY,
    "kitchen": KITCHEN,
}

OBJECTS = {
    (10, 7): "red backpack",
    # In the living room's corner since 2026-09-26. It was at (2, 2) -- the
    # robot's start cell -- which was harmless while objects were labels on
    # the floor and impossible once they became solid (PLAN-ros-alignment.md
    # 3.9): the robot would have started inside it.
    (1, 1): "sofa",
    (10, 6): "refrigerator",
}


def build_starter_world() -> GridWorld:
    return GridWorld(
        layout=LAYOUT,
        rooms=ROOMS,
        objects=dict(OBJECTS),
        robot_x=2,
        robot_y=2,
        heading=Heading.E,
    )
