"""
scaled_house.py -- a house at real-world proportions (PLAN-ros-alignment.md R6).

The starter house's doors are ONE grid cell: 0.30 m. That was sized for the
retired PiCar-X's one-move step, and the chassis chosen was 0.228 x 0.198 m
across its wheels -- about 5 cm of margin per side (the UGV Rover, since
PLAN-ros-alignment 3.21, is 0.253 x 0.231: 3.5 cm). nav2, planning
with that footprint on a 5 cm SLAM map, sealed a starter-house door whenever
SLAM drew a jamb one cell thick: 5 of 6 goals on one run, 1 of 6 on the next.
Real interior doors are 70-90 cm. So this house keeps the same 0.30 m cell
(every verb, collar and test in the repo is measured against it) and gives
the rooms, the hallway and the doors the sizes a real house has:

    ###########################
    #..........#....#.........#
    #.S........#....#.......F.#
    #..........D....D.........#
    #..........D....D.........#
    #..........D....D......B..#
    #..........#....#.........#
    #..........#....#.........#
    ############....###########
    #..........D....D.........#
    #.b........D....D.........#
    #..........D....D.........#
    #..........#....#.........#
    ###########################

Living room 3.0 x 2.1 m, hallway 1.2 m wide, kitchen 2.7 x 2.1 m, doors
0.9 m (three cells), bedroom and study below.
S sofa, F refrigerator, B the red backpack, b bed. The robot starts in the
living room facing east. Selected with SIM_MAP=scaled_house (robot/factory.py);
the starter house stays the default and every test written against it stands.
"""

from sim.grid_world import GridWorld, Heading
from sim.movers import Mover

_ROOM_ROW = "#..........#....#.........#"
_DOOR_ROW = "#..........D....D.........#"

LAYOUT = [
    "###########################",
    _ROOM_ROW,
    _ROOM_ROW,
    _DOOR_ROW,
    _DOOR_ROW,
    _DOOR_ROW,
    _ROOM_ROW,
    _ROOM_ROW,
    "############....###########",
    _DOOR_ROW,
    _DOOR_ROW,
    _DOOR_ROW,
    _ROOM_ROW,
    "###########################",
]

LIVING_ROOM = {(x, y) for x in range(1, 11) for y in range(1, 8)}
HALLWAY = {(x, y) for x in range(12, 16) for y in range(1, 13)}
KITCHEN = {(x, y) for x in range(17, 26) for y in range(1, 8)}
BEDROOM = {(x, y) for x in range(1, 11) for y in range(9, 13)}
STUDY = {(x, y) for x in range(17, 26) for y in range(9, 13)}

ROOMS = {
    "living room": LIVING_ROOM,
    "hallway": HALLWAY,
    "kitchen": KITCHEN,
    "bedroom": BEDROOM,
    "study": STUDY,
}

OBJECTS = {
    (2, 2): "sofa",
    (24, 2): "refrigerator",
    (23, 5): "red backpack",
    (2, 10): "bed",
}


def build_scaled_world() -> GridWorld:
    return GridWorld(
        layout=LAYOUT,
        rooms=ROOMS,
        objects=dict(OBJECTS),
        robot_x=6,
        robot_y=4,
        heading=Heading.E,
    )


# PLAN-ros-alignment.md 3.30 -- people and pets, by SIM_MOVERS=<name>. Each
# value builds a fresh list: a Mover carries its own position and clock.
MOVERS = {
    # A person pacing across the hallway on the middle door row, from the
    # living-room door to the kitchen door and back, one cell a second --
    # straight across every route between the two rooms.
    "hallway_crossing": lambda: [
        Mover("person", [(12, 4), (13, 4), (14, 4), (15, 4), (14, 4), (13, 4)], hop_s=1.0),
    ],
    # 3.31, criterion 3: someone standing in the middle of the kitchen
    # door -- the only way into the room with the backpack -- for 60 s, then
    # walking into the kitchen and staying there. The door is three cells
    # (0.9 m); with the middle one taken, neither gap beside it fits the
    # chassis and nav2's inflation, so the kitchen is unreachable until
    # they go. Into the kitchen, away from a robot waiting in the hallway,
    # so the keep-out never makes them wait for it.
    "kitchen_door_sitter": lambda: [
        Mover("person", [(16, 4), (17, 4), (18, 4), (19, 4), (19, 3), (19, 2)],
              hop_s=1.0, start_s=60.0, loop=False),
    ],
}
