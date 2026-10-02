"""
movers.py -- people and pets in the simulator (PLAN-ros-alignment.md 3.30).

A `Mover` is a named solid object that hops cell to cell around a closed
path, one hop per `hop_s` of SIM time (`GridWorld.sim_time`). It lives in
`GridWorld.objects` like any other object, so everything that senses or
moves -- `solid_cells`, the scan, collision, `MockWorld`'s map, the camera --
sees it with no code of its own. `GridWorld.advance_time()` is what moves it.

**The keep-out is the one rule worth reading twice.** A mover never hops
to a cell CLOSER to the robot than `MOVER_KEEPOUT_M` beyond the chassis'
turning circle -- or, when the robot has come nearer than that by itself,
closer than the mover already is. It may step past at the same distance, or
away; otherwise it waits and retries at its next hop. That keeps
responsibility clean -- a mover never closes the gap, so the robot can only
come closer than the stop line by its OWN motion, which is the thing 3.18's
ground-truth metric measures. (The first version waited whenever the next
cell was inside the keep-out at all, and a person beside a doorway froze
for good with the robot waiting on them.) It is also a fidelity limit, said
here rather than discovered: a mover never approaches the robot, so this
cannot test a pet darting at it.

Whole 30 cm hops, deterministic: the same path on the same clock is in the
same place, which is what lets a test pin it.
"""

import math
from dataclasses import dataclass

# 3.30: no hop into a cell this close to the chassis' turning circle -- the
# same 20 cm as `min_distance_cm`, so a mover can never appear inside the
# stop line.
MOVER_KEEPOUT_M = 0.20


@dataclass
class Mover:
    """A person or pet: `name` (what the camera's detector would call it),
    a closed `path` of 4-adjacent floor cells it walks round, starting at
    `path[0]`, and the sim seconds per hop."""

    name: str
    path: tuple
    hop_s: float = 1.0
    # 3.31: stand still this long before the first hop (someone in a
    # doorway), and walk the path once and stop at its end (they leave).
    start_s: float = 0.0
    loop: bool = True
    index: int = 0
    next_hop_at: float = 0.0
    hops: int = 0
    waits: int = 0

    def __post_init__(self):
        self.path = tuple(tuple(c) for c in self.path)
        if len(self.path) < 2:
            raise ValueError(f"mover {self.name!r}: a path needs at least two cells")
        if self.hop_s <= 0:
            raise ValueError(f"mover {self.name!r}: hop_s must be positive")
        pairs = self.path[1:] + (self.path[:1] if self.loop else ())
        for i, (a, b) in enumerate(zip(self.path, pairs)):
            if abs(a[0] - b[0]) + abs(a[1] - b[1]) != 1:
                raise ValueError(
                    f"mover {self.name!r}: path cells {a} and {b} (step {i}) are not "
                    "4-adjacent -- a mover walks, it does not jump, and a looping "
                    "path closes from its last cell back to its first")

    @property
    def cell(self) -> tuple:
        return self.path[self.index]

    @property
    def done(self) -> bool:
        """A one-way walk that has reached its end."""
        return not self.loop and self.index == len(self.path) - 1

    @property
    def next_cell(self) -> tuple:
        return self.path[(self.index + 1) % len(self.path)]


def point_to_cell_cells(x: float, y: float, cell: tuple) -> float:
    """Distance, in cells, from a point to the nearest point of a cell's
    square (0 inside it)."""
    cx, cy = cell
    return math.hypot(max(cx - x, 0.0, x - (cx + 1)), max(cy - y, 0.0, y - (cy + 1)))
