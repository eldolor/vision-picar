"""
evaluations/goal-learn-396/ask_labels.py -- 3.116.A: room labels as a find
mission would get them, in place of 3.96's stand-in.

The stand-in (`features.Labeller`) names the true room under a candidate
once a quarter of that room is on the brain's map, whether or not the robot
has been there. A real label is a camera frame's answer about the room the
robot is standing in. `AskLabeller` asks as a survey does
(`brain/room_labels.py` `RoomLabeller`, `spacing` mode, uncapped so the
count is measured), each answer the true room at the robot's cell (the sim's
`frame` source), and a candidate's type is read off the labeller's own
`layer()` -- an answer reaches walking distance `REACH_M`, at most one
doorway on, so a room ahead can carry the name of the room the robot asked
from. `noise`: the share of answers replaced by a different type, drawn
uniformly, seeded by the house (3.116 criterion 3's arm).
"""
import math
import random

from brain.frontier import _cell_of
from brain.goal_policy import room_type
from brain.room_labels import NOT_HERE, RoomLabeller, frame_room

TYPES = ("bathroom", "bedroom", "kitchen", "living room")
NOISE_SEED = 3116


class AskLabeller:
    def __init__(self, grid, cell_m: float, index: int, noise: float = 0.0):
        self.grid = grid
        self.cell_m = cell_m
        self.noise = noise
        self.rng = random.Random(NOISE_SEED * 7919 + index)
        self.room_of = {c: name for name, cells in grid.rooms.items() for c in cells}
        self.cells = {}                   # the stand-in's, for map crops: none here
        self.rl = RoomLabeller(self._answer, max_calls=None, mode="spacing",
                               clock=lambda: grid.sim_time)
        self.wrong = 0
        self._layer = None                # (map key, seeds counted, layer)

    @property
    def asks(self) -> int:
        return self.rl.calls

    def _answer(self, frame: dict):
        name = frame_room(frame)
        if name is NOT_HERE or not self.noise:
            return name
        if self.rng.random() < self.noise:
            self.wrong += 1
            return self.rng.choice([t for t in TYPES if t != room_type(name)])
        return name

    def _look(self) -> dict:
        g = self.grid
        return {"room": self.room_of.get((math.floor(g.x), math.floor(g.y)), "unknown")}

    def step(self, runner) -> None:
        """After a tick, as MissionRunner asks during a survey."""
        self.rl.observe(getattr(runner.agent, "_last_map", None), runner.world.get_pose(),
                        self._look)

    def known_rooms(self, m: dict):
        """The labeller's layer on this map (the harness passes it to
        `types` as `known`)."""
        key = (RoomLabeller._key(m), len(self.rl.seeds))
        if self._layer is None or self._layer[0] != key:
            self._layer = (key, self.rl.layer(m) if self.rl.seeds else None)
        return self._layer[1]

    def types(self, m: dict, goals: list, known=None) -> list:
        layer = self.known_rooms(m) if known is None else known
        if not layer:
            return [None] * len(goals)
        w, h = m["width"], m["height"]
        cells, names = layer["cells"], layer["names"]
        out = []
        for x, y in goals:
            mx, my = _cell_of(m, x, y)
            k = -1
            # The goal's own cell, else its 3x3 neighbours (the stand-in's rule).
            for dx, dy in ((0, 0),) + tuple((a, b) for a in (-1, 0, 1) for b in (-1, 0, 1)
                                            if (a, b) != (0, 0)):
                i, j = mx + dx, my + dy
                if 0 <= i < w and 0 <= j < h and cells[j * w + i] >= 0:
                    k = cells[j * w + i]
                    break
            out.append(room_type(names[k]) if k >= 0 else None)
        return out
