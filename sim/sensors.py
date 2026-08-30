"""
sensors.py

Phase S5 (PLAN-sim-hardening.md) -- sensor realism for the simulated
ultrasonic. `GridWorld.distance_ahead()` is exact and quantized to whole
30cm cells, which is why that plan's section 3.2 found `min_distance_cm`
only ever crossed at exactly 0.0cm: any value from 1 to 30 made every
existing test pass identically -- "the safety layer is structurally sound
and completely uncalibrated." `DistanceSensorModel` wraps that exact
reading with the things a real HC-SR04 actually has -- noise, an
occasional bad read, and a real ~2-400cm range -- so the threshold becomes
something that can actually be exercised at a realistic boundary, not
just at "the robot's nose is already inside the wall".

Every parameter defaults to exact, noiseless, always-available behavior --
this is an explicit opt-in (`config/robot.yaml`'s `sim.sensor_noise`), per
the phase's own requirement that existing tests and demos stay unaffected
until someone turns it on. `sim/mock_robot.py` only builds one of these
when that config block is enabled; otherwise `MockRobot.get_distance()`
keeps doing exactly what it always did.

What this deliberately does NOT model, per that plan's own section 3.2
and Q4: a real ultrasonic's ~15-degree cone (still a single ray, matching
`GridWorld`'s own geometry -- a cone needs continuous sub-cell position,
which is phase S6's job and is explicitly deferred there), and continuous
robot position (the noise here jitters the *reading*, not where the robot
actually is in the grid). Both are named in that plan's Q4 as the point
past which measuring the real sensor beats modeling it further.
"""

import random
from dataclasses import dataclass, field


@dataclass
class DistanceSensorModel:
    """Wraps an exact cell-count reading (from `GridWorld.distance_ahead()`)
    into a distance in cm with optional Gaussian noise, dropout, a clamped
    real-world range, and optional read latency.

    `read()` returns `0.0` on a simulated dropout, not `None`. That is a
    deliberate fail-safe choice, not a shortcut: `robot/safety.py` vetoes
    `FORWARD` whenever the reading is below `min_distance_cm`, and no
    sane deployment configures that below 0 -- so a dropped reading
    reported as `0.0` always trips the veto. "The sensor went blind" fails
    toward "stop", never toward "assume clear and keep driving" the way a
    reading of, say, `max_range_cm` would. Threading a real
    `Optional[float]` through `RobotInterface`'s declared `-> float`
    return type, `robot/server.py`'s JSON response, and
    `RemoteRobot.get_distance()`'s `float(...)` coercion would touch every
    layer of the stack for a case `robot/safety.py` already handles
    correctly with this simpler choice.
    """

    cell_cm: float = 30.0
    min_range_cm: float = 2.0
    max_range_cm: float = 400.0
    noise_stddev_cm: float = 0.0
    dropout_rate: float = 0.0
    read_latency_s: float = 0.0
    rng: random.Random = field(default_factory=random.Random)

    def read(self, cells: int) -> float:
        if self.dropout_rate and self.rng.random() < self.dropout_rate:
            return 0.0

        distance = cells * self.cell_cm
        if self.noise_stddev_cm:
            distance += self.rng.gauss(0.0, self.noise_stddev_cm)

        return round(max(self.min_range_cm, min(self.max_range_cm, distance)), 1)
