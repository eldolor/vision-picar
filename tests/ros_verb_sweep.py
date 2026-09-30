"""
tests/ros_verb_sweep.py

The instrument for `PLAN-ros-alignment.md` 3.24, G2: 3.18's and 3.19's
ground-truth sweeps, run through the ROS VERB PATH instead of direct mode.

What runs is the real thing wherever it can be:

* **`robot.ros_drive.RosDriveRobot`** -- the verb executor the robot server
  uses under `drive: ros`, unmodified. It streams twists and closes on the
  encoders.
* **`SafetyController.check_and_execute()`** -- the entry `/action` uses, so a
  verb is pre-vetted exactly as it is live.
* **A fake chain that includes the robot server's safety vet.** twist_mux ->
  diff_drive_controller -> the plugin -> `POST /wheels` is modelled as a queue
  with R4's measured timing (40-150 ms, one in ten at 300 ms); where a twist
  lands, `vet_wheel_velocity()` runs as `POST /wheels` runs it, and every
  50 ms the wheel loop re-vets the standing command and integrates, as
  `robot/server.py`'s `wheel_loop()` does. (`tests/test_ros_drive.py`'s
  `FakeChain` leaves the vet out; it tests the executor, this tests safety.)
* **A virtual clock** replaces `ros_drive`'s `time`, so a sweep of thousands
  of verbs is fast and exactly repeatable. The executor's own logic -- its
  ramps, settle passes and stall rule -- is untouched.

Judged on ground truth by `tests/footprint_sweep.py`'s geometry, never on
the readings the vet uses.
"""

import json
import logging
import math
import random
from collections import deque
from contextlib import contextmanager

import httpx

import robot.ros_drive as ros_drive
from robot.safety import SafetyController, SafetyViolation, VERB_MIN_MOVE_M, VERB_MIN_TURN_DEG
from sim.maps import build_world
from sim.mock_robot import MockRobot
from tests import footprint_sweep as fs

LOOP_S = 0.05                     # robot/server.py's WHEEL_LOOP_INTERVAL_S
DELAY_S = (0.04, 0.15)            # R4's measured spread
SPIKE_S, SPIKE_P = 0.30, 0.10
SPEEDS = (50, 100)                # G2: 0.3 and 0.6 m/s
TURN_DEG = 45                     # a pivot verb's size in the sweep


class VirtualClock:
    """Stands in for the `time` module inside `robot.ros_drive`."""

    def __init__(self, chain):
        self.now = 1000.0
        self.chain = chain

    def monotonic(self):
        return self.now

    def time(self):
        return self.now

    def sleep(self, dt):
        self.chain.run_until(self.now + dt)


class SafeChain:
    """The ROS chain between the executor and the sim, with the robot
    server's safety vet where the server applies it."""

    def __init__(self, inner, safety, seed, on_advance):
        self.inner, self.safety = inner, safety
        self.rng = random.Random(seed)
        self.pending = deque()
        self.clock = VirtualClock(self)
        self.next_tick = self.clock.now + LOOP_S
        self.on_advance = on_advance

    def handler(self, request: httpx.Request) -> httpx.Response:
        body = json.loads(request.content)
        delay = SPIKE_S if self.rng.random() < SPIKE_P else self.rng.uniform(*DELAY_S)
        due = max(self.clock.now + delay, self.pending[-1][0] if self.pending else 0.0)
        self.pending.append((due, body))
        return httpx.Response(200, json={"published": body.get("driver")})

    def _land(self, twist):
        """A twist reaching the wheels: diff_drive_controller's kinematics,
        then `POST /wheels`'s own vet."""
        w = self.inner.get_wheel_state()
        v, om = twist["linear_m_s"], twist["angular_rad_s"]
        half = om * w["track_width_m"] / 2
        left, right = (v - half) / w["wheel_radius_m"], (v + half) / w["wheel_radius_m"]
        left, right, _ = self.safety.vet_wheel_velocity(left, right)
        self.inner.set_wheel_velocity(left, right)

    def _tick(self):
        """One `wheel_loop()` period: re-vet the standing command, integrate."""
        w = self.inner.get_wheel_state()
        left, right = w["left"]["velocity_rad_s"], w["right"]["velocity_rad_s"]
        if left or right:
            nl, nr, reason = self.safety.vet_wheel_velocity(left, right)
            if reason:
                self.inner.set_wheel_velocity(nl, nr)
            before = (self.inner.world.x, self.inner.world.y, self.inner.world.theta)
            self.inner.advance(LOOP_S)
            self.on_advance(before)

    def run_until(self, t):
        while self.next_tick <= t:
            self.clock.now = self.next_tick
            while self.pending and self.pending[0][0] <= self.clock.now:
                self._land(self.pending.popleft()[1])
            self._tick()
            self.next_tick += LOOP_S
        self.clock.now = t


@contextmanager
def _virtual_time(clock):
    real = ros_drive.time
    ros_drive.time = clock
    try:
        yield
    finally:
        ros_drive.time = real


def direct_travel(house, x, y, theta, action, speed=50):
    """What the SAME verb covers from the same pose in direct mode (3.22's
    guarded verb) -- G2's definition of "the way is clear": direct mode
    completes it. Travel in cm, or degrees for a turn."""
    world = build_world(house)
    world.x, world.y, world.theta = x, y, theta
    robot = MockRobot(world, render=False)
    straight = action in ("FORWARD", "REVERSE")
    kw = {"speed": speed, "duration": 0.5} if straight else {"angle": TURN_DEG}
    try:
        SafetyController(robot, 20.0).check_and_execute(action, **kw)
    except SafetyViolation:
        pass
    if straight:
        return math.hypot(world.x - x, world.y - y) * fs.CELL_CM
    return abs(math.degrees((world.theta - theta + math.pi) % (2 * math.pi) - math.pi))


def run(house, x, y, theta, action, speed=50, seed=0):
    """One verb through the ROS path, judged on ground truth. `x`, `y` in
    cells, `theta` in radians."""
    world = build_world(house)
    world.x, world.y, world.theta = x, y, theta
    inner = MockRobot(world, render=False)
    server_safety = SafetyController(inner, 20.0)
    direction = -1 if action == "REVERSE" else +1
    straight = action in ("FORWARD", "REVERSE")
    T0, G0, _ = fs.truth(world, direction)
    stats = {"worst_T_after_move": math.inf, "min_G": G0}

    def on_advance(before):
        moved = math.hypot(world.x - before[0], world.y - before[1]) > 1e-9
        T, G, _ = fs.truth(world, direction)
        stats["min_G"] = min(stats["min_G"], G)
        if moved and straight:
            stats["worst_T_after_move"] = min(stats["worst_T_after_move"], T)

    chain = SafeChain(inner, server_safety, seed, on_advance)
    robot = ros_drive.RosDriveRobot(inner, "http://bridge")
    robot._http = httpx.Client(base_url="http://bridge", transport=httpx.MockTransport(chain.handler))
    entry = SafetyController(robot, 20.0)       # what /action calls under drive: ros
    x0, y0, th0 = world.x, world.y, world.theta
    kw = {"speed": speed, "duration": 0.5} if straight else {"angle": TURN_DEG}
    refused = None
    with _virtual_time(chain.clock):
        try:
            entry.check_and_execute(action, **kw)
        except SafetyViolation as e:
            refused = str(e)
        chain.run_until(chain.clock.now + 1.0)   # let anything in flight land and settle
    travel = math.hypot(world.x - x0, world.y - y0) * fs.CELL_CM
    turned = abs(math.degrees((world.theta - th0 + math.pi) % (2 * math.pi) - math.pi))
    return {"house": house, "x": x, "y": y, "theta": theta, "action": action, "speed": speed,
            "T0": T0, "G0": G0, "travel_cm": travel, "turned_deg": turned, "refused": refused,
            "direct": direct_travel(house, x, y, theta, action, speed), **stats}


logging.getLogger("safety").setLevel(logging.ERROR)   # the pre-vet's refusals are data here


def straight_sweep(houses, starts_per_house, speeds=SPEEDS, headings=range(0, 360, 30)):
    out = []
    for house in houses:
        for x, y in fs.starts(house, starts_per_house):
            for h in headings:
                for action in ("FORWARD", "REVERSE"):
                    for speed in speeds:
                        out.append(run(house, x, y, math.radians(h), action, speed,
                                       seed=hash((house, x, y, h, action, speed)) & 0xffff))
    return out


def pivot_sweep(houses, starts_per_house):
    out = []
    for house in houses:
        for x, y, th in fs.pivot_starts(house, starts_per_house):
            for action in ("LEFT", "RIGHT"):
                out.append(run(house, x, y, th, action,
                               seed=hash((house, x, y, th, action)) & 0xffff))
    return out


def verdicts(straight, pivots):
    """G2's criteria: {name: (ok, detail)}."""
    c1 = [r for r in straight if r["worst_T_after_move"] < fs.T_BAR_CM]
    c2 = [r for r in straight + pivots if r["min_G"] < min(r["G0"], fs.G_BAR_CM) - 1e-6]
    # Parity: a verb that achieved under the minimum must be a REFUSAL, as in direct mode.
    tiny = [r for r in straight if r["travel_cm"] < VERB_MIN_MOVE_M * 100] + \
           [r for r in pivots if r["turned_deg"] < VERB_MIN_TURN_DEG]
    unrefused = [r for r in tiny if not r["refused"]]
    # "Clear" = direct mode completes the full move from the same pose. (First
    # defined as truth's travel-to-contact >= 53 cm, which counted as clear
    # moves the safety layer DELIBERATELY stops short -- its 3 cm side margin
    # and 15 cm cone body are more cautious than truth. See 3.24, G2.)
    clear = [r for r in straight if abs(r["direct"] - 30.0) <= 0.44]
    full = [r for r in clear if abs(r["travel_cm"] - 30.0) <= 0.44]
    return {
        "1 stopping distance": (not c1, f"{len(c1)}/{len(straight)} under {fs.T_BAR_CM} cm"
                                + (f", worst {min(r['worst_T_after_move'] for r in c1):.1f}" if c1 else "")),
        "2 no contact": (not c2, f"{len(c2)}/{len(straight) + len(pivots)} touched"),
        "3 a blocked verb is a refusal": (not unrefused, f"{len(unrefused)}/{len(tiny)} tiny verbs reported as executed"),
        "4 progress": (not clear or len(full) / len(clear) >= 0.95,
                       f"{len(full)}/{len(clear)} clear FORWARD/REVERSE covered 30 cm +/- 4.4 mm"),
    }
