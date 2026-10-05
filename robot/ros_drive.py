"""
robot/ros_drive.py

Phase R4 (`PLAN-ros-alignment.md` 3.13) -- verbs become velocity profiles,
and the ROS chain becomes the ONE writer to the wheels.

`/action` is a verb API -- `FORWARD` is one 0.30 m move, `LEFT 45` a
45-degree pivot -- and every step budget, demo and recorded walk in this repo
was measured against that. ROS moves robots with velocities. So under
`drive: ros` this wrapper turns each verb into a stream of twists, sends them
to the bridge in the ROS container (`service/slam/`, `POST /cmd_vel`), and
closes the loop on the **wheel encoders** it reads from the robot underneath:

    /action FORWARD -> RosDriveRobot.drive_forward()
        -> POST <bridge>/cmd_vel {driver, linear_m_s}   (20 Hz, until the
           encoders say 0.30 m)
        -> twist_mux -> diff_drive_controller -> picar_sim_hardware
        -> POST /wheels (x-driver: ros) -> the robot underneath

The encoders, not the clock, because a timed move over HTTP measures the
network: R4's first open-loop run moved 0.267-0.316 m for a nominal 0.30.

Every READ goes straight to the robot underneath -- the camera, the depth
grid, the scan, the encoders. Only motion goes round through ROS. `stop()`
is the one exception, and deliberately: it zeroes the robot directly FIRST,
and only then -- on a background thread, with a short timeout -- zeroes the
ROS inputs, because a stop that waits on a container is not a stop. (Until
2026-10-02 it posted the zeros first, each with the client's 2 s timeout: a
bridge that accepted connections and never answered held a stop for ~6 s,
and the watchdog's stop runs on the server's event loop, so every route
froze with it. docs-review/SPEC-REVIEW.md, finding 1.) For STOP_HOLD_S after
a stop, a non-zero wheel command arriving from ROS -- the stopped verb's
last twists still in flight through twist_mux and the controller -- is
held at zero, so the direct stop is not undone; the next verb lifts the
hold.

Not a backend of its own: `robot/factory.py` wraps whichever backend the
config names, so this is the only place in `robot/` that knows a bridge
exists, and nothing here imports ROS (`tests/test_ros_containment.py`).
"""

import logging
import math
import threading
import time
from contextlib import contextmanager
from typing import Optional

import httpx

from robot.interface import (DRIVER_MANUAL, VERB_STALL_S, RobotInterface, WheelFeedbackLost,
                             driver_priority)

logger = logging.getLogger("ros_drive")

# What a verb MEANS, mirrored from sim/mock_robot.py so a verb through ROS
# covers exactly what the same verb covers without it (tests pin the pair):
# `speed` 0-100 is a fraction of 2 moves per second, `duration` is quantised
# into whole moves, a move is 0.30 m.
MOVE_M = 0.30
MOVES_PER_SECOND_AT_FULL_SPEED = 2.0
# Pivot rate for a turn verb, and the ramps. Chosen from R4's first live
# run, not by taste: the chain from this loop to the wheels (bridge ->
# twist_mux -> a 20 Hz controller -> HTTP -> a 20 Hz wheel loop) carries
# 40-150 ms of delay, and at 2 rad/s with a gain of 3/s a 45-degree turn came
# out at 59-74 degrees. A proportional ramp over a delay L settles only when
# gain * L is well under 1, and the last in-flight motion is floor-rate * L.
TURN_RATE_RAD_S = 1.2
ANGULAR_GAIN_PER_S = 1.5
LINEAR_GAIN_PER_S = 2.0
MIN_LINEAR_M_S = 0.02
# The floor rate and the turn tolerance were halved and cut from 0.8 deg in
# 3.24 G2: at 0.10 rad/s, the chain's 40-300 ms of delay carried each final
# correction 0.2-1.7 deg on, and only 80% of clear 45-degree turns landed
# within 3.13's 0.64 deg (p95 1.01). At 0.05 rad/s and 0.5 deg: 100%, worst
# 0.49 deg (tests/ros_verb_sweep.py, 240 turns on a virtual clock).
MIN_ANGULAR_RAD_S = 0.05

CONTROL_HZ = 20.0
LINEAR_TOLERANCE_M = 0.004
ANGULAR_TOLERANCE_RAD = math.radians(0.5)
# No encoder progress for VERB_STALL_S while commanding motion: the safety
# vet (or a wall) has stopped the wheels. End the verb rather than push. The
# number is robot/interface.py's, shared with direct mode's verbs (3.35).


def moves_for(speed: int, duration: float) -> int:
    """How many 0.30 m moves a (speed, duration) verb covers -- the rule
    `MockRobot._speed_duration_to_cells()` has used since Phase 0."""
    speed = max(0, min(100, speed))
    moves = (speed / 100.0) * MOVES_PER_SECOND_AT_FULL_SPEED * duration
    return max(1, round(moves)) if speed > 0 and duration > 0 else 0


# A stop zeroes the ROS inputs in the background; each post may wait this
# long, so a hung bridge costs a background thread 1.5 s and the caller
# nothing.
STOP_ZERO_TIMEOUT_S = 0.5
# After a stop, non-zero wheel commands from ROS are held at zero this long.
# It must outlast how long the stopped verb's last twist can keep reaching
# the wheels when the bridge is hung and the zeros never arrive: twist_mux
# holds a silent input for its timeout (0.25 s) and THEN
# diff_drive_controller holds its last command for its own cmd_vel_timeout
# (0.25 s) -- the two ADD (picar_bringup/config/twist_mux.yaml says so) --
# plus one 0.05 s plugin period: 0.55 s. Was 0.4 until the second spec
# review (2026-10-02) caught the two timeouts counted as one.
STOP_HOLD_S = 0.6
# A failed send to the bridge marks it down (handoff 2026-10-02 1d). While
# down, its `/health` is probed in the background at most this often, and
# the first answer marks it up again -- so a person's verbs, running direct
# meanwhile, never wait on it and never send it anything.
BRIDGE_PROBE_S = 0.5
BRIDGE_PROBE_TIMEOUT_S = 0.5


def ros_input_for(driver: Optional[str]) -> str:
    """The bridge's twist_mux input for a driver (handoff 3a, decided
    2026-10-05). Every PERSON -- the twin's D-pad, a teleop operator, an
    unnamed caller, who ranks as a person -- drives on the D-pad's input,
    the highest; `ros` (nav2) on its own; everything else autonomous on the
    brain's. Named inputs used to be looked up by driver name, and a
    `teleop-operator` verb was refused `ros_unavailable` on the bridge's 400
    for an unknown driver. The names are picar_bridge's DRIVER_TOPICS keys
    (tests/test_ros_drive.py reads them from bridge.py)."""
    if driver == "ros":
        return "ros"
    if driver_priority(driver or "") >= DRIVER_MANUAL:
        return "twin-dpad"
    return "brain"


class RosDriveRobot(RobotInterface):
    """Verbs through ROS, reads from the robot underneath."""

    drives_by_velocity = True

    def __init__(self, inner: RobotInterface, bridge_url: str, secret: str = "",
                 timeout_s: float = 2.0):
        self.inner = inner
        self.bridge_url = bridge_url.rstrip("/")
        headers = {"x-app-secret": secret} if secret else {}
        self._http = httpx.Client(base_url=self.bridge_url, headers=headers, timeout=timeout_s)
        self._local = threading.local()
        # A verb in progress is superseded by the next one to START --
        # robot/server.py's M4 arbitration has already decided the newcomer
        # may drive, and the old stream must not resume when it finishes.
        self._generation = 0
        self._gen_lock = threading.Lock()
        self.verbs_through_ros = 0
        # The stop hold: in force while the generation is still the stop's
        # own and the clock is before _hold_until.
        self._hold_gen = -1
        self._hold_until = 0.0
        # Bridge liveness (1d): None while sends succeed; the time of the
        # failure that marked it down otherwise.
        self._bridge_down_since: Optional[float] = None
        self._probed_at = 0.0
        self._probing = threading.Lock()
        # One background zeroing at a time: the watchdog stops every poll
        # while the robot is silent, and each must not add a thread.
        self._zeroing = threading.Lock()

    # ---------- who is driving ----------

    @contextmanager
    def driving_as(self, driver: str):
        """robot/server.py names the driver of each /action, so the twists go
        on that driver's twist_mux input (teleop outranks brain in ROS too).
        The input is chosen by RANK (`ros_input_for`), not by name: twist_mux
        has one input per rank, and M4 has already decided between people
        and autonomy before a verb gets here."""
        previous = getattr(self._local, "driver", None)
        self._local.driver = ros_input_for(driver)
        try:
            yield
        finally:
            self._local.driver = previous

    @property
    def _driver(self) -> str:
        return getattr(self._local, "driver", None) or "brain"

    # ---------- motion, through ROS ----------

    def _send(self, linear: float, angular: float, driver: Optional[str] = None,
              timeout: Optional[float] = None) -> None:
        try:
            r = self._http.post("/cmd_vel", json={"driver": driver or self._driver,
                                                  "linear_m_s": linear, "angular_rad_s": angular},
                                timeout=httpx.USE_CLIENT_DEFAULT if timeout is None else timeout)
            r.raise_for_status()
        except httpx.HTTPStatusError as e:
            # The bridge ANSWERED: a 4xx (an unknown driver's 400, a wrong
            # secret's 401) is proof of life, not an outage -- marking it down
            # would flap against the unauthenticated /health probe (spec
            # review 3, V7). Only a 5xx says the bridge itself is failing.
            if e.response.status_code >= 500:
                self._mark_bridge_down()
            else:
                self._bridge_down_since = None
            raise
        except httpx.TransportError:
            self._mark_bridge_down()
            raise
        self._bridge_down_since = None

    # ---------- is the bridge alive (handoff 2026-10-02 1d) ----------

    def _mark_bridge_down(self) -> None:
        if self._bridge_down_since is None:
            self._bridge_down_since = time.monotonic()
            logger.warning("a send to the bridge failed -- ROS marked down until it answers")

    def bridge_up(self) -> bool:
        """False from a failed send until the bridge answers again.

        The robot server ANDs this into `ros_up()`: the actuator's posts prove
        the end of the chain is alive, this proves the start is. Never blocks:
        while down it starts at most one background `/health` probe per
        BRIDGE_PROBE_S and answers from the last result."""
        if self._bridge_down_since is None:
            return True
        now = time.monotonic()
        if now - self._probed_at >= BRIDGE_PROBE_S and self._probing.acquire(blocking=False):
            self._probed_at = now
            threading.Thread(target=self._probe, daemon=True, name="ros-bridge-probe").start()
        return False

    def _probe(self) -> None:
        try:
            r = self._http.get("/health", timeout=BRIDGE_PROBE_TIMEOUT_S)
            if r.status_code == 200:
                self._bridge_down_since = None
                logger.info("the bridge answers again -- ROS no longer marked down by it")
        except httpx.HTTPError:
            pass
        finally:
            self._probing.release()

    def _begin(self) -> int:
        with self._gen_lock:
            self._generation += 1
            self.verbs_through_ros += 1
            return self._generation

    def _superseded(self, gen: int) -> bool:
        return gen != self._generation

    def _encoders(self):
        w = self.inner.get_wheel_state()
        if not w.get("usable"):
            # 3.34: none, or none that are being measured right now -- either
            # way no verb can be closed on them, and the server says so.
            raise WheelFeedbackLost("drive: ros needs wheel encoders, and this robot's "
                                    "are not measured (none, or no fresh feedback)")
        return (w["left"]["position_rad"], w["right"]["position_rad"],
                w["wheel_radius_m"], w["track_width_m"])

    def _run(self, gen: int, target: float, max_rate: float, min_rate: float,
             tolerance: float, progress, linear: bool, gain: float) -> float:
        """Stream twists until `progress()` is within `tolerance` of `target`
        (signed), the verb is superseded, or the wheels stall. Returns the
        progress made.

        The error is SIGNED and the loop may drive back: after the zero
        lands and the wheels settle, a verb that overshot on a latency spike
        corrects at the floor rate, so what is left is floor-rate x delay
        (about 1 degree, 3 mm) rather than whatever the spike was.
        """
        expected_s = abs(target) / max_rate
        started = time.monotonic()
        deadline = started + 3 * expected_s + 3.0
        done = progress()
        for _settle in range(3):
            last_progress, last_change = done, time.monotonic()
            while time.monotonic() < deadline and not self._superseded(gen):
                done = progress()
                err = target - done
                if abs(err) <= tolerance:
                    break
                if abs(done - last_progress) > tolerance / 4:
                    last_progress, last_change = done, time.monotonic()
                elif time.monotonic() - last_change > VERB_STALL_S:
                    logger.info("verb stalled at %.3f of %.3f -- ended", done, target)
                    deadline = 0.0          # a wall is not something to retry into
                    break
                rate = math.copysign(min(max_rate, max(min_rate, gain * abs(err))), err)
                self._send(rate, 0.0) if linear else self._send(0.0, rate)
                time.sleep(1.0 / CONTROL_HZ)
            if self._superseded(gen):
                break
            self._send(0.0, 0.0)
            # Let the zero land and the wheels settle, then look again.
            time.sleep(3.0 / CONTROL_HZ)
            done = progress()
            if abs(target - done) <= tolerance or time.monotonic() >= deadline:
                break
        logger.info("verb target=%.4f final=%.4f in %.2fs%s", target, done,
                    time.monotonic() - started, " (superseded)" if self._superseded(gen) else "")
        return done

    def _straight(self, moves: int, speed: int) -> dict:
        if moves == 0:
            return {"requested": 0, "moved": 0.0}
        gen = self._begin()
        l0, r0, radius, _ = self._encoders()

        def travelled():
            l, r, _, _ = self._encoders()
            return radius * ((l - l0) + (r - r0)) / 2.0

        v = max(1, min(100, speed)) / 100.0 * MOVES_PER_SECOND_AT_FULL_SPEED * MOVE_M
        moved = self._run(gen, moves * MOVE_M, v, MIN_LINEAR_M_S,
                          LINEAR_TOLERANCE_M, travelled, linear=True, gain=LINEAR_GAIN_PER_S)
        return {"requested": moves, "moved": round(moved / MOVE_M, 4), "moved_m": round(moved, 4)}

    def _turn(self, degrees_right: float) -> dict:
        if degrees_right == 0:
            return {"turned_deg": 0.0}
        gen = self._begin()
        l0, r0, radius, track = self._encoders()

        def turned_ccw():
            l, r, _, _ = self._encoders()
            return radius * ((r - r0) - (l - l0)) / track

        # REP-103: positive yaw is to the LEFT; the project's degrees are
        # positive to the RIGHT. One conversion, here.
        turned = self._run(gen, -math.radians(degrees_right), TURN_RATE_RAD_S,
                           MIN_ANGULAR_RAD_S, ANGULAR_TOLERANCE_RAD, turned_ccw, linear=False,
                           gain=ANGULAR_GAIN_PER_S)
        return {"turned_deg": round(-math.degrees(turned), 2)}

    def drive_forward(self, speed: int = 50, duration: float = 0.5) -> dict:
        return {"action": "drive_forward", "speed": speed, "duration": duration,
                "via": "ros", **self._straight(moves_for(speed, duration), speed)}

    def reverse(self, speed: int = 50, duration: float = 0.5) -> dict:
        return {"action": "reverse", "speed": speed, "duration": duration,
                "via": "ros", **self._straight(-moves_for(speed, duration), speed)}

    def turn_left(self, angle: int = 90) -> dict:
        return {"action": "turn_left", "angle": angle, "via": "ros", **self._turn(-float(angle))}

    def turn_right(self, angle: int = 90) -> dict:
        return {"action": "turn_right", "angle": angle, "via": "ros", **self._turn(float(angle))}

    def stop(self) -> dict:
        """Supersede any verb, zero the robot directly, then zero every ROS
        input in the background -- a stop must not depend on the container
        being alive, or answering."""
        with self._gen_lock:
            self._generation += 1
            self._hold_gen = self._generation
            self._hold_until = time.monotonic() + STOP_HOLD_S
        result = self.inner.stop()
        if self._zeroing.acquire(blocking=False):
            threading.Thread(target=self._zero_ros_inputs, daemon=True,
                             name="ros-stop-zero").start()
        return result

    def _zero_ros_inputs(self) -> None:
        try:
            for driver in ("twin-dpad", "brain", "ros"):
                try:
                    self._send(0.0, 0.0, driver=driver, timeout=STOP_ZERO_TIMEOUT_S)
                except Exception:  # noqa: BLE001 -- the robot is already stopped
                    pass
        finally:
            self._zeroing.release()

    def _holding_stop(self) -> bool:
        return self._generation == self._hold_gen and time.monotonic() < self._hold_until

    # ---------- the ROS actuator's route ----------

    def set_wheel_velocity(self, left_rad_s: float, right_rad_s: float) -> dict:
        if (left_rad_s or right_rad_s) and self._holding_stop():
            # The stopped verb's last twist, still in flight through ROS.
            return self.inner.set_wheel_velocity(0.0, 0.0)
        return self.inner.set_wheel_velocity(left_rad_s, right_rad_s)

    def advance(self, dt: float) -> None:
        return self.inner.advance(dt)

    def pass_time(self, dt: float) -> None:
        """3.30: idle time for the sim body underneath (its movers), if it
        has a clock; a real body has none and this is a no-op."""
        fn = getattr(self.inner, "pass_time", None)
        if fn is not None:
            fn(dt)

    # ---------- everything else reads the robot underneath ----------

    def look_left(self) -> dict:
        return self.inner.look_left()

    def look_right(self) -> dict:
        return self.inner.look_right()

    def look_center(self) -> dict:
        return self.inner.look_center()

    def get_camera_frame(self) -> dict:
        return self.inner.get_camera_frame()

    def get_distance(self) -> float:
        return self.inner.get_distance()

    def get_depth_grid(self) -> dict:
        return self.inner.get_depth_grid()

    def get_odometry(self) -> dict:
        return self.inner.get_odometry()

    def get_wheel_state(self) -> dict:
        return self.inner.get_wheel_state()

    def get_scan(self, max_range_m=None) -> dict:
        return self.inner.get_scan(max_range_m=max_range_m)

    def __getattr__(self, name):
        # Backend extras (MockRobot.world, the sim's truth) stay reachable
        # for the routes that are allowed to know about them.
        return getattr(self.inner, name)
