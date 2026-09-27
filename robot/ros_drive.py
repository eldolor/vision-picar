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
is the one exception, and deliberately: it zeroes the robot directly as well
as through ROS, because a stop that waits on a container is not a stop.

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

from robot.interface import RobotInterface

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
MIN_ANGULAR_RAD_S = 0.10

CONTROL_HZ = 20.0
LINEAR_TOLERANCE_M = 0.004
ANGULAR_TOLERANCE_RAD = math.radians(0.8)
# No encoder progress for this long while commanding motion: the safety
# vet (or a wall) has stopped the wheels. End the verb rather than push.
STALL_S = 0.6


def moves_for(speed: int, duration: float) -> int:
    """How many 0.30 m moves a (speed, duration) verb covers -- the rule
    `MockRobot._speed_duration_to_cells()` has used since Phase 0."""
    speed = max(0, min(100, speed))
    moves = (speed / 100.0) * MOVES_PER_SECOND_AT_FULL_SPEED * duration
    return max(1, round(moves)) if speed > 0 and duration > 0 else 0


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

    # ---------- who is driving ----------

    @contextmanager
    def driving_as(self, driver: str):
        """robot/server.py names the driver of each /action, so the twists go
        on that driver's twist_mux input (teleop outranks brain in ROS too)."""
        previous = getattr(self._local, "driver", None)
        self._local.driver = driver
        try:
            yield
        finally:
            self._local.driver = previous

    @property
    def _driver(self) -> str:
        return getattr(self._local, "driver", None) or "brain"

    # ---------- motion, through ROS ----------

    def _send(self, linear: float, angular: float, driver: Optional[str] = None) -> None:
        r = self._http.post("/cmd_vel", json={"driver": driver or self._driver,
                                              "linear_m_s": linear, "angular_rad_s": angular})
        r.raise_for_status()

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
            raise RuntimeError("drive: ros needs wheel encoders, and this robot has none")
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
                elif time.monotonic() - last_change > STALL_S:
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
            if abs(target - done) <= 2 * tolerance or time.monotonic() >= deadline:
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
        """Supersede any verb, zero every ROS input, and zero the robot
        directly -- a stop must not depend on the container being alive."""
        with self._gen_lock:
            self._generation += 1
        for driver in ("twin-dpad", "brain", "ros"):
            try:
                self._send(0.0, 0.0, driver=driver)
            except Exception:  # noqa: BLE001 -- the direct stop below still happens
                pass
        return self.inner.stop()

    # ---------- the ROS actuator's route ----------

    def set_wheel_velocity(self, left_rad_s: float, right_rad_s: float) -> dict:
        return self.inner.set_wheel_velocity(left_rad_s, right_rad_s)

    def advance(self, dt: float) -> None:
        return self.inner.advance(dt)

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
