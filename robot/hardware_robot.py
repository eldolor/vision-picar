"""
robot/hardware_robot.py

Phase R7 (`PLAN-ros-alignment.md` 3.16), redone for the UGV Rover in 3.25 --
the real robot's MOTORS, as a `RobotInterface` backend: the Rover's ESP32
board (the Waveshare **ROS Driver**, `ugv_base_ros`) over USB serial,
newline-delimited JSON at 115200 baud.

This is the backend `robot/factory.py`'s `mode: hardware` has pointed at
since Phase 0, and it is why hardware day is a config change: the robot
server, its safety vet, its watchdog and the whole ROS chain above it talk to
this exactly as they talk to `MockRobot`. The serial port belongs HERE, not
to a ROS plugin, so `robot/safety.py` stays the last word on every path
(3.15, 3.16).

**The board is only the motors.** Camera, lidar and depth are other devices;
until their drivers exist, `sensors` supplies them -- a real driver on the
robot, the simulated body in the sim (`sim/fake_esp32.py` turns that body's
wheels). With no `sensors` every sensing method answers the interface's
honest "unusable" default.

**What the firmware makes of this backend** (read from its source, 3.25):

* `T:1` is closed-loop wheel speed, m/s, in the Rover's stock mainType 2 --
  no firmware change (the General Driver R7 was first written against was
  open loop there).
* The `1001` frame, streamed at most every 50 ms, carries MEASURED wheel
  speeds and **`odl`/`odr`: each wheel's travel since the board booted, in
  whole centimetres**. Positions are the speeds integrated, then clamped
  into the centimetre each odometer allows (`_anchor()`): the integral alone
  drifts with every lost line and mistimed interval, the odometer alone is
  3.3 degrees of heading per centimetre, and together the error stays under
  a centimetre with sub-centimetre resolution.
* A reboot zeroes the odometers and forgets the host's set-up. Odometry by
  contract never jumps, so a jump in `odl`/`odr` is taken as a reboot: the
  origin moves to absorb it and the set-up is sent again.
* The board's own heartbeat (`T=136`) stops the motors when commands stop,
  independently of this process: set to `HEARTBEAT_MS`, above the robot
  server's watchdog so that one normally acts first.
"""

import json
import math
import os
import select
import termios
import threading
import time
import tty
from typing import Optional

from robot.interface import (
    RobotInterface, unusable_grid, unusable_odometry, unusable_scan)

# The Waveshare UGV Rover (PLAN-ros-alignment.md 3.21): the ROS Driver
# firmware's mainType 2 values, which the sim and the URDF carry too
# (tests/test_urdf.py and tests/test_wall_linters.py pin them together; the
# pulse count, which nothing on the ROS side holds, is pinned by
# tests/test_ros_driver_board.py). 660 = 11 lines x 2 (half quad) x 30:1.
WHEEL_RADIUS_M = 0.040
TRACK_WIDTH_M = 0.172
COUNTS_PER_REV = 660
# Odometers back at zero while the estimate is this far from them: a board
# that rebooted and started counting again (`_absorb_reboot()`).
REBOOT_JUMP_M = 0.03
# Two feedback intervals: a frame later than this is lost, not late, and the
# last speed is no longer a fair guess.
EXTRAPOLATE_MAX_S = 0.10
# The board stops its own motors after this much silence. Longer than the
# robot server's 1 s watchdog on purpose: the server should always act
# first, and this is what acts if the server itself dies.
HEARTBEAT_MS = 1500
# A verb's speed: 2 moves per second at speed 100, as MockRobot's verbs.
MOVE_M = 0.30
MOVES_PER_SECOND_AT_FULL_SPEED = 2.0


def _bucket(odo_cm: int):
    """The travel, m, a truncated whole-centimetre odometer reading allows."""
    if odo_cm > 0:
        return odo_cm / 100.0, (odo_cm + 1) / 100.0
    if odo_cm < 0:
        return (odo_cm - 1) / 100.0, odo_cm / 100.0
    return -0.01, 0.01


def _bucket_centre(odo_cm: int) -> float:
    lo, hi = _bucket(odo_cm)
    return (lo + hi) / 2.0


class HardwareRobot(RobotInterface):
    def __init__(self, port: str, sensors: Optional[RobotInterface] = None,
                 baud: int = 115200):
        self.port = port
        self.sensors = sensors
        self._fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
        tty.setraw(self._fd)
        attrs = termios.tcgetattr(self._fd)
        speed = getattr(termios, f"B{baud}", termios.B115200)
        attrs[4] = attrs[5] = speed
        termios.tcsetattr(self._fd, termios.TCSANOW, attrs)
        self._lock = threading.Lock()
        self._cmd = (0.0, 0.0)          # rad/s, as last commanded
        # Each wheel's travel on the BOARD's odometer, m since it booted,
        # as estimated here; and what to add to make it travel since THIS
        # process started (moved, never jumped, by a board reboot).
        self._board_m = [0.0, 0.0]
        self._origin_m = [0.0, 0.0]
        self._speed = [0.0, 0.0]        # m/s, the board's last report
        self._last_frame_at: Optional[float] = None
        self.frames = 0
        self.board_reboots = 0          # odometer jumps taken as a reboot
        self._path_m = 0.0
        self._running = True
        self._buf = b""
        self._reader_thread = threading.Thread(target=self._reader, daemon=True)
        self._reader_thread.start()
        self._set_up()

    def _set_up(self):
        """The heartbeat and continuous feedback -- sent at start, and again
        after the board reboots, which forgets both."""
        self._send({"T": 136, "cmd": HEARTBEAT_MS})
        self._send({"T": 131, "cmd": 1})

    # ---------- the wire ----------

    def _send(self, obj: dict):
        os.write(self._fd, (json.dumps(obj) + "\n").encode())

    def _reader(self):
        # select() with a timeout rather than a bare blocking read: on macOS,
        # closing a pty while another thread sits in read() on it hangs the
        # close -- the first contract-suite run never got past teardown.
        while self._running:
            try:
                ready, _, _ = select.select([self._fd], [], [], 0.1)
                if not ready:
                    continue
                chunk = os.read(self._fd, 4096)
            except (OSError, ValueError):
                return
            self._buf += chunk
            while b"\n" in self._buf:
                line, self._buf = self._buf.split(b"\n", 1)
                try:
                    frame = json.loads(line)
                except ValueError:
                    continue            # a partial line at start-up, or noise
                if frame.get("T") == 1001:
                    self._on_base_feedback(frame)

    def _on_base_feedback(self, frame: dict):
        now = time.monotonic()
        speed = [float(frame["L"]), float(frame["R"])]
        odo = (frame.get("odl"), frame.get("odr"))
        anchored = all(isinstance(o, int) for o in odo)
        rebooted = False
        with self._lock:
            before = self._travel_m()
            if self._last_frame_at is None:
                if anchored:
                    self._board_m = [_bucket_centre(o) for o in odo]
                    self._origin_m = [-b for b in self._board_m]
            else:
                dt = now - self._last_frame_at
                # Trapezoid over the two reports -- the interpolation the
                # odometer's whole centimetres cannot give.
                for i in range(2):
                    self._board_m[i] += (self._speed[i] + speed[i]) / 2.0 * dt
                if anchored:
                    rebooted = self._absorb_reboot(odo)
                    self._anchor(odo)
            after = self._travel_m()
            # Path is the BODY's travel -- the wheels' average -- so a pivot
            # (wheels opposite) covers no ground, as MockRobot and
            # get_odometry()'s contract say.
            self._path_m += abs((after[0] + after[1]) / 2.0 - (before[0] + before[1]) / 2.0)
            self._speed = speed
            self._last_frame_at = now
            self.frames += 1
        if rebooted:
            self._set_up()

    def _travel_m(self):
        return [self._board_m[i] + self._origin_m[i] for i in range(2)]

    def _travel_now_m(self):
        """The travel as of NOW rather than as of the last frame: up to one
        feedback interval (50 ms) old, which a verb closed on the encoders
        overshoots by. Carried forward on the board's last measured speeds,
        never further than `EXTRAPOLATE_MAX_S`."""
        travel = self._travel_m()
        if self._last_frame_at is None:
            return travel
        age = min(time.monotonic() - self._last_frame_at, EXTRAPOLATE_MAX_S)
        return [travel[i] + self._speed[i] * age for i in range(2)]

    def _anchor(self, odo):
        """Clamp each wheel's estimate into the centimetre its odometer
        allows: `long int odl_cm = (en_odom_l * 100)` truncates toward zero,
        so n > 0 means [n, n+1) cm, n < 0 means (n-1, n], and 0 means (-1, 1)."""
        for i in range(2):
            lo, hi = _bucket(odo[i])
            self._board_m[i] = min(max(self._board_m[i], lo), hi)

    def _absorb_reboot(self, odo) -> bool:
        """A board that rebooted counts from zero again. Move the origin so
        the travel reported here carries on from where it was.

        Both odometers must read (near) zero: a far reading alone is not a
        reboot -- lost lines and frames arriving bunched let the odometer
        move several centimetres between two the host integrated (7 -> 11 cm
        was seen), and that is drift the clamp corrects, not a new origin.
        The board stops its motors as it boots, so its first frame does
        read zero."""
        if not all(abs(o) <= 1 for o in odo):
            return False
        far = any(not (_bucket(o)[0] - REBOOT_JUMP_M <= self._board_m[i]
                       <= _bucket(o)[1] + REBOOT_JUMP_M) for i, o in enumerate(odo))
        if not far:
            return False
        self.board_reboots += 1
        for i in range(2):
            fresh = _bucket_centre(odo[i])
            self._origin_m[i] += self._board_m[i] - fresh
            self._board_m[i] = fresh
        return True

    # ---------- wheels ----------

    def set_wheel_velocity(self, left_rad_s: float, right_rad_s: float) -> dict:
        with self._lock:
            self._cmd = (left_rad_s, right_rad_s)
        # T=1 in closed-loop mode: wheel surface speeds, m/s.
        self._send({"T": 1, "L": round(left_rad_s * WHEEL_RADIUS_M, 5),
                    "R": round(right_rad_s * WHEEL_RADIUS_M, 5)})
        return {"left_rad_s": left_rad_s, "right_rad_s": right_rad_s}

    def advance(self, dt: float) -> None:
        # The board integrates its own motors; the robot server's wheel loop
        # re-sends the standing command instead, which also feeds the
        # board's heartbeat.
        left, right = self._cmd
        if left or right:
            self.set_wheel_velocity(left, right)

    def get_wheel_state(self) -> dict:
        per_rad = COUNTS_PER_REV / (2 * math.pi)
        with self._lock:
            usable = self._last_frame_at is not None
            pos = [m / WHEEL_RADIUS_M for m in self._travel_now_m()]
            cmd = self._cmd
        return {"usable": usable,
                "left": {"position_rad": pos[0], "velocity_rad_s": cmd[0],
                         "counts": int(round(pos[0] * per_rad))},
                "right": {"position_rad": pos[1], "velocity_rad_s": cmd[1],
                          "counts": int(round(pos[1] * per_rad))},
                "wheel_radius_m": WHEEL_RADIUS_M, "track_width_m": TRACK_WIDTH_M,
                "counts_per_rev": COUNTS_PER_REV}

    def get_odometry(self) -> dict:
        with self._lock:
            if self._last_frame_at is None:
                return unusable_odometry()
            left, right = (m / WHEEL_RADIUS_M for m in self._travel_now_m())
            # Encoders know how far the body has TURNED, not which way is
            # north: clockwise-positive from wherever it started, the
            # project's convention for every angle (a left turn is right
            # wheel ahead of left, counter-clockwise, so negated).
            turned_ccw = math.degrees((right - left) * WHEEL_RADIUS_M / TRACK_WIDTH_M)
            return {"usable": True, "distance_m": round(self._path_m, 4),
                    "heading_deg": round((-turned_ccw) % 360.0, 3)}

    # ---------- verbs, as timed wheel commands ----------

    def _run(self, left: float, right: float, seconds: float) -> None:
        # Only reached when a verb is called WITHOUT the safety layer (which
        # carries verbs out itself since 3.22, from `verb_plan()` below). It
        # still must not outlive a stop: it used to re-send its speed every
        # 50 ms regardless, so a /stop mid-verb was overwritten at once.
        stops = self.stop_count
        self.set_wheel_velocity(left, right)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            time.sleep(min(0.05, max(0.0, end - time.monotonic())))
            if self.stop_count != stops:
                return
            self.set_wheel_velocity(left, right)     # feeds the heartbeat
        self.set_wheel_velocity(0.0, 0.0)

    def verb_plan(self, action: str, speed: int = 50, duration: float = 0.5,
                  angle: int = 90) -> Optional[dict]:
        """The verbs above, for the safety layer to carry out and re-vet
        every period (3.22) -- the same speeds and extents, on the wall
        clock."""
        if action in ("FORWARD", "REVERSE"):
            speed = max(0, min(100, speed))
            moves = (speed / 100.0) * MOVES_PER_SECOND_AT_FULL_SPEED * duration
            moves = max(1, round(moves)) if speed > 0 and duration > 0 else 0
            if not moves:
                return None
            sign = 1.0 if action == "FORWARD" else -1.0
            w = sign * speed / 100.0 * MOVES_PER_SECOND_AT_FULL_SPEED * MOVE_M / WHEEL_RADIUS_M
            return {"kind": "straight", "left_rad_s": w, "right_rad_s": w,
                    "target": moves * MOVE_M, "wall_clock": True, "moves": sign * moves}
        if action in ("LEFT", "RIGHT"):
            if not angle:
                return None
            w = 1.2 * TRACK_WIDTH_M / 2 / WHEEL_RADIUS_M          # 1.2 rad/s body
            left, right = (w, -w) if action == "RIGHT" else (-w, w)
            return {"kind": "turn", "left_rad_s": left, "right_rad_s": right,
                    "target": float(abs(angle)), "wall_clock": True}
        return None

    def verb_done(self, action: str, plan: dict, outcome: dict, **kwargs) -> dict:
        short = ({"stopped_short": outcome["ended"], "reason": outcome["reason"]}
                 if outcome["ended"] != "complete" else {})
        if plan["kind"] == "straight":
            sign = 1.0 if plan["moves"] > 0 else -1.0
            return {"action": "drive_forward" if action == "FORWARD" else "reverse",
                    "requested": plan["moves"],
                    "moved": sign * outcome["done"] / MOVE_M, **short}
        turned = outcome["done"] if action == "RIGHT" else -outcome["done"]
        return {"action": "turn_right" if action == "RIGHT" else "turn_left",
                "turned_deg": turned, **short}

    def _move(self, sign: float, speed: int, duration: float) -> dict:
        speed = max(0, min(100, speed))
        moves = (speed / 100.0) * MOVES_PER_SECOND_AT_FULL_SPEED * duration
        moves = max(1, round(moves)) if speed > 0 and duration > 0 else 0
        v = max(1, speed) / 100.0 * MOVES_PER_SECOND_AT_FULL_SPEED * MOVE_M
        w = sign * v / WHEEL_RADIUS_M
        if moves:
            self._run(w, w, moves * MOVE_M / v)
        return {"requested": sign * moves}

    def _pivot(self, degrees_right: float) -> dict:
        w = 1.2 * TRACK_WIDTH_M / 2 / WHEEL_RADIUS_M          # 1.2 rad/s body
        seconds = math.radians(abs(degrees_right)) / 1.2
        s = 1.0 if degrees_right > 0 else -1.0
        self._run(s * w, -s * w, seconds)
        return {"turned_deg": degrees_right}

    def drive_forward(self, speed: int = 50, duration: float = 0.5) -> dict:
        return {"action": "drive_forward", **self._move(1.0, speed, duration)}

    def reverse(self, speed: int = 50, duration: float = 0.5) -> dict:
        return {"action": "reverse", **self._move(-1.0, speed, duration)}

    def turn_left(self, angle: int = 90) -> dict:
        return {"action": "turn_left", **self._pivot(-float(angle))}

    def turn_right(self, angle: int = 90) -> dict:
        return {"action": "turn_right", **self._pivot(float(angle))}

    # How many times `stop()` has been called: a verb in progress -- the
    # safety layer's loop, or `_run()` -- ends the moment it changes (3.22).
    stop_count = 0

    def stop(self) -> dict:
        self.stop_count += 1
        self.set_wheel_velocity(0.0, 0.0)
        return {"action": "stop"}

    # ---------- everything the board is not ----------

    def look_left(self) -> dict:
        return self.sensors.look_left() if self.sensors else {"pan": None}

    def look_right(self) -> dict:
        return self.sensors.look_right() if self.sensors else {"pan": None}

    def look_center(self) -> dict:
        return self.sensors.look_center() if self.sensors else {"pan": None}

    def get_camera_frame(self) -> dict:
        if self.sensors is None:
            raise RuntimeError("no camera driver yet -- the board is only the motors")
        return self.sensors.get_camera_frame()

    def get_distance(self) -> float:
        return self.sensors.get_distance() if self.sensors else 0.0

    def get_depth_grid(self) -> dict:
        return self.sensors.get_depth_grid() if self.sensors else unusable_grid()

    def get_scan(self, max_range_m=None) -> dict:
        return self.sensors.get_scan(max_range_m=max_range_m) if self.sensors else unusable_scan()

    def close(self) -> None:
        try:
            self.stop()
        finally:
            self._running = False
            self._reader_thread.join(timeout=1.0)
            os.close(self._fd)

    def __getattr__(self, name):
        # The sim's body extras (`world`, for the world model's truth) when
        # the sensors ARE the simulated body.
        if name == "sensors":
            raise AttributeError(name)
        return getattr(self.sensors, name)
