"""
robot/hardware_robot.py

Phase R7 (`PLAN-ros-alignment.md` 3.16) -- the real robot's MOTORS, as a
`RobotInterface` backend: the Waveshare ESP32 driver board over USB serial,
newline-delimited JSON at 115200 baud (`HARDWARE-BOM.md` 4.2).

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

**What the firmware makes of this backend** (read from its source, 3.16):

* It assumes the board runs in **closed-loop mode** (`mainType` 3) with THIS
  chassis' constants -- a firmware change. In the stock `mainType` 2 that
  `HARDWARE-BOM.md` 4.2's example selects, `T=1` is open-loop PWM, not m/s.
* The `1001` frame carries wheel SPEEDS only, so wheel POSITIONS are
  integrated here from speed x time -- the lossy way. A firmware change that
  also reports encoder counts would replace this integral with a reading.
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

# HARDWARE-BOM.md 4.3; the track is the same flagged placeholder the sim and
# the URDF carry (tests/test_urdf.py pins them together).
WHEEL_RADIUS_M = 0.0325
TRACK_WIDTH_M = 0.172
COUNTS_PER_REV = 1760
# The board stops its own motors after this much silence. Longer than the
# robot server's 1 s watchdog on purpose: the server should always act
# first, and this is what acts if the server itself dies.
HEARTBEAT_MS = 1500
# A verb's speed: 2 moves per second at speed 100, as MockRobot's verbs.
MOVE_M = 0.30
MOVES_PER_SECOND_AT_FULL_SPEED = 2.0


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
        self._pos = [0.0, 0.0]          # rad, integrated from 1001 speeds
        self._speed = [0.0, 0.0]        # m/s, the board's last report
        self._last_frame_at: Optional[float] = None
        self.frames = 0
        self._path_m = 0.0
        self._running = True
        self._buf = b""
        self._reader_thread = threading.Thread(target=self._reader, daemon=True)
        self._reader_thread.start()
        # Closed loop, the heartbeat, continuous feedback (3.16).
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
        with self._lock:
            if self._last_frame_at is not None:
                dt = now - self._last_frame_at
                # Trapezoid over the two reports: the integral 3.16 calls lossy.
                mean = []
                for i, key in enumerate(("L", "R")):
                    v = (self._speed[i] + float(frame[key])) / 2.0
                    self._pos[i] += v * dt / WHEEL_RADIUS_M
                    mean.append(v)
                # Path is the BODY's travel -- the wheels' average -- so a
                # pivot (wheels opposite) covers no ground, as MockRobot and
                # get_odometry()'s contract say.
                self._path_m += abs((mean[0] + mean[1]) / 2.0) * dt
            self._speed = [float(frame["L"]), float(frame["R"])]
            self._last_frame_at = now
            self.frames += 1

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
            pos, cmd = list(self._pos), self._cmd
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
            left, right = self._pos
            # Encoders know how far the body has TURNED, not which way is
            # north: clockwise-positive from wherever it started, the
            # project's convention for every angle (a left turn is right
            # wheel ahead of left, counter-clockwise, so negated).
            turned_ccw = math.degrees((right - left) * WHEEL_RADIUS_M / TRACK_WIDTH_M)
            return {"usable": True, "distance_m": round(self._path_m, 4),
                    "heading_deg": round((-turned_ccw) % 360.0, 3)}

    # ---------- verbs, as timed wheel commands ----------

    def _run(self, left: float, right: float, seconds: float) -> None:
        self.set_wheel_velocity(left, right)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            time.sleep(min(0.05, max(0.0, end - time.monotonic())))
            self.set_wheel_velocity(left, right)     # feeds the heartbeat
        self.set_wheel_velocity(0.0, 0.0)

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

    def stop(self) -> dict:
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

    def get_scan(self) -> dict:
        return self.sensors.get_scan() if self.sensors else unusable_scan()

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
