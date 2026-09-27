"""
sim/fake_esp32.py

Phase R7 (`PLAN-ros-alignment.md` 3.16) -- the Waveshare General Driver for
Robots (ESP32), faked on a pseudo-terminal.

The real board speaks newline-delimited JSON over USB serial at 115200 baud
(`HARDWARE-BOM.md` 4.2). This fake speaks the same thing on a pty, so the
host code that will talk to the real board -- `robot/hardware_robot.py` --
is written and tested against a serial line before any board exists. It
closes the blocker `PLAN-onboard-perception.md` C3 named: "nothing in this
repo simulates a serial peer".

**It follows the firmware's SOURCE, not a belief about it.** Each rule below
cites the file and function in `waveshareteam/ugv_base_general`,
`General_Driver/` (GPL-3.0) that it mirrors; `tests/test_fake_esp32.py` pins
them. Where the real board does something this project does not want -- T=1
being open-loop PWM in mode 2 -- the fake does it too, because a fake that
behaves better than the board hides exactly the problem it exists to find.

What it models: the host commands T=1 / 11 / 13 / 130 / 131 / 136 / 900; the
heartbeat; the 1001 base-feedback frame; mainType 2's open loop and mainType
3's closed loop, the latter as an ideal PID (the wheel reaches its setpoint
within one board loop). What it does not: IMU, OLED, servos, battery drain,
motor dynamics beyond a speed ceiling.

The wheels it turns are a `MockRobot`'s: the body the rest of the sim sees.
"""

import json
import math
import os
import select
import threading
import time
import tty
from typing import Optional

BOARD_LOOP_S = 0.01            # the firmware's loop() runs at ~100 Hz and above
# Motor model (HARDWARE-BOM.md 4.3): 300 rpm no-load at 12 V on 65 mm wheels.
NO_LOAD_WHEEL_M_S = 300 / 60 * math.pi * 0.065


class FakeEsp32:
    """One board: a pty, a firmware loop, and the body whose wheels it turns.

    `path` is the serial device the host opens. `wheel_diameter_m`,
    `pulses_per_rev` and `track_width_m` are the constants the firmware
    holds per mainType (movtion_module.h mm_settings()); mainType 3 here
    carries THIS chassis' values, modelling the firmware change 3.16 calls
    for -- the stock mode 3 is sized for another robot.
    """

    def __init__(self, body, main_type: int = 3, wheel_diameter_m: float = 0.065,
                 pulses_per_rev: int = 1760, track_width_m: float = 0.172):
        self.body = body
        self.main_type = main_type
        self.wheel_d = wheel_diameter_m
        self.pulses = pulses_per_rev
        self.track = track_width_m
        # ugv_config.h: int HEART_BEAT_DELAY = 3000;
        self.heartbeat_ms = 3000
        self.heartbeat_stopped = False
        self.last_cmd_at = time.monotonic()
        self.feedback_continuous = False
        # Wheel surface speeds, m/s, as the board is currently driving them.
        self.setpoint = [0.0, 0.0]
        self.pwm = [0, 0]
        self.use_pid = main_type == 3
        self.lines_in = 0
        self.frames_out = 0
        self.lock = threading.Lock()
        self._master, slave = os.openpty()
        tty.setraw(slave)
        self.path = os.ttyname(slave)
        self._slave = slave
        self._running = True
        self._buf = b""
        self._threads = [threading.Thread(target=self._reader, daemon=True),
                         threading.Thread(target=self._loop, daemon=True)]
        for t in self._threads:
            t.start()

    # ---------- the serial side ----------

    def _reader(self):
        # select() with a timeout rather than a bare blocking read: on macOS,
        # closing a pty while another thread sits in read() on it hangs the
        # close -- the first contract-suite run never got past teardown.
        while self._running:
            try:
                ready, _, _ = select.select([self._master], [], [], 0.1)
                if not ready:
                    continue
                chunk = os.read(self._master, 4096)
            except (OSError, ValueError):
                return
            if not chunk:
                continue
            self._buf += chunk
            while b"\n" in self._buf:
                line, self._buf = self._buf.split(b"\n", 1)
                self._handle(line.strip())

    def _write(self, obj: dict):
        try:
            os.write(self._master, (json.dumps(obj) + "\n").encode())
            self.frames_out += 1
        except OSError:
            pass

    def _handle(self, line: bytes):
        """uart_ctrl.h jsonCmdReceiveHandler(): one JSON command per line."""
        try:
            cmd = json.loads(line)
        except ValueError:
            return            # the firmware's deserializeJson() fails quietly too
        self.lines_in += 1
        t = cmd.get("T")
        with self.lock:
            if t == 1 and isinstance(cmd.get("L"), (int, float)) and isinstance(cmd.get("R"), (int, float)):
                # CMD_SPEED_CTRL: heartbeatStopFlag = false; lastCmdRecvTime = millis();
                self._touch()
                self._set_goal_speed(float(cmd["L"]), float(cmd["R"]))
            elif t == 11:
                # CMD_PWM_INPUT: usePIDCompute = false; ... leftCtrl(L); rightCtrl(R);
                self.use_pid = False
                self._touch()
                self.pwm = [max(-255, min(255, int(cmd.get("L", 0)))),
                            max(-255, min(255, int(cmd.get("R", 0))))]
            elif t == 13:
                # CMD_ROS_CTRL -> rosCtrl(): setpointA = X - Z*TRACK_WIDTH/2 ...
                self._touch()
                x, z = float(cmd.get("X", 0.0)), float(cmd.get("Z", 0.0))
                self._set_goal_speed(x - z * self.track / 2.0, x + z * self.track / 2.0)
            elif t == 130:
                self._feedback()              # baseInfoFeedback(), once
            elif t == 131:
                self.feedback_continuous = bool(cmd.get("cmd"))
            elif t == 136:
                # CMD_HEART_BEAT_SET -> changeHeartBeatDelay(): HEART_BEAT_DELAY = cmd
                self.heartbeat_ms = int(cmd.get("cmd", 3000))
            elif t == 900:
                self.main_type = int(cmd.get("main", self.main_type))
                self.use_pid = self.main_type == 3

    def _touch(self):
        self.heartbeat_stopped = False
        self.last_cmd_at = time.monotonic()

    # ---------- the firmware ----------

    def _set_goal_speed(self, left: float, right: float):
        """movtion_module.h setGoalSpeed()."""
        if self.main_type == 3:
            self.use_pid = True
            # if(inputLeft < -2.0 || inputLeft > 2.0) return;  (and right)
            if not (-2.0 <= left <= 2.0) or not (-2.0 <= right <= 2.0):
                return
            self.setpoint = [left, right]
        else:
            # usePIDCompute = false; leftCtrl(inputLeft * 512 * spd_rate_A);
            self.use_pid = False
            self.pwm = [max(-255, min(255, int(left * 512))),
                        max(-255, min(255, int(right * 512)))]

    def _heartbeat(self):
        """movtion_module.h heartBeatCtrl()."""
        if (time.monotonic() - self.last_cmd_at) * 1000 > self.heartbeat_ms:
            if not self.heartbeat_stopped:
                self.heartbeat_stopped = True
                self._set_goal_speed(0.0, 0.0)

    def _wheel_speeds(self):
        """What the wheels actually do this loop, m/s."""
        if self.use_pid:
            return list(self.setpoint)           # an ideal PID
        return [p / 255.0 * NO_LOAD_WHEEL_M_S for p in self.pwm]

    def _feedback(self):
        """ugv_advance.h baseInfoFeedback(): T 1001, L/R speeds, IMU, temp, v."""
        speeds = self.measured
        self._write({"T": 1001, "L": round(speeds[0], 4), "R": round(speeds[1], 4),
                     "r": 0.0, "p": 0.0, "y": 0.0, "temp": 25.0, "v": 12.0})

    def _loop(self):
        radius = self.body.get_wheel_state()["wheel_radius_m"]
        last = time.monotonic()
        self.measured = [0.0, 0.0]
        while self._running:
            time.sleep(BOARD_LOOP_S)
            now = time.monotonic()
            dt, last = now - last, now
            with self.lock:
                self._heartbeat()
                speeds = self._wheel_speeds()
                self.body.set_wheel_velocity(speeds[0] / radius, speeds[1] / radius)
                self.body.advance(dt)
                # getLeftSpeed(): pulses since last read x plusesRate / dt. The
                # body integrates truly, so the measured speed IS the speed.
                self.measured = speeds
                if self.feedback_continuous:
                    self._feedback()

    def close(self):
        self._running = False
        for t in self._threads:
            t.join(timeout=1.0)
        for fd in (self._master, self._slave):
            try:
                os.close(fd)
            except OSError:
                pass
