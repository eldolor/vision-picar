"""
sim/fake_esp32.py

Phase R7 (`PLAN-ros-alignment.md` 3.16), redone for the UGV Rover in 3.25 --
the Rover's ESP32 motor board, the Waveshare **ROS Driver for Robots**,
faked on a pseudo-terminal.

The real board speaks newline-delimited JSON over USB serial at 115200 baud.
This fake speaks the same thing on a pty, so the host code that will talk to
the real board -- `robot/hardware_robot.py` -- is written and tested against
a serial line before any board exists.

**It follows the firmware's SOURCE, not a belief about it.** Each rule below
cites the file and function in `waveshareteam/ugv_base_ros`, `ROS_Driver/`
(read at commit `2e7df97`) that it mirrors; `tests/test_fake_esp32.py` and
`tests/test_ros_driver_board.py` pin them. A fake that behaves better than
the board hides exactly what it exists to find -- which is why the odometer
is whole centimetres and the feedback arrives at 20 Hz, not every loop.

Until 3.25 this faked the General Driver (`ugv_base_general`), the board R7
was written for, where `T:1` in the Rover's mainType 2 was open-loop PWM. The
ROS Driver is closed loop in every mainType and reports what the encoders
measured; that is the board the Rover ships with (Waveshare, 2026-09-30).

What it models: the host commands T=0 / 1 / 11 / 13 / 130 / 131 / 136 /
142 / 900; the heartbeat; the 1001 base-feedback frame with measured
speeds and the `odl`/`odr` odometers; integer encoder counts; a reboot
(`reboot()`); a lossy wire (`drop_rate`). What it does not: the PID's
dynamics (it is ideal: the wheel reaches its setpoint within one board loop)
including its `THRESHOLD_PWM` deadband, the IMU (zeros), the OLED, servos,
battery drain.

The wheels it turns are a `MockRobot`'s: the body the rest of the sim sees.
"""

import json
import math
import os
import random
import select
import threading
import time
import tty

BOARD_LOOP_S = 0.01            # the firmware's loop() runs at ~100 Hz and above
# Motor model for T:11 (raw PWM, the one open-loop command left): the UGV
# Rover's rated top speed, 1.3 m/s (Waveshare product page [V]), taken as the
# surface speed at full PWM [I] -- the motor's own rpm is not published.
NO_LOAD_WHEEL_M_S = 1.3

# movtion_module.h mm_settings(): (WHEEL_D, ONE_CIRCLE_PLUSES, TRACK_WIDTH)
# per mainType. 2 is the UGV Rover; its pulse count is commented
# "1650(v=0.90) -> 660(v>=0.93)" -- 660 is 11 lines x 2 (half quad) x 30:1.
MAIN_TYPES = {
    1: (0.0800, 2100, 0.125),   # RaspRover
    2: (0.0800, 660, 0.172),    # UGV Rover
    3: (0.0523, 1092, 0.141),   # UGV Beast
}
# ugv_config.h: int HEART_BEAT_DELAY = 3000; int feedbackFlowExtraDelay = 50;
DEFAULT_HEARTBEAT_MS = 3000
DEFAULT_FEEDBACK_INTERVAL_MS = 50
# The board's side of the wire: what may wait, unread by the host, before
# whole lines start being lost (a USB-serial bridge's buffer, roughly).
OUT_BUFFER_BYTES = 4096


class FakeEsp32:
    """One board: a pty, a firmware loop, and the body whose wheels it turns.

    `path` is the serial device the host opens. `main_type` picks the
    constants the firmware holds (`MAIN_TYPES`); the Rover's is 2, and a
    board set to another type reports odometry in that type's units, as the
    real one would. `drop_rate` loses that share of the board's lines on the
    wire, seeded by `seed`.
    """

    def __init__(self, body, main_type: int = 2, drop_rate: float = 0.0,
                 seed: int = 0):
        self.body = body
        self.drop_rate = drop_rate
        self._rng = random.Random(seed)
        self.lines_in = 0
        self.frames_out = 0
        self.frames_dropped = 0
        self.frames_overflowed = 0
        self.reboots = 0
        # What the board's encoders were at the frames it actually sent:
        # (left_m, right_m) of the body's wheels, for tests to judge against.
        self.sent_truth = []
        self.lock = threading.Lock()
        self._raw_counts_at_boot = [0, 0]
        self._load_main_type(main_type)
        self._boot()
        self._master, slave = os.openpty()
        tty.setraw(slave)
        # The board streams whether or not anyone reads, and a USB-serial
        # bridge drops what the host leaves unread; it never stalls the
        # firmware. A blocking write here would, while holding `lock`.
        os.set_blocking(self._master, False)
        self._out = b""
        self.path = os.ttyname(slave)
        self._slave = slave
        self._running = True
        self._buf = b""
        self._threads = [threading.Thread(target=self._reader, daemon=True),
                         threading.Thread(target=self._loop, daemon=True)]
        for t in self._threads:
            t.start()

    def _load_main_type(self, main_type: int):
        """movtion_module.h mm_settings(); the type is kept in NVS, so it
        survives a reboot."""
        self.main_type = main_type
        self.wheel_d, self.pulses, self.track = MAIN_TYPES[main_type]

    def _boot(self):
        """setup(): everything a power cycle resets. The encoder counters are
        zeroed (initEncoders() -> setCount(0)), so the odometers restart at 0."""
        self.heartbeat_ms = DEFAULT_HEARTBEAT_MS
        self.heartbeat_stopped = False
        self.last_cmd_at = time.monotonic()
        self.feedback_continuous = True           # bool baseFeedbackFlow = 1;
        self.feedback_interval_ms = DEFAULT_FEEDBACK_INTERVAL_MS
        self._last_feedback_at = -math.inf
        self.setpoint = [0.0, 0.0]                # m/s, the PID's targets
        self.pwm = [0, 0]
        self.use_pid = True                       # bool usePIDCompute = true;
        self.measured = [0.0, 0.0]                # speedGetA / speedGetB
        raw = self._raw_counts()
        self._raw_counts_at_boot = raw
        self._last_counts = [0, 0]

    def reboot(self):
        """A power cycle -- a brownout when the motors stall, a USB reset.
        The motors stop, the counters restart from zero and the host's
        set-up (heartbeat, feedback interval) is lost."""
        with self.lock:
            self.reboots += 1
            self._boot()

    # ---------- the encoders ----------

    def _raw_counts(self):
        """The body's wheels as an encoder sees them: whole edges. Positions
        come from `get_wheel_state()`, so `MockRobot.encoder_scale` (R5's
        drift) misleads the board exactly as it would a real one."""
        w = self.body.get_wheel_state()
        r = w["wheel_radius_m"]
        k = self.pulses / (math.pi * self.wheel_d)
        return [int(w[s]["position_rad"] * r * k) for s in ("left", "right")]

    @property
    def counts(self):
        """encoderA.getCount() / encoderB.getCount(): edges since boot."""
        raw = self._raw_counts()
        return [raw[0] - self._raw_counts_at_boot[0], raw[1] - self._raw_counts_at_boot[1]]

    def _odometers_cm(self, counts):
        """getLeftSpeed(): en_odom_l = (pulses / ONE_CIRCLE_PLUSES) * WHEEL_D * PI;
        baseInfoFeedback(): long int odl_cm = (en_odom_l * 100); -- a C cast,
        so truncated toward zero."""
        return [int(c / self.pulses * self.wheel_d * math.pi * 100) for c in counts]

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

    def _write(self, obj: dict) -> bool:
        """Queue one line for the wire. Lines are lost WHOLE -- to `drop_rate`,
        or because the host has left `OUT_BUFFER_BYTES` unread -- never cut,
        so a lost line cannot garble the one after it."""
        if self.drop_rate and self._rng.random() < self.drop_rate:
            self.frames_dropped += 1
            return False
        line = (json.dumps(obj) + "\n").encode()
        if len(self._out) + len(line) > OUT_BUFFER_BYTES:
            self.frames_overflowed += 1       # nobody reading: lost, as on the wire
            return False
        self._out += line
        self.frames_out += 1
        self._flush()
        return True

    def _flush(self):
        try:
            while self._out:
                n = os.write(self._master, self._out)
                self._out = self._out[n:]
        except (BlockingIOError, OSError):
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
            if t == 0:
                # CMD_EMERGENCY_STOP: ... setGoalSpeed(0, 0);
                self._set_goal_speed(0.0, 0.0)
            elif t == 1 and isinstance(cmd.get("L"), (int, float)) and isinstance(cmd.get("R"), (int, float)):
                # CMD_SPEED_CTRL: heartbeatStopFlag = false; lastCmdRecvTime = millis();
                self._touch()
                self._set_goal_speed(float(cmd["L"]), float(cmd["R"]))
            elif t == 11:
                # CMD_PWM_INPUT: usePIDCompute = false; heartbeat...; leftCtrl(L); rightCtrl(R);
                self.use_pid = False
                self._touch()
                self.pwm = [max(-255, min(255, int(cmd.get("L", 0)))),
                            max(-255, min(255, int(cmd.get("R", 0))))]
            elif t == 13:
                # CMD_ROS_CTRL -> rosCtrl(): setpointA = X - Z*TRACK_WIDTH/2 ...
                # and NOTHING about the heartbeat: only T:1 and T:11 feed it.
                x, z = float(cmd.get("X", 0.0)), float(cmd.get("Z", 0.0))
                self._set_goal_speed(x - z * self.track / 2.0, x + z * self.track / 2.0)
            elif t == 130:
                self._feedback()              # baseInfoFeedback(), once
            elif t == 131:
                self.feedback_continuous = bool(cmd.get("cmd"))
            elif t == 136:
                # CMD_HEART_BEAT_SET -> changeHeartBeatDelay(): HEART_BEAT_DELAY = cmd
                self.heartbeat_ms = int(cmd.get("cmd", DEFAULT_HEARTBEAT_MS))
            elif t == 142:
                # CMD_FEEDBACK_FLOW_INTERVAL: feedbackFlowExtraDelay = abs(cmd)
                self.feedback_interval_ms = abs(int(cmd.get("cmd", DEFAULT_FEEDBACK_INTERVAL_MS)))
            elif t == 900:
                # CMD_MM_TYPE_SET -> mm_settings(main, module)
                self._load_main_type(int(cmd.get("main", self.main_type)))

    def _touch(self):
        self.heartbeat_stopped = False
        self.last_cmd_at = time.monotonic()

    # ---------- the firmware ----------

    def _set_goal_speed(self, left: float, right: float):
        """movtion_module.h setGoalSpeed(): closed loop in every mainType."""
        self.use_pid = True
        # if(inputLeft < -2.0 || inputLeft > 2.0) return;  (and right)
        if not (-2.0 <= left <= 2.0) or not (-2.0 <= right <= 2.0):
            return
        self.setpoint = [left, right]

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
        """ugv_advance.h baseInfoFeedback(): rate-limited to one frame per
        feedbackFlowExtraDelay ms, whether streamed or asked for (T:130)."""
        now = time.monotonic()
        if (now - self._last_feedback_at) * 1000 < self.feedback_interval_ms:
            return
        self._last_feedback_at = now
        odl, odr = self._odometers_cm(self.counts)
        frame = {"T": 1001, "L": round(self.measured[0], 4), "R": round(self.measured[1], 4),
                 "ax": 0, "ay": 0, "az": 0, "gx": 0, "gy": 0, "gz": 0,
                 "mx": 0, "my": 0, "mz": 0,
                 "odl": odl, "odr": odr,
                 "v": 1200}                       # int v_int = (int)(loadVoltage_V * 100);
        w = self.body.get_wheel_state()
        truth = (w["left"]["position_rad"] * w["wheel_radius_m"],
                 w["right"]["position_rad"] * w["wheel_radius_m"])
        if self._write(frame):
            self.sent_truth.append(truth)

    def _loop(self):
        radius = self.body.get_wheel_state()["wheel_radius_m"]
        last = time.monotonic()
        while self._running:
            time.sleep(BOARD_LOOP_S)
            now = time.monotonic()
            dt, last = now - last, now
            with self.lock:
                self._flush()
                speeds = self._wheel_speeds()
                self.body.set_wheel_velocity(speeds[0] / radius, speeds[1] / radius)
                self.body.advance(dt)
                # getLeftSpeed(): speedGetA = plusesRate * (count - lastCount) / dt
                # -- MEASURED, so a whole number of edges per loop.
                counts = self.counts
                per_count = math.pi * self.wheel_d / self.pulses
                self.measured = [(counts[i] - self._last_counts[i]) * per_count / dt
                                 for i in range(2)]
                self._last_counts = counts
                if self.feedback_continuous:
                    self._feedback()
                self._heartbeat()

    def close(self):
        self._running = False
        for t in self._threads:
            t.join(timeout=1.0)
        for fd in (self._master, self._slave):
            try:
                os.close(fd)
            except OSError:
                pass
