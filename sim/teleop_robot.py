"""
teleop_robot.py

Phase T1 of PLAN-teleop-robot.md -- a robot body with a live camera and no
motor.

`sim/replay_robot.py` is "a body made of photographs": a walk already
taken, played back one frame per move, honestly documented as open-loop
(the frames don't respond to the action the model chose). `TeleopRobot`
is its live, closed-loop cousin -- the photographs arrive as they're
taken, and the next one really is whatever the human standing in for the
motor photographs *after* reading the model's decision. The phone is the
camera; a person is the motor.

## What this is for

Same question Stage 0 and `replay_robot.py` leave unanswered: does the
whole harness -- mission memory, the step budget, the failure budget, the
arrival signal, `MissionRunner`'s failsafes -- behave correctly when
driven by a real camera, not `MockRobot`'s grid world? A recorded walk
answers that for lifecycle and cost, but is open loop by construction.
This backend closes the loop, using a human's own footsteps as the
actuator `RobotInterface` expects one of.

## What it does NOT do

**No motor.** `drive_forward`/`reverse`/`turn_left`/`turn_right`/`look_*`
log the decided action and return an ack. Nothing moves. The mission's
"movement" is a human reading `MissionRunner.status()`'s `last_action`
and acting on it -- see the twin's Robot view ("drive via brain" mode).

**No distance sensor**, same honest answer `replay_robot.py` gives:
`get_distance()` returns `NO_SENSOR_CM`, far above any configured
`min_distance_cm`, so `robot/safety.py` never vetoes anything here. A
clean teleop mission says nothing about collision avoidance.

**No room label.** A pushed photograph carries no room, so
`frame["room"]` is `"unknown"` and `MissionMemory.visited_rooms` stays
empty -- the same open S2b gap every other vision-policy backend has.

## Stall detection

`get_camera_frame()` is a staleness check, not a wait: it returns the most
recently pushed frame immediately if it arrived within `stall_timeout_s`
(default 15s), or raises `TeleopStall` immediately otherwise -- never
blocks. `ConstrainedAgent.step()` calls `get_camera_frame()` directly --
outside `MissionRunner`'s B3.2 vision-call timeout -- so this backend owns
its own deadline instead.

**Originally implemented as a blocking wait for "a frame newer than the
last one this call returned"** -- wrong, found via a real deployment: that
made `_last_returned_seq` a single cursor shared by *every* caller, not
just an active mission's tick loop. `robot/server.py`'s `GET /frame` is a
generic, repeatable sensor read used by plenty of callers that have
nothing to do with a mission -- Settings' `connect()` health check, the
Sim tab's frame-following, a passive observer -- and once any one of them
"consumed" the current frame, the next one would block for the full
`stall_timeout_s` and then 500, regardless of who was asking or why. A
routine connectivity check taking 15s and failing is exactly what broke
first. Every other backend's `get_camera_frame()` is a cheap, idempotent
read; this one now is too -- staleness is measured from *when a frame was
last pushed*, not from *who last read one*.

`TeleopStall` needs no dedicated handling anywhere: it propagates out of
`agent.step()` into `MissionRunner.tick()`'s existing catch-all, which
already stops the robot and ends the mission `failed` with the stall's
own message in the log. Because this no longer blocks, `stall_timeout_s`
no longer risks racing B3.3's `tick_timeout_s` (a stalled tick now fails
essentially instantly instead of occupying up to `stall_timeout_s` of the
tick's own wall-clock time) -- comfortably under it is simply good manners
now, not a correctness requirement.

A stall ends the mission on the first miss, with no retry budget the way
B3.2's vision failures get one -- deliberate: a missing frame has nothing
to decide on, unlike a flaky model response that might succeed on retry.
"""

import collections
import uuid
import logging
import threading
import time
from typing import Optional

from robot.interface import NO_SENSOR_CM as _NO_SENSOR_CM, RobotInterface

logger = logging.getLogger("teleop_robot")

# Re-exported from robot/interface.py, which is where the contract lives
# now -- this used to be a hand-kept copy of sim/replay_robot.py's.
NO_SENSOR_CM = _NO_SENSOR_CM

DEFAULT_STALL_TIMEOUT_S = 15.0
# 3.90 E: pushed frames kept for a recorder draining them by seq, off the
# decision path (`frames_after()`). 16 is 4 s at 4 fps; a recorder further
# behind loses frames and is told how many.
FRAME_BUFFER = 16


class TeleopStall(RuntimeError):
    """Raised by get_camera_frame() when the last pushed frame is older
    than stall_timeout_s (or none has ever been pushed). See this module's
    docstring -- MissionRunner.tick() catches this through its existing
    generic Exception handler; no dedicated handling belongs there."""


class TeleopRobot(RobotInterface):
    """A robot body made of a live phone camera and a human. See this
    module's docstring."""

    def __init__(self, stall_timeout_s: float = DEFAULT_STALL_TIMEOUT_S):
        self.stall_timeout_s = stall_timeout_s
        self._lock = threading.Lock()
        self._frame: Optional[dict] = None
        self._seq = 0
        self._last_pushed_at: Optional[float] = None
        self._recent = collections.deque(maxlen=FRAME_BUFFER)   # 3.90 E
        # 3.90 E: this body's life. A recorder whose cursor came from another
        # (the robot server restarted) must start over, whatever the seqs say.
        self.boot_id = uuid.uuid4().hex[:12]
        # Bounded (3.76 M2, as GridWorld.log): every push and verb lands
        # here for the robot server's life, and only tests read it.
        self.log = collections.deque(maxlen=2000)

    # ---------- ingress: called only by robot/server.py's POST /teleop/frame ----------

    def push_frame(self, image_base64: str, media_type: str = "image/jpeg") -> dict:
        """Stashes the latest frame. Not part of RobotInterface -- a human
        can push a photo; nothing else in the mission loop calls this
        directly. Detected via hasattr(robot, "push_frame") by
        robot/server.py's POST /teleop/frame, rather than an isinstance
        check, so that file stays backend-agnostic. Returns the new
        sequence number, echoed back to the caller as confirmation."""
        with self._lock:
            self._seq += 1
            seq = self._seq      # read under the lock (3.76 M2): a push between
            self._last_pushed_at = time.monotonic()
            self._frame = {
                "image_base64": image_base64,
                "media_type": media_type,
                # MissionMemory reads this; a live phone frame carries no
                # room label, same honest answer replay_robot.py gives.
                "room": "unknown",
                # 3.90: when it arrived here -- the phone's capture, give or
                # take the upload. Its own clock's, so two stamps' difference
                # is the time between two frames (robot/interface.py).
                "captured_at": time.time(),
                "metadata": {"source": "teleop", "seq": self._seq,
                             # 3.90 E: which life of this body numbered it, so
                             # a decision is never joined across a restart.
                             "boot_id": self.boot_id},
            }
            self._recent.append(self._frame)
        self._record(f"PUSH seq={seq}")
        return {"seq": seq}   # ...here and the return gave both pushes its number

    def latest(self) -> dict:
        """3.90 E: the newest seq and this body's life, without the frames
        (review: a start fetched the whole buffer twice to read one number)."""
        with self._lock:
            return {"latest_seq": self._seq, "boot_id": self.boot_id}

    def frames_after(self, after_seq: int) -> dict:
        """3.90 E: every kept frame newer than `after_seq`, oldest first, for
        a recorder. Decisions never read this -- they take the newest frame
        (`get_camera_frame()`), so a slow brain acts on the present. `lost`
        counts frames newer than `after_seq` that already fell out of the
        buffer: the recorder says so in its walk instead of thinning it
        silently. `reset` is true when `after_seq` is ahead of anything this
        body has numbered -- the robot server restarted and its seq began
        again -- and the frames returned are then every one it kept (review:
        a recorder holding an old seq heard "nothing new" for good). A
        restart that has already numbered past the cursor is seen only by
        `boot_id`, which the recorder compares (review)."""
        after_seq = max(0, int(after_seq))
        with self._lock:
            reset = after_seq > self._seq
            if reset:
                after_seq = 0
            frames = [f for f in self._recent if f["metadata"]["seq"] > after_seq]
            oldest = self._recent[0]["metadata"]["seq"] if self._recent else self._seq + 1
            lost = max(0, oldest - after_seq - 1) if after_seq < self._seq else 0
            return {"frames": frames, "latest_seq": self._seq, "lost": lost, "reset": reset,
                    "boot_id": self.boot_id}

    # ---------- sensing ----------

    def get_camera_frame(self) -> dict:
        with self._lock:
            if self._last_pushed_at is None:
                raise TeleopStall("no frame has ever been pushed -- is the phone capturing?")
            age = time.monotonic() - self._last_pushed_at
            if age > self.stall_timeout_s:
                raise TeleopStall(
                    f"no new frame in {age:.1f}s (limit {self.stall_timeout_s}s) -- "
                    "is the phone still capturing?"
                )
            return self._frame

    def get_distance(self) -> float:
        """No sensor. Safety stays inert here, same as replay_robot.py --
        see this module's docstring."""
        return NO_SENSOR_CM

    # ---------- driving: no motor, just an honest ack ----------
    # A human, not this code, is what makes the next pushed frame
    # different. See this module's docstring's "No motor" section.

    def drive_forward(self, speed: int = 50, duration: float = 0.5) -> dict:
        return self._ack("drive_forward", speed=speed, duration=duration)

    def reverse(self, speed: int = 50, duration: float = 0.5) -> dict:
        return self._ack("reverse", speed=speed, duration=duration)

    def turn_left(self, angle: int = 90) -> dict:
        return self._ack("turn_left", angle=angle)

    def turn_right(self, angle: int = 90) -> dict:
        return self._ack("turn_right", angle=angle)

    def stop(self) -> dict:
        return self._ack("stop")

    # ---------- camera pan ----------
    # Same honest non-answer as replay_robot.py: a pan doesn't change
    # what the phone is pointed at, so there's nothing to show for it.

    def look_left(self) -> dict:
        return self._ack("look_left", pan=-1)

    def look_right(self) -> dict:
        return self._ack("look_right", pan=1)

    def look_center(self) -> dict:
        return self._ack("look_center", pan=0)

    # ---------- internal ----------

    def _ack(self, action: str, **kwargs) -> dict:
        self._record(action.upper())
        return {"action": action, "moved": 0, **kwargs}

    def _record(self, event: str) -> None:
        self.log.append(event)
        logger.info(event)
