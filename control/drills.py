"""
drills.py

Fault injection, so the failsafes can be *shown* rather than only tested.

Every stage of this project is supposed to be verifiable from the digital
twin. For most of Stage 2 that is easy -- start a mission from the phone
and watch the robot move. B3.2 (the AWS link dies) and B3.3 (the brain
loop hangs) are the exceptions: they are the two behaviors that matter
most on hardware, and neither can be provoked by pressing anything. You
would have to unplug the internet at the right moment, or ship a bug.

So the brain accepts an optional `fault` on POST /mission/start, and the
twin exposes it as a "failsafe drill" picker. Each drill breaks exactly
one thing and leaves every other guard in place, so what you watch happen
is the real failsafe firing, not a simulation of one:

    vision_error   every vision call raises      -> B3.2 failure budget
    vision_hang    every vision call never returns -> B3.2 per-call timeout
    tick_hang      the loop stops returning      -> B3.3 dead-man

Two deliberate constraints. **Drills are fail-safe by construction**: each
one can only end a mission with the robot stopped, which is the same thing
the guards do on their own -- there is no drill that makes the robot move.
And they run on *shortened* timeouts (`drill_vision_timeout_s`,
`drill_tick_timeout_s`), because a demonstration that takes 60 seconds of
staring at a phone does not get watched. The guard being exercised is
identical; only the deadline is shorter.

`brain.allow_drills` in config/robot.yaml turns the whole surface off for
a deployment where nobody should be able to end someone else's mission.
"""

import logging
import time
from typing import Callable

from control.mission_runner import MissionRunner

logger = logging.getLogger("drills")

NONE = "none"
VISION_ERROR = "vision_error"
VISION_HANG = "vision_hang"
TICK_HANG = "tick_hang"

FAULTS = (NONE, VISION_ERROR, VISION_HANG, TICK_HANG)

# How far past the deadline a drill overshoots. Long enough that the guard
# is unambiguously what fired, short enough that the abandoned thread is
# gone seconds later rather than lingering for the life of the process.
OVERSHOOT = 3.0


class DrillNotAllowed(RuntimeError):
    """A fault was requested on a brain that has drills switched off."""


class TickHangRunner(MissionRunner):
    """Drill: a mission whose loop stops coming back.

    The first tick runs normally -- so the twin shows the mission alive and
    stepping before it freezes, which is what makes the dead-man's job
    legible -- and every tick after it blocks. Nothing else is disabled:
    the robot is idle and healthy, the brain process is alive and
    responsive, and only B3.3 can tell that anything is wrong. That is
    precisely the failure robot/server.py's watchdog cannot see.
    """

    def __init__(self, *args, hang_s: float = 10.0, **kwargs):
        super().__init__(*args, **kwargs)
        self.hang_s = hang_s
        self._ticks = 0

    def tick(self) -> bool:
        self._ticks += 1
        if self._ticks > 1:
            logger.warning(f"drill: hanging this tick for {self.hang_s}s")
            time.sleep(self.hang_s)
        return super().tick()


def vision_fn_for(fault: str, timeout_s: float) -> Callable[[dict], dict]:
    """The stand-in vision call for a B3.2 drill."""
    if fault == VISION_ERROR:
        def erroring(frame: dict) -> dict:
            raise RuntimeError("drill: simulated vision service failure")
        return erroring

    def hanging(frame: dict) -> dict:
        time.sleep(timeout_s * OVERSHOOT)
        return {}
    return hanging


def apply(fault: str, kwargs: dict, config: dict) -> tuple:
    """Fold a drill into the runner arguments the brain server built.

    Returns (runner_class, kwargs, tick_timeout_s). With no fault this is
    the identity: the production path and the drill path build the same
    object, so a drill cannot quietly become a different mission.
    """
    fault = fault or NONE
    if fault not in FAULTS:
        raise ValueError(f"Unknown fault: {fault!r}. Known drills: {', '.join(FAULTS)}")
    if fault != NONE and not config.get("allow_drills", True):
        raise DrillNotAllowed(
            "Failsafe drills are disabled on this brain (brain.allow_drills)."
        )

    tick_timeout_s = config["tick_timeout_s"]
    if fault == NONE:
        return MissionRunner, kwargs, tick_timeout_s

    logger.warning(f"starting a {fault} failsafe drill")
    if fault == TICK_HANG:
        tick_timeout_s = config["drill_tick_timeout_s"]
        return (
            lambda *a, **kw: TickHangRunner(*a, hang_s=tick_timeout_s * OVERSHOOT, **kw),
            kwargs,
            tick_timeout_s,
        )

    drill_timeout = config["drill_vision_timeout_s"]
    kwargs = {
        **kwargs,
        "vision_fn": vision_fn_for(fault, drill_timeout),
        "vision_timeout_s": drill_timeout,
    }
    return MissionRunner, kwargs, tick_timeout_s
