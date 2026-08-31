"""
replay_robot.py

A robot body made of photographs: a walk you already took, played back
one frame at a time.

Point it at a directory of images -- a walk recorded from the twin's Guide
tab in Robot view, or photos taken by hand -- and it implements
`RobotInterface` well enough to run a whole mission against real pixels.
Each movement command advances to the next frame; the camera returns the
frame you are standing on.

## What this is for

Stage 0 asked whether the vision model can navigate from real photographs
at all, one frame at a time. It cannot answer the next question: whether
the *loop* works -- whether memory, the step budget, the failure budget,
the arrival signal and the cost per mission behave in a real house. That
needs the whole harness running on real pixels, and this is the cheapest
way to get there, because a recorded walk can be replayed a hundred times
while you change a prompt, with no hardware and no walking.

## What it cannot tell you -- read this before trusting a run

**It is open loop.** The frames follow the path *you* walked. If the model
says LEFT at frame 12, frame 13 is still whatever you photographed next,
not what a robot that turned left would have seen. So this measures
memory, lifecycle, cost and wall-clock honestly, and it does **not**
measure navigation. A run where every action looks sensible is evidence
the model reads scenes; it is not evidence a robot would have arrived.

**The safety layer is inert.** A photograph has no distance in it, so
`get_distance()` returns `NO_SENSOR_CM` -- a value far above any
threshold, which means `robot/safety.py` never vetoes anything here. That
is the honest representation of "this backend has no ultrasonic", and it
is why a clean replay says nothing about collision avoidance. Phases S5
and 11 are where that gets tested.

**There is no room.** Photographs carry no room label, so
`MissionMemory.visited_rooms` stays empty and nothing stops a policy
re-searching. See `brain/vision_agent.py`.

## Frame shape

Returns the image-bearing frame contract that phase S2 specifies for
every backend:

    {"image_base64": str, "media_type": str, "metadata": {...}}

`metadata` carries replay bookkeeping -- filename, index, how many frames
remain -- as **debug data no policy may read**, the same rule S2 sets for
the simulator's grid facts. `room` is present and `"unknown"` because
`MissionMemory` looks for it.
"""

import base64
import logging
from pathlib import Path

from robot.interface import NO_SENSOR_CM as _NO_SENSOR_CM, RobotInterface

logger = logging.getLogger("replay_robot")

SUFFIXES = {".jpg", ".jpeg", ".png", ".webp", ".gif"}
MEDIA_TYPES = {
    ".jpg": "image/jpeg", ".jpeg": "image/jpeg", ".png": "image/png",
    ".webp": "image/webp", ".gif": "image/gif",
}

# Re-exported from robot/interface.py, which is where the contract lives
# now. Kept as a module-level name because callers and tests import it from
# here, and because it is genuinely part of this backend's story.
NO_SENSOR_CM = _NO_SENSOR_CM

MOVEMENTS = ("drive_forward", "reverse", "turn_left", "turn_right")


class ReplayRobot(RobotInterface):
    """Plays a directory of images back as if walking through them."""

    def __init__(self, frames_dir, loop: bool = False):
        self.dir = Path(frames_dir)
        self.frames = sorted(
            p for p in self.dir.iterdir() if p.suffix.lower() in SUFFIXES
        ) if self.dir.is_dir() else []
        if not self.frames:
            raise ValueError(
                f"No images in {self.dir} (looked for {', '.join(sorted(SUFFIXES))})"
            )
        self.loop = loop
        self.index = 0
        self.exhausted = False
        self.log: list = []

    # ---------- driving ----------

    def drive_forward(self, speed: int = 50, duration: float = 0.5) -> dict:
        return self._advance("drive_forward", speed=speed, duration=duration)

    def reverse(self, speed: int = 50, duration: float = 0.5) -> dict:
        return self._advance("reverse", speed=speed, duration=duration)

    def turn_left(self, angle: int = 90) -> dict:
        return self._advance("turn_left", angle=angle)

    def turn_right(self, angle: int = 90) -> dict:
        return self._advance("turn_right", angle=angle)

    def stop(self) -> dict:
        self._record("STOP")
        return {"action": "stop"}

    # ---------- camera pan ----------
    # A pan does not advance the walk: you did not photograph the view to
    # your left, so there is nothing to show. Recorded honestly as a no-op
    # rather than silently consuming a frame that faces the wrong way.

    def look_left(self) -> dict:
        return self._pan("look_left")

    def look_right(self) -> dict:
        return self._pan("look_right")

    def look_center(self) -> dict:
        return self._pan("look_center")

    # ---------- sensing ----------

    def get_camera_frame(self) -> dict:
        path = self.frames[self.index]
        data = base64.b64encode(path.read_bytes()).decode()
        return {
            "image_base64": data,
            "media_type": MEDIA_TYPES.get(path.suffix.lower(), "image/jpeg"),
            # MissionMemory reads this; a photograph has no room label.
            "room": "unknown",
            "metadata": {
                "source": "replay",
                "file": path.name,
                "index": self.index,
                "frames_total": len(self.frames),
                "frames_remaining": len(self.frames) - 1 - self.index,
                "exhausted": self.exhausted,
            },
        }

    def get_distance(self) -> float:
        """No sensor. See NO_SENSOR_CM and this module's docstring -- the
        safety layer cannot be exercised by a replay."""
        return NO_SENSOR_CM

    # ---------- internal ----------

    def _advance(self, action: str, **kwargs) -> dict:
        moved = 0
        if self.index + 1 < len(self.frames):
            self.index += 1
            moved = 1
        elif self.loop:
            self.index = 0
            moved = 1
        else:
            # Out of walk. Reported rather than raised: a mission that runs
            # past the end of its recording should finish on its own terms
            # (the policy sees the same frame again, and the step budget or
            # the arrival signal ends it) instead of dying with a traceback
            # halfway through a paid run.
            self.exhausted = True
            logger.info("replay exhausted -- holding on the last frame")
        self._record(f"{action.upper()} index={self.index} moved={moved}")
        return {"action": action, "moved": moved, "index": self.index,
                "exhausted": self.exhausted, **kwargs}

    def _pan(self, action: str) -> dict:
        self._record(action.upper())
        return {"action": action, "pan": 0, "note": "replay has no side views"}

    def _record(self, event: str) -> None:
        self.log.append(event)
        logger.info(event)
