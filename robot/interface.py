"""
interface.py

The abstract contract brain/ is allowed to depend on. Both the simulation
backend (sim/mock_robot.py) and the eventual real hardware backend
(robot/hardware_robot.py, added in Phase 11) implement this exact set of
methods. brain/ never imports a backend directly -- only this interface,
via robot/factory.py.
"""

from abc import ABC, abstractmethod


class RobotInterface(ABC):
    @abstractmethod
    def drive_forward(self, speed: int, duration: float) -> dict: ...

    @abstractmethod
    def reverse(self, speed: int, duration: float) -> dict: ...

    @abstractmethod
    def turn_left(self, angle: int) -> dict: ...

    @abstractmethod
    def turn_right(self, angle: int) -> dict: ...

    @abstractmethod
    def stop(self) -> dict: ...

    @abstractmethod
    def look_left(self) -> dict: ...

    @abstractmethod
    def look_right(self) -> dict: ...

    @abstractmethod
    def look_center(self) -> dict: ...

    @abstractmethod
    def get_camera_frame(self) -> dict:
        """One camera frame. **Must carry pixels** -- phase S2.

            {"image_base64": str,            # the frame, base64
             "media_type": "image/jpeg",     # or whatever it really is
             "room": str,                    # MissionMemory reads this
             "metadata": {...},              # provenance; policies ignore it
             ...}                            # backend-specific extras

        The image keys are the contract. They exist because the whole
        design rests on this interface not changing when hardware lands
        (`robot/factory.py`), and a Pi camera returns bytes -- so a
        backend that answered with grid facts alone was
        `PLAN-sim-hardening.md` 2.1's blocker, guaranteeing the one
        abstraction brain/ depends on would have to change.

        Anything a backend adds beyond those keys is its own business and
        **no policy on the hardware path may read it.** `MockRobot` puts
        grid coordinates there for the rule-based agent (section 2.2); a
        camera cannot produce them, so a vision policy that reads them is
        cheating and will fail on a real robot.

        Raising is allowed and meaningful: `TeleopRobot` raises when its
        frames have gone stale, and `robot/server.py`'s `/frame` turns
        that into a 503 rather than pretending it has a picture.
        """

    @abstractmethod
    def get_distance(self) -> float: ...
