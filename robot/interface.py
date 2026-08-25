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
    def get_camera_frame(self) -> dict: ...

    @abstractmethod
    def get_distance(self) -> float: ...
