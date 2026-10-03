"""
tests/test_verb_stop_race.py

Handoff 2026-10-02, item 2c: the stop race in `carry_out_verb`.

A `stop()` landing between the verb's `stop_count` check and its
`set_wheel_velocity()` was overwritten by the re-command, and the next
period saw the stop and skipped the final zeroing -- the wheels stayed
commanded until the watchdog, about a second later. The body below injects
the stop in exactly that window, on the verb's Nth re-command.
"""

import pytest

from robot.interface import carry_out_verb
from sim.maps.scaled_house import build_scaled_world
from sim.mock_robot import MockRobot


class _StopInTheWindow(MockRobot):
    """Calls stop() just before the Nth non-zero wheel command lands."""

    def __init__(self, *a, at=3, **kw):
        super().__init__(*a, **kw)
        self.at, self.commands = at, 0

    def set_wheel_velocity(self, left, right):
        if left or right:
            self.commands += 1
            if self.commands == self.at:
                self.stop()
        return super().set_wheel_velocity(left, right)


def _commanded(robot):
    w = robot.get_wheel_state()
    return w["left"]["velocity_rad_s"], w["right"]["velocity_rad_s"]


@pytest.mark.parametrize("verb", ["FORWARD", "LEFT"])
def test_a_stop_in_the_window_leaves_the_wheels_at_zero(verb):
    robot = _StopInTheWindow(build_scaled_world(), render=False, at=3)
    plan = robot.verb_plan(verb, angle=90) if verb == "LEFT" else robot.verb_plan(verb)
    result = carry_out_verb(robot, plan)
    assert result["ended"] == "stopped", result
    assert _commanded(robot) == (0.0, 0.0), "the stop was overwritten by the verb"
