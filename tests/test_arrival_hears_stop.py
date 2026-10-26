"""
tests/test_arrival_hears_stop.py

`PLAN-ros-alignment.md` 3.104: a person's Stop (3.84's latch) or D-pad
take-over while the arrival's identity confirmation waits is heard before the
mission is finished: it ends `cancelled`, never `found`. The brain hears the
latch only as a refused verb, and between the arrival and `_finish(FOUND)` it
sent none.

In process over the simulator: the robot latches when the confirmation is
asked, as a person pressing Stop while it waits would.
"""

import json
import logging

import pytest

from brain.intervention_log import InterventionLog
from brain.perceive import FrameReportedPipeline
from brain.tiered import TRIGGER_ARRIVAL, TieredVision
from control.mission_runner import CANCELLED, FOUND, MissionRunner
from robot.interface import Preempted, StoppedByPerson
from robot.safety import SafetyViolation
from sim.mock_robot import MockRobot
from tests.conftest import mock_world_for
from tests.test_bearing_turns import TARGET, _quiet_cloud
from tests.test_cloud_health import _placed


@pytest.fixture(autouse=True)
def _quiet_logs():
    logging.disable(logging.WARNING)
    yield
    logging.disable(logging.NOTSET)


# What the robot server answers a brain verb after the press (robot/server.py):
# a person's Stop latch, a person holding the robot, a veto, the mission's own
# nav2 goal still holding, or no answer at all.
REFUSALS = {
    "stop": StoppedByPerson("twin-dpad pressed Stop -- autonomy stays stopped until a new "
                            "mission starts"),
    "dpad": Preempted("twin-dpad is driving"),
    "veto": SafetyViolation("forward clearance 12.0 cm < 20.0 cm"),
    "own goal": Preempted("ros is driving (nav2 goal is active)"),
    "autonomous": Preempted("brain is driving"),
    "down": ConnectionError("robot server unreachable"),
}


class PressedDuringConfirm(MockRobot):
    """Once `press()` is called, every brain verb gets `refusal`; the stop
    the runner sends at its end still lands."""

    def __init__(self, grid, refusal=None):
        super().__init__(grid, render=False)
        self.refusal, self.pressed, self.probes = refusal, False, 0

    def press(self):
        self.pressed = True

    def _refuse(self):
        if self.pressed and self.refusal is not None:
            raise self.refusal

    def look_center(self):
        if self.pressed:
            self.probes += 1
        self._refuse()
        return super().look_center()


def _mission(refusal=None, press=True, log_to=None):
    robot = PressedDuringConfirm(_placed(), refusal)

    def confirm(frame):
        if press:
            robot.press()               # the person presses while the call is out
        return _quiet_cloud(frame)      # ... and the cloud then says yes

    tier = TieredVision(FrameReportedPipeline(TARGET), _quiet_cloud, steer_on_sight=True,
                        hold_goal=True, confirm_vision_fn=confirm)
    runner = MissionRunner(robot, target_object=TARGET, max_steps=60, policy="tiered",
                           vision_fn=tier, world=mock_world_for(robot))
    if log_to is not None:
        runner.intervention_log = InterventionLog(log_to)
    runner.start()
    while runner.tick():
        pass
    assert tier.stats.triggers.get(TRIGGER_ARRIVAL), "never reached arrival -- proves nothing"
    return runner.status(), robot, runner


# ---------- criterion 1: a person in that window ends it cancelled ----------


def test_a_persons_stop_during_the_confirmation_ends_cancelled_not_found(tmp_path):
    path = tmp_path / "interventions.jsonl"
    status, _, runner = _mission(REFUSALS["stop"], log_to=path)
    assert runner.intervention_done.wait(10), "the intervention line never landed"
    assert status["outcome"] == CANCELLED, status["log_tail"][-3:]
    assert status["cancelled_by"] == "stop", status
    assert "twin-dpad" in status["end_reason"], status
    [line] = [json.loads(x) for x in path.read_text().splitlines()]
    assert (line["intervention"]["kind"], line["intervention"]["via"]) == ("stop", "robot_stop")


def test_a_persons_take_over_during_the_confirmation_ends_cancelled_by_the_dpad():
    status, _, _ = _mission(REFUSALS["dpad"])
    assert status["outcome"] == CANCELLED, status["log_tail"][-3:]
    assert status["cancelled_by"] == "dpad", status


# ---------- criterion 2: nothing else changes a found ----------


def test_without_a_press_it_is_still_found():
    status, robot, _ = _mission(REFUSALS["stop"], press=False)
    assert status["outcome"] == FOUND, status["log_tail"][-3:]


@pytest.mark.parametrize("kind", ["veto", "own goal", "autonomous", "down"])
def test_a_veto_its_own_goal_an_autonomous_holder_or_no_answer_keeps_the_found(kind):
    """Review of c0432da (both passes): only a person's refusal ends it; an
    autonomous holder at the probe is no person and read `failed` there.""" 
    status, robot, _ = _mission(REFUSALS[kind])
    assert status["outcome"] == FOUND, (kind, status["log_tail"][-3:])
    assert robot.probes == 1, robot.probes
