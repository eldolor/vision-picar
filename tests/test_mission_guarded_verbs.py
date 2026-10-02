"""
tests/test_mission_guarded_verbs.py

Found 2026-10-01 while measuring `PLAN-ros-alignment.md` 3.31: an
IN-PROCESS mission's verbs bypassed 3.22's guarded verbs. `MissionRunner`
wraps its robot in `_HaltGate`, which did not forward `verb_plan()`, so the
safety layer saw None and called the raw verb -- turns with no pivot vetting
(3.19), forwards checked once instead of every period. On ground truth, six
of twenty furnished-home missions then turned a corner into furniture.
These pin that a mission's verbs go through `SafetyController.run_verb()`
and that a stop made through the gate still ends a verb in progress.
"""

import math

from robot import safety as safety_mod
from sim.maps import build_world
from sim.mock_robot import MockRobot
from tests.conftest import fresh_mock_runner, mock_world_for
from control.mission_runner import MissionRunner, _HaltGate


def test_a_missions_verbs_are_carried_out_by_the_safety_layer(monkeypatch):
    planned = []
    orig = safety_mod.SafetyController.run_verb

    def counting(self, plan):
        planned.append(plan["kind"])
        return orig(self, plan)

    monkeypatch.setattr(safety_mod.SafetyController, "run_verb", counting)
    runner = fresh_mock_runner(max_steps=30, target_object="red backpack")
    runner.start()
    while runner.tick():
        pass
    assert "turn" in planned and "straight" in planned, planned


def test_the_gate_forwards_the_body_s_plan_and_stop_count():
    body = MockRobot(build_world("starter_house"), render=False)
    gate = _HaltGate(body, lambda: True)
    assert gate.verb_plan("LEFT", angle=45) == body.verb_plan("LEFT", angle=45)
    before = gate.stop_count
    gate.stop()
    assert gate.stop_count == before + 1


def test_a_finished_mission_refuses_a_plan():
    body = MockRobot(build_world("starter_house"), render=False)
    gate = _HaltGate(body, lambda: False)
    try:
        gate.verb_plan("FORWARD")
    except Exception as e:  # noqa: BLE001
        assert "mission is over" in str(e)
    else:
        raise AssertionError("a gate whose mission is over handed out a plan")
