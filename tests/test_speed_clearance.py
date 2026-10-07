"""PLAN-ros-alignment.md 3.44 -- the stop distance follows speed.

A straight move ASKED at `CREEP_M_S` or slower keeps `CREEP_MARGIN_CM`
(3 cm); every other move keeps `max(min_distance_cm, stop_distance_cm(v))`,
so today's speeds keep exactly 20 cm and a faster one keeps more. Judged on
ground truth (`tests/footprint_sweep.py`), never on the readings the veto
uses. The full sweeps and their numbers are in the plan entry; these are
the pinned samples.
"""

import logging
import math

import pytest

import tests.creep_sweep as cs
import tests.footprint_sweep as fs
from robot.safety import (CREEP_M_S, CREEP_MARGIN_CM, FOOTPRINT_LENGTH_M, MAX_LINEAR_M_S,
                          SafetyController, SafetyViolation, stop_distance_cm)
from sim.maps import build_world
from sim.mock_robot import MockRobot, WHEEL_RADIUS_M

HOUSES = ("starter_house", "scaled_house", "home_first_floor")


@pytest.fixture(autouse=True)
def _quiet():
    # One warning per refusal, and the sweeps refuse thousands of times.
    logger = logging.getLogger("safety")
    level = logger.level
    logger.setLevel(logging.ERROR)
    yield
    logger.setLevel(level)


# ---------- the rule ----------

def test_todays_speeds_keep_exactly_twenty_centimetres():
    safety = SafetyController(None, 20.0)
    for v in (0.05, 0.1, 0.2, 0.3):
        assert safety.required_clearance_cm(v) == 20.0
        assert safety.required_clearance_cm(-v) == 20.0
    assert safety.required_clearance_cm(None) == 20.0       # unknown speed: the full bar


def test_a_creep_needs_three_centimetres_only_when_it_was_asked():
    safety = SafetyController(None, 20.0)
    assert safety.required_clearance_cm(CREEP_M_S, creep=True) == CREEP_MARGIN_CM
    assert safety.required_clearance_cm(-0.01, creep=True) == CREEP_MARGIN_CM
    # The same speed, not asked (a nav2 twist slowing near its goal): full bar.
    assert safety.required_clearance_cm(CREEP_M_S, creep=False) == 20.0
    # Asked at creep but commanded faster (a ramp's overshoot): full bar.
    assert safety.required_clearance_cm(0.1, creep=True) == 20.0


def test_a_faster_move_needs_more_room():
    safety = SafetyController(None, 20.0)
    assert safety.required_clearance_cm(0.4) == pytest.approx(25.0)
    assert safety.required_clearance_cm(0.5) == pytest.approx(35.5)
    assert safety.required_clearance_cm(0.6) == pytest.approx(48.0)
    assert stop_distance_cm(0.0) == CREEP_MARGIN_CM


# ---------- criterion 3: slowed is not creep ----------

def _facing_something(house="scaled_house", lo=30.0, hi=50.0):
    """A seeded start whose truth puts something lo-hi cm straight ahead."""
    for seed in range(20):
        for x, y in fs.starts(house, 20, seed):
            for k in range(fs.HEADINGS):
                world = build_world(house)
                world.x, world.y, world.theta = x, y, math.radians(k * 15)
                if lo <= fs.truth(world, +1)[0] <= hi:
                    return world
    raise AssertionError("no start found")


@pytest.mark.parametrize("v", [0.2, 0.02])
def test_an_unasked_twist_keeps_the_full_bar_however_slow(v):
    """nav2's twists are never asked at creep speed, so they keep 20 cm --
    including one nav2 itself slowed below CREEP_M_S near its goal. Red
    (0.02) on a version that keys the creep margin on the commanded speed
    rather than the ask: that twist would then drive to 3 cm. The vet keeps
    no state between periods, so its OWN slowing can cost at most one
    period past the line; the verb path's look-ahead is pinned below."""
    world = _facing_something()
    robot = MockRobot(world, render=False)
    safety = SafetyController(robot, 20.0)
    w = v / WHEEL_RADIUS_M
    for _ in range(int(25.0 / (v * 100 * fs.PERIOD_S)) + 20):
        left, right, _ = safety.vet_wheel_velocity(w, w)
        robot.set_wheel_velocity(left, right)
        robot.advance(fs.PERIOD_S)
    clearance = safety.forward_clearance()[0]
    assert clearance >= 19.4, clearance          # 3.22's VETO_BAR_CM: the line, less a quantum
    assert fs.truth(world, +1)[0] >= fs.T_BAR_CM


def test_a_verb_the_look_ahead_slowed_still_stops_at_twenty():
    """A verb asked at speed 50 is slowed below CREEP_M_S in its last
    periods by `run_verb()`'s look-ahead; it must still stop at 20 cm, then
    be refused there. Red on a version that keys the bar on the slowed
    per-period speed."""
    world = _facing_something()
    robot = MockRobot(world, render=False)
    safety = SafetyController(robot, 20.0)
    with pytest.raises(SafetyViolation):
        for _ in range(4):
            safety.check_and_execute("FORWARD", speed=50, duration=0.5)
    assert safety.forward_clearance()[0] >= 19.4
    assert fs.truth(world, +1)[0] >= fs.T_BAR_CM


def test_the_asked_speed_does_not_outlive_its_verb():
    world = _facing_something()
    robot = MockRobot(world, render=False)
    safety = SafetyController(robot, 20.0)
    with pytest.raises(SafetyViolation):
        for _ in range(20):
            safety.check_and_execute("FORWARD", speed=cs.CREEP_SPEED, duration=0.5)
    assert safety._asked_v_m_s is None and not safety._creep_asked
    # The wheel loop afterwards (nav2): the full bar, so a twist is refused.
    w = 0.1 / WHEEL_RADIUS_M
    left, right, reason = safety.vet_wheel_velocity(w, w)
    assert (left, right) == (0.0, 0.0) and reason


# ---------- criterion 2: a creep never touches ----------

@pytest.mark.parametrize("path,lidar", [("verb", False), ("verb", True), ("wheels", False)])
def test_criterion_2_a_creep_never_comes_within_two_centimetres(path, lidar):
    res = cs.sweep(starts_per_house=40, path=path, lidar=lidar)
    v = cs.verdicts(res)
    assert v["2 creep never touches"][0], v
    assert v["2 creep keeps 2 cm"][0], v
    # The progress half -- red on the 20 cm bar for every speed (3.43's stalls).
    assert v["progress: creep closes in"][0], v


def test_criterion_2_the_creep_sweep_sees_a_margin_that_is_too_small(monkeypatch):
    # The instrument can fail: no margin at all lets a creep come within 2 cm.
    monkeypatch.setattr("robot.safety.CREEP_MARGIN_CM", 0.0)
    res = cs.sweep(houses=HOUSES[:1], starts_per_house=40, path="verb")
    assert not cs.verdicts(res)["2 creep keeps 2 cm"][0]


# ---------- criterion 4: the bar grows above 20 cm ----------

@pytest.mark.parametrize("v,lidar", [(0.4, False), (0.5, False), (0.5, True)])
def test_criterion_4_faster_runs_stop_short_and_never_touch(v, lidar):
    res = fs.sweep(HOUSES, 3, lidar=lidar, speed_m_s=v)
    verdicts = fs.verdicts(res)
    assert verdicts["1 stopping distance"][0], verdicts
    assert verdicts["2 no contact"][0], verdicts
    # And the bar did grow: every run stops further out than 20 cm allows.
    worst = min(r["T_after_move"] for r in res)
    need = SafetyController(None, 20.0).required_clearance_cm(v)
    assert worst >= need - 5.0, (worst, need)


def test_criterion_4_the_scan_hint_covers_the_top_speeds_bar():
    """The simulator reports beams past the safety layer's range hint as
    "nothing there", so a hint shorter than the bar makes the sim blind
    where the car sees. Red with today's fixed 0.6 m hint (0.47 m past the
    front edge, under 0.6 m/s's 0.48 m bar plus a period's travel)."""
    safety = SafetyController(None, 20.0)
    need_m = (safety.required_clearance_cm(MAX_LINEAR_M_S) + MAX_LINEAR_M_S * 100 * fs.PERIOD_S) / 100
    assert safety.scan_hint_m() >= FOOTPRINT_LENGTH_M / 2 + need_m
