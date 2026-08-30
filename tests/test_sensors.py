"""
tests/test_sensors.py

Phase S5 (PLAN-sim-hardening.md) -- DistanceSensorModel unit tests, plus
one integration test tying it back to robot/safety.py's threshold, since
"min_distance_cm is demonstrably load-bearing" (that plan's definition of
done, item 4) is a claim about the safety layer, not about this module in
isolation.

Run with: pytest tests/test_sensors.py -v
"""

import random

import pytest

from sim.sensors import DistanceSensorModel


def test_default_model_reproduces_exact_cell_math_within_range():
    """No noise, no dropout configured -- matches the exact cells * 30cm
    formula for any in-range reading. (This is not what MockRobot.get_
    distance() uses by default -- see sim/mock_robot.py: a robot's
    `sensor` is None unless sim.sensor_noise.enabled is set, which skips
    this class entirely and keeps the old formula exact everywhere,
    including at 0. This class's own default min_range_cm is a real S5
    feature -- see the next test.)"""
    sensor = DistanceSensorModel()
    assert sensor.read(3) == 90.0


def test_the_range_floor_applies_even_with_no_noise_or_dropout():
    """Clamping to the real sensor's minimum range is one of S5's named
    features in its own right, not a side effect of noise -- a real
    HC-SR04 cannot report 0.0cm any more than it can report -5cm."""
    sensor = DistanceSensorModel(min_range_cm=2.0)
    assert sensor.read(0) == 2.0


def test_noise_jitters_around_the_true_distance():
    sensor = DistanceSensorModel(noise_stddev_cm=5.0, rng=random.Random(1))
    readings = [sensor.read(5) for _ in range(50)]

    assert len(set(readings)) > 1, "noise should produce varying readings, not one repeated value"
    average = sum(readings) / len(readings)
    assert 100.0 < average < 200.0, "should stay centred near the true 150cm, not drift wildly"


def test_dropout_fails_safe_toward_zero_not_toward_clear():
    """0.0cm always trips a safety veto (no sane deployment configures
    min_distance_cm below 0), which is the fail-safe direction -- see this
    module's docstring for why that beats reporting a missing value."""
    sensor = DistanceSensorModel(dropout_rate=1.0)
    for _ in range(10):
        assert sensor.read(10) == 0.0


def test_no_dropout_configured_never_drops(monkeypatch):
    """dropout_rate=0.0 (the default) must never roll the dice at all --
    not merely roll it and almost-always lose."""
    sensor = DistanceSensorModel(dropout_rate=0.0)
    monkeypatch.setattr(sensor.rng, "random", lambda: 0.0)  # would "drop" if the check ran
    assert sensor.read(5) == 150.0


def test_range_is_clamped_to_the_real_sensors_span():
    sensor = DistanceSensorModel(min_range_cm=2.0, max_range_cm=50.0)
    assert sensor.read(0) >= 2.0
    assert sensor.read(100) <= 50.0


# ---------- tying it back to the safety layer ----------


def test_min_distance_cm_is_load_bearing_at_a_non_multiple_of_30():
    """PLAN-sim-hardening.md 3.2's own finding: without a sensor model,
    min_distance_cm is only ever crossed at exactly 0.0cm -- any value
    from 1 to 30 makes every existing test pass identically. A noisy
    reading (here, a fixed +7cm offset so the assertion doesn't depend on
    a seed landing where it needs to) makes the threshold load-bearing at
    a value that is neither 0 nor a multiple of 30."""

    class FixedOffsetRng(random.Random):
        def gauss(self, mu, sigma):
            return 7.0

    sensor = DistanceSensorModel(noise_stddev_cm=1.0, rng=FixedOffsetRng())
    reading = sensor.read(3)  # 3 cells * 30cm + 7cm offset = 97.0

    assert reading == 97.0
    assert reading % 30.0 != 0.0, "noise moved the reading off the old quantization"

    from robot.safety import SafetyController, SafetyViolation

    class FixedDistanceRobot:
        def get_distance(self):
            return reading

        def stop(self):
            return {"action": "stop"}

        def drive_forward(self, speed=50, duration=0.5):
            return {"action": "drive_forward"}

    # A threshold on either side of 97.0 -- neither 0 nor a multiple of
    # 30 -- must actually change the outcome.
    blocked = SafetyController(FixedDistanceRobot(), min_distance_cm=100.0)
    with pytest.raises(SafetyViolation):
        blocked.check_and_execute("FORWARD")

    allowed = SafetyController(FixedDistanceRobot(), min_distance_cm=90.0)
    result = allowed.check_and_execute("FORWARD")
    assert result["action"] == "drive_forward"
