"""PLAN-ros-alignment.md 3.42 criterion 4 -- safety under a real lidar's
timing, on ground truth (3.18's method): the vet reads the D500's scan
through `robot/lidar_ld19.py`, every point cast at its own instant from a
MOVING robot, 0-100 ms old, with no depth grid and no scalar sensor (4c).

Measured 2026-10-06 on the same samples 3.18/3.19 pin:
* straight runs: 0/432 under 18 cm, 0 contacts, progress 97.7% -- the same
  as the simulator's instantaneous scan with its depth grid;
* pivots, BEFORE the fix: 105/120 within 1 cm (worst 0.0) -- the guard
  checked the next 50 ms against a scan it treated as current, and the
  same pivots with a fresh scan read 0;
* pivots, AFTER ageing the rotation by the encoders' turn since the scan
  (`SafetyController._turn_since_scan`): 0/120 within 1 cm, 24/24 with room
  turned into it.
"""

import pytest

import tests.footprint_sweep as fs

HOUSES = ("starter_house", "scaled_house", "home_first_floor")


@pytest.fixture(scope="module")
def straight():
    return fs.sweep(HOUSES, 3, lidar=True)


@pytest.fixture(scope="module")
def pivots():
    return fs.pivot_sweep(HOUSES, 20, lidar=True)


def test_straight_runs_stop_short_and_never_touch(straight):
    v = fs.verdicts(straight)
    assert v["1 stopping distance"][0], v
    assert v["2 no contact"][0], v
    assert v["3 progress"][0], v


def test_pivots_never_close_within_a_centimetre(pivots):
    bad = [r for r in pivots if r["min_G"] < min(r["G0"], fs.G_BAR_CM) - 1e-6]
    assert not bad, f"{len(bad)}/{len(pivots)}"


def test_pivots_with_room_still_turn_into_it(pivots):
    eligible = [r for r in pivots if r["room"] >= fs.FREE_ANGLE_MIN_DEG]
    short = [r for r in eligible
             if r["turned"] < min(r["room"], fs.PIVOT_REACH_DEG) - fs.FREE_ANGLE_SLACK_DEG]
    assert len(eligible) >= 20 and len(short) / len(eligible) <= 0.05


def test_without_the_turn_since_the_scan_the_guard_fails(monkeypatch):
    # The finding, kept red: rotation not aged -> pivots come within 1 cm.
    monkeypatch.setattr(fs._LidarTimed, "turn_since_scan", lambda self: None)
    monkeypatch.setattr(fs._LidarTimed, "sensor_age_s", lambda self: None)
    piv = fs.pivot_sweep(HOUSES[:1], 20, lidar=True)
    assert any(r["min_G"] < min(r["G0"], fs.G_BAR_CM) - 1e-6 for r in piv)
