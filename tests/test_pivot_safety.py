"""
tests/test_pivot_safety.py

`PLAN-ros-alignment.md` 3.19 -- pivots. A rectangle's corners sit 17.1 cm
from the rotation centre and its sides 11.55 cm (UGV Rover, 3.20; 15.1 and
9.9 on the 2WD chassis 3.19 was built on), so a pivot sweeps a ring
beyond the sides; rotation used to be exempt from the veto. One test per
criterion, written before the fix, confirmed red against the code before it.
Judged on ground truth (`tests/footprint_sweep.py`).
"""

import pytest

from tests import footprint_sweep as fs

HOUSES = ("starter_house", "scaled_house", "home_first_floor")
STARTS_PER_HOUSE = 20


@pytest.fixture(scope="module")
def pivots():
    return fs.pivot_sweep(HOUSES, STARTS_PER_HOUSE)


def _show(rs):
    return [{k: (round(v, 2) if isinstance(v, float) else v) for k, v in r.items()} for r in rs[:3]]


def test_criterion_1_no_contact_from_turning(pivots):
    bad = [r for r in pivots if r["min_G"] < min(r["G0"], fs.G_BAR_CM) - 1e-6]
    assert not bad, f"{len(bad)}/{len(pivots)} pivots touched something: {_show(bad)}"


def test_criterion_2_turning_is_not_frozen(pivots):
    eligible = [r for r in pivots if r["room"] >= fs.FREE_ANGLE_MIN_DEG]
    assert len(eligible) >= 20, "the sample must contain pivots with room"
    short = [r for r in eligible
             if r["turned"] < min(r["room"], fs.PIVOT_REACH_DEG) - fs.FREE_ANGLE_SLACK_DEG]
    share = 1 - len(short) / len(eligible)
    assert share >= 0.95, f"only {share:.1%} of pivots with room turned into it: {_show(short)}"


def test_criterion_3_a_robot_close_to_something_can_still_turn_away(pivots):
    by_start = {}
    for r in pivots:
        by_start.setdefault((r["house"], r["x"], r["y"], r["theta"]), []).append(r)
    stuck = [rs for rs in by_start.values()
             if rs[0]["G0"] < 3.0 and max(r["room"] for r in rs) >= fs.FREE_ANGLE_MIN_DEG
             and max(r["turned"] for r in rs) < fs.FREE_ANGLE_MIN_DEG]
    assert not stuck, f"{len(stuck)} close starts could turn neither way: {_show([rs[0] for rs in stuck])}"


def test_criterion_4_the_sim_does_not_let_a_pivot_into_anything():
    free = fs.pivot_sweep(HOUSES, STARTS_PER_HOUSE, clamp=False)
    bad = [r for r in free if r["max_P"] > fs.PENETRATION_BAR_CM]
    assert not bad, f"{len(bad)}/{len(free)} unclamped pivots went into something: " \
                    f"{_show(sorted(bad, key=lambda r: -r['max_P']))}"
