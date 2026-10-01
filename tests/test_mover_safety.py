"""
tests/test_mover_safety.py

PLAN-ros-alignment.md 3.30, criterion 2, pinned on a sample: with a mover
crossing its path, the robot never brings itself under 18 cm of
travel-to-contact, never touches anything and never ends inside anything --
judged on ground truth (`tests/mover_sweep.py`, 3.18's geometry). The full
sweep is `python -m tests.demo_mover_sweep`.

The second test is the bar's teeth: the same sample with the safety clamp
switched off must FAIL it, or a pass would only mean the mover never got
in the way.
"""

import pytest

from tests.mover_sweep import sweep, verdicts

SAMPLE = dict(houses=["scaled_house"], starts_per_house=2, headings=8)


@pytest.fixture(scope="module")
def clamped():
    return sweep(**SAMPLE)


def test_no_contact_the_robot_caused_with_a_mover_crossing(clamped):
    v = verdicts(clamped)
    for bar in ("travel-to-contact after a robot move >= 18 cm",
                "no contact (gap >= 1 cm)", "no penetration"):
        ok, detail = v[bar]
        assert ok, f"{bar}: {detail}"


def test_the_sample_actually_meets_the_mover(clamped):
    ok, detail = verdicts(clamped)["exercised"]
    met = sum(r["nearest_mover_cm"] <= 30.0 for r in clamped)
    assert ok and met >= len(clamped) // 2, detail


def test_without_the_clamp_the_same_sample_fails():
    v = verdicts(sweep(**SAMPLE, clamp=False))
    assert not v["travel-to-contact after a robot move >= 18 cm"][0]
