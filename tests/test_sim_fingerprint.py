"""
tests/test_sim_fingerprint.py -- PLAN-ros-alignment.md 3.74 criterion A1.

A house with no object heights senses exactly what it did before 3.74:
scans (full and safety-hinted), the fake lidar's beams, the depth grid's
level row, the scalar, the detections and the discovered map, over a
seeded 500-step drive in each of the four houses. The digests were taken
on `dev` at 2d54e78, before any 3.74 code. A change that moves one is a
change to how the simulator senses its world, which must be deliberate:
re-pin it in the commit that makes it, and say why.

In SLOW_FILES (~20 s): it guards the simulator, not a hot path.
"""
import pytest

from tests.sim_fingerprint import fingerprint

PINNED = {
    "starter_house": "2abc39351fd2346626c8f6bd575d13079e1a443cd01cb20b6086d9fb45a23947",
    "scaled_house": "15cf233091d482afd617c7b79f8121529708e7fe7b96cf2c7192f08adaff05f2",
    "home_first_floor": "89a73bcb386f63d4cd972b5ff62d8576eae1f533d9280e0555deffe362bdaa0a",
    "complex_house": "299d40cbec0e7bb82be94e0e7fc8e56f59b2bc5dabdc54b77e5e86eb8adaef5a",
}


@pytest.mark.parametrize("house", sorted(PINNED))
def test_a_house_without_heights_senses_what_it_did(house):
    assert fingerprint(house) == PINNED[house]
