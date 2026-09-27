"""
tests/test_pan_safety.py

`PLAN-ros-alignment.md` 3.18, part 2 -- the flaky wall stop of 3.17, which
turned out to be the camera. A preempted frontier mission left the pan at 90
degrees; the depth grid, and so the forward veto's path cone, is cast along
the CAMERA; the robot drove north guarded by a cone looking west. One test
per criterion, written before the fix and confirmed red against the code
3.17 found it in.
"""

import pytest

from robot.interface import ZONE_RANGE, unusable_scan
from robot.safety import SafetyController
from tests import footprint_sweep as fs

HOUSES = ("starter_house", "scaled_house", "home_first_floor")
STARTS_PER_HOUSE = 2
PANS = (-1.0, 1.0)   # fully left, fully right (x 90 degrees)


@pytest.fixture(scope="module")
def centred():
    return fs.sweep(HOUSES, STARTS_PER_HOUSE, directions=(+1,))


@pytest.fixture(scope="module", params=PANS, ids=["panned-left", "panned-right"])
def panned(request):
    return fs.sweep(HOUSES, STARTS_PER_HOUSE, directions=(+1,), pan=request.param)


def test_criterion_1_the_path_guard_does_not_turn_with_the_camera(panned):
    verdict = fs.verdicts(panned)
    for name in ("1 stopping distance", "2 no contact"):
        ok, detail = verdict[name]
        assert ok, f"camera panned {panned[0]['pan']:+}: {name}: {detail}"


def test_criterion_2_a_panned_camera_never_stops_the_robot_earlier(centred, panned):
    early = [(c, p) for c, p in zip(centred, panned) if p["travel"] < c["travel"] - 0.5]
    assert not early, (
        f"{len(early)}/{len(centred)} runs stopped earlier with the camera panned "
        f"{panned[0]['pan']:+}: " + str([(c["house"], round(c["x"], 2), round(c["y"], 2), c["heading"],
                                          round(c["travel"], 1), round(p["travel"], 1)) for c, p in early[:4]]))


class _PannedNoScan:
    """A robot whose camera faces 90 degrees left, with a depth grid that
    sees a clear field there, and no lidar at all."""

    def get_depth_grid(self):
        return {"rows": 1, "cols": 8, "fov_deg": 60.0, "pan_deg": -90.0,
                "zones": [{"status": ZONE_RANGE, "distance_cm": 200.0}] * 8}

    def get_distance(self):
        return 200.0

    def get_scan(self, max_range_m=None):
        return unusable_scan()


def test_criterion_3_no_scan_and_a_camera_facing_away_refuses_forward():
    clearance, source = SafetyController(_PannedNoScan()).forward_clearance()
    assert clearance is not None and clearance < 20.0, (
        f"a forward move was waved through on a camera looking elsewhere: {clearance} ({source})")
    assert "not_observed" in source


def test_a_centred_camera_still_answers_exactly_as_before():
    """Criterion 3's refusal must not leak into the ordinary case: pan 0 is
    the M3 path cone, unchanged."""
    class Centred(_PannedNoScan):
        def get_depth_grid(self):
            return {**super().get_depth_grid(), "pan_deg": 0.0}
    assert SafetyController(Centred()).forward_clearance() == (200.0, "depth_grid")
