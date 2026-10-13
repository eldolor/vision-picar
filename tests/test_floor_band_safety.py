"""
tests/test_floor_band_safety.py -- PLAN-ros-alignment.md 3.74 part B,
seeded samples of the sweeps in `tests/floor_band_sweep.py` (the full ones
are recorded in the plan section). Ground truth throughout.

In SLOW_FILES: a sweep, not a hot path.
"""
from tests import floor_band_sweep as F
from tests.footprint_sweep import T_BAR_CM


def test_b1_a_low_object_stops_every_run_and_fails_red_without_the_band():
    placements = F.placements(20, seed=7)
    on = [F.run(*p, s, band=True) for p in placements for s in F.SPEEDS]
    assert min(r["T_after_move"] for r in on) >= T_BAR_CM
    assert all(r["min_G"] >= 1.0 for r in on)
    off = [F.run(*p, s, band=False) for p in placements for s in F.SPEEDS]
    assert sum(r["min_G"] < 1.0 for r in off) > 0, "the scan alone must miss the sock"


def test_b2a_with_nothing_low_the_band_changes_no_exact_run():
    from tests.footprint_sweep import starts
    for house in ("scaled_house", "home_first_floor"):
        (x, y), = starts(house, 1, seed=3)
        for h in range(0, 360, 45):
            on = F.progress_run(house, x, y, h, 0.2, band=True)
            off = F.progress_run(house, x, y, h, 0.2, band=False)
            assert abs(on["travel"] - off["travel"]) < 1e-6, (house, h)
