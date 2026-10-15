"""
tests/test_procthor_sweep.py -- PLAN-ros-alignment.md 3.66 criterion 5:
`explore` over the first 100 usable ProcTHOR houses (tests/procthor_sweep.py).
On SLOW_FILES: it runs a hundred missions.
"""
from control.mission_runner import FOUND
from tests.procthor_sweep import sweep


def test_explore_ends_cleanly_and_finds_in_new_houses():
    rows = sweep(100)
    assert len(rows) == 100
    bad = [r for r in rows if r["outcome"] in ("exception", "running") or r.get("capped")]
    assert not bad, bad                                            # 5a: 100%
    # 5b's bar was 90 and FAILED at 84 (2026-10-10; the 16 misses are
    # classified in 3.66's entry). 3.69 fixed 15 of them: 99, the miss left
    # is house 27 (a passage with nowhere to stop; recorded in 3.69).
    assert sum(r["outcome"] == FOUND for r in rows) >= 99


def test_a_sweep_can_start_past_the_first_houses(monkeypatch):
    """3.70 M5 runs usable houses 200-299: `start` indexes the usable list."""
    import tests.procthor_sweep as ps
    monkeypatch.setattr(ps, "usable", lambda: list(range(1000, 1400)))
    monkeypatch.setattr(ps, "run_house", lambda i: {"index": i})
    assert [r["index"] for r in ps.sweep(3, start=200)] == [1200, 1201, 1202]


def test_an_unsurveyed_house_still_works_on_unseen_houses():
    """3.70 M5: with no survey and no saved map, explore over usable houses
    200-299 (never tuned on) still meets 3.66's bar of 90. Measured
    2026-10-14: 98 found, 100 ended cleanly; the misses are 378 (3.69's,
    undiagnosed) and 471, both `exhausted`."""
    rows = sweep(100, start=200)
    bad = [r for r in rows if r["outcome"] in ("exception", "running") or r.get("capped")]
    assert not bad, bad
    assert sum(r["outcome"] == FOUND for r in rows) >= 98
