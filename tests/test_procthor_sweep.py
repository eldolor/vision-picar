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
