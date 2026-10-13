"""
tests/test_survey_sweep.py -- 3.70 S1/S2's sim numbers, pinned (about 80 s
per mode; in tools/plan_section.py's SLOW_FILES).

The 9 surveys of `tests/survey_sweep.py` (3 spread starts in each named
house, FakeNav for nav2). Recorded 2026-10-13 in 3.70's "Records: S1-S3":
* every survey ends `exhausted` with coverage >= 95% (S1, met);
* `rooms`: label calls median 10, max 19 -- never past the cap of 20;
  rooms >= 95% labelled 22 of 90 (S2 FAILED; pinned as no regression);
* `spacing`: calls median 28, max 40; rooms >= 95% labelled 36 of 90.
"""

import statistics

import pytest

from tests import survey_sweep as sw


@pytest.fixture(scope="module", params=["rooms", "spacing"])
def rows(request):
    return request.param, [sw.run(h, st, mode=request.param)
                           for h in sw.HOUSES for st in sw.starts(h)]


def test_every_sim_survey_ends_exhausted_with_the_house_mapped(rows):
    _mode, rs = rows
    assert [r["outcome"] for r in rs] == ["exhausted"] * 9, rs
    assert all(r["coverage"] >= 0.95 for r in rs), [r["coverage"] for r in rs]


def test_label_calls_and_where_labels_land_do_not_regress(rows):
    mode, rs = rows
    calls = [r["label_calls"] for r in rs]
    good = sum(v >= 0.95 for r in rs for v in r["per_room"].values())
    if mode == "rooms":
        assert statistics.median(calls) <= 10 and max(calls) <= 20, calls
        assert good >= 22, good
    else:
        assert statistics.median(calls) <= 28 and max(calls) <= 40, calls
        assert good >= 36, good
