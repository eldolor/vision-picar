"""
tests/test_procthor_houses.py -- sim/maps/procthor.py, ProcTHOR floor plans
as simulator houses (PLAN-ros-alignment.md 3.66). Pins the conversion's
criteria (1-4, 6) on a fixed sample; the 1,000-house numbers are in the
plan entry, measured by evaluations/procthor-366/measure.py. The sweep
(criterion 5) is tests/test_procthor_sweep.py, on SLOW_FILES.
"""
import statistics
import time
from collections import deque

import pytest

from sim.maps import build_world
from sim.maps.procthor import TARGET, convert, houses, object_name, parse_name, usable

SAMPLE = range(100)


@pytest.fixture(scope="module")
def sample():
    return [convert(i) for i in SAMPLE]


def test_it_is_registered_by_name():
    assert build_world("procthor:val:7").map_name == "procthor:val:7"
    assert parse_name("procthor:val:12") == 12
    assert parse_name("procthor:train:12") is None
    with pytest.raises(ValueError, match="procthor:val"):
        build_world("procthor:other:1")


def test_all_val_houses_are_committed():
    assert len(houses()) == 1000


def test_the_test_split_is_its_own_set_of_houses():
    """3.65 amendment 3: `procthor:test:<i>` is the `test` split, kept out of
    every tuning run; `parse_name` still means `val` only."""
    from sim.maps.procthor import parse_split_name
    assert parse_split_name("procthor:test:7") == ("test", 7)
    assert parse_split_name("procthor:val:7") == ("val", 7)
    assert parse_name("procthor:test:7") is None
    assert build_world("procthor:test:7").map_name == "procthor:test:7"
    assert len(houses("test")) == 1000 and houses("test") is not houses()
    assert len(usable("test")) == 613
    a, b = build_world("procthor:test:7"), build_world("procthor:val:7")
    assert a.layout != b.layout


def test_the_train_split_is_its_own_set_of_houses():
    """3.88 B: `procthor:train:<i>`, ProcTHOR's `train` split; its first
    1,000 gave 615 usable (B's fresh test houses). 3.96 amendment 2 imports
    all 10,000 (6,097 usable), and the first 1,000 convert exactly as before,
    so 3.88's houses are the same houses."""
    from sim.maps.procthor import parse_split_name
    assert parse_split_name("procthor:train:7") == ("train", 7)
    assert build_world("procthor:train:7").map_name == "procthor:train:7"
    assert len(houses("train")) == 10000
    u = usable("train")
    assert len(u) == 6097
    assert len([h for h in u if h < 1000]) == 615
    assert len([h for h in u if 1000 <= h < 2000]) == 588   # 3.96's test houses
    a, b, c = (build_world(f"procthor:{s}:7") for s in ("train", "test", "val"))
    assert a.layout != b.layout and a.layout != c.layout


def test_the_outside_is_closed(sample):
    for cv in sample:
        layout = cv.world.layout
        assert set(layout[0]) == {"#"} and set(layout[-1]) == {"#"}
        assert all(row[0] == "#" and row[-1] == "#" for row in layout)


def test_topology_is_kept(sample):
    """Criterion 1 on the sample: no door lost, no wall leaking."""
    kept = sum(cv.grid_joins() == cv.joins for cv in sample)
    assert kept / len(sample) >= 0.95


def test_the_outline_is_kept(sample):
    """Criterion 2 on the sample."""
    ok = sum(abs(cv.outline_area_m2 / cv.source_area_m2 - 1) <= 0.05 for cv in sample)
    assert ok / len(sample) >= 0.95


def test_the_furniture_is_kept(sample):
    """Criterion 3 on the sample."""
    assert (sum(cv.placed_objects for cv in sample)
            / sum(cv.floor_objects for cv in sample)) >= 0.98


def test_the_target_is_in_another_room_and_reachable(sample):
    for cv in sample:
        w = cv.world
        where = [c for c, v in w.objects.items() if v == TARGET]
        assert len(where) == 1, cv.index
        start = (w.robot_x, w.robot_y)
        rooms = [n for n, cells in w.rooms.items() if start in cells]
        if len(w.rooms) > 1 and rooms:
            assert where[0] not in w.rooms[rooms[0]], cv.index
        free = {(i, j) for j, row in enumerate(w.layout) for i, c in enumerate(row)
                if c == "." and (i, j) not in w.objects} | set(where)
        seen, q = {start}, deque([start])
        while q:
            i, j = q.popleft()
            for n in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
                if n in free and n not in seen:
                    seen.add(n)
                    q.append(n)
        assert where[0] in seen, cv.index


def test_a_house_builds_the_same_twice():
    a, b = build_world("procthor:val:42"), build_world("procthor:val:42")
    assert a.layout == b.layout and a.objects == b.objects and a.rooms == b.rooms
    assert (a.x, a.y, a.theta) == (b.x, b.y, b.theta)


def test_a_house_builds_quickly():
    """Criterion 6: < 0.5 s median."""
    times = []
    for i in range(0, 1000, 100):
        t = time.perf_counter()
        convert(i)
        times.append(time.perf_counter() - t)
    assert statistics.median(times) < 0.5


def test_usable_houses_are_usable():
    """Criterion 4's rule, re-checked on the instrument for a few of the
    pinned list, so the list cannot drift from the converter."""
    pytest.importorskip("scipy")
    from tests.chassis_fit import SAFETY_MARGIN_M, UGV_ROVER, fit
    pinned = usable()
    assert len(pinned) >= 100 and pinned == sorted(pinned)
    for index in pinned[:3]:
        got = fit(f"procthor:val:{index}", *UGV_ROVER, SAFETY_MARGIN_M)
        assert got["start_free"]
        assert not [n for n, r in got["rooms"].items() if not r["reached"]]
        assert len(got["rooms"]) >= 3


@pytest.mark.parametrize("kind,name", [("DiningTable", "dining table"), ("TVStand", "tv stand"),
                                       ("BasketBall", "basketball"), ("LivingRoom", "living room"),
                                       ("Fridge", "fridge")])
def test_object_names(kind, name):
    assert object_name(kind) == name


def test_the_importer_refuses_a_source_it_did_not_pin(tmp_path, monkeypatch):
    from tools import procthor_import
    url, _ = procthor_import.SOURCES["assets"]
    (tmp_path / url.rsplit("/", 1)[1]).write_bytes(b"{}")
    with pytest.raises(ValueError, match="sha256"):
        procthor_import.fetch("assets", str(tmp_path))


def test_an_index_past_the_split_is_a_value_error():
    with pytest.raises(ValueError, match="procthor:val"):
        build_world("procthor:val:1000")


def test_movers_in_a_procthor_house_are_refused_by_name(monkeypatch):
    from sim.maps import build_movers
    with pytest.raises(ValueError, match="this house has none"):
        build_movers("procthor:val:7", "hallway_crossing")


def test_a_bad_download_is_not_kept(tmp_path, monkeypatch):
    """A truncated fetch must not poison the cache: the next run fetches again."""
    import io
    from tools import procthor_import
    monkeypatch.setattr(procthor_import.urllib.request, "urlopen",
                        lambda url, timeout: io.BytesIO(b"truncated"))
    with pytest.raises(ValueError, match="sha256"):
        procthor_import.fetch("assets", str(tmp_path))
    assert list(tmp_path.iterdir()) == []
