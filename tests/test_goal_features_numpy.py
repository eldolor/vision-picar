"""3.109 criterion 1 (fast tier): the numpy `features()` gives exactly the
loop's answers -- on one live SLAM map (both seen sets) and at the edges the
loop clips: a goal on the map's border, a goal off the map, seen cells off
the map. The six maps and real missions are `test_goal_features_sweep.py`."""
import random

from brain.goal_policy import features, goal_context
from tests import frontier_compare as fc
from tests import goal_features_compare as gc
from tests import goal_features_reference as ref
from world.interface import CELL_FREE, CELL_OCCUPIED, CELL_UNKNOWN


def test_one_live_map_gives_the_old_answers():
    path = fc.MAPS[0]
    rng = random.Random(f"3109:{path.name}")
    m = fc.load(path)
    seens = (set(), fc.seen_set(m, 8, rng))
    bad, n = [], 0
    for x, y in fc.poses(m, 4, rng):
        pose = {"x_m": x, "y_m": y, "heading_deg": rng.uniform(0, 360)}
        for seen in seens:
            goals = gc.candidates(m, x, y, seen)
            n += len(goals)
            bad += gc.differences(m, pose, goals, seen)
    assert n > 50
    assert bad == []


def _small_map():
    w, h = 30, 24
    rng = random.Random(3109)
    cells = [rng.choice((CELL_FREE, CELL_FREE, CELL_UNKNOWN, CELL_OCCUPIED)) for _ in range(w * h)]
    return {"width": w, "height": h, "resolution_m": 0.1, "origin_x_m": -1.0,
            "origin_y_m": 0.5, "cells": cells}


def test_the_edges_give_the_old_answers():
    m = _small_map()
    w, h, res = m["width"], m["height"], m["resolution_m"]
    ox, oy = m["origin_x_m"], m["origin_y_m"]
    goals = [(ox + 0.05, oy + 0.05), (ox + w * res - 0.05, oy + h * res - 0.05),
             (ox + 1.5, oy + 1.2), (ox - 0.7, oy + 1.0), (ox + 50.0, oy - 50.0)]
    seen = {(0, 0), (5, 5), (29, 23), (-3, 4), (40, 2), (7, 100)}
    pose = {"x_m": ox + 1.0, "y_m": oy + 1.0, "heading_deg": 33.0}
    for s in (set(), seen):
        ctx = goal_context(m, s)
        for g in goals:
            for kind in ("frontier", "view"):
                want = ref.features(m, pose, g, kind=kind, path_m=2.0, size_m=1.0,
                                    area_m2=0.5, seen_cells=s)
                for got in (features(m, pose, g, kind=kind, path_m=2.0, size_m=1.0,
                                     area_m2=0.5, seen_cells=s, ctx=ctx),
                            features(m, pose, g, kind=kind, path_m=2.0, size_m=1.0,
                                     area_m2=0.5, seen_cells=s)):
                    assert got == want
                    assert [type(v) for v in got] == [type(v) for v in want]


def test_a_context_from_another_map_or_seen_set_is_refused():
    import pytest
    m = _small_map()
    pose = {"x_m": 0.0, "y_m": 1.0, "heading_deg": 0.0}
    seen = {(1, 1)}
    ctx = goal_context(m, seen)
    other = {**m, "cells": list(m["cells"])}
    with pytest.raises(ValueError):
        features(other, pose, (0.0, 1.0), kind="frontier", path_m=1.0, seen_cells=seen, ctx=ctx)
    with pytest.raises(ValueError):
        features(m, pose, (0.0, 1.0), kind="frontier", path_m=1.0, seen_cells={(2, 2)}, ctx=ctx)
    with pytest.raises(ValueError):
        goal_context(m, {(1, 2, 3)})
