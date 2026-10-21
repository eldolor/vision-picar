"""
tests/test_goal_tap_oracle.py -- 3.88 B's oracle and cloning
(`evaluations/goal-tap-388/`): the oracle's cost is the true floor path to
the target, ties keep the pool's order, and the cloning fit recovers a
ranking it was shown.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "evaluations", "goal-tap-388"))
from b import agreement, fit                                  # noqa: E402
from oracle import INF, cost_at, distance_field, oracle_pick, run  # noqa: E402
from sim.grid_world import GridWorld                         # noqa: E402
from sim.maps.procthor import TARGET, usable                 # noqa: E402
from tests.test_explore import CELL                          # noqa: E402

LAYOUT = [
    "#######",
    "#.....#",
    "#.###.#",
    "#.....#",
    "#######",
]


def _grid():
    return GridWorld(layout=LAYOUT, rooms={"room": set()}, objects={(5, 3): TARGET})


def test_the_cost_is_the_floor_path_around_walls_not_the_straight_line():
    d = distance_field(_grid())
    assert d[(5, 3)] == 0 and d[(5, 1)] == 2
    # (3, 1) is 2 cells from the target in a straight line, 4 round the wall.
    assert d[(3, 1)] == 4
    assert cost_at(d, 3.5 * CELL, 1.5 * CELL) == 4
    # A wall cell takes its cheapest open neighbour's (8 around), plus one:
    # (4, 3), one from the target.
    assert cost_at(d, 3.5 * CELL, 2.5 * CELL) == 2
    assert cost_at(d, 50 * CELL, 50 * CELL) == INF


def test_ties_and_an_unreachable_pool_keep_the_rules_order():
    assert oracle_pick([3.0, 1.0, 1.0]) == 1
    assert oracle_pick([INF, INF]) == 0
    assert oracle_pick([2.0, INF, 2.0]) == 0


def test_the_fit_recovers_a_ranking_it_was_shown():
    rng = np.random.default_rng(0)
    true = np.array([-3.0, 0, 0, 2.0, 0, 1.0, 0, 0, 0, 0])
    choices = []
    for _ in range(300):
        f = rng.random((int(rng.integers(2, 8)), 10))
        costs = list(-(f @ true))
        choices.append({"feats": f.tolist(), "costs": costs,
                        "oracle": int(np.argmax(f @ true)), "pick": 0})
    w = fit(choices, steps=1500)
    assert agreement(choices, w)["strict"] > 0.9
    assert agreement(choices, [0.0] * 10)["strict"] < 0.5


def test_an_oracle_mission_records_its_choices_and_finds():
    row = run(usable()[300])
    assert row["outcome"] == "found"
    for c in row["choices"]:
        assert len(c["feats"]) == len(c["costs"]) >= 2
        assert c["pick"] == c["oracle"]
