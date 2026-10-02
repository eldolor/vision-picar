"""
tests/test_frontier.py

PLAN-ros-alignment.md 3.31 -- the pure half of the explore policy:
frontiers on an occupancy grid, which of them the chassis can reach, and
the retry rule the explore policy and the stuck detector share.
"""

import pytest

from brain.frontier import (RetryBook, find_frontiers, frontier_cells, known_near)
from world.interface import CELL_FREE as F, CELL_OCCUPIED as O, CELL_UNKNOWN as U


def grid(rows, res=0.05):
    """A map from strings: '.' free, '#' occupied, '?' unknown."""
    code = {".": F, "#": O, "?": U}
    cells = [code[ch] for row in rows for ch in row]
    return {"usable": True, "map_id": "t", "map_version": 1, "resolution_m": res,
            "width": len(rows[0]), "height": len(rows), "origin_x_m": 0.0,
            "origin_y_m": 0.0, "cells": cells}


def centre(c, res=0.05):
    return (c + 0.5) * res


# A room 30 x 20 cells (1.5 m x 1.0 m) at 5 cm, its east wall broken by a
# 12-cell (0.6 m) doorway into unknown.
ROOM = (["#" * 32]
        + ["#" + "." * 30 + "#"] * 4
        + ["#" + "." * 30 + "?"] * 12
        + ["#" + "." * 30 + "#"] * 4
        + ["#" * 32])


def test_a_frontier_is_seen_floor_next_to_unknown():
    m = grid(ROOM)
    f = frontier_cells(m)
    assert f == {(30, y) for y in range(5, 17)}


def test_the_doorway_is_offered_with_a_goal_on_known_floor():
    m = grid(ROOM)
    found = find_frontiers(m, centre(3), centre(10))
    assert len(found) == 1
    gx, gy = found[0].goal
    col, row = int(gx / 0.05), int(gy / 0.05)
    assert m["cells"][row * m["width"] + col] == F
    assert found[0].size_m == pytest.approx(12 * 0.05)
    assert found[0].distance_m > 1.0


def test_a_crack_narrower_than_the_robot_is_not_a_frontier():
    rows = list(ROOM)
    for y in range(5, 17):
        rows[y] = rows[y][:-1] + ("?" if y in (10, 11) else "#")
    assert find_frontiers(grid(rows), centre(3), centre(10)) == []


def test_a_frontier_the_chassis_cannot_reach_is_not_offered():
    # A wall across the room with a 3-cell (15 cm) slot: the doorway beyond
    # is known floor away, but no 22 cm-clear path leads to it.
    rows = [list(r) for r in ROOM]
    for y in range(1, 21):
        rows[y][15] = "." if y in (9, 10, 11) else "#"
    m = grid(["".join(r) for r in rows])
    assert find_frontiers(m, centre(3), centre(10)) == []


def test_nearer_and_larger_frontiers_rank_first():
    rows = [list(r) for r in ROOM]
    for y in range(1, 8):      # a second opening (0.35 m) in the west wall
        rows[y][0] = "?"
    m = grid(["".join(r) for r in rows])
    near_west = find_frontiers(m, centre(4), centre(4))
    assert len(near_west) == 2
    assert near_west[0].score <= near_west[1].score


def test_an_unusable_map_has_no_frontiers():
    assert find_frontiers({"usable": False}, 0.0, 0.0) == []


def test_known_near_counts_seen_cells():
    m = grid(ROOM)
    assert known_near(m, centre(30), centre(10), 0.1) > 0
    assert known_near({"usable": False}, 0, 0, 1.0) == 0


# ---------- the retry rule ----------


def test_a_failure_cools_down_then_comes_back():
    book = RetryBook(cooldown_s=30, limit=3)
    assert book.available(1.0, 1.0, now=0)
    book.fail(1.0, 1.0, now=0)
    assert not book.available(1.0, 1.0, now=10)
    assert book.available(1.0, 1.0, now=31)
    assert book.cooling(now=10)
    # past its cooldown it is offered again, not waited on
    assert not book.cooling(now=31)


def test_a_failure_covers_its_neighbourhood_not_the_house():
    book = RetryBook(radius_m=0.6)
    book.fail(1.0, 1.0, now=0)
    assert not book.available(1.3, 1.0, now=1)
    assert book.available(3.0, 1.0, now=1)


def test_dropped_after_the_limit_and_no_longer_cooling():
    book = RetryBook(cooldown_s=30, limit=3)
    for t in (0, 40, 80):
        book.fail(1.0, 1.0, now=t)
    assert book.dropped(1.0, 1.0)
    assert not book.available(1.0, 1.0, now=1000)
    assert not book.cooling(now=1000)


def test_the_map_growing_near_a_failure_brings_it_back_early():
    book = RetryBook(cooldown_s=30, early_growth=0.25)
    book.fail(1.0, 1.0, now=0, known=100)
    assert not book.available(1.0, 1.0, now=5, known_now=lambda x, y: 110)
    assert book.available(1.0, 1.0, now=5, known_now=lambda x, y: 130)


def test_next_ready_in_counts_down():
    book = RetryBook(cooldown_s=30)
    book.fail(0, 0, now=0)
    assert book.next_ready_in(now=12) == pytest.approx(18)
