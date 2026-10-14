"""
tests/test_survey.py

PLAN-ros-alignment.md 3.70 Part A: survey mode, in process.

* S1: a mission with no target and job `survey` runs explore's frontier loop
  and ends Exhausted -- never Found, never a target-confirmation call.
* S2: rooms are labelled during the survey (brain/room_labels.py): one label
  call on entering space no label covers, a cap per survey, and the labels
  attached to the map as `room_labels`.
"""

import pytest

from control import mission_runner as mr
from control.mission_runner import EXHAUSTED, FOUND, MissionRunner
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests.test_explore import FakeNav


def _drive(runner, max_ticks=20000):
    ticks = 0
    while runner.tick():
        ticks += 1
        assert ticks < max_ticks, "runner never terminated"
    assert runner.report_done.wait(10), "the report never landed"
    return runner.status()


def _survey(house="scaled_house", **kw):
    grid = build_world(house)
    robot = MockRobot(grid, render=False)
    nav = FakeNav(robot)
    runner = MissionRunner(robot, job="survey", policy="explore", navigator=nav,
                           world=MockWorld(grid), clock=lambda: grid.sim_time,
                           idle=robot.pass_time, **kw)
    return runner, robot, grid


# ---------- S1: a survey has no target and ends Exhausted ----------

def test_a_survey_needs_no_target_and_ends_exhausted():
    runner, _robot, _grid = _survey()
    runner.start()
    s = _drive(runner)
    assert s["outcome"] == EXHAUSTED, s["log_tail"]
    assert s["job"] == "survey"
    assert s["target_object"] is None


def test_a_survey_never_ends_found_and_never_asks_to_confirm_a_target(monkeypatch):
    calls = []
    runner, _robot, _grid = _survey()
    monkeypatch.setattr(runner, "_guarded_confirm", lambda *a, **k: calls.append(a))
    runner.agent.arrival_confirm_fn = runner._guarded_confirm
    runner.start()
    s = _drive(runner)
    assert s["outcome"] != FOUND
    assert calls == []


def test_a_survey_with_a_target_is_refused():
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    with pytest.raises(ValueError, match="survey"):
        MissionRunner(robot, job="survey", target_object="red backpack", policy="explore",
                      navigator=FakeNav(robot), world=MockWorld(grid))


@pytest.mark.parametrize("policy", ["frontier", "tiered", "vision"])
def test_a_survey_runs_only_under_explore(policy):
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    with pytest.raises(ValueError, match="explore"):
        MissionRunner(robot, job="survey", policy=policy, world=MockWorld(grid),
                      vision_fn=lambda f: {})


def test_an_unknown_job_is_refused():
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    with pytest.raises(ValueError, match="job"):
        MissionRunner(robot, job="tidy", target_object="x", world=MockWorld(grid))


def test_a_find_mission_is_still_the_default_and_still_needs_a_target():
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    with pytest.raises(ValueError, match="target"):
        MissionRunner(robot, world=MockWorld(grid))
    runner = MissionRunner(robot, target_object="x", world=MockWorld(grid))
    assert runner.status()["job"] == "find"


def test_a_survey_takes_the_survey_time_limit():
    runner, _robot, _grid = _survey(mission_time_limit_s=1800.0)
    assert runner.mission_time_limit_s == 1800.0
    assert runner.max_steps == mr.derived_max_steps(1800.0)


# ---------- S2: room labels, one ask per room, capped ----------

from brain.room_labels import RoomLabeller, UNCLEAR, frame_room, segment  # noqa: E402
from world.interface import CELL_FREE, CELL_OCCUPIED  # noqa: E402


def _two_rooms():
    """Two 3 m x 3 m rooms side by side, 0.1 m cells, joined by a 0.9 m door
    in the wall between them (x = 3.0-3.1 m)."""
    res, w, h = 0.1, 63, 32
    cells = []
    for j in range(h):
        for i in range(w):
            edge = i in (0, w - 1) or j in (0, h - 1)
            wall = i == 31 and not 12 <= j < 21
            cells.append(CELL_OCCUPIED if edge or wall else CELL_FREE)
    return {"usable": True, "map_id": "m", "map_version": 1, "resolution_m": res,
            "width": w, "height": h, "origin_x_m": 0.0, "origin_y_m": 0.0, "cells": cells}


def _pose(x, y):
    return {"usable": True, "x_m": x, "y_m": y, "heading_deg": 0.0}


def test_the_map_is_cut_into_rooms_at_the_door():
    seg, _d = segment(_two_rooms())
    assert seg[16, 15] != seg[16, 47]
    assert seg[16, 15] >= 0 and seg[16, 47] >= 0


def test_one_ask_per_room_and_none_from_the_doorway():
    asked = []
    lab = RoomLabeller(lambda f: asked.append(f) or f["room"], mode="rooms")
    m = _two_rooms()
    # In the doorway the camera may be looking at either room: no ask.
    assert lab.observe(m, _pose(3.05, 1.6), lambda: {"room": "hall"}) is None
    for x in (1.5, 1.2, 2.0):                       # three steps in the left room
        lab.observe(m, _pose(x, 1.6), lambda: {"room": "den"})
    assert len(asked) == 1
    assert lab.observe(m, _pose(4.7, 1.6), lambda: {"room": "kitchen"}) == "kitchen"
    assert lab.calls == 2
    layer = lab.layer(m)
    names = layer["names"]
    left = layer["cells"][16 * m["width"] + 15]
    right = layer["cells"][16 * m["width"] + 47]
    assert names[left] == "den" and names[right] == "kitchen"


def test_the_cap_leaves_the_rest_unlabelled_and_never_asks_past_it():
    asked = []
    lab = RoomLabeller(lambda f: asked.append(1) or "den", max_calls=1, mode="rooms")
    m = _two_rooms()
    lab.observe(m, _pose(1.5, 1.6), dict)
    assert lab.observe(m, _pose(4.7, 1.6), dict) is None
    assert len(asked) == 1 and lab.capped
    layer = lab.layer(m)
    assert layer["cells"][16 * m["width"] + 47] == -1


def test_unclear_names_nothing_and_is_not_asked_again():
    asked = []
    lab = RoomLabeller(lambda f: asked.append(1) or None, mode="rooms")
    m = _two_rooms()
    for x in (1.5, 1.2):
        lab.observe(m, _pose(x, 1.6), dict)
    assert len(asked) == 1
    assert lab.seeds[0][2] == UNCLEAR
    assert lab.layer(m)["names"] == []


def test_spacing_mode_asks_again_only_beyond_its_spacing():
    asked = []
    lab = RoomLabeller(lambda f: asked.append(1) or "den", mode="spacing", spacing_m=2.0)
    m = _two_rooms()
    lab.observe(m, _pose(0.5, 1.6), dict)
    lab.observe(m, _pose(2.0, 1.6), dict)           # 1.5 m on: covered
    assert len(asked) == 1
    lab.observe(m, _pose(2.6, 1.6), dict)           # 2.1 m on: not covered
    assert len(asked) == 2


def test_spacing_labels_stop_at_the_doorway_not_halfway_between_asks():
    # den asked 2.5 m from the door, kitchen 1 m past it: halfway by walking
    # is inside the den, but the room ends at the door.
    lab = RoomLabeller(lambda f: f["room"], mode="spacing")
    m = _two_rooms()
    lab.observe(m, _pose(0.5, 1.6), lambda: {"room": "den"})
    lab.observe(m, _pose(4.0, 1.6), lambda: {"room": "kitchen"})
    layer = lab.layer(m)
    row = [layer["names"][k] if k >= 0 else None
           for k in layer["cells"][16 * m["width"]:17 * m["width"]]]
    assert set(row[1:31]) == {"den"}, row
    assert set(row[32:62]) == {"kitchen"}, row


def _row(layer, m, j=16):
    return [layer["names"][k] if k >= 0 else None
            for k in layer["cells"][j * m["width"]:(j + 1) * m["width"]]]


def test_spacing_unclear_holds_its_room_against_a_label_through_the_door():
    lab = RoomLabeller(lambda f: f["room"], mode="spacing")
    m = _two_rooms()
    lab.observe(m, _pose(1.5, 1.6), lambda: {"room": "den"})
    lab.observe(m, _pose(4.7, 1.6), lambda: {"room": None})       # unclear
    row = _row(lab.layer(m), m)
    assert set(row[32:62]) == {None}, row


def _three_rooms():
    """`_two_rooms` with a third 3 m room on the right, through a second
    0.9 m door (x = 6.2-6.3 m)."""
    res, w, h = 0.1, 94, 32
    cells = []
    for j in range(h):
        for i in range(w):
            edge = i in (0, w - 1) or j in (0, h - 1)
            wall = i in (31, 62) and not 12 <= j < 21
            cells.append(CELL_OCCUPIED if edge or wall else CELL_FREE)
    return {"usable": True, "map_id": "m3", "map_version": 1, "resolution_m": res,
            "width": w, "height": h, "origin_x_m": 0.0, "origin_y_m": 0.0, "cells": cells}


def test_spacing_capped_leaves_rooms_two_doorways_on_unlabelled():
    lab = RoomLabeller(lambda f: "den", mode="spacing", max_calls=1)
    m = _three_rooms()
    lab.observe(m, _pose(1.5, 1.6), dict)
    row = _row(lab.layer(m), m)
    assert set(row[1:31]) == {"den"}, row
    assert set(row[63:93]) == {None}, row


def test_spacing_keeps_a_seed_whose_cell_is_no_longer_floor():
    lab = RoomLabeller(lambda f: "den", mode="spacing")
    m = _two_rooms()
    lab.observe(m, _pose(1.55, 1.65), dict)
    k = 16 * m["width"] + 15
    moved = {**m, "map_version": 2, "cells": m["cells"][:k] + [CELL_OCCUPIED] + m["cells"][k + 1:]}
    row = _row(lab.layer(moved), moved)
    assert set(row[1:15]) == {"den"}, row


def test_spacing_asks_again_through_a_doorway_even_within_its_spacing():
    asked = []
    lab = RoomLabeller(lambda f: asked.append(1) or f["room"], mode="spacing", spacing_m=2.0)
    m = _two_rooms()
    lab.observe(m, _pose(2.0, 1.6), lambda: {"room": "den"})
    lab.observe(m, _pose(3.8, 1.6), lambda: {"room": "kitchen"})   # ~1.8 m on, another room
    assert len(asked) == 2


def test_a_survey_counts_its_label_calls_and_stops_asking_at_the_cap():
    calls = []
    runner, _robot, _grid = _survey(room_label_fn=lambda f: calls.append(1) or f.get("room"),
                                    room_label_max_calls=2, room_label_mode="spacing")
    runner.start()
    s = _drive(runner)
    assert s["outcome"] == EXHAUSTED                # the cap ends labelling, not the survey
    assert len(calls) == 2
    assert s["survey"]["label_calls"] == 2 and s["survey"]["capped"] is True


def test_a_survey_names_the_room_it_starts_in():
    runner, _robot, grid = _survey()
    first_room = grid.room_at(int(grid.x), int(grid.y))
    runner.start()
    s = _drive(runner)
    assert s["survey"]["label_calls"] >= 1
    if first_room:
        assert s["survey"]["seeds"][0][2] == first_room


# ---------- review fixes (code-review + bug pass) ----------

def test_a_label_call_that_raises_skips_the_label_and_never_wedges_the_survey():
    def boom(frame):
        raise RuntimeError("cloud down")
    runner, _robot, _grid = _survey(room_label_fn=boom)
    runner.start()
    s = _drive(runner)
    assert s["outcome"] == EXHAUSTED and not s["running"]


def test_a_frame_read_that_raises_skips_the_label(monkeypatch):
    runner, _robot, _grid = _survey()
    def no_frame():
        raise ConnectionError("GET /frame failed")
    monkeypatch.setattr(runner, "_label_frame", no_frame)
    runner.start()
    s = _drive(runner)
    assert s["outcome"] == EXHAUSTED and s["survey"]["label_calls"] == 0


def test_a_frame_with_no_room_and_no_label_source_spends_no_call():
    """On the car no label source is wired yet: an ask must not burn the cap
    on an `unclear` it never asked anyone for."""
    lab = RoomLabeller(frame_room, max_calls=3, mode="rooms")
    m = _two_rooms()
    assert lab.observe(m, _pose(1.5, 1.6), lambda: {"image": "..."}) is None
    assert lab.calls == 0 and lab.seeds == []


def test_no_ask_once_the_mission_has_stopped():
    asked = []
    lab = RoomLabeller(lambda f: asked.append(1) or "den", mode="rooms")
    m = _two_rooms()
    assert lab.observe(m, _pose(1.5, 1.6), lambda: None) is None   # look() refused
    assert asked == [] and lab.calls == 0


def test_the_cap_is_checked_before_any_segmentation(monkeypatch):
    import brain.room_labels as rl
    lab = RoomLabeller(lambda f: "den", max_calls=1, mode="rooms")
    m = _two_rooms()
    lab.observe(m, _pose(1.5, 1.6), dict)
    monkeypatch.setattr(rl, "segment", lambda *a, **k: (_ for _ in ()).throw(AssertionError))
    lab._cache = (None, None, None, None)
    lab.observe({**m, "map_version": 2}, _pose(4.7, 1.6), dict)
    assert lab.capped


def test_segmentation_is_rate_limited_on_the_labeller_s_clock(monkeypatch):
    import brain.room_labels as rl
    runs = []
    real = rl.segment
    monkeypatch.setattr(rl, "segment", lambda *a, **k: runs.append(1) or real(*a, **k))
    t = [0.0]
    lab = RoomLabeller(lambda f: "den", clock=lambda: t[0], mode="rooms")
    m = _two_rooms()
    for v in range(1, 6):                           # five new map versions in 1 s
        t[0] += 0.2
        lab.observe({**m, "map_version": v}, _pose(1.5, 1.6), dict)
    assert len(runs) == 1
    t[0] += rl.RESEGMENT_S
    lab.observe({**m, "map_version": 9}, _pose(1.5, 1.6), dict)
    assert len(runs) == 2


def test_the_survey_s_labels_are_a_map_layer():
    runner, _robot, _grid = _survey()
    runner.start()
    _drive(runner)
    layer = runner.room_labels()
    assert layer is not None and layer["names"]
    assert len(layer["cells"]) == runner.world.get_map()["width"] * runner.world.get_map()["height"]


def test_a_boolean_survey_time_limit_is_refused_at_load(tmp_path):
    from control.brain_config import load_brain_config
    p = tmp_path / "robot.yaml"
    p.write_text("brain:\n  survey_time_limit_s: true\n")
    with pytest.raises(ValueError, match="survey_time_limit_s"):
        load_brain_config(str(p))


# ---------- second review pass ----------

def _grown(m, extra=10):
    """The same rooms on a map grown `extra` cells to the west and south, its
    origin moved to match -- what slam_toolbox's map does as it explores."""
    w, h = m["width"] + extra, m["height"] + extra
    cells = [-1] * (w * h)
    for j in range(m["height"]):
        for i in range(m["width"]):
            cells[(j + extra) * w + i + extra] = m["cells"][j * m["width"] + i]
    res = m["resolution_m"]
    return {**m, "width": w, "height": h, "cells": cells, "map_version": m["map_version"] + 1,
            "origin_x_m": m["origin_x_m"] - extra * res, "origin_y_m": m["origin_y_m"] - extra * res}


def test_a_growing_map_is_not_recut_on_every_tick(monkeypatch):
    import brain.room_labels as rl
    runs, real = [], rl.segment
    monkeypatch.setattr(rl, "segment", lambda *a, **k: runs.append(1) or real(*a, **k))
    t = [0.0]
    lab = RoomLabeller(lambda f: "den", clock=lambda: t[0], mode="rooms")
    m = _two_rooms()
    for _ in range(4):
        t[0] += 0.5
        m = _grown(m, 2)
        lab.observe(m, _pose(1.5, 1.6), dict)
    assert len(runs) == 1


def test_a_grown_map_still_finds_the_robot_s_room_by_position():
    t = [0.0]
    lab = RoomLabeller(lambda f: "den", clock=lambda: t[0], mode="rooms")
    m = _two_rooms()
    lab.observe(m, _pose(1.5, 1.6), dict)
    t[0] += 1.0
    g = _grown(m, 10)
    assert lab.observe(g, _pose(1.2, 1.6), dict) is None        # same room: asked already
    assert lab.calls == 1


def test_spacing_never_asks_on_floor_its_cached_distances_have_not_seen():
    t = [0.0]
    asked = []
    lab = RoomLabeller(lambda f: asked.append(1) or "den", mode="spacing", clock=lambda: t[0])
    m = _two_rooms()
    half = {**m, "cells": [c if (k % m["width"]) < 15 else -1 for k, c in enumerate(m["cells"])]}
    lab.observe(half, _pose(0.5, 1.6), dict)
    t[0] += 0.2
    lab.observe(half, _pose(0.7, 1.6), dict)        # distances cached on the half map
    t[0] += 0.5                                     # inside RESEGMENT_S: the cache stands
    lab.observe({**m, "map_version": 2}, _pose(1.8, 1.6), dict)   # 1.3 m on, newly mapped
    assert len(asked) == 1


def test_no_label_source_is_remembered_and_no_frame_is_fetched_again():
    looks = []
    lab = RoomLabeller(frame_room, mode="rooms")
    m = _two_rooms()
    for x in (1.5, 1.2, 4.7):
        lab.observe(m, _pose(x, 1.6), lambda: looks.append(1) or {"image": "..."})
    assert len(looks) == 1 and lab.calls == 0


def test_a_frame_off_the_rooms_spends_no_call_and_waits_before_asking_again():
    t = [0.0]
    looks = []
    lab = RoomLabeller(frame_room, clock=lambda: t[0], mode="rooms")
    m = _two_rooms()
    lab.observe(m, _pose(1.5, 1.6), lambda: looks.append(1) or {"room": "unknown"})
    lab.observe(m, _pose(1.5, 1.6), lambda: looks.append(1) or {"room": "unknown"})
    assert lab.calls == 0 and len(looks) == 1
    t[0] += 60
    assert lab.observe(m, _pose(1.5, 1.6), lambda: {"room": "den"}) == "den"


def test_a_failing_label_source_backs_off_and_does_not_burn_the_cap():
    t = [0.0]
    def down(frame):
        raise ConnectionError("cloud down")
    lab = RoomLabeller(down, max_calls=20, clock=lambda: t[0], mode="rooms")
    m = _two_rooms()
    for _ in range(50):                             # 50 ticks, 0.2 s apart
        t[0] += 0.2
        try:
            lab.observe(m, _pose(1.5, 1.6), dict)
        except ConnectionError:
            pass
    assert lab.calls <= 2


def test_the_map_layer_is_cut_from_the_map_it_is_given(monkeypatch):
    import brain.room_labels as rl
    t = [0.0]
    lab = RoomLabeller(lambda f: "den", clock=lambda: t[0], mode="rooms")
    m = _two_rooms()
    lab.observe(m, _pose(1.5, 1.6), dict)
    cut = []
    real = rl.segment
    monkeypatch.setattr(rl, "segment", lambda mm, *a, **k: cut.append(mm["map_version"])
                        or real(mm, *a, **k))
    lab.layer({**m, "map_version": 2})              # newer, same size, inside RESEGMENT_S
    assert cut == [2]


def test_capped_is_set_on_the_ask_that_reaches_the_cap():
    lab = RoomLabeller(lambda f: "den", max_calls=1, mode="rooms")
    lab.observe(_two_rooms(), _pose(1.5, 1.6), dict)
    assert lab.capped


def test_a_stop_after_the_frame_never_reaches_the_label_source():
    asked = []
    alive = [True]
    lab = RoomLabeller(lambda f: asked.append(1) or "den", mode="rooms")
    def look():
        alive[0] = False                            # the stop lands while the frame is read
        return {"room": "den"}
    assert lab.observe(_two_rooms(), _pose(1.5, 1.6), look, alive=lambda: alive[0]) is None
    assert asked == [] and lab.calls == 0


def test_spacing_does_not_ask_again_beside_a_seed_whose_cell_turned_occupied():
    t = [0.0]
    asked = []
    lab = RoomLabeller(lambda f: asked.append(1) or "den", mode="spacing", clock=lambda: t[0])
    m = _two_rooms()
    lab.observe(m, _pose(1.55, 1.65), dict)
    k = 16 * m["width"] + 15
    t[0] += 10.0
    moved = {**m, "map_version": 2, "cells": m["cells"][:k] + [CELL_OCCUPIED] + m["cells"][k + 1:]}
    lab.observe(moved, _pose(1.65, 1.65), dict)
    assert len(asked) == 1
