"""
tests/test_floor_hazards.py -- PLAN-ros-alignment.md 3.74 part A: the
simulator's objects can be lower than the lidar's plane.

Criterion A2: a 5 cm object in front of the robot is absent from the scan
and present in the depth camera's floor band at its true range, +/- one
cell, from the band's minimum range out to 1.5 m. A1 (a house with no
heights senses exactly what it did on `dev`) is `tests/test_sim_fingerprint.py`.
"""
import pytest

from robot import interface
from sim.grid_world import LIDAR_Z_M, ROBOT_HALF_CELL, GridWorld, Heading, LowObjectDetector
from sim.mock_robot import (DEFAULT_CELL_CM, FLOOR_BAND_FOV_DEG, FLOOR_BAND_MAX_CM,
                            FLOOR_BAND_MIN_CM, FLOOR_BAND_ORIGIN_X_CM, MockRobot)
from sim.mock_world import CELL_OCCUPIED, MockWorld

CORRIDOR = ["#" * 14, "#" + "." * 12 + "#", "#" + "." * 12 + "#", "#" + "." * 12 + "#", "#" * 14]


def _robot(gap_cells=None, label="sock", heights=None):
    """The robot at (1, 2) facing east down a corridor; an object `gap_cells`
    free cells ahead of it (None: no object)."""
    objects = {} if gap_cells is None else {(2 + gap_cells, 2): label}
    world = GridWorld(CORRIDOR, {"hall": set()}, objects=objects, robot_x=1, robot_y=2,
                      heading=Heading.E,
                      object_heights={"sock": 0.05} if heights is None else heights)
    return MockRobot(world, render=False)


def _centre(zones):
    return zones[len(zones) // 2 - 1: len(zones) // 2 + 1]


def _true_clearance_cm(gap_cells):
    """Robot centre (1.5) to the object's near face (2 + gap): the level
    row's measure, from the robot's cell edge."""
    return (2 + gap_cells - 1.5 - ROBOT_HALF_CELL) * DEFAULT_CELL_CM


def _true_band_range_cm(gap_cells):
    """The lens (`FLOOR_BAND_ORIGIN_X_CM` ahead of the centre) to the
    object's near face: what the floor band measures."""
    return (2 + gap_cells - 1.5) * DEFAULT_CELL_CM - FLOOR_BAND_ORIGIN_X_CM


def test_the_lidar_plane_is_the_xacros():
    assert LIDAR_Z_M == pytest.approx(0.120)


@pytest.mark.parametrize("gap_cells", [0, 1, 2, 3, 4, 5])
def test_a_low_object_is_under_the_scan_and_in_the_floor_band(gap_cells):
    robot = _robot(gap_cells)
    wall = _robot(None)
    # Under every beam: the scan is the empty corridor's, to the beam.
    assert robot.get_scan()["ranges_m"] == wall.get_scan()["ranges_m"]
    assert robot.get_scan(max_range_m=0.6)["ranges_m"] == wall.get_scan(max_range_m=0.6)["ranges_m"]
    # ...and the camera's level row and scalar do not range it either.
    assert robot.get_depth_grid()["zones"] == wall.get_depth_grid()["zones"]
    assert robot.get_distance() == wall.get_distance()

    truth = _true_band_range_cm(gap_cells)
    floor = robot.get_depth_grid()["floor"]
    for zone in _centre(floor["zones"]):
        if truth < FLOOR_BAND_MIN_CM:
            # Inside the stereo's MinZ: unmeasurable, never clear (M3).
            assert zone["status"] == interface.ZONE_UNUSABLE
        elif truth <= FLOOR_BAND_MAX_CM:
            assert zone["status"] == interface.ZONE_RANGE
            assert zone["distance_cm"] == pytest.approx(truth, abs=DEFAULT_CELL_CM)
        else:
            assert zone["status"] == interface.ZONE_NO_TARGET


def test_the_band_covers_minz_to_one_and_a_half_metres():
    """A2's range end to end: some gap is first measurable at or just past
    MinZ, and every gap out to 1.5 m reads in range."""
    measured = []
    for gap in range(0, 6):
        zone = _centre(_robot(gap).get_depth_grid()["floor"]["zones"])[0]
        if zone["status"] == interface.ZONE_RANGE:
            measured.append(_true_band_range_cm(gap))
    assert measured and min(measured) < FLOOR_BAND_MIN_CM + DEFAULT_CELL_CM
    assert max(measured) >= FLOOR_BAND_MAX_CM - DEFAULT_CELL_CM


def test_without_the_object_the_band_reads_the_far_wall_or_nothing():
    floor = _robot(None).get_depth_grid()["floor"]
    assert floor["fov_deg"] == FLOOR_BAND_FOV_DEG and floor["pan_deg"] == 0.0
    # The far wall is 3.3 m off: past the band's 1.5 m.
    assert all(z["status"] == interface.ZONE_NO_TARGET for z in _centre(floor["zones"]))


def test_the_band_is_body_fixed_and_the_level_row_pans():
    robot = _robot(2)
    robot.look_left()
    grid = robot.get_depth_grid()
    assert grid["pan_deg"] != 0.0
    assert grid["floor"]["pan_deg"] == 0.0
    assert _centre(grid["floor"]["zones"])[0]["status"] == interface.ZONE_RANGE


def test_a_tall_object_is_seen_by_everything():
    robot = _robot(2, label="chair")
    assert robot.get_scan()["ranges_m"] != _robot(None).get_scan()["ranges_m"]
    assert _centre(robot.get_depth_grid()["zones"])[0]["distance_cm"] == pytest.approx(
        _true_clearance_cm(2), abs=DEFAULT_CELL_CM)


def test_a_low_object_is_still_solid_to_the_body():
    """Driving onto a sock is a contact on ground truth, not a pass."""
    robot = _robot(1)
    robot.drive_forward(speed=100, duration=3.0)
    assert robot.world.x < 2 + 1  # stopped short of the sock's cell
    assert any("BLOCKED" in line for line in robot.world.log)


def test_the_map_never_shows_a_low_object():
    robot = _robot(2)
    world = MockWorld(robot.world)
    world.observe()
    m = world.get_map()
    assert m["cells"][2 * m["width"] + 4] != CELL_OCCUPIED


def test_heights_go_by_label_so_a_moved_object_stays_low():
    robot = _robot(2)
    robot.world.move_object((4, 2), (6, 2))
    assert robot.world.low_cells == {(6, 2)}


def test_the_camera_still_detects_a_low_object():
    labels = [d["label"] for d in _robot(3).world.frame_description()["detections"]]
    assert labels == ["sock"]


def test_the_stand_in_detector_misses_and_invents():
    robot = _robot(3)
    robot.world.low_detector = LowObjectDetector(recall=0.0, false_positive=0.0)
    assert robot.world.frame_description()["detections"] == []
    robot.world.low_detector = LowObjectDetector(recall=1.0, false_positive=1.0)
    labels = [d["label"] for d in robot.world.frame_description()["detections"]]
    assert "sock" in labels and len(labels) == 2


def test_the_stand_in_detector_touches_only_low_objects():
    robot = _robot(3, label="chair")
    robot.world.low_detector = LowObjectDetector(recall=0.0, false_positive=0.0)
    assert [d["label"] for d in robot.world.frame_description()["detections"]] == ["chair"]


def test_the_stand_in_detector_rates_over_many_frames():
    det = LowObjectDetector(recall=0.8, false_positive=0.05, seed=3)
    kept = sum(det.keep() for _ in range(4000))
    spurious = sum(det.spurious_detection(1.0) is not None for _ in range(4000))
    assert 0.77 < kept / 4000 < 0.83
    assert 0.035 < spurious / 4000 < 0.065


def test_the_hazards_house(monkeypatch):
    from sim.maps import build_movers, build_world
    from sim.maps.complex_house import HAZARDS_FT, LOW_HEIGHTS_M, OBJECTS

    monkeypatch.delenv("SIM_HAZARD_RECALL", raising=False)
    monkeypatch.delenv("SIM_HAZARD_FP", raising=False)
    world = build_world("complex_house_hazards")
    assert world.map_name == "complex_house_hazards"
    assert world.low_detector is None
    assert len(world.objects) == len(OBJECTS) + len(HAZARDS_FT)
    assert {world.objects[c] for c in world.low_cells} <= set(LOW_HEIGHTS_M)
    assert {name for name, _, _ in HAZARDS_FT} <= {world.objects[c] for c in world.low_cells}
    # The target stays tall: the lidar ranges it at arrival.
    assert "red backpack" not in LOW_HEIGHTS_M
    # A variant shares its house's movers.
    from sim.maps import complex_house
    for scenario in getattr(complex_house, "MOVERS", {}):
        assert build_movers("complex_house_hazards", scenario) is not None

    monkeypatch.setenv("SIM_HAZARD_RECALL", "0.5")
    assert build_world("complex_house_hazards").low_detector.recall == 0.5


def test_the_rover_still_reaches_every_room_of_the_hazards_house():
    pytest.importorskip("scipy")
    from tests.chassis_fit import SAFETY_MARGIN_M, UGV_ROVER, fit
    got = fit("complex_house_hazards", *UGV_ROVER, SAFETY_MARGIN_M)
    assert got["start_free"]
    assert not [name for name, r in got["rooms"].items() if not r["reached"]]


def test_the_band_does_not_shift_the_level_rows_noise():
    """3.74 review: with S5's noise on, a house with no heights reads the
    same level row and scalar as before -- the band has its own stream."""
    import random

    from sim.sensors import DistanceSensorModel

    # The scalar sequence must not depend on whether grids were read with a
    # band beside them; compare against a model the band never touches.
    robot = _robot(None, heights={})
    robot.sensor = DistanceSensorModel(noise_stddev_cm=3.0, dropout_rate=0.2, rng=random.Random(9))
    level = [(robot.get_depth_grid()["zones"], robot.get_distance()) for _ in range(20)]
    robot2 = _robot(None, heights={})
    robot2.sensor = DistanceSensorModel(noise_stddev_cm=3.0, dropout_rate=0.2, rng=random.Random(9))
    robot2._floor_band = lambda cols, cell_cm: {}
    level2 = [(robot2.get_depth_grid()["zones"], robot2.get_distance()) for _ in range(20)]
    assert level == level2


def test_noise_never_carries_a_band_reading_past_its_reach():
    from sim.sensors import DistanceSensorModel
    import random
    for seed in range(30):
        robot = _robot(4)  # true clearance ~135 cm
        robot.sensor = DistanceSensorModel(noise_stddev_cm=20.0, rng=random.Random(seed))
        for zone in robot.get_depth_grid()["floor"]["zones"]:
            assert zone["distance_cm"] is None or zone["distance_cm"] <= FLOOR_BAND_MAX_CM


def test_low_cells_follow_a_replaced_object_table():
    robot = _robot(2)
    first = robot.world.low_cells
    assert robot.world.low_cells is first  # cached
    robot.world.move_object((4, 2), (5, 2))
    assert robot.world.low_cells == {(5, 2)}
    assert robot.world.lidar_solid_cells == frozenset()


# ---------------- part B: the floor band in robot/safety.py ----------------

from robot.safety import SafetyController  # noqa: E402


def test_the_band_vetoes_a_forward_the_scan_would_allow():
    robot = _robot(1)  # a sock ~45 cm from the lens, under every beam
    safety = SafetyController(robot, 20.0)
    assert safety.footprint_clearance(+1)[1] == "scan_footprint_no_target"
    cm, source = safety.floor_clearance()
    assert source == "floor_band" and cm < 40
    assert safety.forward_clearance()[1].startswith("floor_band")


def test_no_band_changes_nothing():
    """A grid with no `floor` block -- every real body today."""
    robot = _robot(1)
    safety = SafetyController(robot, 20.0)
    plain = robot.get_depth_grid()
    plain.pop("floor")
    assert safety.floor_clearance(plain) == (None, "no_floor_band")


def test_unusable_zones_never_enter_the_comparison():
    """Never compared (M3); and since 3.76 K1 a band that saw nothing says
    so, rather than reading as a clear corridor."""
    safety = SafetyController(_robot(None), 20.0)
    grid = {"floor": {"fov_deg": 73.0, "pan_deg": 0.0, "origin_x_cm": 6.5,
                      "zones": [{"status": interface.ZONE_UNUSABLE, "distance_cm": None}] * 32}}
    assert safety.floor_clearance(grid) == (None, "floor_band_unusable")


def test_a_tight_corridor_does_not_stop_the_robot():
    """A one-cell (30 cm) corridor: its walls are 15 cm off the centre-line,
    just outside the 23.1 cm chassis' corridor (11.55 + 3 cm). Measured
    from the bumper instead of the lens (the first cut), they read as in
    it and stopped the robot."""
    layout = ["#" * 14, "#" + "." * 12 + "#", "#" * 14]
    world = GridWorld(layout, {"hall": set()}, robot_x=1, robot_y=1, heading=Heading.E)
    robot = MockRobot(world, render=False)
    safety = SafetyController(robot, 20.0)
    for _ in range(8):
        cm, _source = safety.floor_clearance()
        assert cm is None or cm > safety.required_clearance_cm(0.2)
        robot.drive_forward(speed=50, duration=0.5)


def _grid(*zones, origin=6.5, fov=73.0, cols=None):
    """A floor block with the given (distance_cm, bearing_deg) zones."""
    zs = [{"status": interface.ZONE_RANGE, "distance_cm": d, "bearing_deg": b} for d, b in zones]
    return {"floor": {"fov_deg": fov, "pan_deg": 0.0, "origin_x_cm": origin,
                      "min_range_cm": 20.0, "zones": zs}}


def test_a_return_beside_the_body_stops_only_inside_its_width():
    """As `footprint_clearance()` judges the scan (R6's frozen-at-a-jamb
    lesson): beside the chassis, the 3 cm side margin does not apply."""
    safety = SafetyController(_robot(None), 20.0)
    # A point level with the front edge, 12.5 cm off the centre-line: inside
    # the margin (14.55) but outside the width (11.55) -> no stop.
    beside = _grid((_m_hyp(12.65 - 6.5 - 1.0, 12.5), _m_brg(12.65 - 6.5 - 1.0, 12.5)))
    assert safety.floor_clearance(beside) == (None, "floor_band_no_target")
    inside = _grid((_m_hyp(12.65 - 6.5 - 1.0, 10.0), _m_brg(12.65 - 6.5 - 1.0, 10.0)))
    assert safety.floor_clearance(inside) == (0.0, "floor_band")
    ahead = _grid((_m_hyp(12.65 - 6.5 + 10.0, 13.5), _m_brg(12.65 - 6.5 + 10.0, 13.5)))
    assert safety.floor_clearance(ahead) == (pytest.approx(10.0, abs=0.1), "floor_band")


def _m_hyp(x, y):
    import math as _m
    return _m.hypot(x, y)


def _m_brg(x, y):
    import math as _m
    return _m.degrees(_m.atan2(y, x))


def test_a_null_bearing_falls_back_to_the_zone_centre():
    safety = SafetyController(_robot(None), 20.0)
    grid = {"floor": {"fov_deg": 73.0, "origin_x_cm": 6.5, "zones": [
        {"status": interface.ZONE_RANGE, "distance_cm": 40.0, "bearing_deg": None}]}}
    assert safety.floor_clearance(grid)[1] == "floor_band"
    assert safety.floor_clearance({"floor": {"fov_deg": None, "zones": [{}]}}) == (None, "no_floor_band")


def test_band_and_scan_points_share_one_frame():
    """A wall off to one side: every band return in range lies on a scan
    return -- the two sensors put it on the SAME side. A first cut compared
    them mirrored (bearings clockwise-positive, body y left-positive)."""
    import math as _m

    from robot.safety import scan_points_cm
    layout = ["#" * 8, "#......#", "#......#", "#......#", "#" * 8]
    world = GridWorld(layout, {"r": set()}, robot_x=2, robot_y=1, heading=Heading.E)
    world.theta = _m.radians(20)   # the wall above at an angle, to the LEFT
    robot = MockRobot(world, render=False)
    pts = list(scan_points_cm(robot.get_scan()))
    band = robot.get_depth_grid()["floor"]
    hits = 0
    for z in band["zones"]:
        if z["status"] != interface.ZONE_RANGE:
            continue
        b = _m.radians(z["bearing_deg"])
        px, py = band["origin_x_cm"] + z["distance_cm"] * _m.cos(b), -z["distance_cm"] * _m.sin(b)
        # 2 cm, plus the scan's 1-degree beam spacing at that range (corners).
        tol = 2.0 + z["distance_cm"] * _m.radians(1.0) + 0.5
        assert min(_m.hypot(px - a, py - c) for a, c in pts) <= tol
        hits += 1
    assert hits >= 5
