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
                            FLOOR_BAND_MIN_CM, MockRobot)
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
    """Robot centre (1.5) to the object's near face (2 + gap), less the half
    cell every depth reading here is measured from."""
    return (2 + gap_cells - 1.5 - ROBOT_HALF_CELL) * DEFAULT_CELL_CM


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

    truth = _true_clearance_cm(gap_cells)
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
            measured.append(_true_clearance_cm(gap))
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
