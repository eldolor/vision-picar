"""
Run with: pytest tests/test_mock_robot.py -v

Covers the Phase 0 success criteria (every mock function logs a state
change and returns a plausible result) and the Phase 3 safety-layer
requirement (AI's FORWARD is overridden when too close to an obstacle).
"""

import pytest
from sim.grid_world import GridWorld, Heading
from sim.mock_robot import MockRobot
from robot.safety import SafetyController, SafetyViolation


@pytest.fixture
def open_world():
    """A simple open room with no obstacles nearby, robot facing East."""
    layout = [
        "##########",
        "#........#",
        "#........#",
        "#........#",
        "##########",
    ]
    world = GridWorld(
        layout=layout,
        rooms={"room": {(x, y) for x in range(1, 9) for y in range(1, 4)}},
        robot_x=1,
        robot_y=2,
        heading=Heading.E,
    )
    return world


@pytest.fixture
def wall_ahead_world():
    """Robot facing directly into a wall two cells away."""
    layout = [
        "#####",
        "#...#",
        "#...#",
        "#####",
    ]
    world = GridWorld(
        layout=layout,
        rooms={"room": {(1, 1), (1, 2), (2, 1), (2, 2), (3, 1), (3, 2)}},
        robot_x=1,
        robot_y=1,
        heading=Heading.E,
    )
    return world


def test_drive_forward_moves_and_logs(open_world):
    robot = MockRobot(open_world)
    start_pos = (open_world.robot_x, open_world.robot_y)

    result = robot.drive_forward(speed=100, duration=1.0)

    assert result["action"] == "drive_forward"
    assert result["moved"] > 0
    assert (open_world.robot_x, open_world.robot_y) != start_pos
    assert any("MOVE" in entry for entry in open_world.log)


def test_stopped_by_wall(open_world):
    robot = MockRobot(open_world)
    # Drive far enough that it should hit the East wall and stop early.
    result = robot.drive_forward(speed=100, duration=10.0)
    assert open_world.robot_x <= 8  # never passes through the wall
    assert any("BLOCKED" in entry for entry in open_world.log)


def test_turn_left_right_are_inverses(open_world):
    robot = MockRobot(open_world)
    original = open_world.heading
    robot.turn_left(90)
    robot.turn_right(90)
    assert open_world.heading == original


def test_look_left_right_center_change_pan(open_world):
    robot = MockRobot(open_world)
    assert robot.look_left()["pan"] == -1
    assert robot.look_right()["pan"] == 1
    assert robot.look_center()["pan"] == 0


def test_get_distance_returns_plausible_cm(open_world):
    robot = MockRobot(open_world)
    distance = robot.get_distance()
    assert isinstance(distance, float)
    assert distance > 0


def test_get_camera_frame_returns_structured_scene(open_world):
    robot = MockRobot(open_world)
    frame = robot.get_camera_frame()
    assert "room" in frame
    assert "free_space_cells" in frame
    assert "objects_visible" in frame


def test_safety_blocks_forward_when_wall_close(wall_ahead_world):
    robot = MockRobot(wall_ahead_world)
    safety = SafetyController(robot, min_distance_cm=200)  # deliberately strict

    with pytest.raises(SafetyViolation):
        safety.check_and_execute("FORWARD", speed=100, duration=1.0)

    # Position must not have changed -- the veto happened before the move.
    assert (wall_ahead_world.robot_x, wall_ahead_world.robot_y) == (1, 1)


def test_safety_allows_forward_when_clear(open_world):
    robot = MockRobot(open_world)
    safety = SafetyController(robot, min_distance_cm=20)

    result = safety.check_and_execute("FORWARD", speed=100, duration=0.5)
    assert result["action"] == "drive_forward"


# ---------- Phase S4/S5 -- realtime and sensor noise (both opt-in) ----------


def test_settle_is_a_noop_by_default(open_world):
    """The regression this whole feature must never cause: every existing
    test and demo script stays exactly as fast as it always was."""
    import time

    robot = MockRobot(open_world)
    start = time.monotonic()
    robot.drive_forward(speed=50, duration=2.0)
    assert time.monotonic() - start < 0.5


def test_settle_actually_sleeps_under_realtime(open_world):
    import time

    robot = MockRobot(open_world, realtime=True)
    start = time.monotonic()
    robot.drive_forward(speed=50, duration=0.2)
    assert time.monotonic() - start >= 0.15


def test_get_distance_is_unaffected_by_realtime_alone(open_world):
    """realtime and sensor noise are independent knobs -- turning on
    realtime must not also start jittering distance readings."""
    robot = MockRobot(open_world, realtime=True)
    distance = robot.get_distance()
    assert distance % 30.0 == 0.0, "still the old exact multiple-of-30 behavior"


def test_get_distance_uses_the_configured_sensor_model(open_world):
    from sim.sensors import DistanceSensorModel

    sensor = DistanceSensorModel(dropout_rate=1.0)  # always fails, deterministically
    robot = MockRobot(open_world, sensor=sensor)
    assert robot.get_distance() == 0.0, "a dropout reads as 0.0 -- see sim/sensors.py"


def test_read_latency_only_applies_under_realtime(open_world):
    import time

    from sim.sensors import DistanceSensorModel

    sensor = DistanceSensorModel(read_latency_s=0.2)
    instant = MockRobot(open_world, realtime=False, sensor=sensor)
    start = time.monotonic()
    instant.get_distance()
    assert time.monotonic() - start < 0.1, "no realtime, no sleep, even with latency configured"

    real = MockRobot(open_world, realtime=True, sensor=sensor)
    start = time.monotonic()
    real.get_distance()
    assert time.monotonic() - start >= 0.15
