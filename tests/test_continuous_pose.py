"""
tests/test_continuous_pose.py

Phase R0 (`PLAN-ros-alignment.md`) -- the pose is continuous and the motion
is differential-drive kinematics.

Two suites' worth of things are deliberately NOT here.
`tests/test_robot_contract.py` already pins `get_odometry()`'s shape, its
monotonicity and the rule that a pivot covers no ground, for every backend;
`tests/test_world_contract.py` already pins `get_pose()`'s. Repeating either
against `MockRobot` would be writing one backend's tests twice with the noun
changed, which is the thing that file's own docstring warns against. What is
here is the part those suites cannot see, because it is about the simulator's
job rather than the contract's:

  * that a non-cardinal pose EXISTS and survives being read back, which is
    the single thing P25's A/B needed and could not get;
  * that the legacy cell-and-cardinal view of it still answers, because a
    great deal of this project legitimately thinks in cells and none of it
    was asked to change;
  * that the kinematics are the real ones -- a straight line does not depend
    on the track width, a pivot does, and the encoders agree with the pose
    because one is the inverse of the other;
  * that collision is continuous, and that the invariant the discrete mover
    guaranteed by construction (the robot's cell is never a wall) still
    holds now that it is guaranteed by arithmetic instead.

Run with: pytest tests/test_continuous_pose.py -v
"""

import math

import pytest

from sim import renderer
from sim.grid_world import (
    CELL_WALL,
    PAN_ANGLE_RAD,
    ROBOT_HALF_CELL,
    GridWorld,
    Heading,
)
from sim.maps.starter_house import build_starter_world
from sim.mock_robot import (
    DEFAULT_CELL_M,
    ENCODER_COUNTS_PER_REV,
    TRACK_WIDTH_M,
    WHEEL_MAX_RAD_S,
    WHEEL_RADIUS_M,
    MockRobot,
)
from sim.mock_world import MockWorld

OPEN_LAYOUT = [
    "##########",
    "#........#",
    "#........#",
    "#........#",
    "#........#",
    "##########",
]
OPEN_ROOMS = {"room": {(x, y) for x in range(1, 9) for y in range(1, 5)}}


def open_world(**kwargs):
    kwargs.setdefault("robot_x", 4)
    kwargs.setdefault("robot_y", 2)
    kwargs.setdefault("heading", Heading.E)
    return GridWorld(layout=OPEN_LAYOUT, rooms=OPEN_ROOMS, **kwargs)


def open_robot(**kwargs):
    return MockRobot(open_world(**kwargs), render=False)


# ---------- the angle conventions, pinned against each other ----------


@pytest.mark.parametrize("heading", list(Heading))
def test_a_cardinal_headings_angle_is_the_renderers_own_angle(heading):
    """`Heading.angle_rad()` is derived from `compass_deg()`, and
    `sim/renderer.py`'s `HEADING_ANGLE` is a line-for-line port of the
    JavaScript's table. Two statements of one mapping, so they get pinned
    together -- the same reason `CELL_M * 100 == DEFAULT_CELL_CM` is a test.
    If this fails, a rendered frame and the pose it was rendered from have
    started disagreeing about which way the robot faces.
    """
    assert heading.angle_rad() == pytest.approx(
        renderer.HEADING_ANGLE[heading.name])


@pytest.mark.parametrize("heading,expected", [
    (Heading.N, 0.0), (Heading.E, 90.0), (Heading.S, 180.0), (Heading.W, 270.0),
])
def test_heading_deg_is_a_compass_bearing(heading, expected):
    """Clockwise from north, positive to the robot's right -- the one
    convention every angle in this project uses. Cardinal cases only,
    because they are the ones with an unarguable right answer."""
    assert open_world(heading=heading).heading_deg == pytest.approx(expected)


def test_turning_right_increases_the_compass_bearing():
    """The sign that 'looks right at 0 and 180' if it is wrong, which is
    why it gets its own test rather than being inferred from the table
    above. A right turn is clockwise, so the bearing goes up."""
    world = open_world(heading=Heading.N)
    world.turn_right(30)
    assert world.heading_deg == pytest.approx(30.0)
    world.turn_left(60)
    assert world.heading_deg == pytest.approx(330.0), "and wraps, not negative"


def test_the_pan_swings_the_view_and_not_the_body():
    """What `look_left()` has always meant, now as an angle. The body's
    `theta` must not move: a pan moves the camera and no wheels."""
    world = open_world(heading=Heading.N)
    body = world.theta
    world.look_right()
    assert world.theta == body
    assert world.view_angle() == pytest.approx(
        renderer.normalize_angle(body + PAN_ANGLE_RAD))
    world.look_center()
    assert world.view_angle() == pytest.approx(body)


# ---------- the pose is continuous, which is the whole phase ----------


def test_a_turn_that_is_not_a_quarter_turn_really_turns_that_far():
    """**The test R0 exists for.** `brain/goal_pose.py` ships default OFF
    because the sim "turns in 90-degree quanta against a 10-degree centre
    band, so a target off a cardinal direction can never be centred" --
    every turn overshot and flipped the error's sign, and both arms of
    P25's A/B just alternated LEFT/RIGHT. Before this, `turn_left(45)`
    rounded to a whole quarter-turn and moved 90 degrees.
    """
    robot = open_robot(heading=Heading.E)
    robot.turn_left(45)
    assert robot.world.heading_deg == pytest.approx(45.0)
    robot.turn_left(10)
    assert robot.world.heading_deg == pytest.approx(35.0)
    assert robot.get_odometry()["heading_deg"] == pytest.approx(35.0)


def test_driving_at_a_non_cardinal_heading_lands_off_the_grid_lines():
    """The other half: a continuous heading is worth nothing if travel
    along it snaps back to a cell centre."""
    robot = open_robot(heading=Heading.E)
    robot.turn_right(45)
    before = (robot.world.x, robot.world.y)
    robot.drive_forward(speed=100, duration=1.0)
    dx = robot.world.x - before[0]
    dy = robot.world.y - before[1]
    assert dx == pytest.approx(dy, abs=1e-6), "45 degrees means equal parts"
    assert dx > 0.5, "it actually moved"
    assert robot.world.x % 1 != pytest.approx(0.5, abs=1e-9)


def test_the_pose_the_world_publishes_is_the_continuous_one():
    """N1 shipped `MockWorld.get_pose()` against a quantised pose and
    predicted that C2/R0 would make it smooth "without changing one line on
    either side of the wall". This is the check that the prediction held --
    the map view draws a robot between cells, at a non-multiple-of-90
    bearing, through the contract it already had.
    """
    grid = build_starter_world()
    world = MockWorld(grid)
    robot = MockRobot(grid, render=False)
    robot.turn_left(30)
    robot.drive_forward(speed=40, duration=0.4)
    pose = world.get_pose()
    assert pose["usable"] is True
    assert pose["heading_deg"] % 90 != pytest.approx(0.0, abs=1e-6)
    assert pose["x_m"] == pytest.approx(grid.x * 0.30)
    assert pose["y_m"] == pytest.approx(grid.y * 0.30)


def test_a_pan_still_does_not_move_the_published_pose():
    """Re-pinned here because `get_pose()` now reads a float off the world
    rather than a cardinal enum, and reading `view_angle()` by mistake would
    be an easy and very plausible slip."""
    grid = build_starter_world()
    world = MockWorld(grid)
    before = world.get_pose()["heading_deg"]
    grid.look_left()
    assert world.get_pose()["heading_deg"] == before


# ---------- the cell-and-cardinal view still answers ----------


def test_the_cell_view_reads_and_writes_the_continuous_pose():
    """`world.robot_x += 1` is the idiom several tests and the starter map
    use, and it is a statement about CELLS with no opinion about where in
    the cell to land -- so it snaps to the middle, which is where the
    discrete mover always left the robot anyway."""
    world = open_world()
    assert (world.robot_x, world.robot_y) == (4, 2)
    assert (world.x, world.y) == (4.5, 2.5)
    world.robot_x += 1
    assert world.x == 5.5
    world.x = 6.9
    assert world.robot_x == 6, "the cell view floors rather than rounding"


def test_the_cardinal_view_is_the_nearest_heading():
    """Lossy on purpose. Every consumer of `heading` was cardinal-only
    before R0 and none of them was asked to change; the ones that want the
    real answer read `heading_deg`."""
    world = open_world(heading=Heading.N)
    world.turn_right(20)
    assert world.heading is Heading.N
    world.turn_right(40)  # 60 degrees: past the halfway point to East
    assert world.heading is Heading.E
    assert world.heading_deg == pytest.approx(60.0)


def test_a_quarter_turn_leaves_the_cardinal_view_lossless():
    """The property that keeps every pre-R0 test meaningful: drive the world
    in quarter turns and the cardinal heading is still exact, not 89.97."""
    world = open_world(heading=Heading.N)
    for expected in (Heading.E, Heading.S, Heading.W, Heading.N):
        world.turn_right()
        assert world.heading is expected
        assert world.heading_deg == pytest.approx(expected.compass_deg())


# ---------- the kinematics are the real ones ----------


def test_equal_wheel_velocities_drive_straight_and_ignore_the_track_width():
    """The track width is the one chassis constant that is a PLACEHOLDER
    (`HARDWARE-BOM.md` 4.3 says to measure it). This pins the blast radius
    of getting it wrong: a straight line does not depend on it at all, so a
    wrong value can make the sim turn at the wrong rate and can never make
    it travel the wrong distance.
    """
    robot = open_robot(heading=Heading.E)
    heading_before = robot.world.heading_deg
    robot.set_wheel_velocity(WHEEL_MAX_RAD_S, WHEEL_MAX_RAD_S)
    result = robot.step(0.5)
    assert robot.world.heading_deg == pytest.approx(heading_before)
    assert result["moved_m"] == pytest.approx(
        WHEEL_MAX_RAD_S * WHEEL_RADIUS_M * 0.5, rel=1e-6)


def test_opposite_wheel_velocities_pivot_at_the_rate_the_track_width_implies():
    """omega = (v_right - v_left) / track, and the sign of it is the bug
    worth a test of its own: `omega` is REP-103's body yaw rate, which is
    counter-clockwise positive -- a turn to the robot's LEFT -- while the
    grid's `theta` increases clockwise. Right wheel forward must therefore
    DECREASE the compass bearing.
    """
    robot = open_robot(heading=Heading.E)
    dt = 0.25
    robot.set_wheel_velocity(-WHEEL_MAX_RAD_S, WHEEL_MAX_RAD_S)
    result = robot.step(dt)
    expected_deg = math.degrees(
        2 * WHEEL_MAX_RAD_S * WHEEL_RADIUS_M / TRACK_WIDTH_M * dt)
    assert result["turned_deg"] == pytest.approx(expected_deg, rel=1e-6)
    assert robot.world.heading_deg == pytest.approx(
        (90.0 - expected_deg) % 360.0, rel=1e-6)
    assert result["moved_m"] == pytest.approx(0.0, abs=1e-12)


def test_one_wheel_stopped_draws_an_arc_rather_than_a_pivot_or_a_line():
    """The case neither of the two above covers, and the one a discrete
    mover could not represent at all: both the position and the heading
    change, in one command."""
    robot = open_robot(heading=Heading.E)
    start = (robot.world.x, robot.world.y, robot.world.heading_deg)
    robot.set_wheel_velocity(0.0, WHEEL_MAX_RAD_S)
    robot.step(0.2)
    assert (robot.world.x, robot.world.y) != start[:2]
    assert robot.world.heading_deg != pytest.approx(start[2])


def test_the_encoders_are_the_inverse_of_the_motion_that_happened():
    """`get_wheel_state()` and `get_odometry()` are one motion described
    twice, so they have to be reconcilable -- the forward kinematics of the
    wheel positions must give back the body's path length. A drift here is
    what makes a `diff_drive_controller` fight its own odometry.
    """
    robot = open_robot(heading=Heading.E)
    robot.drive_forward(speed=100, duration=1.0)
    robot.turn_left(37)
    robot.drive_forward(speed=60, duration=0.5)

    state = robot.get_wheel_state()
    left_m = state["left"]["position_rad"] * WHEEL_RADIUS_M
    right_m = state["right"]["position_rad"] * WHEEL_RADIUS_M
    assert (left_m + right_m) / 2 == pytest.approx(
        robot.get_odometry()["distance_m"], abs=1e-3)


def test_encoder_counts_are_the_position_in_the_units_the_board_reports():
    """1760 counts per revolution at 4x quadrature (`HARDWARE-BOM.md` 4.3),
    so R7's fake ESP32 has something to serialise and R2 has something to
    publish. A count is not a new measurement, it is a unit change."""
    robot = open_robot(heading=Heading.E)
    robot.drive_forward(speed=100, duration=1.0)
    state = robot.get_wheel_state()
    assert state["counts_per_rev"] == ENCODER_COUNTS_PER_REV
    for side in ("left", "right"):
        expected = state[side]["position_rad"] / (2 * math.pi) * ENCODER_COUNTS_PER_REV
        assert state[side]["counts"] == pytest.approx(expected, abs=1)


def test_a_standing_wheel_command_persists_until_stop_cancels_it():
    """A velocity API means there is now something for `stop()` to cancel,
    and every failsafe in `control/` (B3.1-B3.3) depends on `stop()` really
    stopping. Before R0 a move was instantaneous and `stop()` was a log
    line, so a robot with a standing command and a hung brain would have
    kept integrating forward on the next tick.
    """
    robot = open_robot(heading=Heading.E)
    robot.set_wheel_velocity(WHEEL_MAX_RAD_S, WHEEL_MAX_RAD_S)
    robot.step(0.1)
    moved_once = robot.get_odometry()["distance_m"]
    robot.step(0.1)
    assert robot.get_odometry()["distance_m"] > moved_once, "command persists"

    robot.stop()
    settled = robot.get_odometry()["distance_m"]
    robot.step(0.5)
    assert robot.get_odometry()["distance_m"] == settled
    assert robot.get_wheel_state()["left"]["velocity_rad_s"] == 0.0


def test_a_verb_still_covers_the_ground_it_always_did():
    """The compatibility promise that keeps every step budget, recorded
    demo and safety threshold in this repo meaningful. `speed=100` for one
    second is two cells, as `CELLS_PER_SECOND_AT_FULL_SPEED` has said since
    Phase 0 -- now because the wheels turned that far rather than because
    the robot was teleported twice.
    """
    robot = open_robot(robot_x=1, robot_y=2, heading=Heading.E)
    result = robot.drive_forward(speed=100, duration=1.0)
    assert result["moved"] == pytest.approx(2.0, abs=1e-6)
    assert robot.world.robot_x == 3
    assert robot.get_odometry()["distance_m"] == pytest.approx(
        2 * DEFAULT_CELL_M, abs=1e-4)


# ---------- collision, now continuous ----------


def test_a_move_is_capped_where_the_robots_front_meets_the_wall():
    """Continuous clearance, which the discrete mover could not express: it
    asked "is the next CELL passable", so being 4cm from a wall was not a
    representable state."""
    world = open_world(robot_x=1, robot_y=2, heading=Heading.E)
    moved = world.translate(20)
    wall_face = 9.0  # cells: the layout's east wall starts at index 9
    assert world.x == pytest.approx(wall_face - ROBOT_HALF_CELL, abs=0.06)
    assert moved == pytest.approx(world.x - 1.5, abs=1e-9)
    assert any("BLOCKED" in entry for entry in world.log)


def test_an_ordinary_step_into_the_last_free_cell_is_not_called_a_collision():
    """The regression this file has to guard, because the arithmetic invites
    it. `renderer.cast_ray()` overshoots by up to one `FPV_STEP`, and
    `get_depth_grid()` deliberately subtracts that to avoid overstating
    clearance to a safety veto. Subtracting it here too would leave every
    one-cell step 1.5cm short of the cell it aimed for and log a wall it
    never touched.
    """
    world = open_world(robot_x=7, robot_y=2, heading=Heading.E)
    assert world.translate(1) == pytest.approx(1.0)
    assert world.robot_x == 8
    assert not any("BLOCKED" in entry for entry in world.log)


def test_the_robots_cell_is_never_a_wall_however_hard_it_is_driven():
    """The invariant the discrete mover got by construction and this one
    has to earn. Swept over every cardinal and a couple of diagonals, so a
    sign error in the arc integration cannot hide in one direction.
    """
    grid = build_starter_world()
    robot = MockRobot(grid, render=False)
    for turn in (0, 37, 53, 90, 90, 90, 45):
        robot.turn_right(turn)
        for _ in range(12):
            robot.drive_forward(speed=100, duration=1.0)
            cell = grid._cell(grid.robot_x, grid.robot_y)
            assert cell != CELL_WALL, (
                f"drove into a wall at ({grid.x:.2f}, {grid.y:.2f}) "
                f"heading {grid.heading_deg:.1f}")


def test_reversing_is_checked_against_the_wall_behind():
    """The direction a single forward-facing ray cannot see, and the one a
    cell-walk got for free by negating a step."""
    world = open_world(robot_x=1, robot_y=2, heading=Heading.E)
    moved = world.translate(-5)
    assert moved > -1.0, "the west wall is one cell behind"
    assert world.robot_x == 1
    assert any("BLOCKED" in entry for entry in world.log)


def test_a_zero_command_is_a_no_op_everywhere_it_can_be_given():
    """Every entry point has to tolerate being asked for nothing -- a
    velocity API is polled, and a controller that has just arrived at its
    goal sends zeros at 20Hz. Cheap to pin and expensive to discover."""
    robot = open_robot(heading=Heading.E)
    pose = (robot.world.x, robot.world.y, robot.world.theta)
    assert robot.world.translate(0) == 0.0
    assert robot.world.rotate(0) == 0.0
    robot.step(0)
    robot.step(-1)
    assert robot.drive_forward(speed=0, duration=0.5)["moved"] == 0.0
    assert robot.turn_left(0)["heading_deg"] == pytest.approx(90.0)
    assert (robot.world.x, robot.world.y, robot.world.theta) == pose
    assert robot.get_odometry()["distance_m"] == 0.0


def test_a_world_can_be_built_from_a_real_pose_without_rounding_it():
    """The constructor still takes a cell and a cardinal, because every
    caller does -- but a caller that HAS a continuous pose should not have to
    round it to say so. R2's ground-truth ghost and R5's drifted odometry
    both need this."""
    world = GridWorld(layout=OPEN_LAYOUT, rooms=OPEN_ROOMS,
                      x=4.25, y=2.75, theta=math.radians(-20))
    assert (world.x, world.y) == (4.25, 2.75)
    assert world.heading_deg == pytest.approx(70.0)
    assert (world.robot_x, world.robot_y) == (4, 2)


# ---------- the readouts the twin draws (R0's UI proof) ----------


def test_the_depth_strip_is_cast_from_the_continuous_view_angle():
    """R0's stated proof is that "the FPV and depth strip track smoothly".
    Both are cast from `view_angle()` now; this pins the depth half, which
    is the one with numbers in it. A 45-degree turn in a square room must
    move every zone, where a cardinal-rounded angle would move none.
    """
    robot = open_robot(robot_x=2, robot_y=2, heading=Heading.E)
    before = [z["distance_cm"] for z in robot.get_depth_grid()["zones"]]
    robot.turn_right(45)
    after = [z["distance_cm"] for z in robot.get_depth_grid()["zones"]]
    assert before != after


def test_the_rendered_frame_is_cast_from_the_continuous_view_angle():
    """The FPV half of the same proof. Cheaper and stronger than comparing
    JPEGs: `wall_profile()` is the geometry the picture is painted from, and
    `tests/test_renderer_parity.py` already pins it against the browser.
    """
    world = open_world(robot_x=2, robot_y=2, heading=Heading.E)
    before = renderer.wall_profile(world.layout, world.x, world.y,
                                   world.view_angle())
    world.turn_right(45)
    after = renderer.wall_profile(world.layout, world.x, world.y,
                                 world.view_angle())
    assert before != after


# ---------- perception and sightings, without cells ----------


def test_an_object_off_a_cardinal_direction_is_perceived():
    """The cell walk this replaced looked exactly three cells down a
    cardinal, so an object at 45 degrees was invisible from every pose.
    Perception is now the renderer's own visibility test -- the object is
    reported because it is in the picture."""
    world = GridWorld(layout=OPEN_LAYOUT, rooms=OPEN_ROOMS,
                      objects={(4, 1): "red backpack"},
                      robot_x=2, robot_y=3, heading=Heading.E)
    assert "red backpack" not in world.frame_description()["objects_visible"]
    world.turn_left(45)
    assert "red backpack" in world.frame_description()["objects_visible"]


def test_the_robot_does_not_perceive_what_it_is_standing_on():
    """The renderer's angle to a point at distance zero is 'dead ahead',
    so without the footprint cut the robot would see the object underneath
    it. A detector cannot."""
    world = GridWorld(layout=OPEN_LAYOUT, rooms=OPEN_ROOMS,
                      objects={(4, 2): "sofa"}, robot_x=4, robot_y=2)
    assert world.frame_description()["objects_visible"] == []


def test_a_sighting_is_anchored_to_the_worlds_pose_not_a_cell():
    """A sighting is a statement about the HOUSE, so it carries a map-frame
    pose with the map it is in -- the shape a `PoseStamped` in `map` will
    have at R3. It used to be a grid cell read off the camera frame."""
    from brain.agent import ObjectSearchAgent
    from brain.memory import MissionMemory

    robot = MockRobot(build_starter_world(), render=False)
    memory = MissionMemory(mission="Find the red backpack.",
                           target_object="red backpack")
    agent = ObjectSearchAgent(robot, memory, min_distance_cm=30,
                              world=MockWorld(robot.world))
    report = agent.run_mission(max_steps=150)

    assert report["found"] is True
    pose = report["sighting"].position
    assert set(pose) == {"x_m", "y_m", "heading_deg", "map_id"}
    assert pose["map_id"] == "sim-grid"
