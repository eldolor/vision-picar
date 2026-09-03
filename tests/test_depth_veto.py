"""
tests/test_depth_veto.py

Phase M3 (PLAN-microduck-transplants.md) -- the tri-state zone, and a
centre-zone veto.

M2 put a depth grid on `RobotInterface` with nobody reading it. This is
the consumer: `robot/safety.py` reduces the zones the next move crosses to
one number and compares that to `min_distance_cm`, exactly as it always
compared `get_distance()`. The shape of the safety layer does not change.
What changes is which sensor answers, and what happens when it cannot.

The rule the whole phase rests on, from Microduck's `tof/src/lib.rs`:

    ST's status byte is the difference between "nothing is there" and "I
    could not tell", and collapsing them loses the distinction a map most
    needs: empty space is information, an unusable measurement is not.

`sim/sensors.py` collapses them for the *scalar* and is right to -- a lone
float has no way to say the second thing, so `0.0` fails toward stop. The
grid has a way to say it, and a veto that read a failed zone as `0.0`
would stop on every dropout. With eight zones a dropout is eight times as
likely to land somewhere as it is on one beam, so the collapse would make
the grid worse than the sensor it improves on. That is what makes the
tri-state load-bearing rather than tidy.

Run with: pytest tests/test_depth_veto.py -v
"""

import logging
import math
import random

import pytest

from robot.interface import ZONE_NO_TARGET, ZONE_RANGE, ZONE_UNUSABLE
from robot.safety import (
    CHASSIS_WIDTH_CM,
    PATH_FRACTION,
    PATH_HALF_ANGLE_DEG,
    PATH_REFERENCE_CM,
    SafetyController,
    SafetyViolation,
    path_zone_indices,
)
from sim import renderer
from sim.grid_world import GridWorld, Heading
from sim.maps.starter_house import LAYOUT, OBJECTS, ROOMS
from sim.mock_robot import MockRobot
from sim.sensors import DistanceSensorModel


def world_at(x, y, heading):
    return GridWorld(layout=LAYOUT, rooms=ROOMS, objects=dict(OBJECTS),
                     robot_x=x, robot_y=y, heading=heading)


def robot_at(x, y, heading, sensor=None):
    return MockRobot(world_at(x, y, heading), render=False, sensor=sensor)


class GridRobot:
    """A robot that publishes exactly the grid it is given. Duck-typed
    rather than a RobotInterface subclass, so a test can state one grid and
    nothing else -- the reduction under test does not care where the zones
    came from, and neither should the test."""

    def __init__(self, zones, rows=1, scalar=999.0):
        self.zones = zones
        self.rows = rows
        self.scalar = scalar
        self.stopped = 0
        self.distance_calls = 0

    def get_depth_grid(self):
        return {"rows": self.rows, "cols": len(self.zones) // self.rows,
                "zones": self.zones}

    def get_distance(self):
        self.distance_calls += 1
        return self.scalar

    def stop(self):
        self.stopped += 1
        return {"action": "stop"}

    def drive_forward(self, speed=50, duration=0.5):
        return {"action": "drive_forward"}


def rng(cm):
    return {"status": ZONE_RANGE, "distance_cm": cm}


NO_TARGET = {"status": ZONE_NO_TARGET, "distance_cm": None}
UNUSABLE = {"status": ZONE_UNUSABLE, "distance_cm": None}


# ---------- which zones are "the path" ----------


def test_the_path_is_the_middle_of_the_grid_and_not_all_of_it():
    """Both neighbours of this choice are real failures. The whole grid
    vetoes every corridor (the outer rays read the walls beside the robot);
    a single centre zone is the one beam `get_distance()` already was."""
    idx = path_zone_indices(1, 8)
    assert idx == [2, 3, 4, 5]
    assert 0 not in idx and 7 not in idx
    assert len(idx) == round(8 * PATH_FRACTION)


def test_every_row_is_in_the_path_until_something_can_reject_the_floor():
    """Rows are not narrowed here. The sim publishes rows: 1, and on real
    hardware which rows matter is M8's floor geometry, which this layer does
    not have. Taking all rows is the conservative reading."""
    assert path_zone_indices(2, 4) == [1, 2, 5, 6]


def test_a_one_zone_grid_still_has_a_path():
    """max(1, ...) -- a degenerate grid must not reduce to no zones at all
    and silently fall through to the scalar."""
    assert path_zone_indices(1, 1) == [0]


# ---------- the three outcomes ----------


def test_a_measured_path_is_the_nearest_of_its_own_zones():
    bot = GridRobot([rng(300), rng(40), rng(31), rng(55), rng(60), rng(300)])
    clearance, source = SafetyController(bot).path_clearance()
    assert (clearance, source) == (31, "depth_grid")
    assert bot.distance_calls == 0, (
        "the grid answered, so the scalar must not also be fetched -- over "
        "RemoteRobot that is a second HTTP round trip per FORWARD")


def test_side_zones_are_informational_and_never_veto():
    """The corridor case, which is why this phase reduces the middle rather
    than the whole grid: in a 30cm corridor the outermost rays of a
    60-degree cone read the side walls at ~18cm, well under the 20cm
    threshold, on every single step of a legal traverse."""
    bot = GridRobot([rng(18), rng(18), rng(300), rng(300), rng(300), rng(300),
                     rng(18), rng(18)])
    safety = SafetyController(bot, min_distance_cm=20.0)
    assert safety.path_clearance() == (300, "depth_grid")
    assert safety.check_and_execute("FORWARD")["action"] == "drive_forward"


def test_nothing_within_range_is_a_fact_and_never_a_veto():
    """`no_target` is information about the room -- empty space -- and must
    not be read as either a distance or a failure."""
    bot = GridRobot([rng(10), NO_TARGET, NO_TARGET, rng(10)], scalar=0.0)
    safety = SafetyController(bot, min_distance_cm=20.0)
    clearance, source = safety.path_clearance()
    assert clearance is None and source == "depth_grid_no_target"
    assert safety.check_and_execute("FORWARD")["action"] == "drive_forward"
    assert bot.distance_calls == 0, (
        "an answered path must not consult the scalar -- here the scalar is "
        "0.0 and would have vetoed a demonstrably clear path")


def test_an_unusable_zone_is_dropped_rather_than_read_as_near_or_far():
    """The rule the phase exists for. `sim/sensors.py` would have handed a
    failed *scalar* 0.0; a failed zone carries no number at all, and the
    remaining zones answer."""
    bot = GridRobot([rng(300), UNUSABLE, UNUSABLE, rng(45), rng(300), rng(300)])
    safety = SafetyController(bot, min_distance_cm=20.0)
    assert safety.path_clearance() == (45, "depth_grid")
    assert safety.check_and_execute("FORWARD")["action"] == "drive_forward"


def test_a_wholly_blind_path_falls_back_to_the_scalar_and_still_fails_safe():
    """Case 3. Every path zone unusable is the one situation the grid cannot
    answer, and the answer is the pre-M3 veto -- including the scalar's own
    0.0-on-dropout collapse, so a blind robot still stops."""
    bot = GridRobot([rng(300), UNUSABLE, UNUSABLE, rng(300)], scalar=0.0)
    safety = SafetyController(bot, min_distance_cm=20.0)
    assert safety.path_clearance() == (0.0, "distance_sensor")
    with pytest.raises(SafetyViolation):
        safety.check_and_execute("FORWARD")
    assert bot.stopped == 1


def test_a_backend_with_no_grid_at_all_behaves_exactly_as_before():
    """Every duck-typed robot in this suite predating M2, and every
    RobotInterface backend with no depth sensor. Nothing regresses."""

    class ScalarOnly:
        def get_distance(self):
            return 5.0

        def stop(self):
            return {"action": "stop"}

        def drive_forward(self, speed=50, duration=0.5):
            return {"action": "drive_forward"}

    safety = SafetyController(ScalarOnly(), min_distance_cm=20.0)
    assert safety.path_clearance() == (5.0, "distance_sensor")
    with pytest.raises(SafetyViolation):
        safety.check_and_execute("FORWARD")


# ---------- against the real simulator ----------


def test_the_collar_still_fires_against_a_wall_in_the_grid_world():
    """The case the veto exists for, end to end through MockRobot rather
    than a stub: nose against the living room's east wall."""
    safety = SafetyController(robot_at(3, 1, Heading.E), min_distance_cm=20.0)
    with pytest.raises(SafetyViolation):
        safety.check_and_execute("FORWARD")


def test_one_cell_of_clearance_is_not_vetoed_anywhere_it_used_to_be_legal():
    """The regression that would matter most: a wider veto that refuses the
    ordinary step. Every pose in the starter house with at least one free
    cell ahead must still be allowed."""
    world = world_at(1, 1, Heading.N)
    allowed = blocked = 0
    for y in range(1, len(LAYOUT) - 1):
        for x in range(1, len(LAYOUT[0]) - 1):
            for heading in Heading:
                bot = robot_at(x, y, heading)
                if bot.world._cell(x, y) == "#":
                    continue
                if bot.get_distance() < 30.0:
                    continue  # genuinely against a wall -- not this test's case
                safety = SafetyController(bot, min_distance_cm=20.0)
                try:
                    safety.check_and_execute("FORWARD")
                    allowed += 1
                except SafetyViolation:
                    blocked += 1
    assert allowed > 100, f"only {allowed} poses tested -- the sweep is broken"
    assert blocked == 0, (
        f"{blocked} of {allowed + blocked} poses with a free cell ahead were "
        "vetoed -- the path cone is too wide to drive through this house"
    )
    assert world is not None


def test_a_doorway_is_still_passable():
    """The failure mode on the other side of PATH_FRACTION. The living
    room's door is one cell wide, and a cone wide enough to catch both
    jambs would make the house impossible to cross."""
    for x in (1, 2, 3):
        safety = SafetyController(robot_at(x, 2, Heading.E), min_distance_cm=20.0)
        assert safety.check_and_execute("FORWARD")["action"] == "drive_forward", (
            f"blocked approaching the doorway from ({x}, 2)")


# ---------- dropout, in the simulator that now produces it ----------


def test_dropout_produces_unusable_zones_rather_than_zero():
    """S5's scalar reads a dropout as 0.0 and always trips the veto. The
    grid says so instead, which is what lets the veto ignore it."""
    sensor = DistanceSensorModel(dropout_rate=1.0, rng=random.Random(1))
    grid = robot_at(2, 1, Heading.E, sensor=sensor).get_depth_grid()
    assert all(z["status"] == ZONE_UNUSABLE for z in grid["zones"])
    assert all(z["distance_cm"] is None for z in grid["zones"]), (
        "an unusable zone carrying a number is exactly the collapse this "
        "phase removes"
    )


def test_no_sensor_configured_still_produces_an_exact_grid():
    """The default path is untouched: no rng draws, no unusable zones."""
    grid = robot_at(2, 1, Heading.E).get_depth_grid()
    assert not any(z["status"] == ZONE_UNUSABLE for z in grid["zones"])


def test_dropout_does_not_stop_the_robot_dead_the_way_the_scalar_does():
    """M3's "done when", and the measurable payoff of the whole phase.

    With one beam, a 30% dropout rate vetoes ~30% of legal moves -- the
    reading is 0.0 and there is nothing else to consult. With eight zones
    it takes all four path zones failing at once, which is 0.3^4 -- under
    one percent. Same sensor, same dropout rate, two orders of magnitude
    fewer spurious stops.

    Set `PATH_FRACTION` aside and read a failed zone as 0.0 and this goes
    red immediately: it would veto more often than the scalar, not less.
    """
    dropout = 0.3
    trials = 400

    scalar_bot = robot_at(2, 1, Heading.E, sensor=DistanceSensorModel(
        dropout_rate=dropout, rng=random.Random(11)))
    scalar_blocked = 0
    for _ in range(trials):
        # The pre-M3 veto, stated directly: no grid, just the beam.
        if scalar_bot.get_distance() < 20.0:
            scalar_blocked += 1

    grid_bot = robot_at(2, 1, Heading.E, sensor=DistanceSensorModel(
        dropout_rate=dropout, rng=random.Random(11)))
    safety = SafetyController(grid_bot, min_distance_cm=20.0)
    grid_blocked = 0
    for _ in range(trials):
        # path_clearance() rather than check_and_execute(): the veto is the
        # comparison, and executing 400 real FORWARDs would drive the robot
        # into the wall on move two and then measure that instead. The
        # scalar loop above is the same comparison, stated the same way.
        clearance, _ = safety.path_clearance()
        if clearance is not None and clearance < safety.min_distance_cm:
            grid_blocked += 1

    assert scalar_blocked > trials * 0.2, (
        f"the scalar only lost {scalar_blocked}/{trials} -- the premise of "
        "this test is that dropout hurts it badly")
    assert grid_blocked < scalar_blocked / 10, (
        f"grid vetoed {grid_blocked}/{trials} vs the scalar's {scalar_blocked} "
        "-- the grid is supposed to survive dropout the scalar cannot")


# ---------- selecting the path by angle, not by fraction (5.1) ----------
#
# PATH_FRACTION was a sound proxy for an angle while every sensor
# considered had a narrow forward field. PLAN-onboard-perception.md 1.2
# buys a 360-degree lidar, on which the middle half of the columns is
# +/-90 degrees -- the whole forward hemisphere, which is the
# permanently-vetoed-in-every-corridor failure the module's own comment
# warns about. These pin the angular rule that replaces it.


def test_the_angular_rule_picks_the_same_zones_as_the_fraction_did_in_the_sim():
    """The change is of input, not of behaviour. On the sim's 60-degree
    8-column grid both rules select the middle four, which is what makes
    every other test in this file still meaningful."""
    assert path_zone_indices(1, 8, 60) == path_zone_indices(1, 8) == [2, 3, 4, 5]


def test_a_360_degree_grid_selects_the_path_and_not_the_hemisphere():
    """The defect 5.1 found, stated as a test. A 72-zone lidar at 5 degrees
    per bin must yield a cone a chassis width wide, not 36 zones spanning
    +/-90 degrees -- which would read the walls beside the robot and veto
    every corridor it is supposed to drive down."""
    idx = path_zone_indices(1, 72, 360)
    assert idx == [33, 34, 35, 36, 37, 38]
    # Straddling straight ahead, and narrow.
    assert len(idx) < 72 * PATH_FRACTION / 4
    # What the pre-5.1 rule would have done, for contrast.
    assert len(path_zone_indices(1, 72)) == 36


def test_the_cone_is_symmetric_and_falls_short_of_the_chassis_by_under_one_slice():
    """The rule's justification is that the cone brackets the chassis at one
    move's travel -- and on a grid this coarse it *nearly* does, which is
    worth pinning rather than rounding away.

    Zones are selected by their CENTRE bearing, so the outermost selected
    zone contributes only to its own edge: on the sim's 8-column
    60-degree grid that edge is 15.00 degrees against a required 15.38, and
    the cone spans 16.1cm where the chassis is 16.5. Selecting by zone
    *edge* instead would pull in the +/-18.75 zones, widening the cone to
    +/-30 degrees -- which reads the walls beside the robot and is the
    corridor failure two tests below already forbid.

    **This shortfall is not new.** The pre-5.1 fraction rule selected the
    identical four zones, so the geometry is unchanged; 5.1 only made the
    selection correct on a sensor whose columns do not span 60 degrees.
    The gap is bounded by half a slice and is M10's to measure against a
    real sensor's real field of view.
    """
    fov, cols = 60, 8
    idx = path_zone_indices(1, cols, fov)
    bearings = [-fov / 2 + fov * (i + 0.5) / cols for i in idx]

    assert bearings == [-b for b in reversed(bearings)], "the cone must be centred"

    edge = max(abs(b) for b in bearings) + (fov / cols) / 2
    span_cm = 2 * PATH_REFERENCE_CM * math.tan(math.radians(edge))
    shortfall = CHASSIS_WIDTH_CM - span_cm
    assert 0 < shortfall < 1.0, (
        f"cone spans {span_cm:.1f}cm at {PATH_REFERENCE_CM}cm against a "
        f"{CHASSIS_WIDTH_CM}cm chassis -- expected a sub-centimetre shortfall")
    assert edge < PATH_HALF_ANGLE_DEG, "selection is by centre, so the edge falls inside"


def test_the_cone_under_covers_the_chassis_at_the_stop_threshold():
    """The honest limit of a fixed-angle cone, recorded so M10 measures it
    rather than rediscovering it.

    The cone is sized at one move's travel (30cm). At the 20cm stop
    threshold the same angles subtend only ~10.7cm, against a 16.5cm
    chassis -- so an obstacle at the chassis corner *at the threshold* sits
    outside the path zones and is not what the veto reads. Pre-existing:
    the fraction rule selected the same zones. The real answer is a cone
    that widens as range shortens, which needs a sensor whose geometry is
    known -- M10, not the grid world.
    """
    fov, cols = 60, 8
    idx = path_zone_indices(1, cols, fov)
    edge = max(abs(-fov / 2 + fov * (i + 0.5) / cols) for i in idx) + (fov / cols) / 2
    at_threshold = 2 * 20.0 * math.tan(math.radians(edge))
    assert at_threshold < CHASSIS_WIDTH_CM, (
        "if this ever passes, the cone geometry changed and M10's note above "
        "needs revisiting")


def test_a_grid_coarser_than_the_cone_still_has_a_path():
    """A 360-degree grid chopped into 8 bins has no zone within the cone at
    all. It must fall back to the zone(s) pointing most nearly ahead rather
    than returning nothing and silently dropping through to the scalar --
    the same promise `max(1, ...)` makes on the fraction branch."""
    idx = path_zone_indices(1, 8, 360)
    assert idx == [3, 4], "the two bins straddling straight ahead"


def test_an_undeclared_field_of_view_still_works_for_a_narrow_sensor():
    """Every backend predating 5.1 omits `fov_deg`, and all of them have
    narrow forward fields where the fraction is a fair proxy. They must
    keep working unchanged."""
    assert path_zone_indices(1, 8, None) == [2, 3, 4, 5]
    assert path_zone_indices(2, 4, None) == [1, 2, 5, 6]
    assert path_zone_indices(1, 8, 0) == [2, 3, 4, 5]


def test_a_wide_grid_without_a_field_of_view_warns_rather_than_guessing(caplog):
    """The silent-fallback class M7 exists to remove. Falling back is the
    right behaviour -- there is nothing else to do -- but doing it quietly
    on a lidar is how the hemisphere bug ships."""
    import robot.safety as safety_module

    safety_module._warned_missing_fov = False
    with caplog.at_level(logging.WARNING, logger="safety"):
        path_zone_indices(1, 72, None)
    assert any("fov_deg" in r.message for r in caplog.records), (
        "a 72-column grid with no declared field of view must say so")


def test_the_simulator_declares_the_field_of_view_it_actually_casts_on():
    """`MockRobot` casts its zones on `renderer.FPV_FOV`. Publishing any
    other number would aim the veto's cone somewhere the rays never went."""
    grid = robot_at(2, 1, Heading.E).get_depth_grid()
    assert grid["fov_deg"] == pytest.approx(math.degrees(renderer.FPV_FOV))
