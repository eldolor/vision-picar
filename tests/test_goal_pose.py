"""P7c item 2 / P25: a sighting that survives the robot turning."""

import math

import pytest

from brain.goal_pose import GoalPose, OdomTracker, Pose


def drive(tracker, distance_m, heading_deg):
    tracker.update({"usable": True, "distance_m": distance_m,
                    "heading_deg": heading_deg})
    return tracker.pose


# ---------- the failure this exists for ----------

def test_a_bearing_survives_a_pivot_without_a_new_detection():
    """P25's defect, stated as a test.

    An egocentric bearing is wrong the moment the robot turns, so the policy
    needs a fresh detection just to know where the target went -- and on six
    rig walks the command changed every frame. Anchored once, the bearing
    stays correct through a pivot with the detector never running again.
    """
    t = OdomTracker()
    drive(t, 0.0, 0.0)
    goal = GoalPose()
    goal.sight(t.pose, bearing_deg=20.0, range_m=3.0)

    drive(t, 0.0, 20.0)          # pivot onto it. No detection in between.

    assert goal.bearing_from(t.pose) == pytest.approx(0.0, abs=1e-6)
    assert goal.sightings == 1   # still ONE detection, many frames later


def test_driving_toward_the_target_closes_the_range_and_holds_the_bearing():
    t = OdomTracker()
    drive(t, 0.0, 0.0)
    goal = GoalPose()
    goal.sight(t.pose, bearing_deg=0.0, range_m=4.0)

    drive(t, 1.5, 0.0)

    assert goal.bearing_from(t.pose) == pytest.approx(0.0, abs=1e-6)
    assert goal.distance_from(t.pose) == pytest.approx(2.5, abs=1e-6)


def test_driving_past_the_target_puts_it_behind(): 
    """The case a direction-only sighting cannot represent, which is why a
    range is worth having and why `is_point` says whether one was."""
    t = OdomTracker()
    drive(t, 0.0, 0.0)
    goal = GoalPose()
    goal.sight(t.pose, bearing_deg=0.0, range_m=1.0)

    drive(t, 3.0, 0.0)

    assert abs(goal.bearing_from(t.pose)) == pytest.approx(180.0, abs=1e-6)


# ---------- the honest degradation ----------

def test_without_a_range_a_sighting_is_a_direction_and_says_so():
    """Monocular frames carry no metric range (the whole reason
    `bearing-only` shipped), so this is the common case, not the edge."""
    t = OdomTracker()
    drive(t, 0.0, 0.0)
    goal = GoalPose()
    goal.sight(t.pose, bearing_deg=30.0)

    assert goal.held and not goal.is_point
    assert goal.bearing_from(t.pose) == pytest.approx(30.0)
    # Exact under rotation -- which is what breaks a bearing between frames.
    drive(t, 0.0, 30.0)
    assert goal.bearing_from(t.pose) == pytest.approx(0.0, abs=1e-6)


def test_a_direction_has_no_distance_and_returns_None_rather_than_a_number():
    """None is not "far". A caller comparing it to an arrival threshold
    should fail loudly, not stop early -- same argument as
    `unusable_odometry()` and `NO_SENSOR_CM`."""
    t = OdomTracker()
    drive(t, 0.0, 0.0)
    goal = GoalPose()
    goal.sight(t.pose, bearing_deg=10.0)

    assert goal.distance_from(t.pose) is None


# ---------- odometry the backends actually return ----------

def test_unusable_odometry_is_counted_and_never_read_as_standing_still():
    """`ReplayRobot` and `TeleopRobot` return nothing else -- a phone has no
    encoders -- so this is the path the real-pixels walks take. Treating it
    as a zero-distance move would let a policy trust a stale bearing forever
    while believing it had measured the robot staying put."""
    t = OdomTracker()
    drive(t, 2.0, 90.0)
    before = Pose(t.pose.x, t.pose.y, t.pose.heading_deg)

    assert t.update({"usable": False, "distance_m": None,
                     "heading_deg": None}) is None
    assert t.update({}) is None

    assert (t.pose.x, t.pose.y, t.pose.heading_deg) == (
        before.x, before.y, before.heading_deg)
    assert t.dropped == 2 and t.samples == 1


def test_path_length_going_backwards_re_anchors_instead_of_integrating():
    """The interface promises monotonically non-decreasing path length. A
    decrease means a restart or a swapped backend, and integrating a
    negative step would drive the dead-reckoned pose backwards along a
    heading the robot never travelled."""
    t = OdomTracker()
    drive(t, 5.0, 0.0)
    drive(t, 6.0, 0.0)
    x_after = t.pose.x

    drive(t, 1.0, 0.0)           # server restarted

    assert t.pose.x == pytest.approx(x_after)


def test_a_turn_adds_heading_and_no_distance():
    """True on a differential chassis (1.1) and the reason the tracker may
    use the new heading for the segment just travelled."""
    t = OdomTracker()
    drive(t, 1.0, 0.0)
    drive(t, 1.0, 90.0)

    assert t.pose.x == pytest.approx(0.0, abs=1e-6)
    assert t.pose.y == pytest.approx(0.0, abs=1e-6)
    assert t.pose.heading_deg == 90.0


# ---------- drift correction ----------

def test_a_new_sighting_replaces_the_old_one():
    """Re-detection corrects drift rather than being averaged with it: a
    detector that has just seen the target up close is better informed than
    one that saw it across the room, and the older anchor is the stale one."""
    t = OdomTracker()
    drive(t, 0.0, 0.0)
    goal = GoalPose()
    goal.sight(t.pose, bearing_deg=40.0, range_m=5.0)
    goal.sight(t.pose, bearing_deg=0.0, range_m=2.0)

    assert goal.sightings == 2
    assert goal.bearing_from(t.pose) == pytest.approx(0.0, abs=1e-6)
    assert goal.distance_from(t.pose) == pytest.approx(2.0, abs=1e-6)


def test_bearings_wrap_rather_than_accumulating():
    t = OdomTracker()
    drive(t, 0.0, 170.0)
    goal = GoalPose()
    goal.sight(t.pose, bearing_deg=30.0)      # world 200 -> -160

    assert goal.direction_deg == pytest.approx(-160.0)
    assert goal.bearing_from(t.pose) == pytest.approx(30.0, abs=1e-6)


def test_nothing_held_answers_None_rather_than_zero():
    goal = GoalPose()
    assert not goal.held
    assert goal.bearing_from(Pose()) is None
    goal.sight(Pose(), 10.0)
    goal.clear()
    assert goal.bearing_from(Pose()) is None


def test_the_heading_convention_is_not_assumed():
    """`MockRobot` starts at heading 90 and a LEFT turn decreases it, which
    is the opposite of the mathematical convention the integration uses.
    The mismatch mirrors the internal map and must change no answer -- only
    bearings and ranges leave this module."""
    t = OdomTracker()
    drive(t, 0.0, 90.0)                     # the sim's real start heading
    goal = GoalPose()
    goal.sight(t.pose, bearing_deg=25.0, range_m=2.0)

    drive(t, 0.0, 0.0)                      # a left turn, compass-style
    assert goal.bearing_from(t.pose) == pytest.approx(115.0, abs=1e-6)

    drive(t, 0.0, 90.0)                     # back to where it was sighted
    assert goal.bearing_from(t.pose) == pytest.approx(25.0, abs=1e-6)
    assert goal.distance_from(t.pose) == pytest.approx(2.0, abs=1e-6)
