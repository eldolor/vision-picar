"""
tests/test_inventory.py -- brain/inventory.py, the object inventory
(PLAN-ros-alignment.md 3.46). Geometry, the belief rules and fusion, on
hand-built scans; the house-scale numbers are tests/inventory_sweep.py's.
"""
import math

import pytest

from brain import inventory as inv
from brain.inventory import Inventory, range_at, scan_returns
from robot.safety import LIDAR_X_M


def _pose(x=0.0, y=0.0, h=0.0, map_id="m1"):
    return {"usable": True, "x_m": x, "y_m": y, "heading_deg": h, "map_id": map_id}


def _scan(returns: dict = None, default=None):
    """A 360-beam scan from the LIDAR; `returns` maps a body-centre bearing
    (deg) to a body-centre range (m). Placed back into the lidar's frame so
    the inventory's own conversion is what is tested."""
    ranges = [default] * 360
    for bearing, r in (returns or {}).items():
        b = math.radians(bearing)
        fwd, right = r * math.cos(b) - LIDAR_X_M, r * math.sin(b)
        a = round(math.degrees(math.atan2(right, fwd)))
        ranges[(a + 180) % 360] = math.hypot(fwd, right)
    return {"usable": True, "angle_min_deg": -180.0, "angle_increment_deg": 1.0,
            "range_min_m": 0.0, "range_max_m": 12.0, "ranges_m": ranges}


def test_scan_is_moved_to_the_body_centre():
    r = range_at(scan_returns(_scan({0: 2.0})), 0.0)
    assert r == pytest.approx(2.0, abs=1e-6)


def test_no_return_is_no_range():
    assert range_at(scan_returns(_scan()), 0.0) is None
    assert scan_returns({"usable": False}) == []


def test_placement_follows_the_compass_convention():
    """Heading 90 faces east (+x); y grows south. A thing dead ahead at
    2 m from (1, 1) is at (3, 1); 90 deg to the right of that is south."""
    i = Inventory()
    i.observe([{"label": "mug", "bearing_deg": 0.0}], _pose(1, 1, 90), _scan({0: 2.0}))
    lm = i.landmarks[0]
    assert (lm.x, lm.y) == pytest.approx((3.0, 1.0), abs=0.01)
    i = Inventory()
    i.observe([{"label": "mug", "bearing_deg": 0.0}], _pose(1, 1, 90),
              _scan({90: 2.0}), pan_deg=90.0)
    lm = i.landmarks[0]
    assert (lm.x, lm.y) == pytest.approx((1.0, 3.0), abs=0.01)


def test_one_pose_is_one_look():
    """3.46 criterion 5's pin: fifty frames from one pose are one hit.
    Red on a per-frame count."""
    i = Inventory()
    for _ in range(50):
        i.observe([{"label": "mug", "bearing_deg": 0.0}], _pose(), _scan({0: 2.0}))
    assert len(i.landmarks) == 1
    assert i.landmarks[0].hits == 1
    assert i.landmarks[0].belief == pytest.approx(inv._sigmoid(inv.L_HIT))
    assert not i.report()["reported"]


def test_two_viewpoints_report_it():
    i = Inventory()
    i.observe([{"label": "mug", "bearing_deg": 0.0}], _pose(0, 0, 90), _scan({0: 2.0}))
    i.observe([{"label": "mug", "bearing_deg": 0.0}], _pose(0.6, 0, 90), _scan({0: 1.4}))
    lm = i.landmarks[0]
    assert lm.hits == 2 and lm.belief >= inv.REPORT_BELIEF
    assert [d["label"] for d in i.report()["reported"]] == ["mug"]


def test_turning_alone_is_a_new_view():
    i = Inventory()
    i.observe([{"label": "mug", "bearing_deg": 0.0}], _pose(0, 0, 90), _scan({0: 2.0}))
    i.observe([{"label": "mug", "bearing_deg": -20.0}], _pose(0, 0, 110),
              _scan({-20: 2.0}))
    assert i.landmarks[0].hits == 1          # 20 deg: the same look
    i.observe([{"label": "mug", "bearing_deg": -10.0}], _pose(0, 0, 100 + 30),
              _scan({-40: 2.0}), pan_deg=-30.0)
    assert len(i.landmarks) == 1 and i.landmarks[0].hits == 2


def _seen_twice():
    i = Inventory()
    i.observe([{"label": "mug", "bearing_deg": 0.0}], _pose(0, 0, 90), _scan({0: 2.5}))
    i.observe([{"label": "mug", "bearing_deg": 0.0}], _pose(0.6, 0, 90), _scan({0: 1.9}))
    return i


def test_one_miss_between_hits_is_forgiven():
    """Amendment 2: a detector that drops a box once must not sink a thing
    seen twice under the reporting bar (sweep 1's recall failure)."""
    i = _seen_twice()
    before = i.landmarks[0].belief
    i.observe([], _pose(1.2, 0, 90), _scan({0: 1.3}))
    assert i.landmarks[0].misses == 0 and i.landmarks[0].belief == before
    assert i.report()["reported"]


def test_two_misses_in_a_row_both_count():
    i = _seen_twice()
    before = i.landmarks[0].score
    i.observe([], _pose(0.6, 0.6, 90), _scan({0: 1.9}))
    i.observe([], _pose(1.2, 0, 90), _scan({0: 1.3}))
    lm = i.landmarks[0]
    assert lm.misses == 2 and lm.score == pytest.approx(before + 2 * inv.L_MISS)


def test_a_hit_clears_a_pending_miss():
    i = _seen_twice()
    i.observe([], _pose(1.2, 0, 90), _scan({0: 1.3}))                     # pending
    i.observe([{"label": "mug", "bearing_deg": 0.0}], _pose(1.2, 0, 90),
              _scan({0: 1.3}))                                             # a hit
    i.observe([], _pose(0.6, 0.6, 90), _scan({0: 1.95}))                  # pending again
    assert len(i.landmarks) == 1 and i.landmarks[0].hits == 3
    assert i.landmarks[0].misses == 0


def test_occluded_is_not_a_miss():
    i = Inventory()
    i.observe([{"label": "mug", "bearing_deg": 0.0}], _pose(0, 0, 90), _scan({0: 2.0}))
    i.observe([], _pose(0.6, 0, 90), _scan({0: 0.5}))    # a chair in between
    assert i.landmarks[0].misses == 0


def test_a_frame_the_detector_skipped_says_nothing():
    i = Inventory()
    i.observe([{"label": "mug", "bearing_deg": 0.0}], _pose(0, 0, 90), _scan({0: 2.0}))
    i.observe(None, _pose(0.6, 0, 90), _scan({0: 1.4}))
    assert i.landmarks[0].misses == 0 and i.frames == 1


def test_out_of_view_is_not_a_miss():
    i = Inventory()
    i.observe([{"label": "mug", "bearing_deg": 0.0}], _pose(0, 0, 90), _scan({0: 2.0}))
    i.observe([], _pose(0.6, 0, 270), _scan({180: 1.4}))
    assert i.landmarks[0].misses == 0


def test_a_wrong_label_is_a_vote_not_an_object():
    i = Inventory()
    for k, label in enumerate(["mug", "mug", "vase"]):
        i.observe([{"label": label, "bearing_deg": 0.0}], _pose(0.6 * k, 0, 90),
                  _scan({0: 2.0 - 0.6 * k}))
    assert len(i.landmarks) == 1
    lm = i.landmarks[0]
    assert lm.label == "mug" and dict(lm.votes) == {"mug": 2, "vase": 1}
    assert lm.disputed


def test_a_long_thing_stays_one_landmark():
    """A sofa's front seen from along its length: points 0.4 m apart chain."""
    i = Inventory()
    for k in range(6):
        i.observe([{"label": "sofa", "bearing_deg": 0.0}], _pose(0.4 * k, 0, 180),
                  _scan({0: 1.0}))
    assert len(i.landmarks) == 1


def test_a_point_between_two_landmarks_bridges_them():
    """3.46 sweep 1's defect: a bed's two ends became two beds because a
    point near both joined the first. Red without `_absorb()`."""
    i = Inventory()
    i.observe([{"label": "bed", "bearing_deg": 0.0}], _pose(0, 0, 0), _scan({0: 2.0}))
    i.observe([{"label": "bed", "bearing_deg": 0.0}], _pose(0.8, 0, 0), _scan({0: 2.0}))
    assert len(i.landmarks) == 2                  # 0.8 m apart: not yet linked
    i.observe([{"label": "bed", "bearing_deg": 0.0}], _pose(0.4, 0, 0), _scan({0: 2.0}))
    assert len(i.landmarks) == 1
    lm = i.landmarks[0]
    assert len(lm.points) == 3 and lm.votes["bed"] == 2
    assert lm.score == pytest.approx(inv.L_HIT)   # the larger, never the sum


def test_a_merge_inside_one_frame_counts_that_look_once():
    """3.47, found by review: one frame's first detection gives A its hit,
    the second matches B and folds A into it. `_absorb` carries A's score
    and hits over, so B taking the hit as well counted one look twice
    (4 hits from 3 looks). Red without the `joined` hand-over in observe()."""
    i = Inventory()
    i.observe([{"label": "sofa", "bearing_deg": 0.0}], _pose(0, 0, 0), _scan({0: 2.0}))
    i.observe([{"label": "sofa", "bearing_deg": 0.0}], _pose(0.8, 0, 0), _scan({0: 2.0}))
    assert len(i.landmarks) == 2                  # B at x=0, A at x=0.8
    # A new viewpoint for both: one detection lands by A, the next between
    # them -- it matches B first and bridges A in.
    i.observe([{"label": "sofa", "bearing_deg": 9.0}, {"label": "sofa", "bearing_deg": 0.0}],
              _pose(0.4, 1.0, 0), _scan({9: 3.04, 0: 3.0}))
    (lm,) = i.landmarks
    assert lm.hits == 3 and lm.votes["sofa"] == 3
    assert lm.score == pytest.approx(2 * inv.L_HIT)   # A's or B's one look, then this one
    # ...and the merged landmark remembers this frame as its last viewpoint,
    # so the same look from the same spot next frame counts nothing.
    i.observe([{"label": "sofa", "bearing_deg": 0.0}], _pose(0.4, 1.0, 0), _scan({0: 3.0}))
    assert i.landmarks[0].hits == 3


def test_a_merge_never_loses_the_bigger_landmarks_own_look():
    """3.47, found by the review of the fixes: when `lm` had the larger
    score and this frame was a new view for it, the merge's max() kept
    `lm`'s old score, so the look was lost. The merged object gets this
    look once -- neither twice nor not at all."""
    i = Inventory()
    # B at (0, -2), seen from two viewpoints 0.6 m apart: two looks.
    i.observe([{"label": "sofa", "bearing_deg": 0.0}], _pose(0, 0, 0), _scan({0: 2.0}))
    i.observe([{"label": "sofa", "bearing_deg": 17.0}], _pose(-0.6, 0, 0), _scan({17: 2.09}))
    (b,) = i.landmarks
    before = b.score
    assert before == pytest.approx(2 * inv.L_HIT)
    # One frame from a new spot: the first detection starts A (0.8 m off,
    # unlinked); the second lands between them and bridges A into B.
    i.observe([{"label": "sofa", "bearing_deg": 9.0}, {"label": "sofa", "bearing_deg": 0.0}],
              _pose(0.4, 1.0, 0), _scan({9: 3.04, 0: 3.0}))
    (lm,) = i.landmarks
    assert lm.score == pytest.approx(min(inv.SCORE_CLAMP, before + inv.L_HIT))


def test_two_things_apart_are_two_landmarks():
    i = Inventory()
    i.observe([{"label": "mug", "bearing_deg": -20.0}, {"label": "shoe", "bearing_deg": 20.0}],
              _pose(0, 0, 0), _scan({-20: 2.0, 20: 2.0}))
    assert sorted(lm.label for lm in i.landmarks) == ["mug", "shoe"]


def test_no_range_is_held_unplaced():
    i = Inventory()
    i.observe([{"label": "mug", "bearing_deg": 0.0}], _pose(), _scan())
    i.observe([{"label": "mug", "bearing_deg": 0.0}], _pose(), _scan({0: 6.0}))
    assert not i.landmarks and i.unplaced == 2


def test_another_maps_pose_is_refused():
    i = Inventory()
    i.observe([{"label": "mug", "bearing_deg": 0.0}], _pose(), _scan({0: 2.0}))
    i.observe([{"label": "shoe", "bearing_deg": 0.0}], _pose(map_id="m2"), _scan({0: 2.0}))
    assert [lm.label for lm in i.landmarks] == ["mug"] and i.no_pose == 1


def test_an_unusable_pose_places_nothing():
    i = Inventory()
    i.observe([{"label": "mug", "bearing_deg": 0.0}], {"usable": False}, _scan({0: 2.0}))
    assert not i.landmarks and i.no_pose == 1


def test_frame_detections_group_cells_into_objects():
    frame = {"detections": [
        {"label": "sofa", "bearing_deg": -10.0, "distance_m": 2.0},
        {"label": "sofa", "bearing_deg": -5.0, "distance_m": 2.0},
        {"label": "sofa", "bearing_deg": 0.0, "distance_m": 2.0},
        {"label": "chair", "bearing_deg": 25.0, "distance_m": 2.0},
        {"label": "chair", "bearing_deg": -25.0, "distance_m": 2.0}]}
    got = inv.frame_detections(frame)
    assert [(d["label"], round(d["bearing_deg"], 1)) for d in got] == [
        ("chair", -25.0), ("chair", 25.0), ("sofa", -5.0)]


def test_a_frame_with_no_detector_is_none_not_empty():
    assert inv.frame_detections({"image_base64": "..."}) is None
    assert inv.frame_detections({"detections": []}) == []


def test_summary_names_what_was_reported():
    i = Inventory()
    i.observe([{"label": "mug", "bearing_deg": 0.0}], _pose(0, 0, 90), _scan({0: 2.0}))
    i.observe([{"label": "mug", "bearing_deg": 0.0}], _pose(0.6, 0, 90), _scan({0: 1.4}))
    assert i.summary().startswith("inventory: 1 reported, 0 candidates from 2 frames -- mug")
