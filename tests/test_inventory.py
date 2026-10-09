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
    """3.55, found by review: one frame's first detection gives A its hit,
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
    """3.55, found by the review of the fixes: when `lm` had the larger
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


def test_a_merge_counts_the_look_once_in_the_other_order_too():
    """3.55, found by the Thermos pass on the third round: with the OLDER,
    smaller landmark A first in the list, a detection hits A, the next one
    bridges B (larger, a new view from here) into A -- and B's look was
    lost (merged 2L instead of 3L). Each landmark's due is computed before
    the merge now, whichever survives it."""
    i = Inventory()
    # A at (0, -2): one look. B at (0.8, -2): two looks, created after A.
    i.observe([{"label": "sofa", "bearing_deg": 0.0}], _pose(0, 0, 0), _scan({0: 2.0}))
    i.observe([{"label": "sofa", "bearing_deg": 0.0}], _pose(0.8, 0, 0), _scan({0: 2.0}))
    i.observe([{"label": "sofa", "bearing_deg": -17.0}], _pose(1.4, 0, 0), _scan({-17: 2.09}))
    a, b = i.landmarks
    assert (a.score, b.score) == pytest.approx((inv.L_HIT, 2 * inv.L_HIT))
    # A new spot for both: the first detection lands by A only, the second
    # between them, matching A (first in the list) and bridging B in.
    i.observe([{"label": "sofa", "bearing_deg": -9.0}, {"label": "sofa", "bearing_deg": 0.0}],
              _pose(0.4, 1.0, 0), _scan({-9: 3.04, 0: 3.0}))
    (lm,) = i.landmarks
    assert lm.score == pytest.approx(3 * inv.L_HIT)    # B's two looks and this one
    assert lm.hits == 4 and lm.votes["sofa"] == 4      # 1 + 2 before, this frame once


def test_one_frame_is_one_hit_and_one_vote_even_when_it_bridges_itself():
    """3.55, found by the Thermos pass on the fourth round: two detections
    of one object, placed 0.7 m apart, each became a landmark and took a
    hit; a third between them bridged the two, and `_absorb` summed their
    hits and votes -- two looks from one frame (and only in some detection
    orders). The score was already right."""
    for order in ([-10.0, 10.0, 0.0], [0.0, -10.0, 10.0], [10.0, 0.0, -10.0]):
        i = Inventory()
        i.observe([{"label": "stool", "bearing_deg": b} for b in order], _pose(0, 0, 0),
                  _scan({-10: 2.03, 10: 2.03, 0: 2.0}))
        (lm,) = i.landmarks
        assert (lm.hits, dict(lm.votes)) == (1, {"stool": 1}), order
        assert lm.score == pytest.approx(inv.L_HIT), order


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


# ---------- 3.60: the seven open review claims, reproduced ----------

def _two_stools_one_frame():
    """Two detections of one stool, placed 0.7 m apart in ONE frame: two
    landmarks, each counted on that frame (they do not bridge yet)."""
    i = Inventory()
    i.observe([{"label": "stool", "bearing_deg": -10.0}, {"label": "stool", "bearing_deg": 10.0}],
              _pose(0, 0, 0), _scan({-10: 2.03, 10: 2.03}))
    assert len(i.landmarks) == 2
    return i


def test_a_later_frame_join_keeps_one_look_per_frame():
    """3.60 claim 1: landmarks counted separately on one frame and joined on
    a LATER frame summed their hits and votes -- frame 1 counted twice
    (hits 2 from one frame; 3.46 sweep at seed 3463: 2 such landmarks).
    The joining frame is from the same spot, so it adds no look."""
    i = _two_stools_one_frame()
    i.observe([{"label": "stool", "bearing_deg": 0.0}], _pose(0, 0, 0), _scan({0: 2.0}))
    (lm,) = i.landmarks
    assert (lm.hits, dict(lm.votes)) == (1, {"stool": 1})
    assert lm.score == pytest.approx(inv.L_HIT)


def test_no_usable_scan_infers_no_miss():
    """3.60 claim 2: with no scan there is no occlusion test and no
    placement, so nothing seen can join -- every landmark in view took a
    miss. A frame without a scan says nothing about what is there."""
    for scan in ({"usable": False}, _scan()):
        i = _seen_twice()
        for x in (1.2, 1.8):                       # two new views in a row
            i.observe([{"label": "mug", "bearing_deg": 0.0}], _pose(x, 0, 90), scan)
        assert i.landmarks[0].misses == 0, scan.get("usable")


def test_seen_without_a_range_is_not_a_miss():
    """3.60, found by /code-review on claim 2's fix: a usable scan with no
    return at the mug's bearing left the detection unplaced, and the mug
    took a miss for a frame that saw it."""
    i = _seen_twice()
    for x in (1.2, 1.8):
        i.observe([{"label": "mug", "bearing_deg": 0.0}], _pose(x, 0, 90), _scan({90: 1.0}))
    assert i.landmarks[0].misses == 0


def test_a_far_detection_does_not_spare_a_miss_the_lidar_proves():
    """3.60, found by the Thermos bug pass on the fix above: a chair 4.5 m
    behind where a chair WAS (beyond `MAX_RANGE_M`, so unplaced) spared the
    gone one, though the lidar saw straight through its spot."""
    i = Inventory()
    i.observe([{"label": "chair", "bearing_deg": 0.0}], _pose(0, 0, 90), _scan({0: 2.5}))
    i.observe([{"label": "chair", "bearing_deg": 0.0}], _pose(0.6, 0, 90), _scan({0: 1.9}))
    for x in (1.2, 1.8):                            # gone; another chair far behind
        i.observe([{"label": "chair", "bearing_deg": 0.0}], _pose(x, 0, 90),
                  _scan({0: 4.5 + 2.5 - x}))
    assert i.landmarks[0].misses == 2


def test_a_short_scan_reads_as_no_returns():
    """3.60: `scan_returns` keeps its defaults on a scan missing fields."""
    assert scan_returns({"usable": True, "ranges_m": None}) == []
    assert range_at(scan_returns({"usable": True, "ranges_m": [2.0] * 360}), 0.0) is not None


def test_a_resighting_from_near_the_last_hit_clears_a_pending_miss():
    """3.60 claim 3: the pending miss was cleared only by a COUNTED hit, so
    seeing it again from near the last hit's spot left it pending and the
    next miss counted both. Seen in between is not gone."""
    i = _seen_twice()                              # last hit at (0.6, 0)
    i.observe([], _pose(1.2, 0, 90), _scan({0: 1.3}))               # pending
    i.observe([{"label": "mug", "bearing_deg": 0.0}], _pose(0.7, 0, 90),
              _scan({0: 1.8}))                     # seen; not a new view
    i.observe([], _pose(0.6, 0.6, 90), _scan({0: 1.95}))            # one miss
    assert i.landmarks[0].misses == 0


def _see(i, pose, points, label="bed"):
    """Detect `label` at each map point from `pose`, with the scan's returns
    put where those points are."""
    seen = [inv._bearing_to(pose, x, y) for x, y in points]
    i.observe([{"label": label, "bearing_deg": b} for b, _ in seen], pose,
              _scan({round(b): r for b, r in seen}))


def test_a_later_frame_join_keeps_the_newest_view():
    """3.60 claim 5: `_absorb` kept the surviving landmark's own last
    viewpoint and dropped the absorbed one's. A was last seen from (0, 0),
    B from (0.4, 0); once they are one object, (0.55, 0) is 0.15 m from
    that object's last look and is not a new one -- it was counted, judged
    against A's spot alone."""
    i = Inventory()
    _see(i, _pose(0, 0, 0), [(0.0, -2.0)])              # A
    _see(i, _pose(0.4, 0, 0), [(0.8, -2.0)])            # B, 0.8 m off: apart
    assert len(i.landmarks) == 2
    _see(i, _pose(0.2, 0, 0), [(0.4, -2.0)])            # bridges; new to neither
    (lm,) = i.landmarks
    assert lm.hits == 2
    _see(i, _pose(0.55, 0, 0), [(0.4, -2.0)])
    assert i.landmarks[0].hits == 2


def test_a_later_frame_join_keeps_the_newest_miss_view():
    """3.60 claim 5, the miss half: the merged landmark keeps the part's
    NEWER miss viewpoint. A was last missed from (-0.6, 0.6), B later from
    (1.4, 0.6); a miss from (1.5, 0.6) is no new look at the merged object."""
    i = Inventory()
    _see(i, _pose(0, 0, 0), [(0.0, -2.0)])                   # A
    _see(i, _pose(0.8, 0, 0), [(0.8, -2.0)])                 # B; A missed (pending)
    i.observe([], _pose(-0.6, 0.6, 0), _scan({0: 3.0}))     # A missed again: counted; B out of view
    i.observe([], _pose(1.4, 0.6, 0), _scan({0: 3.0}))      # B missed (pending); A out of view
    a, b = i.landmarks
    assert (a.misses, b.misses) == (2, 0) and a._miss_frame < b._miss_frame
    _see(i, _pose(0.4, 0.1, 0), [(0.4, -2.0)])               # bridges; new to neither
    (lm,) = i.landmarks
    assert lm._miss_pose["x_m"] == 1.4
    lm._pending_miss = lm._miss_frame                        # as if one miss were held
    misses = lm.misses
    i.observe([], _pose(1.5, 0.6, 0), _scan({0: 3.0}))
    assert i.landmarks[0].misses == misses


def test_a_merge_never_counts_one_frame_as_hit_and_miss():
    """3.60, found by /code-review: A's miss counted on the frame that
    started B; once they are one object, that frame saw it."""
    i = Inventory()
    _see(i, _pose(0, 0, 0), [(0.0, -2.0)])                   # A
    i.observe([], _pose(-0.6, 0, 0), _scan({0: 3.0}))       # A missed: pending
    _see(i, _pose(0.8, 0, 0), [(0.8, -2.0)])                 # B; A's miss counted
    a, b = i.landmarks
    assert a.misses == 2
    _see(i, _pose(0.4, 0.05, 0), [(0.4, -2.0)])              # bridges
    (lm,) = i.landmarks
    assert not (lm._missed & lm._looks.keys()) and lm.misses == 1

def test_the_nearest_return_in_the_gate_places_it():
    """3.60 claim 4, a DESIGN verdict pinned: the inventory places by the
    NEAREST return within the gate, where arrival takes the median. Here a
    jamb 1 deg off the bearing wins over the object behind it. On seed
    3463 the median raised recall (93.5 -> 95.7%) but cost precision
    (97.1 -> 95.7%) and duplicates (1.08 -> 1.17), so nearest stays."""
    i = Inventory()
    i.observe([{"label": "mug", "bearing_deg": 0.0}], _pose(), _scan({0: 2.0, 1: 0.6, -1: 2.0}))
    assert range_at(scan_returns(_scan({0: 2.0, 1: 0.6, -1: 2.0})), 0.0) == pytest.approx(0.6, abs=1e-6)
    (lm,) = i.landmarks
    assert math.hypot(lm.x, lm.y) == pytest.approx(0.6, abs=0.01)
