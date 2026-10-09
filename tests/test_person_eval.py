"""tests/test_person_eval.py -- 3.52's person-recall scorer, no models."""

from tools import person_eval as pe

W1 = "woven-laundry-basket-20260908-212419"
W5 = "woven-laundry-basket-20260908-215252"


def _all():
    return {(w, n) for w in pe.WALKS for n in range(0, 120)}


def test_the_adjudicated_truth_is_43_person_frames_and_4_excluded():
    assert len(pe.person_frames()) == 43
    assert len(pe.excluded_frames()) == 4
    assert not pe.person_frames() & pe.excluded_frames()


def test_recall_counts_only_verified_hits():
    fired = {(W1, 66): [[0, 0, 1, 1, 0.3]], (W1, 77): [[9, 9, 10, 10, 0.3]]}
    unverified = pe.score(fired, _all())
    assert unverified["person_fired"] == 2 and unverified["person_hits"] == 2
    # 0077's box is on the gaming chair beside the person: not a hit.
    verified = pe.score(fired, _all(), verified_hits={(W1, 66)})
    assert verified["person_hits"] == 1
    assert verified["recall"] == round(1 / 43, 3)


def test_blanket_only_frames_are_neither_hits_nor_false_fires():
    fired = {(W5, 49): [[0, 0, 1, 1, 0.5]], (W5, 50): [[0, 0, 1, 1, 0.5]]}
    s = pe.score(fired, _all())
    assert s["person_fired"] == 0 and s["false_fires"] == 0


def test_yoloe_is_recognised_by_file_name_not_full_path():
    assert pe.is_yoloe("/Users/x/vision-picar/yoloe-11s-seg.pt")
    assert pe.is_yoloe("yoloe-11s-seg.pt")
    assert not pe.is_yoloe("/Users/x/yoloe-dir/yolo11s.pt")


def test_a_box_on_a_person_free_frame_is_a_false_fire():
    s = pe.score({(W1, 0): [[0, 0, 1, 1, 0.1]]}, _all())
    assert s["false_fires"] == 1
    assert s["negative_frames"] == len(_all()) - 43 - 4
