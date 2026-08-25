"""
Run with: pytest tests/test_rooms.py -v
"""

from brain.rooms import identify_room


def test_identifies_kitchen_from_features():
    assert identify_room(["refrigerator", "white cabinets"]) == "kitchen"


def test_identifies_living_room_from_features():
    assert identify_room(["sofa", "television"]) == "living room"


def test_returns_unknown_for_no_objects():
    assert identify_room([]) == "unknown"


def test_returns_unknown_for_unrecognized_objects():
    assert identify_room(["mysterious artifact"]) == "unknown"


def test_matching_is_case_insensitive():
    assert identify_room(["REFRIGERATOR", "White Cabinets"]) == "kitchen"


def test_partial_substring_match():
    # "kitchen sink" should still register a hit against feature "sink"
    assert identify_room(["kitchen sink", "oven"]) == "kitchen"


def test_picks_highest_scoring_room_on_mixed_signals():
    # Two kitchen features vs one living-room feature -- kitchen should win.
    assert identify_room(["refrigerator", "oven", "sofa"]) == "kitchen"
