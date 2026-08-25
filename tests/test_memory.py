"""
Run with: pytest tests/test_memory.py -v
"""

from brain.memory import MissionMemory


def test_records_visited_room():
    memory = MissionMemory(mission="Find a red backpack.", target_object="red backpack")
    frame = {"room": "kitchen", "position": (10, 7)}
    scene = {"important_objects": []}
    memory.record_observation(step=0, frame=frame, scene=scene)
    assert "kitchen" in memory.visited_rooms
    assert not memory.found


def test_unknown_room_not_recorded():
    memory = MissionMemory(mission="Explore.", target_object=None)
    frame = {"room": "unknown", "position": (4, 2)}
    memory.record_observation(step=0, frame=frame, scene={"important_objects": []})
    assert memory.visited_rooms == set()


def test_finds_target_object_case_insensitive():
    memory = MissionMemory(mission="Find a red backpack.", target_object="Red Backpack")
    frame = {"room": "kitchen", "position": (10, 7)}
    scene = {"important_objects": ["red backpack"]}
    memory.record_observation(step=5, frame=frame, scene=scene)
    assert memory.found is True
    assert memory.found_sighting.room == "kitchen"
    assert memory.found_sighting.step == 5


def test_does_not_match_unrelated_object():
    memory = MissionMemory(mission="Find a red backpack.", target_object="red backpack")
    scene = {"important_objects": ["refrigerator", "sofa"]}
    memory.record_observation(step=0, frame={"room": "kitchen"}, scene=scene)
    assert memory.found is False


def test_found_stays_true_once_set():
    memory = MissionMemory(mission="Find a red backpack.", target_object="red backpack")
    memory.record_observation(
        step=0, frame={"room": "kitchen"}, scene={"important_objects": ["red backpack"]}
    )
    memory.record_observation(step=1, frame={"room": "hallway"}, scene={"important_objects": []})
    assert memory.found is True
    assert memory.found_sighting.step == 0  # first sighting preserved


def test_summary_before_any_room_searched():
    memory = MissionMemory(mission="Find a red backpack.", target_object="red backpack")
    assert memory.summary() == "No rooms searched yet."


def test_summary_after_search_no_target():
    memory = MissionMemory(mission="Find a red backpack.", target_object="red backpack")
    memory.mark_room_searched("living room")
    assert memory.summary() == "Living room searched. No red backpack found."


def test_summary_after_found():
    memory = MissionMemory(mission="Find a red backpack.", target_object="red backpack")
    memory.mark_room_searched("kitchen")
    memory.record_observation(
        step=0, frame={"room": "kitchen"}, scene={"important_objects": ["red backpack"]}
    )
    assert "Found." in memory.summary()


def test_as_context_matches_harness_format():
    memory = MissionMemory(mission="Find a red backpack.", target_object="red backpack")
    memory.mark_room_searched("living room")
    context = memory.as_context(
        current_observation="Hallway with doorway on right.",
        available_tools=["look_left", "look_right", "forward", "turn_right", "stop"],
    )
    assert "MISSION: Find a red backpack." in context
    assert "MEMORY: Living room searched." in context
    assert "CURRENT OBSERVATION: Hallway with doorway on right." in context
    assert "AVAILABLE TOOLS: look_left(), look_right()" in context
