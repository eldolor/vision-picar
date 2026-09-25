"""
Run with: pytest tests/test_object_search.py -v
"""

from sim.mock_robot import MockRobot
from sim.maps.starter_house import build_starter_world
from brain.agent import ObjectSearchAgent
from brain.memory import MissionMemory
from tests.conftest import mock_world_for


def test_scans_on_first_entry_to_a_room():
    world = build_starter_world()
    robot = MockRobot(world)
    memory = MissionMemory(mission="Find the red backpack.", target_object="red backpack")
    agent = ObjectSearchAgent(robot, memory, min_distance_cm=30,
                         world=mock_world_for(robot))

    # Step 0 is the very first observation in the starting room (living
    # room) -- it should trigger the scan sequence immediately.
    agent.step()
    assert agent.history[0].action == "LOOK_LEFT"
    agent.step()
    assert agent.history[1].action == "LOOK_RIGHT"
    agent.step()
    assert agent.history[2].action == "LOOK_CENTER"


def test_does_not_scan_same_room_twice():
    world = build_starter_world()
    robot = MockRobot(world)
    memory = MissionMemory(mission="Find the red backpack.", target_object="red backpack")
    agent = ObjectSearchAgent(robot, memory, min_distance_cm=30,
                         world=mock_world_for(robot))

    agent.run(max_steps=3)  # burns through the exact first scan sequence
    assert memory.searched_rooms == {"living room"}
    scan_actions_so_far = [r.action for r in agent.history if r.action in ObjectSearchAgent.SCAN_SEQUENCE]
    assert scan_actions_so_far == ["LOOK_LEFT", "LOOK_RIGHT", "LOOK_CENTER"]

    # Run several more steps while still in the living room -- as long
    # as no new room is entered, the scan must not fire again.
    agent.run(max_steps=10)
    scan_actions_total = [r.action for r in agent.history if r.action in ObjectSearchAgent.SCAN_SEQUENCE]
    still_in_living_room = all(
        r.frame.get("room") == "living room" for r in agent.history[3:]
    )
    if still_in_living_room:
        assert scan_actions_total == ["LOOK_LEFT", "LOOK_RIGHT", "LOOK_CENTER"]


def test_does_not_scan_when_no_target_object():
    world = build_starter_world()
    robot = MockRobot(world)
    memory = MissionMemory(mission="Go to the hallway.", target_room="hallway")
    agent = ObjectSearchAgent(robot, memory, min_distance_cm=30,
                         world=mock_world_for(robot))

    agent.step()
    # No target_object set -- should fall straight through to normal
    # frontier navigation, not a LOOK_* scan.
    assert agent.history[0].action not in ObjectSearchAgent.SCAN_SEQUENCE


def test_finds_backpack_and_stops_scan_queue():
    world = build_starter_world()
    robot = MockRobot(world)
    memory = MissionMemory(mission="Find the red backpack.", target_object="red backpack")
    agent = ObjectSearchAgent(robot, memory, min_distance_cm=30,
                         world=mock_world_for(robot))

    report = agent.run_mission(max_steps=150)

    assert report["found"] is True
    # Once found, decide() must not still be mid-scan.
    assert agent._pending_scan == []


def test_scanning_still_respects_safety():
    """LOOK_* actions never move the robot, so they should always
    execute regardless of distance -- confirms the scan doesn't get
    accidentally vetoed."""
    world = build_starter_world()
    robot = MockRobot(world)
    memory = MissionMemory(mission="Find the red backpack.", target_object="red backpack")
    agent = ObjectSearchAgent(robot, memory, min_distance_cm=200,  # deliberately strict
                              world=mock_world_for(robot))

    agent.step()
    assert agent.history[0].executed is True
