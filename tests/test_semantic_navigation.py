"""
Run with: pytest tests/test_semantic_navigation.py -v
"""

from sim.mock_robot import MockRobot
from sim.maps.starter_house import build_starter_world
from brain.agent import MissionAgent
from brain.memory import MissionMemory
from tests.conftest import mock_world_for


def test_reaches_target_room_with_no_object_target():
    world = build_starter_world()
    robot = MockRobot(world)
    memory = MissionMemory(mission="Go to the hallway.", target_room="hallway")
    agent = MissionAgent(robot, memory, min_distance_cm=30,
                         world=mock_world_for(robot))

    report = agent.run_mission(max_steps=50)

    assert report["room_reached"] is True
    assert report["mission_complete"] is True
    assert report["found"] is False  # no object target was ever set
    assert "hallway" in memory.visited_rooms


def test_stops_immediately_if_already_in_target_room():
    world = build_starter_world()  # robot starts in the living room
    robot = MockRobot(world)
    memory = MissionMemory(mission="Go to the living room.", target_room="living room")
    agent = MissionAgent(robot, memory, min_distance_cm=30,
                         world=mock_world_for(robot))

    report = agent.run_mission(max_steps=50)

    assert report["room_reached"] is True
    assert report["steps_taken"] == 1  # first observation already satisfies it


def test_can_combine_object_and_room_goals():
    world = build_starter_world()
    robot = MockRobot(world)
    memory = MissionMemory(
        mission="Find the red backpack in the kitchen.",
        target_object="red backpack",
        target_room="kitchen",
    )
    agent = MissionAgent(robot, memory, min_distance_cm=30,
                         world=mock_world_for(robot))

    report = agent.run_mission(max_steps=100)

    # Either goal completes the mission -- whichever happens first.
    assert report["mission_complete"] is True
    assert report["found"] or report["room_reached"]


def test_object_only_mission_unaffected_by_room_goal_logic():
    """Regression check: missions with no target_room behave exactly as
    they did before Phase 5's changes."""
    world = build_starter_world()
    robot = MockRobot(world)
    memory = MissionMemory(mission="Find the red backpack.", target_object="red backpack")
    agent = MissionAgent(robot, memory, min_distance_cm=30,
                         world=mock_world_for(robot))

    report = agent.run_mission(max_steps=100)

    assert report["found"] is True
    assert report["room_reached"] is False  # never set, never triggered
