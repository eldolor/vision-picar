"""
Run with: pytest tests/test_mission_agent.py -v
"""

from sim.mock_robot import MockRobot
from sim.maps.starter_house import build_starter_world
from brain.agent import MissionAgent
from brain.memory import MissionMemory


def test_mission_agent_finds_backpack_in_starter_house():
    world = build_starter_world()  # red backpack lives in the kitchen
    robot = MockRobot(world)
    memory = MissionMemory(mission="Find the red backpack.", target_object="red backpack")
    agent = MissionAgent(robot, memory, min_distance_cm=30)

    report = agent.run_mission(max_steps=100)

    assert report["found"] is True
    assert report["sighting"].object_name == "red backpack"
    # The agent can spot it from up to 3 cells away (frame_description's
    # look-ahead), so it may sight it from an adjacent room before
    # physically entering the kitchen -- that's realistic, not a bug.


def test_mission_agent_stops_once_found_even_if_more_steps_allowed():
    world = build_starter_world()
    robot = MockRobot(world)
    memory = MissionMemory(mission="Find the red backpack.", target_object="red backpack")
    agent = MissionAgent(robot, memory, min_distance_cm=30)

    report = agent.run_mission(max_steps=100)
    steps_when_found = report["steps_taken"]

    # decide() should keep returning STOP once found -- confirm directly.
    assert agent.decide({"safest_direction": "FORWARD"}) == "STOP"
    assert steps_when_found <= 100


def test_mission_agent_reports_failure_when_target_absent():
    world = build_starter_world()
    robot = MockRobot(world)
    memory = MissionMemory(mission="Find the blue umbrella.", target_object="blue umbrella")
    agent = MissionAgent(robot, memory, min_distance_cm=30)

    report = agent.run_mission(max_steps=15)  # too few steps / wrong target

    assert report["found"] is False
    assert report["sighting"] is None


def test_mission_agent_records_rooms_visited_and_searched():
    world = build_starter_world()
    robot = MockRobot(world)
    memory = MissionMemory(mission="Find the red backpack.", target_object="red backpack")
    agent = MissionAgent(robot, memory, min_distance_cm=30)

    agent.run_mission(max_steps=100)

    assert "living room" in memory.visited_rooms
    assert len(memory.actions) == len(agent.history)
