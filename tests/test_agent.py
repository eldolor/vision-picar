"""
Run with: pytest tests/test_agent.py -v
"""

import pytest
from sim.grid_world import GridWorld, Heading
from sim.mock_robot import MockRobot
from brain.agent import ConstrainedAgent, ALLOWED_ACTIONS
from sim.maps.starter_house import build_starter_world


@pytest.fixture
def open_world():
    layout = [
        "##########",
        "#........#",
        "#........#",
        "#........#",
        "##########",
    ]
    return GridWorld(
        layout=layout,
        rooms={"room": {(x, y) for x in range(1, 9) for y in range(1, 4)}},
        robot_x=1,
        robot_y=2,
        heading=Heading.E,
    )


def test_step_returns_allowed_action(open_world):
    robot = MockRobot(open_world)
    agent = ConstrainedAgent(robot)
    result = agent.step()
    assert result.action in ALLOWED_ACTIONS


def test_agent_never_lets_robot_hit_a_wall(open_world):
    robot = MockRobot(open_world)
    agent = ConstrainedAgent(robot, min_distance_cm=30)
    agent.run(max_steps=40)
    # The safety layer should have intercepted every risky FORWARD before
    # grid_world.translate() ever had a chance to report a collision.
    assert not any("BLOCKED" in entry for entry in open_world.log)


def test_agent_explores_starter_house_without_collision():
    world = build_starter_world()
    robot = MockRobot(world)
    agent = ConstrainedAgent(robot, min_distance_cm=30)
    agent.run(max_steps=60)
    assert not any("BLOCKED" in entry for entry in world.log)
    # It should have actually moved, not just sat still the whole time.
    executed_forwards = [
        r for r in agent.history if r.action == "FORWARD" and r.executed
    ]
    assert len(executed_forwards) > 0


def test_stuck_breaker_forces_turn_after_repeated_stops():
    world = build_starter_world()
    robot = MockRobot(world)

    def always_stop(frame):
        return {
            "obstacles_ahead": ["wall"],
            "free_space": "none",
            "doorway_visible": False,
            "important_objects": [],
            "safest_direction": "STOP",
        }

    agent = ConstrainedAgent(robot, vision_fn=always_stop, max_consecutive_stops=2)
    agent.run(max_steps=3)

    assert agent.history[0].action == "STOP"
    assert agent.history[1].action == "STOP"
    assert agent.history[2].action == "RIGHT"


def test_decide_falls_back_to_stop_on_unknown_action(open_world):
    robot = MockRobot(open_world)
    agent = ConstrainedAgent(robot)
    action = agent.decide({"safest_direction": "DIAGONAL_TELEPORT"})
    assert action == "STOP"


def test_history_records_scene_and_frame(open_world):
    robot = MockRobot(open_world)
    agent = ConstrainedAgent(robot)
    agent.run(max_steps=5)
    assert len(agent.history) == 5
    for step in agent.history:
        assert "room" in step.frame
        assert "safest_direction" in step.scene
