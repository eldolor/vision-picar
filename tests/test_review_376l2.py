"""
tests/test_review_376l2.py

`PLAN-ros-alignment.md` 3.76, step L2: the bugs the review of the brain's
decision code (`brain/agent.py`, `frontier.py`, `goal_policy.py`,
`goal_pose.py`, `memory.py`, `mission_report.py`, `room_labels.py`,
`rooms.py`, `vision_agent.py`, whole files) confirmed. Each test was written
before its fix and seen to fail on `dev` for the reason it names.
"""

from brain.agent import MissionAgent
from brain.arrival import ARRIVED, SOURCE_PERCEPTION
from brain.memory import MissionMemory
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld

TARGET = "red backpack"


# ---------- L2.1: a refused arrival forgotten after one flicker ----------


def test_a_refused_perception_arrival_is_not_paid_for_twice_at_the_same_place():
    """Only a local-led arrival remembered where it was refused. A
    perception-led one (the default tiered search) cleared the refusal on
    the first frame that was not an arrival -- one detector miss -- and the
    next arrival at the same object bought the cloud's question again."""
    grid = build_world("scaled_house")
    asked = []

    def confirm(frame, local_ok=True):
        asked.append(local_ok)
        return {"confirmed": False, "cloud_called": True, "reason": "not it"}

    agent = MissionAgent(MockRobot(grid, render=False),
                         MissionMemory(mission="m", target_object=TARGET),
                         arrival_confirm_fn=confirm, world=MockWorld(grid))
    readout = {"state": ARRIVED, "source": SOURCE_PERCEPTION, "bearing_deg": 0.0,
               "range_m": 0.35}
    assert agent._confirm_identity(dict(readout), {})["state"] == "refused"
    agent._identity_refused = False                 # one frame the detector missed
    assert agent._confirm_identity(dict(readout), {})["state"] == "refused"
    assert len(asked) == 1, f"{len(asked)} paid questions about one object"


# ---------- L2.2: the cloud's room names, as given ----------


def _with_room_guesses(*guesses):
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    answers = iter(guesses)

    def vision(frame):
        return {"safest_direction": "STOP", "important_objects": [],
                "_navigate": {"room_guess": next(answers)}}

    agent = MissionAgent(robot, MissionMemory(mission="m", target_object=TARGET),
                         vision_fn=vision)
    for _ in guesses:
        frame = robot.get_camera_frame()
        frame.pop("room", None)                      # a real camera's frame
        robot.get_camera_frame = (lambda f=frame: dict(f))
        agent.step()
    return agent.memory


def test_one_room_named_two_ways_is_one_room_and_unclear_is_none():
    """`room_guess` is free text ("short room-type label"); only the exact
    string "unclear" was dropped, so "Living room" and "living room" were two
    rooms in the report and "Unclear" was a room."""
    memory = _with_room_guesses("Living room", "living room", "Unclear", "unknown")
    assert memory.visited_rooms == {"living room"}, memory.visited_rooms


# ---------- L2.5: the stuck-breaker on the frame the target is found ----------


def test_the_found_frame_is_not_turned_away_from():
    """`arrived_scene` answers STOP; after three STOPs the stuck-breaker made
    it an executed RIGHT on the very frame memory then marked found."""
    from brain.agent import ConstrainedAgent, StepResult
    grid = build_world("scaled_house")
    agent = MissionAgent(MockRobot(grid, render=False),
                         MissionMemory(mission="m", target_object=TARGET))
    agent.history = [StepResult(step=i, frame={}, scene={}, action="STOP",
                                executed=True, detail="") for i in range(3)]
    scene = {"safest_direction": "STOP", "_navigate": {"target_reached": True}}
    assert ConstrainedAgent.decide(agent, scene) == "STOP"
