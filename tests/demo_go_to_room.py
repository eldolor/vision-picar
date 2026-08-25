"""
demo_go_to_room.py

Phase 5 (sim) demo: "go to the kitchen" -- a semantic, room-level goal
instead of an object search. Same MissionAgent as Phase 4; the only
difference is MissionMemory being given a target_room instead of (or
alongside) a target_object.

Run with: python -m tests.demo_go_to_room
"""

from robot.factory import get_robot
from brain.agent import MissionAgent
from brain.memory import MissionMemory
from brain.rooms import identify_room


def main():
    robot = get_robot()
    memory = MissionMemory(mission="Go to the kitchen.", target_room="kitchen")
    agent = MissionAgent(robot, memory, min_distance_cm=30)

    print(f"=== {memory.mission} ===\n")
    report = agent.run_mission(max_steps=100)

    print(f"Steps taken: {report['steps_taken']}")
    print(f"Rooms visited: {report['rooms_visited']}")

    if report["room_reached"]:
        print(f"\nREACHED target room: {memory.target_room}")
    else:
        print(f"\nDid NOT reach {memory.target_room} within the step budget.")

    # identify_room() is what a real VLM pipeline would use instead of
    # the grid-world's ground-truth room label -- shown here purely to
    # demonstrate it against the final observation, not to drive control flow.
    last_scene = agent.history[-1].scene
    guessed_room = identify_room(last_scene.get("important_objects", []))
    print(f"\n(For comparison: identify_room() guessed '{guessed_room}' "
          f"from the last frame's visible objects -- grid-world's ground "
          f"truth was used for actual navigation, as noted in the plan.)")


if __name__ == "__main__":
    main()
