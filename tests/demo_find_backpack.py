"""
demo_find_backpack.py

The second checkpoint demo from the build plan: "find the red backpack"
-- searches, remembers, reports success/failure. Fully autonomous, no
hardware, running entirely against the grid-world.

Run with: python -m tests.demo_find_backpack
"""

from robot.factory import get_robot
from brain.agent import MissionAgent
from brain.memory import MissionMemory


def main():
    robot = get_robot()
    memory = MissionMemory(mission="Find the red backpack.", target_object="red backpack")
    agent = MissionAgent(robot, memory, min_distance_cm=30)

    print(f"=== {memory.mission} ===\n")
    report = agent.run_mission(max_steps=100)

    print(f"Steps taken: {report['steps_taken']}")
    print(f"Rooms visited: {report['rooms_visited']}")
    print(f"Rooms searched: {report['rooms_searched']}\n")

    if report["found"]:
        sighting = report["sighting"]
        print(f"FOUND: {sighting.object_name} in the {sighting.room}, "
              f"at step {sighting.step} (position {sighting.position}).")
    else:
        print(f"NOT FOUND after {report['steps_taken']} steps.")

    print(f"\nMemory summary: {memory.summary()}")


if __name__ == "__main__":
    main()
