"""
demo_active_search.py

Phase 6 (sim) demo: object search using ObjectSearchAgent's active
look_left/look_right/look_center scan on entering each new room, instead
of Phase 4's passive "whatever's in the forward frame" detection.

Run with: python -m tests.demo_active_search
"""

from robot.factory import get_robot
from world.factory import get_world
from brain.agent import ObjectSearchAgent
from brain.memory import MissionMemory


def main():
    robot = get_robot()
    memory = MissionMemory(mission="Find the red backpack.", target_object="red backpack")
    # The world, as robot/server.py builds it: since 3.2 the rule-based
    # policy reads its pose from WorldInterface, and without one it degrades
    # to the right-hand rule -- which is how this demo came to end NOT FOUND
    # (handoff 5a). With it: found in 61 steps, the reference trace.
    agent = ObjectSearchAgent(robot, memory, min_distance_cm=30,
                              world=get_world(robot=robot))

    print(f"=== {memory.mission} (active scanning) ===\n")
    report = agent.run_mission(max_steps=150)

    scan_steps = [r for r in agent.history if r.action in ObjectSearchAgent.SCAN_SEQUENCE]
    print(f"Steps taken: {report['steps_taken']}")
    print(f"Look-around scans performed: {len(scan_steps)} pan actions "
          f"across {len(scan_steps) // 3} room entries")
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
