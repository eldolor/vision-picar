"""
demo_explore.py

The first real checkpoint demo from the build plan: "explore this room
without hitting anything" -- fully autonomous, no hardcoded plan, no
hardware, running entirely against the grid-world.

Run with: python -m tests.demo_explore
"""

import logging
from robot.factory import get_robot
from brain.agent import ConstrainedAgent

logging.basicConfig(level=logging.INFO, format="%(message)s")


def main():
    robot = get_robot()
    agent = ConstrainedAgent(robot, min_distance_cm=30)

    print("=== Autonomous explore demo (Phase 2 sim) ===\n")
    history = agent.run(max_steps=25)

    print("\n=== Summary ===")
    action_counts = {}
    for step in history:
        action_counts[step.action] = action_counts.get(step.action, 0) + 1
    for action, count in sorted(action_counts.items()):
        print(f"{action}: {count}")

    rooms_visited = {step.frame["room"] for step in history}
    print(f"\nRooms visited: {rooms_visited}")

    collisions = [r for r in history if not r.executed]
    print(f"Safety interventions: {len(collisions)}")
    print("No hardware, no wall strikes -- ready for Phase 3 (formalize safety) "
          "and Phase 4 (agent harness / mission memory).")


if __name__ == "__main__":
    main()
