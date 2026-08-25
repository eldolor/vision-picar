"""
demo_manual_loop.py

Not an automated test -- a runnable demo of the mock robot + safety
layer working together, standing in for the "explore without hitting
anything" checkpoint demo before any agent/VLM logic exists.

Run with: python -m tests.demo_manual_loop
"""

from robot.factory import get_robot
from robot.safety import SafetyController, SafetyViolation


def main():
    robot = get_robot()
    safety = SafetyController(robot, min_distance_cm=30)

    print("Starting frame:")
    print(robot.get_camera_frame())

    plan = ["FORWARD", "FORWARD", "LEFT", "FORWARD", "FORWARD", "FORWARD"]

    for action in plan:
        print(f"\n>>> Executing: {action}")
        try:
            result = safety.check_and_execute(action)
            print(result)
        except SafetyViolation as e:
            print(f"SAFETY OVERRIDE: {e}")
            continue

        frame = robot.get_camera_frame()
        print(f"Now in: {frame['room']}, free space: {frame['free_space_cells']} cells")
        if frame["objects_visible"]:
            print(f"Objects visible: {frame['objects_visible']}")


if __name__ == "__main__":
    main()
