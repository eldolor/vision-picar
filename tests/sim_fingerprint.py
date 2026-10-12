"""A fingerprint of what the simulator's sensors report on a fixed drive
(PLAN-ros-alignment.md 3.74 criterion A1).

The drive is seeded, so the same code gives the same digest; a change to
how the sim senses its world changes it. 3.74 pins the digests taken on
`dev` before object heights existed, so a house with no heights must sense
exactly what it did."""

import hashlib
import json
import random

HOUSES = ("starter_house", "scaled_house", "home_first_floor", "complex_house")


def fingerprint(house: str, steps: int = 500, seed: int = 374) -> str:
    from sim.fake_lidar import world_caster
    from sim.maps import build_world
    from sim.mock_robot import MockRobot
    from sim.mock_world import MockWorld

    robot = MockRobot(build_world(house), render=False)
    world = MockWorld(robot.world)
    lidar = world_caster(robot.world)
    rng = random.Random(seed)
    digest = hashlib.sha256()
    verbs = (lambda: robot.drive_forward(duration=0.5), lambda: robot.turn_left(30),
             lambda: robot.turn_right(45), lambda: robot.look_left(), lambda: robot.look_center())
    for i in range(steps):
        rng.choice(verbs)()
        grid = {k: v for k, v in robot.get_depth_grid().items() if k != "floor"}
        reading = {"grid": grid, "distance": robot.get_distance(),
                   "near": robot.get_scan(max_range_m=0.6)["ranges_m"],
                   "detections": robot.world.frame_description()["detections"]}
        if i % 50 == 0:
            reading["scan"] = robot.get_scan()["ranges_m"]
            reading["lidar"] = [lidar(a, 0.0) for a in range(0, 360, 2)]
            world.observe()
            reading["map"] = world.get_map()
        digest.update(json.dumps(reading, sort_keys=True).encode())
    return digest.hexdigest()


if __name__ == "__main__":
    for house in HOUSES:
        print(house, fingerprint(house))
