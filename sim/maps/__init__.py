"""The houses the simulator can load, by name -- robot/factory.py reads
SIM_MAP and builds one of these; the demos that score against ground truth
read the same variable, so the robot and its judge are in the same house."""


def build_world(name: str = "starter_house"):
    if name == "starter_house":
        from sim.maps.starter_house import build_starter_world
        return build_starter_world()
    if name == "scaled_house":
        from sim.maps.scaled_house import build_scaled_world
        return build_scaled_world()
    if name == "home_first_floor":
        from sim.maps.home_first_floor import build_home_world
        return build_home_world()
    raise ValueError(f"unknown SIM_MAP {name!r} (starter_house | scaled_house | home_first_floor)")
