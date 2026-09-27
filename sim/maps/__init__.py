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
    raise ValueError(f"unknown SIM_MAP {name!r} (starter_house | scaled_house)")
