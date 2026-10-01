"""
tests/test_wall_linters.py

The two costs of the ROS wall, measured so they cannot grow unnoticed.

`tests/test_ros_containment.py` keeps ROS *inside* service/slam/. It says
nothing about the price of that: things that must now exist on both sides
of the wall, and a bridge that could quietly grow into a second ROS. Both
were named as the wall's costs on 2026-09-27; this file is their linter.

**1. Things that exist twice.** Every concept that has a definition on
BOTH sides is listed in `DUPLICATES`, with a check that the two agree.
Three rules:

  * a listed duplicate must AGREE across the wall (a check per entry --
    drift between two copies is the actual failure a duplicate causes);
  * an UNLISTED one is detected where it can be: a non-round number that
    appears as a literal in ROS config and as a constant in the project's
    Python is almost always a hand-copied physical constant (wheel radius,
    track width), so it must be listed or it fails;
  * the list has a budget. Raising `MAX_DUPLICATES` is allowed, in a
    commit that says why -- it is meant to be a decision, never drift.

**2. The bridge becoming a thin copy of ROS.** `picar_bridge` is the only
door through the wall, so it is where (b+) would turn into (c):

  * a route budget (`MAX_BRIDGE_ROUTES`), same rule as above;
  * NO generic routes -- a route that takes a topic, service, action,
    node or parameter NAME from the caller is ROS re-exported over HTTP,
    which is exactly "a thin copy", so it is banned outright;
  * every route has a consumer outside the container -- a route nothing
    calls is growth with no reason;
  * every route is listed in the bridge's own docstring, so the door's
    inventory is readable in one place.

All static: nothing here needs ROS or the container.
Run with: pytest tests/test_wall_linters.py -v
"""

import re
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parent.parent
SLAM = ROOT / "service/slam"
CONFIG = SLAM / "src/picar_bringup/config"
XACRO = SLAM / "src/picar_description/urdf/picar.urdf.xacro"
BRIDGE = SLAM / "src/picar_bridge/picar_bridge/bridge.py"

# Where the project's side of a duplicate may live (and where a bridge
# route's consumer may be). tests/ counts as a consumer: R3's /tf and /pan
# are measurement hooks and exist for the tests that use them.
PROJECT_DIRS = ("robot", "sim", "world", "control", "brain")
CONSUMER_DIRS = PROJECT_DIRS + ("tests", "web-twin")

# ---------- budgets: raise deliberately, never to make a test pass ----------

# 2026-09-27: ten concepts, all listed below. An eleventh is a decision.
# 2026-10-01 (3.27): eleven -- the lidar's mounting offset. The URDF must
# carry it (TF places every scan with it) and robot/safety.py must too (it
# judges distance to the chassis off the scan BEFORE ROS, by design, 3.16),
# so it cannot live on one side only.
MAX_DUPLICATES = 11
# 2026-09-27: thirteen method+path pairs (R4-R6). A fourteenth is a decision.
MAX_BRIDGE_ROUTES = 13


# ---------- reading both sides ----------

def _yaml(name):
    return yaml.safe_load((CONFIG / name).read_text())


def _params(name, node):
    return _yaml(name)[node]["ros__parameters"]


def _xacro(name):
    m = re.search(rf'<xacro:property name="{name}"\s+value="([^"]+)"', XACRO.read_text())
    assert m, f"xacro property {name} is gone"
    return float(m.group(1))


def _footprint_xy(text):
    """A nav2 footprint string, or collision_monitor's flat point list, as
    (half_length, half_width)."""
    nums = [abs(float(v)) for v in re.findall(r"-?\d+\.\d+", str(text))]
    return max(nums[0::2]), max(nums[1::2])


# ---------- the registry: every concept that exists on both sides ----------

def _wheel_radius():
    from robot import hardware_robot
    from sim.mock_robot import WHEEL_RADIUS_M
    values = {
        "xacro": _xacro("wheel_radius"),
        "controllers.yaml": _params("controllers.yaml", "diff_drive_controller")["wheel_radius"],
        "sim/mock_robot.py": WHEEL_RADIUS_M,
        "robot/hardware_robot.py": hardware_robot.WHEEL_RADIUS_M,
    }
    return len(set(values.values())) == 1, values


def _wheel_separation():
    from robot import hardware_robot
    from sim.mock_robot import TRACK_WIDTH_M
    values = {
        "xacro": _xacro("wheel_separation"),
        "controllers.yaml": _params("controllers.yaml", "diff_drive_controller")["wheel_separation"],
        "sim/mock_robot.py": TRACK_WIDTH_M,
        "robot/hardware_robot.py": hardware_robot.TRACK_WIDTH_M,
    }
    return len(set(values.values())) == 1, values


def _driver_order():
    """twist_mux's priorities must RANK the drivers the way robot/server.py's
    M4 arbitration does: a person above every autonomous source, and the
    brain and nav2 level (two layers, not rivals)."""
    from robot.interface import DRIVER_PRIORITY
    mux = _params("twist_mux.yaml", "twist_mux")["topics"]
    ros = {"twin-dpad": mux["teleop"]["priority"], "brain": mux["brain"]["priority"],
           "ros": mux["nav"]["priority"]}
    ours = {d: DRIVER_PRIORITY[d] for d in ros}
    def order(p):
        return sorted(p, key=lambda d: (-p[d], d)), {(a, b): p[a] > p[b] for a in p for b in p}
    agree = order(ros)[1] == order(ours)[1]
    return agree, {"twist_mux": ros, "robot/interface.py": ours}


def _control_rate():
    from robot.ros_drive import CONTROL_HZ
    from robot.server import WHEEL_LOOP_INTERVAL_S
    values = {
        "controllers.yaml update_rate": float(_params("controllers.yaml", "controller_manager")["update_rate"]),
        "nav2.yaml controller_frequency": float(_params("nav2.yaml", "controller_server")["controller_frequency"]),
        "robot/server.py wheel loop": round(1.0 / WHEEL_LOOP_INTERVAL_S, 6),
        "robot/ros_drive.py CONTROL_HZ": float(CONTROL_HZ),
    }
    return len(set(values.values())) == 1, values


def _silence_stops():
    """R4 criterion 6: silence stops the wheels within 0.5 s. twist_mux holds
    a silent input for its timeout and diff_drive_controller then waits its
    own, so the two ADD. And each must outlast a couple of periods of the
    20 Hz stream, or ordinary jitter stops a moving robot."""
    from robot.ros_drive import CONTROL_HZ
    mux = max(t["timeout"] for t in _params("twist_mux.yaml", "twist_mux")["topics"].values())
    ddc = _params("controllers.yaml", "diff_drive_controller")["cmd_vel_timeout"]
    period = 1.0 / CONTROL_HZ
    ok = mux + ddc <= 0.5 and min(mux, ddc) >= 2 * period
    return ok, {"twist_mux timeout": mux, "cmd_vel_timeout": ddc, "sum": mux + ddc,
                "stream period": period}


def _lidar_range():
    from sim.mock_robot import LIDAR_RANGE_M
    values = {"slam.yaml max_laser_range": _params("slam.yaml", "slam_toolbox")["max_laser_range"],
              "sim/mock_robot.py LIDAR_RANGE_M": LIDAR_RANGE_M}
    return len(set(values.values())) == 1, values


def _max_speed():
    from sim.mock_robot import WHEEL_MAX_RAD_S, WHEEL_RADIUS_M
    ddc = _params("controllers.yaml", "diff_drive_controller")
    values = {"controllers.yaml linear max": ddc["linear.x.max_velocity"],
              "sim wheel ceiling": round(WHEEL_MAX_RAD_S * WHEEL_RADIUS_M, 6)}
    return abs(values["controllers.yaml linear max"] - values["sim wheel ceiling"]) < 1e-6, values


def _footprint():
    """nav2 plans with, and collision_monitor checks, a rectangle that must be
    the URDF's body: its outer length by its outer width. Since 3.18
    robot/safety.py vets with the same rectangle (and sim/grid_world.py
    collides with it), so its copy is checked here too."""
    from robot.safety import FOOTPRINT_LENGTH_M, FOOTPRINT_WIDTH_M
    nav = _yaml("nav2.yaml")
    shapes = {
        "local_costmap": _footprint_xy(nav["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"]),
        "global_costmap": _footprint_xy(nav["global_costmap"]["global_costmap"]["ros__parameters"]["footprint"]),
        "collision_monitor": _footprint_xy(nav["collision_monitor"]["ros__parameters"]["FootprintApproach"]["points"]),
        "robot/safety.py": (round(FOOTPRINT_LENGTH_M / 2, 6), round(FOOTPRINT_WIDTH_M / 2, 6)),
    }
    urdf = (round(_xacro("body_length") / 2, 6), round(_xacro("body_width") / 2, 6))
    ok = all(abs(s[0] - urdf[0]) < 1e-6 and abs(s[1] - urdf[1]) < 1e-6 for s in shapes.values())
    return ok, {**shapes, "urdf (half length, half width)": urdf}


def _collision_veto():
    """Deliberately two collars in SERIES, safety.py last (R6). What must
    hold: both still exist, and robot/safety.py is still the one that sees
    every path -- the only thing a static check can see is that it is not
    gone."""
    safety = (ROOT / "robot/safety.py").read_text()
    nav = _yaml("nav2.yaml")
    ok = "def vet_wheel_velocity" in safety and "collision_monitor" in nav
    return ok, {"robot/safety.py vet_wheel_velocity": "def vet_wheel_velocity" in safety,
                "nav2.yaml collision_monitor": "collision_monitor" in nav}


def _map_thresholds():
    """ROS's 0-100 occupancy becomes our tri-state in ONE place. A second
    copy of the thresholds anywhere else is the drift this entry exists to
    catch."""
    offenders = []
    for d in PROJECT_DIRS:
        for f in (ROOT / d).rglob("*.py"):
            if f.name == "ros_world.py":
                continue
            if re.search(r"\b(OCCUPIED_AT|FREE_BELOW)\b", f.read_text()):
                offenders.append(str(f.relative_to(ROOT)))
    return not offenders, {"thresholds outside world/ros_world.py": offenders}


def _lidar_offset():
    """The lidar sits ahead of base_link (3.27). TF places every beam with
    the xacro's laser_x; robot/safety.py moves returns into the body frame
    with LIDAR_X_M, and the sim casts from it."""
    from robot.safety import LIDAR_X_M
    values = {"xacro laser_x": _xacro("laser_x"), "robot/safety.py LIDAR_X_M": LIDAR_X_M}
    return len(set(values.values())) == 1, values


DUPLICATES = {
    "lidar mounting offset": _lidar_offset,
    "wheel radius": _wheel_radius,
    "wheel separation": _wheel_separation,
    "driver priority order (M4 vs twist_mux)": _driver_order,
    "control rate (20 Hz)": _control_rate,
    "silence stops the wheels (timeouts)": _silence_stops,
    "lidar range": _lidar_range,
    "linear speed ceiling": _max_speed,
    "chassis footprint": _footprint,
    "collision veto (safety.py + collision_monitor)": _collision_veto,
    "occupancy thresholds (map format)": _map_thresholds,
}

# The value each registered physical constant carries, so the unlisted-
# duplicate detector knows these are accounted for.
REGISTERED_VALUES = {0.172, 0.253, 0.231, 0.1265, 0.1155, 0.0425}


def test_duplicates_are_within_budget():
    assert len(DUPLICATES) <= MAX_DUPLICATES, (
        f"{len(DUPLICATES)} concepts now exist on both sides of the ROS wall "
        f"(budget {MAX_DUPLICATES}). Either remove one side, or raise "
        "MAX_DUPLICATES in a commit that says why.")


@pytest.mark.parametrize("name", sorted(DUPLICATES))
def test_each_duplicate_agrees_across_the_wall(name):
    ok, values = DUPLICATES[name]()
    assert ok, f"'{name}' disagrees across the ROS wall: {values}"


def _is_round(v):
    """0.25, 20, 0.05 -- values chosen, not measured. A hand-copied physical
    constant (0.0325, 0.172) is not round, which is what makes it findable."""
    v = abs(v)
    return v == 0 or abs(v * 20 - round(v * 20)) < 1e-9 or abs(v * 100 - round(v * 100)) < 1e-9


def _ros_literals():
    out = {}
    for f in list(CONFIG.glob("*.yaml")) + [XACRO]:
        for m in re.finditer(r"(?<![\w.])-?(\d+\.\d+)(?![\w.])", f.read_text()):
            v = float(m.group(1))
            if not _is_round(v):
                out.setdefault(round(v, 6), set()).add(f.name)
    return out


def _project_constants():
    out = {}
    for d in PROJECT_DIRS:
        for f in (ROOT / d).rglob("*.py"):
            for m in re.finditer(r"^([A-Z][A-Z0-9_]+)\s*=\s*(-?\d+\.\d+)\s*(#.*)?$",
                                 f.read_text(), re.M):
                v = float(m.group(2))
                if not _is_round(v):
                    out.setdefault(round(abs(v), 6), set()).add(
                        f"{f.relative_to(ROOT)}:{m.group(1)}")
    return out


def test_no_unlisted_physical_constant_is_copied_across_the_wall():
    ros, ours = _ros_literals(), _project_constants()
    shared = {v: (sorted(ros[v]), sorted(ours[v])) for v in ros.keys() & ours.keys()}
    unlisted = {v: w for v, w in shared.items() if v not in REGISTERED_VALUES}
    assert not unlisted, (
        "a non-round value appears in ROS config AND as a project constant -- "
        f"almost certainly a copied physical constant: {unlisted}. Register it "
        "in DUPLICATES with a check that the copies agree (and add it to "
        "REGISTERED_VALUES), or derive one side from the other.")


# ---------- the bridge ----------

def _bridge_routes():
    """(METHOD, path) for every route picar_bridge's handler serves."""
    src = BRIDGE.read_text()
    routes = set()
    for method in ("GET", "POST", "DELETE", "PUT", "PATCH"):
        m = re.search(rf"def do_{method}\(self\).*?(?=\n        def |\n    return Handler)", src, re.S)
        if not m:
            continue
        for p in re.findall(r'path\s*==\s*"(/[^"]*)"', m.group(0)):
            routes.add((method, p))
    return routes


def test_the_route_scanner_sees_the_bridge():
    # A regex that silently matches nothing would pass every test below.
    assert ("POST", "/cmd_vel") in _bridge_routes()
    assert ("GET", "/slam/map") in _bridge_routes()


def test_bridge_routes_are_within_budget():
    routes = _bridge_routes()
    assert len(routes) <= MAX_BRIDGE_ROUTES, (
        f"picar_bridge serves {len(routes)} routes (budget {MAX_BRIDGE_ROUTES}): "
        f"{sorted(routes)}. A growing door is how the wall becomes a thin copy "
        "of ROS -- raise MAX_BRIDGE_ROUTES only in a commit that says why.")


GENERIC_NAMES = r"(topic|service|action|node|param|parameter|msg_type|type)"


def test_bridge_has_no_generic_routes():
    """A route that takes a ROS NAME from the caller re-exports ROS itself.
    Every route must mean one thing in the project's vocabulary."""
    src = BRIDGE.read_text()
    offenders = re.findall(rf'(?:q|query|body|params)\.get\(\s*["\']{GENERIC_NAMES}["\']', src)
    offenders += [p for _, p in _bridge_routes() if re.search(rf"/{GENERIC_NAMES}s?(/|$)", p)
                  and p not in ("/nav/stats",)]
    assert not offenders, f"generic ROS pass-through in picar_bridge: {offenders}"


def test_every_bridge_route_has_a_consumer_outside_the_container():
    corpus = ""
    for d in CONSUMER_DIRS:
        for f in (ROOT / d).rglob("*"):
            if f.suffix in (".py", ".js") and f.is_file():
                corpus += f.read_text(errors="ignore")
    unused = sorted(p for _, p in _bridge_routes() if f'"{p}' not in corpus and f"'{p}" not in corpus
                    and f"{p}?" not in corpus)
    assert not unused, f"bridge routes nothing outside service/slam/ calls: {unused}"


def test_every_bridge_route_is_in_its_docstring():
    doc = BRIDGE.read_text().split('"""')[1]
    missing = sorted(f"{m} {p}" for m, p in _bridge_routes() if p not in doc)
    assert not missing, f"undocumented picar_bridge routes: {missing}"
