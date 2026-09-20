"""
tests/test_ros_containment.py

Phase N1 (PLAN-mapping.md) -- the wall around ROS, enforced.

`PLAN-onboard-perception.md` 3.3 chose **(b+)**: ROS 2 as ONE isolated
service exposing a tiny API, "and nothing else in the system knows ROS
exists." The alternative it rejected, (c), is described as a thing that
"swallows the project" -- its own IPC, build system, node lifecycle and
test model.

Nothing about (b+) is self-enforcing. It degrades into (c) one reasonable
shortcut at a time: a TF lookup here because the conversion was awkward,
an `rclpy` import there because the message type was already right. Each
step looks locally sensible and the wall is gone by the end of it.

So the rule is written down once and tested:

    **Nothing outside service/slam/ may import rclpy.**

This is the same shape as the constraint that has kept `control/` clean
since B2 -- `tests/test_brain_server.py` imports the brain in a
subprocess and inspects `sys.modules` -- with one difference in method.
That test can run the import because the modules it forbids are
installed. `rclpy` is NOT installed here and never will be on a laptop:
it lives inside the SLAM container. So this is a SOURCE scan rather than
an import check, which has a useful property -- it fails on the line
someone writes, not on the machine that happens to have ROS.

**Written before any ROS exists**, deliberately. A rule added after the
first violation is a cleanup; a rule added before is a door.

Run with: pytest tests/test_ros_containment.py -v
"""

import re
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent

# The one place ROS is allowed to exist. Does not exist yet -- N6 creates
# it -- and the rule is in force either way, which is the point.
ROS_ALLOWED_PREFIX = "service/slam"

# Packages that only exist inside a ROS installation. `rclpy` is the
# binding itself; the rest are the ones a well-meaning shortcut reaches
# for first, because they carry the message types.
ROS_MODULES = (
    "rclpy",
    "rosbag2_py",
    "tf2_ros",
    "tf2_py",
    "nav_msgs",
    "geometry_msgs",
    "sensor_msgs",
    "std_msgs",
    "nav2_simple_commander",
    "slam_toolbox",
)

# `import x` / `import x.y` / `from x import ...` / `from x.y import ...`
_IMPORT = re.compile(
    r"^\s*(?:from\s+(?P<from>[A-Za-z_][\w.]*)\s+import\b"
    r"|import\s+(?P<import>[A-Za-z_][\w.]*))",
    re.MULTILINE,
)

# Directories with no source of ours in them.
_SKIP_DIRS = {
    ".git",
    ".venv",
    "venv",
    "node_modules",
    "__pycache__",
    ".pytest_cache",
    "recordings",
    "evaluations",
}


def _python_files():
    for path in REPO_ROOT.rglob("*.py"):
        rel = path.relative_to(REPO_ROOT)
        if _SKIP_DIRS & set(rel.parts):
            continue
        yield rel, path


def _imported_roots(source: str):
    for match in _IMPORT.finditer(source):
        module = match.group("from") or match.group("import")
        if module:
            yield module.split(".")[0]


def test_nothing_outside_the_slam_service_imports_ros():
    """(b+)'s entire value over (c), as one assertion.

    If this fails, read the failure as a design question rather than a
    lint error: whatever needed the ROS type either belongs inside
    service/slam/, or the conversion belongs there and the value should
    cross the wall as JSON over HTTP like every other thing in this
    project.
    """
    offenders = []
    for rel, path in _python_files():
        if str(rel).startswith(ROS_ALLOWED_PREFIX):
            continue
        # This file names the modules in order to forbid them.
        if rel == Path("tests/test_ros_containment.py"):
            continue
        source = path.read_text(encoding="utf-8", errors="ignore")
        for root in _imported_roots(source):
            if root in ROS_MODULES:
                offenders.append(f"{rel}: imports {root}")

    assert not offenders, (
        "ROS has leaked out of service/slam/ -- PLAN-mapping.md's containment "
        "rule. (b+) becomes (c) exactly here:\n  " + "\n  ".join(offenders)
    )


def test_the_world_backend_for_ros_is_an_http_client_not_a_node():
    """`world/ros_world.py` is the module most likely to break the rule,
    because it is the one whose job is to talk to ROS. It talks to the
    SLAM service's HTTP API; it does not join the ROS graph.

    Skips until N6 writes it -- a rule that cannot yet be broken still
    belongs here, so the day the file appears it is already guarded.
    """
    path = REPO_ROOT / "world" / "ros_world.py"
    if not path.exists():
        pytest.skip("world/ros_world.py arrives in N6")
    roots = set(_imported_roots(path.read_text(encoding="utf-8")))
    assert not roots & set(ROS_MODULES)


def test_the_repo_has_no_ros_yet_and_this_test_says_so():
    """A canary for the skip above, so 'nothing imports ROS' cannot pass
    vacuously forever without anyone noticing which case they are in.

    `tests/test_serverless_routes.py` was blind to every router-mounted
    route on FastAPI 0.141 and passed vacuously for weeks; the lesson
    taken from it was to make the vacuous case say so out loud.
    """
    slam = REPO_ROOT / ROS_ALLOWED_PREFIX
    if slam.exists():
        pytest.skip("service/slam/ exists -- the rule is now load-bearing")
    assert not slam.exists()
