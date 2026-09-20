"""A function that cannot import is a 500 with nothing useful in the log.

This is the dependency version of tests/test_serverless_routes.py, and it
exists because the first deploy of the vision function failed exactly this
way. `service/lambda/requirements-vision.txt` had been hand-written as
fastapi/pydantic/mangum, which missed `pillow-heif` -- the iPhone HEIC
decoder that `service/vision_analyze/app.py` imports at module scope. The
function never started:

    [ERROR] Runtime.ImportModuleError: Unable to import module
            'vision_handler': No module named 'pillow_heif'

From the outside that is `{"message":"Internal Server Error"}` and a 500,
which looks like an application bug rather than a packaging one.

The vision half is now derived rather than listed -- build.sh installs the
service's own requirements.txt -- so the check there is that the derivation
is still wired up. The walks half has no requirements file of its own to
derive from (control/ is covered by the repo-root requirements, which
include pytest and playwright), so its list is curated and this test is what
keeps it honest.
"""

import ast
import sys
from pathlib import Path

import pytest

from tests.conftest import REPO_ROOT

LAMBDA = REPO_ROOT / "service" / "lambda"

# Import name -> distribution name, where they differ.
DISTRIBUTION = {
    "yaml": "pyyaml",
    "PIL": "pillow",
    "pillow_heif": "pillow-heif",
}

# Provided by the Lambda runtime image, deliberately not bundled: shipping a
# copy costs ~10MB for something usually older than AWS ships.
IN_RUNTIME = {"boto3", "botocore"}

# First-party packages. Not "third party" and so never a requirements
# line -- `world` joined the list when N1 split world state out of
# RobotInterface and control/remote_world.py started importing it.
LOCAL = {"control", "robot", "brain", "sim", "world", "tests", "service",
         "app", "vision_core", "rooms_core"}


def third_party_imports(paths):
    mods = set()
    for p in paths:
        tree = ast.parse(p.read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for a in node.names:
                    mods.add(a.name.split(".")[0])
            elif isinstance(node, ast.ImportFrom):
                if node.level == 0 and node.module:
                    mods.add(node.module.split(".")[0])
    return {m for m in mods
            if m not in sys.stdlib_module_names and m not in LOCAL}


def listed(requirements: Path) -> set:
    out = set()
    for line in requirements.read_text().splitlines():
        line = line.split("#")[0].strip()
        if line:
            # strip extras and version pins: uvicorn[standard]>=1 -> uvicorn
            out.add(line.split("[")[0].split("=")[0].split(">")[0]
                        .split("<")[0].strip().lower())
    return out


def test_every_third_party_import_in_control_is_packaged_for_the_walks_function():
    needed = third_party_imports(sorted((REPO_ROOT / "control").glob("*.py")))
    have = listed(LAMBDA / "requirements-walks.txt")
    missing = sorted(
        DISTRIBUTION.get(m, m) for m in needed
        if m not in IN_RUNTIME and DISTRIBUTION.get(m, m).lower() not in have)
    assert not missing, (
        f"control/ imports these but requirements-walks.txt does not list them: "
        f"{missing}. The function will fail at import with "
        f"Runtime.ImportModuleError and answer 500.")


def test_the_vision_function_installs_the_services_own_requirements():
    """Derived, not duplicated. A second copy of this list is what shipped
    a function that could not import."""
    build = (LAMBDA / "build.sh").read_text()
    code = "\n".join(ln for ln in build.splitlines()
                     if not ln.lstrip().startswith("#"))
    assert "service/vision_analyze/requirements.txt" in code, (
        "build.sh must install the vision service's own requirements rather "
        "than a hand-maintained copy")


def test_the_vision_services_requirements_still_cover_its_imports():
    """Guards the file build.sh now derives from -- if the service grows an
    import without updating its own requirements, the container deploy and
    the Lambda deploy break together, and this says so first."""
    needed = third_party_imports(
        sorted((REPO_ROOT / "service" / "vision_analyze").glob("*.py")))
    have = listed(REPO_ROOT / "service" / "vision_analyze" / "requirements.txt")
    missing = sorted(
        DISTRIBUTION.get(m, m) for m in needed
        if m not in IN_RUNTIME and DISTRIBUTION.get(m, m).lower() not in have)
    assert not missing, (
        f"service/vision_analyze imports these but its requirements.txt does "
        f"not list them: {missing}")


@pytest.mark.parametrize("name", ["requirements-vision.txt", "requirements-walks.txt"])
def test_mangum_is_packaged_for_both_functions(name):
    """The ASGI-to-Lambda adapter. Without it the handler module itself is
    the thing that cannot import."""
    assert "mangum" in listed(LAMBDA / name)


def test_boto3_is_not_bundled():
    """It is in the runtime image. Bundling it is ~10MB of package for a copy
    that is usually older than the one AWS ships."""
    for name in ("requirements-vision.txt", "requirements-walks.txt"):
        assert "boto3" not in listed(LAMBDA / name)
