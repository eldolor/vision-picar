"""
tests/test_alb_routes.py

Every public route must have a load-balancer path that reaches it.

Both public services here share one ALB listener and are routed by EXACT
path patterns (see CLAUDE.md section 6 -- the alternative was a second load
balancer, which would have cost about as much as everything else in this
project). The consequence is that adding a route to a FastAPI app is only
half the job: without a matching path pattern in that stack's ListenerRule,
the request falls through to whichever service owns the wildcard and comes
back 404.

That failure has shipped four times -- /app.js, /admin.js,
/recording/finish, /recording/summary -- and each time it looked like a bug
in the page rather than a gap in the infrastructure, because the page loads
fine and one feature is silently dead. It is exactly the sort of mistake a
test should catch and a person should not have to remember.

This walks the real app's routes and the real template's patterns and
asserts they agree. It parses the YAML textually rather than with a CFN
library, because the templates are full of intrinsic functions
(!ImportValue, !Sub) that a plain loader cannot resolve and none of them
appear inside a path pattern.
"""

import re
from pathlib import Path

import pytest

from tests.conftest import REPO_ROOT

CFN = REPO_ROOT / "cloudformation"


def alb_path_patterns(template: Path) -> set:
    """Every path-pattern value in every ListenerRule in a template."""
    text = template.read_text()
    patterns = set()
    for block in re.finditer(r"Field:\s*path-pattern\s*\n\s*Values:\s*\n((?:\s*(?:#[^\n]*|-\s*\S+)\n)+)",
                             text):
        for line in block.group(1).splitlines():
            line = line.strip()
            if line.startswith("- "):
                patterns.add(line[2:].strip())
    return patterns


def covered(route: str, patterns: set) -> bool:
    for p in patterns:
        if p == route:
            return True
        if p.endswith("*") and route.startswith(p[:-1]):
            return True
    return False


def app_routes(app, prefix: str = "") -> set:
    """Concrete GET/POST/PUT/DELETE paths of a FastAPI app, with templated
    segments ({walk}) replaced by a sample value so they can be matched
    against a path pattern."""
    out = set()
    for r in app.routes:
        path = getattr(r, "path", None)
        methods = getattr(r, "methods", set()) or set()
        if not path or not (methods & {"GET", "POST", "PUT", "DELETE"}):
            continue
        if path in ("/openapi.json", "/docs", "/redoc", "/docs/oauth2-redirect"):
            continue
        out.add(re.sub(r"\{[^}]+\}", "sample", path))
    return out


# Routes deliberately NOT given a public path, with the reason. Anything not
# listed here is a bug.
INTENTIONALLY_UNROUTED = {
    # A target group's own health check hits the task IP directly and bypasses
    # listener routing entirely, so /health does not need a public path -- see
    # CLAUDE.md section 6. The twin and teleop-brain DO route theirs, because
    # the twin's UI polls them from the browser; admin's page does not.
    ("admin.yaml", "/health"),
    ("brain.yaml", "/health"),
    # robot/server.py serves two deployments. This route only does anything
    # under mode: teleop, which is the teleop-robot stack -- and that stack
    # does route it, at its own prefix. The twin runs mode: sim, where it
    # would 400 anyway.
    ("twin.yaml", "/teleop/frame"),
}


def unrouted(template: str, routes: set, patterns: set) -> list:
    return sorted(r for r in routes
                  if not covered(r, patterns) and (template, r) not in INTENTIONALLY_UNROUTED)


def test_every_admin_route_is_reachable_through_the_load_balancer():
    from control.admin_server import create_app

    patterns = alb_path_patterns(CFN / "admin.yaml")
    missing = unrouted("admin.yaml", app_routes(create_app()), patterns)
    assert not missing, (
        f"admin routes with no ALB path pattern in cloudformation/admin.yaml: {missing}. "
        "They will 404 in production while working locally."
    )


def test_every_brain_route_is_reachable_through_the_load_balancer():
    from control.brain_server import create_app

    for template, prefix in (("brain.yaml", ""), ("teleop-brain.yaml", "/teleop-brain")):
        patterns = alb_path_patterns(CFN / template)
        routes = {prefix + r for r in app_routes(create_app())}
        missing = unrouted(template, {r.replace(prefix, "", 1) if prefix else r
                                      for r in routes}, 
                           {p.replace(prefix, "", 1) if prefix else p for p in patterns})
        assert not missing, (
            f"brain routes with no ALB path pattern in cloudformation/{template}: {missing}")


def test_the_twins_own_routes_are_reachable_through_the_load_balancer():
    from robot.server import create_app

    patterns = alb_path_patterns(CFN / "twin.yaml")
    missing = unrouted("twin.yaml", app_routes(create_app()), patterns)
    assert not missing, (
        f"twin routes with no ALB path pattern in cloudformation/twin.yaml: {missing}")


def test_the_teleop_robot_deployment_serves_the_same_app_completely():
    """robot/server.py is deployed twice at different prefixes, and a route
    added for one is easy to forget for the other. /teleop-robot/app.js was
    missing exactly that way -- the page rendered and did nothing."""
    from robot.server import create_app

    patterns = alb_path_patterns(CFN / "teleop-robot.yaml")
    routes = {"/teleop-robot" + r for r in app_routes(create_app())}
    missing = sorted(r for r in routes if not covered(r, patterns))
    assert not missing, (
        f"teleop-robot routes with no ALB path pattern: {missing}")


def test_the_checker_would_actually_notice_a_missing_path():
    """Guard against this file quietly passing because its parser found
    nothing: a route that is definitely not in any template must fail."""
    patterns = alb_path_patterns(CFN / "admin.yaml")
    assert patterns, "no path patterns parsed out of admin.yaml -- the parser is broken"
    assert not covered("/definitely-not-routed", patterns)
    assert covered("/recording/walks/sample", patterns)
