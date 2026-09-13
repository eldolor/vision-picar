"""Every public route must survive BOTH hops to reach its function.

The successor to tests/test_alb_routes.py, and it exists for exactly the
same reason that one does: adding a route to a FastAPI app is only part of
the job, and the failure when you forget the rest is quiet. The page still
loads and one feature is silently dead. That shipped five times against the
load balancer (/app.js, /admin.js, /recording/finish, /recording/summary,
/teleop-robot/app.js).

Serverless does not remove that failure mode, it doubles it. A request now
crosses two routing tables before reaching Python:

    CloudFront behaviour   decides S3-static vs the API origin
                           -> miss, and an API call is answered with the
                              SPA's index.html, i.e. HTML where the client
                              expected JSON
    API Gateway route      decides WHICH of the two functions answers
                           -> miss, and the gateway returns its own
                              {"message":"Not Found"}, which looks like the
                              app 404ing a path it does not have

Both are checked here against the real apps and the real template. The
template is parsed textually rather than with a CloudFormation library
because it is full of intrinsic functions (!Sub, !GetAtt, !ImportValue)
that a plain YAML loader refuses, and none of them appear inside a path
pattern or a route key.
"""

import re
import sys
from pathlib import Path

import pytest

from tests.conftest import REPO_ROOT

# service/vision_analyze is deliberately FLAT -- app.py imports `vision_core`
# as a top-level module, matching its Dockerfile's "COPY app.py vision_core.py
# rooms_core.py ." into one WORKDIR (CLAUDE.md section 6: the duplication and
# the self-contained build context are on purpose). Its own suite does the
# same sys.path insert for the same reason.
#
# tests/test_alb_routes.py never needed this because vision-analyze was the
# ALB's fallthrough and had no explicit patterns to check. Under API Gateway
# it has explicit routes, so the check now applies -- and it is inherently a
# cross-boundary one: the routes live on one side and the template on the
# other. Importing for a test does not affect what the build context contains.
_VISION_DIR = REPO_ROOT / "service" / "vision_analyze"
if str(_VISION_DIR) not in sys.path:
    sys.path.insert(0, str(_VISION_DIR))

TEMPLATE = REPO_ROOT / "cloudformation" / "serverless.yaml"


def api_route_keys() -> set:
    """Every `RouteKey: "METHOD /path"` in the template."""
    text = TEMPLATE.read_text()
    return {m.group(1).strip()
            for m in re.finditer(r'RouteKey:\s*"([^"]+)"', text)}


def cloudfront_patterns() -> set:
    """Every PathPattern in a CacheBehavior."""
    text = TEMPLATE.read_text()
    return {m.group(1) for m in re.finditer(r'PathPattern:\s*"([^"]+)"', text)}


def _flatten(routes) -> list:
    """Every real route, including those behind an `include_router()`.

    FastAPI changed the shape of `app.routes` between 0.136 and 0.141: the
    older version flattens an included router's routes into the app, the
    newer one appends one `_IncludedRouter` wrapper that has no `.path`.
    This function used to `continue` past anything without a `.path`, so
    on the newer FastAPI it silently skipped **every route mounted via
    include_router** -- `/metrics/runs` and `/metrics/summary`, in this
    repo, which made the gateway's `ANY /metrics/{proxy+}` look like an
    orphan pointing at nothing.

    That direction fails loudly. The dangerous direction is the other one:
    this module exists because a route with no gateway entry 404s in
    production, and a blind enumerator would report that everything is
    covered while checking an empty set. Recurse, and keep working on both
    versions -- the repo is currently run under two Pythons with two
    FastAPIs, which is how this hid.
    """
    out = []
    for r in routes or []:
        if getattr(r, "path", None) is not None:
            out.append(r)
            continue
        inner = getattr(r, "original_router", None)
        if inner is not None:
            out.extend(_flatten(getattr(inner, "routes", [])))
    return out


def app_routes(app) -> set:
    """(method, concrete path) for a FastAPI app, templated segments
    replaced by a sample value so they can be matched against a pattern."""
    out = set()
    for r in _flatten(getattr(app, "routes", [])):
        path = getattr(r, "path", None)
        methods = getattr(r, "methods", set()) or set()
        if not path:
            continue
        for m in methods & {"GET", "POST", "PUT", "DELETE"}:
            if path in ("/openapi.json", "/docs", "/redoc", "/docs/oauth2-redirect"):
                continue
            out.add((m, path))
    return out


def test_the_route_enumerator_can_see_through_an_included_router():
    """A guard on the guard. If this returns an empty set the whole module
    passes vacuously, which is worse than any single route being wrong."""
    from fastapi import APIRouter, FastAPI
    app = FastAPI()
    router = APIRouter()

    @router.get("/deep/route")
    async def _deep():
        return {}

    app.include_router(router)
    assert ("GET", "/deep/route") in app_routes(app), (
        "app_routes() cannot see routes mounted via include_router -- see "
        "_flatten() on why that makes this module pass vacuously")


def _matches(path: str, pattern: str) -> bool:
    """CloudFront/API-Gateway style matching: exact, or a trailing wildcard.

    A FastAPI templated segment ({walk}) is treated as matching anything,
    since the pattern has to cover every value it can take.
    """
    concrete = re.sub(r"\{[^}]+\}", "X", path)
    if pattern.endswith(("*", "{proxy+}")):
        stem = pattern.rstrip("*").replace("{proxy+}", "")
        return concrete.startswith(stem)
    return concrete == re.sub(r"\{[^}]+\}", "X", pattern)


def covered_by_gateway(method: str, path: str, keys: set) -> bool:
    for key in keys:
        parts = key.split(None, 1)
        if len(parts) != 2:
            continue
        kmethod, kpath = parts
        if kmethod not in ("ANY", method):
            continue
        if _matches(path, kpath):
            return True
    return False


def covered_by_cloudfront(path: str, patterns: set) -> bool:
    return any(_matches(path, p) for p in patterns)


# Routes deliberately NOT exposed, each with the reason it is safe.
# Anything reaching these tests that is not listed here is a bug.
NOT_PUBLIC = {
    # Two services sit behind one gateway and "/health" can only route to
    # one of them; it goes to vision. The walks service answers the same
    # thing at /recording/health, which IS routed, so control/health.py can
    # still reach each half.
    ("walks", "GET", "/health"),
    # The console's own page and script are static assets now: they are
    # synced to the S3 bucket and served by the default behaviour, not by
    # the function. The FastAPI routes remain so `uvicorn control.admin_server:app`
    # still works locally with no S3 at all.
    ("walks", "GET", "/admin"),
    ("walks", "GET", "/admin.js"),
}


@pytest.fixture(scope="module")
def vision_app():
    from app import app  # flat module -- see the sys.path note above
    return app


@pytest.fixture(scope="module")
def walks_app():
    from control.admin_server import create_app
    return create_app()


def _check(service, app, expect_gateway=True):
    keys, patterns = api_route_keys(), cloudfront_patterns()
    missing_gw, missing_cf = [], []
    for method, path in sorted(app_routes(app)):
        if (service, method, path) in NOT_PUBLIC:
            continue
        if expect_gateway and not covered_by_gateway(method, path, keys):
            missing_gw.append(f"{method} {path}")
        if not covered_by_cloudfront(path, patterns):
            missing_cf.append(f"{method} {path}")
    return missing_gw, missing_cf


def test_every_vision_route_has_a_gateway_route_and_a_cloudfront_behaviour(vision_app):
    gw, cf = _check("vision", vision_app)
    assert not gw, (f"vision routes with no API Gateway RouteKey: {gw}. "
                    "The gateway will answer its own 404.")
    assert not cf, (f"vision routes with no CloudFront PathPattern: {cf}. "
                    "These fall through to the static origin and return index.html.")


def test_every_walks_route_has_a_gateway_route_and_a_cloudfront_behaviour(walks_app):
    gw, cf = _check("walks", walks_app)
    assert not gw, (f"walks routes with no API Gateway RouteKey: {gw}. "
                    "The gateway will answer its own 404.")
    assert not cf, (f"walks routes with no CloudFront PathPattern: {cf}. "
                    "These fall through to the static origin and return index.html.")


def test_the_recording_write_routes_are_present_in_the_walks_service(walks_app):
    """They moved off the brain when the walks left EFS. If this fails, the
    Pi has been handed the job of writing to S3 again -- see
    control/recording_routes.py for why that is the wrong place for it."""
    paths = {p for _m, p in app_routes(walks_app)}
    assert "/recording/frame" in paths
    assert "/recording/finish" in paths


def test_no_gateway_route_points_at_a_path_no_app_serves(vision_app, walks_app):
    """The other direction: a RouteKey with no route behind it is a path
    that 502s instead of 404ing, which is a worse thing to debug."""
    served = {p for _m, p in app_routes(vision_app) | app_routes(walks_app)}
    orphans = []
    for key in api_route_keys():
        _method, kpath = key.split(None, 1)
        if not any(_matches(p, kpath) for p in served):
            orphans.append(key)
    assert not orphans, f"API Gateway routes with no app route behind them: {orphans}"


def test_the_static_default_is_not_shadowed_by_a_wildcard_api_behaviour():
    """A behaviour like "/*" on the API origin would send the SPA itself to
    Lambda. CloudFront picks the most specific match, so this is about not
    writing a pattern broad enough to swallow index.html."""
    for p in cloudfront_patterns():
        assert p != "/*", "a /* behaviour would route the SPA to the API origin"
