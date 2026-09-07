"""
tests/test_tunnel_proxy.py

The one-port fan-out that lets a single free-tier tunnel reach both local
services (`service/tunnel/proxy.py`).

Worth testing for a reason this project has already been bitten by: the
routing rule is a string prefix, and a prefix that matches too eagerly
sends robot traffic to the brain, where it 404s. That is the same quiet
class of failure as the ALB path patterns in CLAUDE.md section 6 -- the
page loads, one feature is silently dead.

No network: the upstream client is replaced, so what is under test is the
routing decision and the header handling, not httpx.
"""

import pytest
from fastapi.testclient import TestClient

from service.tunnel import proxy as tunnel_proxy


class FakeResponse:
    def __init__(self, url, headers):
        self.status_code = 200
        self.content = b'{"ok":true}'
        self.headers = {"content-type": "application/json",
                        "content-length": "11", "x-echo-url": url,
                        "x-echo-host": headers.get("host", "<absent>"),
                        "x-echo-secret": headers.get("x-app-secret", "<absent>")}


@pytest.fixture
def client(monkeypatch):
    seen = {}

    class FakeClient:
        async def request(self, method, url, content=None, headers=None):
            seen["method"], seen["url"] = method, url
            seen["headers"] = dict(headers or {})
            seen["body"] = content
            return FakeResponse(url, headers or {})

    monkeypatch.setattr(tunnel_proxy, "client", FakeClient())
    return TestClient(tunnel_proxy.app), seen


def test_brain_paths_go_to_the_brain(client):
    c, seen = client
    c.get("/brain/mission/status")
    assert seen["url"] == tunnel_proxy.BRAIN + "/brain/mission/status"


def test_the_bare_brain_prefix_goes_to_the_brain(client):
    """`/brain` with no trailing segment is the health check the twin makes
    when you paste the URL into Settings."""
    c, seen = client
    c.get("/brain")
    assert seen["url"] == tunnel_proxy.BRAIN + "/brain"


def test_everything_else_goes_to_the_robot(client):
    c, seen = client
    for path in ("/health", "/frame", "/action", "/depth", "/teleop/frame"):
        c.get(path)
        assert seen["url"] == tunnel_proxy.ROBOT + path, path


def test_a_path_that_merely_starts_with_the_word_is_not_the_brain(client):
    """`/brainstem` is not `/brain/`. A prefix test written as a bare
    startswith() sends this to the wrong service, where it 404s -- and the
    twin reports "brain unreachable" about a route the robot owns."""
    c, seen = client
    c.get("/brainstem")
    assert seen["url"] == tunnel_proxy.ROBOT + "/brainstem"


def test_the_query_string_survives(client):
    c, seen = client
    c.get("/frame?full=1")
    assert seen["url"].endswith("/frame?full=1")


def test_the_secret_is_forwarded_but_the_tunnel_host_is_not(client):
    """The secret has to reach the upstream or every call 401s. The Host
    must NOT: it is the tunnel's, and forwarding it makes uvicorn's own
    redirects point back out through the tunnel."""
    c, seen = client
    c.get("/health", headers={"x-app-secret": "s3cret"})
    assert seen["headers"].get("x-app-secret") == "s3cret"
    assert "host" not in {k.lower() for k in seen["headers"]}


def test_a_post_body_is_passed_through(client):
    c, seen = client
    c.post("/brain/mission/start", json={"target_object": "red backpack"})
    assert b"red backpack" in seen["body"]
    assert seen["method"] == "POST"


def test_an_unreachable_upstream_is_a_502_naming_it(monkeypatch):
    """Not a 500. Which half is down is the first thing you need to know,
    and the twin renders this detail verbatim."""
    import httpx

    class DeadClient:
        async def request(self, *a, **k):
            raise httpx.ConnectError("connection refused")

    monkeypatch.setattr(tunnel_proxy, "client", DeadClient())
    resp = TestClient(tunnel_proxy.app).get("/brain/health")
    assert resp.status_code == 502
    assert tunnel_proxy.BRAIN in resp.json()["detail"]
