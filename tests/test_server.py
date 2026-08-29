"""
Run with: pytest tests/test_server.py -v
"""

import pytest
from fastapi.testclient import TestClient
from robot.server import create_app, watchdog_should_stop


def make_client():
    app = create_app()
    return TestClient(app)


@pytest.fixture
def teleop_config(tmp_path):
    """A robot.yaml pointed at mode: teleop (PLAN-teleop-robot.md), for
    tests that need TeleopRobot rather than the default MockRobot."""
    config = tmp_path / "robot.yaml"
    config.write_text("mode: teleop\n")
    return str(config)


def make_teleop_client(teleop_config):
    return TestClient(create_app(teleop_config))


def test_forward_action_executes_when_clear():
    with make_client() as client:
        resp = client.post("/action", json={"action": "FORWARD", "speed": 50, "duration": 0.5})
        assert resp.status_code == 200
        body = resp.json()
        assert body["executed"] is True
        assert body["result"]["action"] == "drive_forward"


def test_unknown_action_returns_400():
    with make_client() as client:
        resp = client.post("/action", json={"action": "FLY"})
        assert resp.status_code == 400


def test_stop_endpoint_always_works():
    with make_client() as client:
        resp = client.post("/stop")
        assert resp.status_code == 200
        assert resp.json()["executed"] is True


def test_distance_endpoint():
    with make_client() as client:
        resp = client.get("/distance")
        assert resp.status_code == 200
        assert "distance_cm" in resp.json()


def test_frame_endpoint():
    with make_client() as client:
        resp = client.get("/frame")
        assert resp.status_code == 200
        assert "room" in resp.json()


def test_teleop_frame_round_trips_through_frame_endpoint(teleop_config):
    """Proves T2's whole point: GET /frame needs no changes at all -- it's
    already just robot.get_camera_frame(), whatever the backend is."""
    with make_teleop_client(teleop_config) as client:
        pushed = client.post(
            "/teleop/frame", json={"image_base64": "BASE64DATA", "media_type": "image/jpeg"}
        )
        assert pushed.status_code == 200
        assert pushed.json()["received"] is True

        pulled = client.get("/frame")
        assert pulled.status_code == 200
        assert pulled.json()["image_base64"] == "BASE64DATA"


def test_teleop_frame_rejected_when_not_in_teleop_mode():
    """A mode: sim (or hardware) server has nothing to push a frame into
    -- must fail loudly, not silently drop it."""
    with make_client() as client:
        resp = client.post(
            "/teleop/frame", json={"image_base64": "BASE64DATA"}
        )
        assert resp.status_code == 400


def test_frame_endpoint_reports_a_stalled_teleop_robot_clearly(teleop_config):
    """Found via a real deployment: an uncaught TeleopStall reached callers
    as an opaque 500, and RemoteRobot's error formatting swallowed the
    actual message by the time it reached a mission's log. GET /frame must
    translate any get_camera_frame() failure into a specific response --
    without importing TeleopStall itself, so this file stays backend-
    agnostic (see the route's own comment)."""
    with make_teleop_client(teleop_config) as client:
        resp = client.get("/frame")
        assert resp.status_code == 503
        assert "no frame has ever been pushed" in resp.json()["detail"]


def test_route_prefix_env_var_prepends_every_route(monkeypatch):
    """PLAN-teleop-robot.md: a second robot/server.py instance shares the
    twin's load balancer by claiming a distinct path prefix instead of the
    twin's already-claimed literal paths. Unset (all tests above), this
    changes nothing -- confirmed here rather than assumed."""
    monkeypatch.setenv("ROUTE_PREFIX", "/teleop-robot")
    with make_client() as client:
        assert client.get("/teleop-robot/").status_code == 200
        assert client.get("/teleop-robot/health").status_code == 200
        assert client.post("/teleop-robot/stop").status_code == 200
        # The bare, unprefixed paths must not also work -- otherwise this
        # instance would collide with the twin's own ListenerRule claims.
        assert client.get("/health").status_code == 404
        assert client.post("/stop").status_code == 404


def test_health_endpoint_reports_command_age():
    with make_client() as client:
        client.post("/stop")
        resp = client.get("/health")
        body = resp.json()
        assert body["status"] == "ok"
        assert body["seconds_since_last_command"] < 1.0
        assert "watchdog_timeout_s" in body
        assert body["mode"] == "sim"


def test_health_reports_teleop_mode(teleop_config):
    """A client (Robot view's "drive via brain" start check) needs to tell
    a mode: sim server apart from a mode: teleop one -- both answer /health
    the same otherwise."""
    with make_teleop_client(teleop_config) as client:
        assert client.get("/health").json()["mode"] == "teleop"


def test_safety_blocks_forward_over_wifi_same_as_local_agent():
    """A human driving over Wi-Fi gets the exact same collision
    protection an AI decision does -- drive straight at the wall
    repeatedly and confirm the safety layer eventually vetoes it."""
    with make_client() as client:
        blocked = False
        for _ in range(15):
            resp = client.post(
                "/action", json={"action": "FORWARD", "speed": 100, "duration": 1.0}
            )
            if resp.json()["executed"] is False:
                blocked = True
                break
        assert blocked is True


def test_watchdog_should_stop_pure_logic():
    assert watchdog_should_stop(last_command_at=0.0, now=2.0, timeout_s=1.0) is True
    assert watchdog_should_stop(last_command_at=0.0, now=0.5, timeout_s=1.0) is False
    assert watchdog_should_stop(last_command_at=10.0, now=10.5, timeout_s=1.0) is False


def test_protected_routes_require_secret_when_set(monkeypatch):
    """require_secret() must be inert when APP_SHARED_SECRET is unset (all
    tests above rely on that), but once set, movement/sensing routes must
    reject requests with no or the wrong x-app-secret header -- this is
    what makes it safe to deploy this server publicly (service/twin/)."""
    monkeypatch.setenv("APP_SHARED_SECRET", "correct-horse-battery-staple")
    with make_client() as client:
        no_header = client.post("/action", json={"action": "STOP"})
        assert no_header.status_code == 401

        wrong_header = client.post(
            "/action", json={"action": "STOP"}, headers={"x-app-secret": "wrong"}
        )
        assert wrong_header.status_code == 401

        bodies = {"/action": {"action": "STOP"}, "/teleop/frame": {"image_base64": "X"}}
        for method, path in [
            ("post", "/action"), ("post", "/stop"), ("get", "/distance"),
            ("get", "/frame"), ("post", "/teleop/frame"),
        ]:
            resp = getattr(client, method)(
                path,
                **({"json": bodies[path]} if path in bodies else {}),
                headers={"x-app-secret": "wrong"},
            )
            assert resp.status_code == 401, f"{path} did not reject a wrong secret"


def test_protected_routes_accept_correct_secret(monkeypatch):
    monkeypatch.setenv("APP_SHARED_SECRET", "correct-horse-battery-staple")
    with make_client() as client:
        resp = client.post(
            "/action",
            json={"action": "STOP"},
            headers={"x-app-secret": "correct-horse-battery-staple"},
        )
        assert resp.status_code == 200


def test_health_and_root_stay_open_even_when_secret_set(monkeypatch):
    """The ALB health check can't send custom headers, and the page has to
    load before a user can enter the secret in the UI -- both / and
    /health must stay reachable with no header at all."""
    monkeypatch.setenv("APP_SHARED_SECRET", "correct-horse-battery-staple")
    with make_client() as client:
        assert client.get("/health").status_code == 200
        assert client.get("/").status_code == 200


def test_cors_headers_present_for_browser_requests():
    """The web twin calls this API from a browser -- confirm the preflight
    and actual response both carry CORS headers, or fetch() from Safari
    will silently fail with no useful error in the twin's UI."""
    with make_client() as client:
        preflight = client.options(
            "/action",
            headers={
                "origin": "http://localhost:5500",
                "access-control-request-method": "POST",
            },
        )
        assert preflight.status_code in (200, 204)
        assert "access-control-allow-origin" in {k.lower() for k in preflight.headers.keys()}

        resp = client.post(
            "/action",
            json={"action": "STOP"},
            headers={"origin": "http://localhost:5500"},
        )
        assert "access-control-allow-origin" in {k.lower() for k in resp.headers.keys()}


def test_sensing_does_not_feed_the_watchdog():
    """A read is not a command. The twin polls /frame continuously while it
    observes a brain-driven mission; if that counted, an open phone would
    silently hold off the one failsafe that catches motors left running."""
    with make_client() as client:
        client.post("/action", json={"action": "STOP"})
        client.get("/frame")
        client.get("/distance")
        after_reads = client.get("/health").json()["seconds_since_last_command"]

        client.post("/stop")
        after_command = client.get("/health").json()["seconds_since_last_command"]

        assert after_command <= after_reads, "a command must reset the watchdog clock"
        assert client.get("/frame").status_code == 200
        assert client.get("/health").json()["seconds_since_last_command"] >= after_command, (
            "sensing reads must not reset the watchdog clock"
        )
