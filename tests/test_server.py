"""
Run with: pytest tests/test_server.py -v
"""

from fastapi.testclient import TestClient
from robot.server import create_app, watchdog_should_stop


def make_client():
    app = create_app()
    return TestClient(app)


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


def test_health_endpoint_reports_command_age():
    with make_client() as client:
        client.post("/stop")
        resp = client.get("/health")
        body = resp.json()
        assert body["status"] == "ok"
        assert body["seconds_since_last_command"] < 1.0
        assert "watchdog_timeout_s" in body


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
