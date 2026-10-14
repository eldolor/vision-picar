"""
tests/test_review_376h3.py

PLAN 3.76 batch H, part 3: the tunnel (`service/tunnel/`). Each test is a
confirmed finding from the whole-file review, red on `dev` before its fix.
"""

import re
import subprocess
from pathlib import Path

import httpx
import pytest
from fastapi.testclient import TestClient

from service.tunnel import proxy as tunnel_proxy

ROOT = Path(__file__).resolve().parents[1]


class Upstream:
    """Stands in for httpx.AsyncClient: records what reached it."""

    def __init__(self, status=200, headers=None):
        self.seen = {}
        self.status = status
        self.headers = headers or {"content-type": "application/json"}

    async def request(self, method, url, content=None, headers=None):
        self.seen.update(method=method, url=url, headers=dict(headers or {}), body=content)
        return httpx.Response(self.status, headers=self.headers, content=b'{"ok":true}')


@pytest.fixture
def upstream(monkeypatch):
    up = Upstream()
    monkeypatch.setattr(tunnel_proxy, "client", up)
    return up


# ---------- proxy.py ----------

def test_h3_4_an_encoded_slash_or_hash_reaches_the_upstream_still_encoded(upstream):
    """Built from Starlette's DECODED path, `%2F` became a path separator
    and `%23` a fragment (httpx drops it): `/recording/walks/a%23b` reached
    the robot as `/recording/walks/a`."""
    c = TestClient(tunnel_proxy.app)
    c.get("/recording/walks/a%23b/meta")
    assert upstream.seen["url"] == tunnel_proxy.ROBOT + "/recording/walks/a%23b/meta"
    c.get("/brain/x%2Fy?q=1")
    assert upstream.seen["url"] == tunnel_proxy.BRAIN + "/brain/x%2Fy?q=1"


def test_h3_5_hop_by_hop_request_headers_are_not_forwarded(upstream):
    """Only `host` and `content-length` were dropped; `connection`,
    `transfer-encoding` and the rest describe the phone's hop, not ours."""
    c = TestClient(tunnel_proxy.app)
    c.post("/action", content=b"{}", headers={
        "connection": "keep-alive, x-foo", "keep-alive": "timeout=5", "te": "trailers",
        "upgrade": "h2c", "proxy-authorization": "Basic x", "x-foo": "1",
        "x-app-secret": "s"})
    sent = {k.lower() for k in upstream.seen["headers"]}
    assert not sent & {"connection", "keep-alive", "te", "upgrade",
                       "transfer-encoding", "proxy-authorization", "x-foo"}
    assert "x-app-secret" in sent


def test_h3_2_a_body_over_the_cap_is_refused_before_it_is_buffered(upstream):
    """Unauthenticated, the proxy read any body whole into memory before the
    upstream's secret check; on the Jetson (296 MB free under load, 3.58)
    one large POST could take the stack down."""
    big = b"x" * (tunnel_proxy.MAX_BODY_BYTES + 1)
    r = TestClient(tunnel_proxy.app).post("/teleop/frame", content=big)
    assert r.status_code == 413
    assert upstream.seen == {}


def _asgi_post(path, chunks):
    """Drive the app directly with the body in many ASGI messages, as
    uvicorn delivers it (TestClient joins a generator into one)."""
    import asyncio
    sent = []
    msgs = [{"type": "http.request", "body": c, "more_body": True} for c in chunks]
    msgs.append({"type": "http.request", "body": b"", "more_body": False})

    async def receive():
        return msgs.pop(0) if msgs else {"type": "http.disconnect"}

    async def send(m):
        sent.append(m)

    scope = {"type": "http", "method": "POST", "path": path, "raw_path": path.encode(),
             "query_string": b"", "headers": [], "http_version": "1.1",
             "scheme": "http", "server": ("t", 80), "client": ("c", 1), "root_path": ""}
    asyncio.run(tunnel_proxy.app(scope, receive, send))
    return next(m["status"] for m in sent if m["type"] == "http.response.start")


def test_h3_2_a_streamed_body_over_the_cap_is_refused_too(upstream):
    """No Content-Length (chunked): the cap holds while reading."""
    n = tunnel_proxy.MAX_BODY_BYTES // 65536 + 2
    assert _asgi_post("/teleop/frame", [b"x" * 65536] * n) == 413
    assert upstream.seen == {}
    assert tunnel_proxy._inflight == 0


def test_h3_r2_bodies_in_flight_together_are_bounded(upstream, monkeypatch):
    """16 MB each did not bound twenty at once (the fixes' review)."""
    monkeypatch.setattr(tunnel_proxy, "_inflight", tunnel_proxy.MAX_INFLIGHT_BYTES - 10)
    assert _asgi_post("/teleop/frame", [b"x" * 100]) == 503
    assert upstream.seen == {}
    assert tunnel_proxy._inflight == tunnel_proxy.MAX_INFLIGHT_BYTES - 10


def test_h3_r2_the_proxys_own_errors_carry_cors(monkeypatch):
    """A cross-origin twin read a bare "Load failed" instead of the detail."""
    class Dead:
        async def request(self, *a, **k):
            raise httpx.ConnectError("refused")
    monkeypatch.setattr(tunnel_proxy, "client", Dead())
    c = TestClient(tunnel_proxy.app)
    assert c.get("/brain/health").headers.get("access-control-allow-origin") == "*"
    big = b"x" * (tunnel_proxy.MAX_BODY_BYTES + 1)
    assert c.post("/teleop/frame", content=big).headers.get("access-control-allow-origin") == "*"


def test_h3_r2_a_raw_non_ascii_byte_is_encoded_once(upstream):
    url = tunnel_proxy._forward_path("/recording/walks/é".encode(), "recording/walks/é",
                                     tunnel_proxy.ROBOT)
    assert url == "/recording/walks/%C3%A9"
    assert str(httpx.URL(tunnel_proxy.ROBOT + url)).endswith("/walks/%C3%A9")


def test_h3_r2_an_encoded_separator_after_vision_still_reaches_the_route(upstream):
    TestClient(tunnel_proxy.app).get("/vision%2Fnavigate")
    assert upstream.seen["url"] == tunnel_proxy.VISION + "/navigate"


def test_h3_2_a_frame_at_the_routes_own_limit_still_passes(upstream):
    """The cap sits above the largest legitimate body: a 4 MB frame
    (recording_routes.MAX_FRAME_BYTES) as base64 in JSON."""
    body = b"x" * (4 * 1024 * 1024 * 4 // 3 + 1024)
    r = TestClient(tunnel_proxy.app).post("/recording/walks/w/frames", content=body)
    assert r.status_code == 200
    assert len(upstream.seen["body"]) == len(body)


def test_h3_7_the_502_is_valid_json_whatever_the_error_says(monkeypatch):
    class Dead:
        async def request(self, *a, **k):
            raise httpx.ConnectError('refused "quoted" \\ and\nnewline')
    monkeypatch.setattr(tunnel_proxy, "client", Dead())
    r = TestClient(tunnel_proxy.app).get("/brain/health")
    assert r.status_code == 502
    assert tunnel_proxy.BRAIN in r.json()["detail"]


def test_h3_6_an_upstream_redirect_stays_on_the_tunnel(monkeypatch):
    """Host is not forwarded, so a FastAPI slash redirect named
    `http://127.0.0.1:8001/...` -- unreachable from the phone."""
    up = Upstream(status=307, headers={
        "location": tunnel_proxy.BRAIN + "/brain/mission/status?x=1"})
    monkeypatch.setattr(tunnel_proxy, "client", up)
    r = TestClient(tunnel_proxy.app).get("/brain/mission/status/", follow_redirects=False)
    assert r.status_code == 307
    assert r.headers["location"] == "/brain/mission/status?x=1"


# ---------- run.sh ----------

def _run_secrets_block(tmp_path, secrets_text):
    """Run run.sh's own secrets block against a stand-in secrets file."""
    text = (ROOT / "service/tunnel/run.sh").read_text()
    block = re.search(r"^# --- secrets ---$.*?^# --- end secrets ---$",
                      text, re.S | re.M).group(0)
    home = tmp_path / "home"
    home.mkdir()
    (home / ".vision-picar-local-secrets").write_text(secrets_text)
    script = "set -euo pipefail\n" + block + "\necho \"APP=$APP_SHARED_SECRET VIS=$VISION_SHARED_SECRET\"\n"
    return subprocess.run(["bash", "-c", script], capture_output=True, text=True,
                          env={"HOME": str(home), "PATH": "/usr/bin:/bin"})


def test_h3_1_an_empty_local_secret_refuses_to_start(tmp_path):
    """`LOCAL_SECRET=` passed `set -u` and left require_secret() inert on
    both servers -- on the public internet through the tunnel."""
    r = _run_secrets_block(tmp_path, "LOCAL_SECRET=\nVISION_SECRET=v\n")
    assert r.returncode != 0
    assert "LOCAL_SECRET" in r.stderr + r.stdout


def test_h3_3_no_vision_secret_still_starts(tmp_path):
    """Offline is a mode (3.49): a secrets file without VISION_SECRET
    aborted on `unbound variable` before any server started."""
    r = _run_secrets_block(tmp_path, "LOCAL_SECRET=abcdefghijklmnop\n")
    assert r.returncode == 0, r.stderr
    assert "APP=abcdefghijklmnop VIS=" in r.stdout


# ---------- restart.sh ----------

def test_h3_9_restart_does_not_put_the_secret_on_a_command_line():
    """`curl -H "x-app-secret: $LOCAL_SECRET"` shows in `ps` to every user."""
    text = (ROOT / "service/tunnel/restart.sh").read_text()
    assert not re.search(r'curl[^\n]*-H "x-app-secret: \$', text)


def test_h3_8_restart_checks_the_proxy_and_that_the_stack_is_still_up():
    """OK was printed once robot and brain answered, even while the proxy
    (what ngrok points at) had died and run.sh was tearing the rest down."""
    text = (ROOT / "service/tunnel/restart.sh").read_text()
    verify = text[text.index("# ---- verify"):]
    assert "127.0.0.1:8080" in verify
    assert verify.count("127.0.0.1:8080/health") >= 2   # and again after 5 s


def test_h3_11_the_vision_preflight_allows_the_ngrok_header(upstream):
    """app.js adds `ngrok-skip-browser-warning` on an ngrok host; the fixed
    allow-list left it out and the preflight failed as "Load failed"."""
    r = TestClient(tunnel_proxy.app).options("/vision/navigate", headers={
        "origin": "https://d.cloudfront.net", "access-control-request-method": "POST",
        "access-control-request-headers": "content-type,x-app-secret,ngrok-skip-browser-warning"})
    assert r.status_code == 204
    assert "ngrok-skip-browser-warning" in r.headers["access-control-allow-headers"]


def test_h3_16_robot_replies_do_not_get_a_wildcard_origin_added(upstream):
    """The robot answers CORS itself (`server.allowed_origins`); a `*`
    added here overrode a narrowed allow-list for everything tunnelled."""
    r = TestClient(tunnel_proxy.app).get("/health", headers={"origin": "https://evil.test"})
    assert "access-control-allow-origin" not in r.headers


def test_vision_paths_still_lose_their_prefix(upstream):
    c = TestClient(tunnel_proxy.app)
    c.post("/vision/navigate?x=1", content=b"{}")
    assert upstream.seen["url"] == tunnel_proxy.VISION + "/navigate?x=1"
    c.get("/vision")
    assert upstream.seen["url"] == tunnel_proxy.VISION + "/"


def test_h3_12_an_invalid_upstream_url_is_a_502_not_a_500(monkeypatch):
    class Bad:
        async def request(self, *a, **k):
            raise httpx.InvalidURL("bad")
    monkeypatch.setattr(tunnel_proxy, "client", Bad())
    assert TestClient(tunnel_proxy.app).get("/x").status_code == 502


def test_h3_14_the_bucket_lookup_is_bounded_in_time(tmp_path):
    """No timeout: on a network that drops packets at boot the CLI's 60 s
    connect timeout and retries held every server back for minutes."""
    from tests.test_inventory_mission import _bucket_from_run_sh
    args = tmp_path / "args"
    assert _bucket_from_run_sh(tmp_path, None, aws=f'echo "$@ $AWS_MAX_ATTEMPTS" > {args}; echo b') == "b"
    seen = args.read_text()
    assert "--cli-connect-timeout" in seen and "--cli-read-timeout" in seen
    assert seen.strip().endswith("1")


def test_h3_15_cleanup_waits_for_its_children():
    """Exiting straight after the TERM let systemd (KillMode=mixed) SIGKILL
    the servers mid-shutdown."""
    text = (ROOT / "service/tunnel/run.sh").read_text()
    body = re.search(r"^cleanup\(\) \{.*?^\}", text, re.M | re.S)
    assert body and "kill -0" in body.group(0)


def test_h3_17_a_stale_lidar_link_is_removed_and_a_missing_one_is_fatal():
    text = (ROOT / "service/tunnel/run.sh").read_text()
    block = text[text.index('export ROBOT_LIDAR='):text.index('(fake D500)')]
    assert block.index('rm -f "$ROBOT_LIDAR"') < block.index("python -m sim.fake_lidar")
    assert "exit 1" in block


def test_h3_13_restart_will_not_take_over_the_systemd_unit():
    text = (ROOT / "service/tunnel/restart.sh").read_text()
    guard = text.index("systemctl is-active picar-stack")
    assert "activating" in text[guard:guard + 300]
    assert guard < text.index("kill $old")


def test_h3_r2_an_unset_local_secret_gets_the_message_not_bash(tmp_path):
    r = _run_secrets_block(tmp_path, "VISION_SECRET=v\n")
    assert r.returncode != 0
    assert "LOCAL_SECRET" in r.stderr and "unbound" not in r.stderr


def test_h3_r2_a_stop_cleans_up_once():
    """`cleanup; exit 0` in the INT/TERM trap ran cleanup, then the EXIT
    trap ran it again: two bounded waits on a hung child."""
    text = (ROOT / "service/tunnel/run.sh").read_text()
    assert re.search(r"^trap 'exit 0' INT TERM", text, re.M)
    assert re.search(r"^trap cleanup EXIT", text, re.M)


def test_h3_r2_restart_rechecks_through_the_proxy_not_pgrep():
    text = (ROOT / "service/tunnel/restart.sh").read_text()
    tail = text[text.index("sleep 5"):]
    assert "revision http://127.0.0.1:8080/health" in tail


def test_h3_r3_an_upstream_500_is_readable_cross_origin(monkeypatch):
    """Starlette's ServerErrorMiddleware answers a 500 outside the CORS
    middleware; H3-16's scoping left it with no allow-origin at all."""
    monkeypatch.setattr(tunnel_proxy, "client", Upstream(status=500, headers={"content-type": "text/plain"}))
    r = TestClient(tunnel_proxy.app).get("/brain/mission/status", headers={"origin": "https://x.test"})
    assert r.status_code == 500
    assert r.headers.get("access-control-allow-origin") == "*"
