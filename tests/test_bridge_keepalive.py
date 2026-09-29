"""
tests/test_bridge_keepalive.py

picar_bridge's HTTP client (PLAN-ros-alignment.md 3.24, G1), on a laptop:
it must reuse one connection, survive the server restarting, keep threads
apart, and report a timeout as a timeout. The live half -- the bridge
republishing scans at >= 9.5 Hz -- is `tests/test_ros_chain_live.py`.
"""

import http.server
import socket
import sys
import threading
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "service/slam/src/picar_bridge"))
from picar_bridge.keepalive import KeepAliveClient  # noqa: E402


class _Server:
    """A keep-alive HTTP/1.1 server that counts the connections it accepts."""

    def __init__(self, port=0, delay=0.0):
        outer = self
        self.accepted = 0
        self.open_sockets = []
        self.delay = delay
        self.paths = []

        class Handler(http.server.BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def setup(self):
                outer.accepted += 1
                outer.open_sockets.append(self.request)
                super().setup()

            def do_GET(self):
                time.sleep(outer.delay)
                outer.paths.append((self.path, self.headers.get("x-app-secret")))
                body = b'{"ok": true, "path": "%s"}' % self.path.encode()
                self.send_response(404 if self.path.endswith("/missing") else 200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *a):
                pass

        self.httpd = http.server.ThreadingHTTPServer(("127.0.0.1", port), Handler)
        self.httpd.daemon_threads = True
        self.port = self.httpd.server_address[1]
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def stop(self):
        """Like a real restart: the listener AND every open connection go."""
        self.httpd.shutdown()
        self.httpd.server_close()
        for sock in self.open_sockets:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            sock.close()


@pytest.fixture
def server():
    s = _Server()
    yield s
    s.stop()


def test_many_requests_share_one_connection(server):
    c = KeepAliveClient(f"http://127.0.0.1:{server.port}", {"x-app-secret": "s3"})
    for i in range(50):
        assert c.get_json(f"/scan?i={i}")["ok"] is True
    assert c.connections_opened == 1 and server.accepted == 1
    assert all(secret == "s3" for _, secret in server.paths)


def test_a_path_prefix_is_kept(server):
    c = KeepAliveClient(f"http://127.0.0.1:{server.port}/brain")
    assert c.get_json("/mission/status")["path"] == "/brain/mission/status"


def test_a_server_restart_costs_one_reconnect_not_a_failure():
    s = _Server()
    port = s.port
    c = KeepAliveClient(f"http://127.0.0.1:{port}")
    assert c.get_json("/scan")["ok"]
    s.stop()
    s2 = _Server(port=port)
    try:
        assert c.get_json("/scan")["ok"], "a stale kept-open connection must be retried once"
        assert c.connections_opened == 2
    finally:
        s2.stop()


def test_threads_do_not_share_a_connection(server):
    c = KeepAliveClient(f"http://127.0.0.1:{server.port}")
    errors = []

    def worker(n):
        try:
            for i in range(20):
                assert c.get_json(f"/t{n}/{i}")["path"] == f"/t{n}/{i}"
        except Exception as e:  # noqa: BLE001
            errors.append(e)

    threads = [threading.Thread(target=worker, args=(n,)) for n in range(4)]
    [t.start() for t in threads]
    [t.join() for t in threads]
    assert not errors
    assert c.connections_opened == 4


def test_a_timeout_is_reported_as_a_timeout_and_not_retried(server):
    server.delay = 0.3
    c = KeepAliveClient(f"http://127.0.0.1:{server.port}", timeout=0.1)
    t0 = time.monotonic()
    with pytest.raises((socket.timeout, TimeoutError)):
        c.get_json("/scan")
    assert time.monotonic() - t0 < 0.25, "a timeout must not be retried"
    server.delay = 0.0
    assert c.get_json("/scan")["ok"], "the next call opens a fresh connection"


def test_an_error_status_raises(server):
    c = KeepAliveClient(f"http://127.0.0.1:{server.port}")
    with pytest.raises(RuntimeError, match="404"):
        c.get_json("/missing")
    assert c.get_json("/scan")["ok"], "the connection is still usable after a 404"


def test_nothing_but_plain_http():
    with pytest.raises(ValueError):
        KeepAliveClient("https://example.com")
