"""picar_bridge/keepalive.py -- the bridge's HTTP client to the robot server
and the brain, over KEPT-OPEN connections (PLAN-ros-alignment.md 3.24, G1).

Plain Python -- no rclpy -- so `tests/test_bridge_keepalive.py` runs it on a
laptop, the way `convert.py` is tested.

**Why not `urllib`, which the bridge used until 2026-09-29.** `urllib` opens
a NEW connection for every request. Through Docker Desktop's port-forwarding
on macOS, a new connection from the container to the host stalls past the
bridge's 0.5 s timeout about one time in ten (41 of 433 `/scan` polls,
measured); over one kept-open connection, 0 of 595. Each stall was a scan
never published, which is how the live suite's 5 Hz scan test failed. And
the failure was MISREPORTED: the IPv4 connect timed out, Python fell back to
the host's IPv6 address, which the container cannot route, and raised
"Network is unreachable" -- the last error, not the real one. The C++ wheel
plugin (libcurl) has always kept its connection open, which is why the
wheels never showed this.

One connection per THREAD: the bridge spins a `MultiThreadedExecutor`, so two
timers can poll at once, and an `http.client.HTTPConnection` must not be
shared between threads mid-request.
"""

import http.client
import json
import threading
from urllib.parse import urlparse

# Errors that mean the kept-open connection went stale (the server restarted,
# or closed an idle keep-alive) -- worth ONE retry on a fresh connection.
# A timeout is deliberately not one of them: it is the failure being
# reported, and retrying it would double the time a caller waits.
_STALE = (http.client.RemoteDisconnected, http.client.BadStatusLine,
          BrokenPipeError, ConnectionResetError, ConnectionAbortedError)


class KeepAliveClient:
    """GET JSON from one base URL over a kept-open connection per thread."""

    def __init__(self, base_url: str, headers: dict = None, timeout: float = 0.5):
        u = urlparse(base_url)
        if u.scheme != "http":
            raise ValueError(f"KeepAliveClient speaks plain http, not {u.scheme!r}")
        self.host, self.port = u.hostname, u.port or 80
        self.prefix = u.path.rstrip("/")
        self.headers = dict(headers or {})
        self.timeout = timeout
        self._local = threading.local()
        # How many connections have been opened, across all threads -- what
        # the tests (and anyone debugging a stall) read.
        self.connections_opened = 0
        self._count_lock = threading.Lock()

    def _connection(self):
        conn = getattr(self._local, "conn", None)
        if conn is None:
            conn = http.client.HTTPConnection(self.host, self.port, timeout=self.timeout)
            self._local.conn = conn
            with self._count_lock:
                self.connections_opened += 1
        return conn

    def _drop(self):
        conn = getattr(self._local, "conn", None)
        if conn is not None:
            conn.close()
        self._local.conn = None

    def get_json(self, path: str):
        """The decoded JSON body of GET `path`. Raises on a non-2xx status,
        a timeout, or a connection that fails twice."""
        for attempt in (0, 1):
            conn = self._connection()
            try:
                conn.request("GET", self.prefix + path, headers=self.headers)
                resp = conn.getresponse()
                body = resp.read()
            except _STALE:
                self._drop()
                if attempt:
                    raise
                continue
            except Exception:
                self._drop()        # a half-read connection is not reusable
                raise
            if not 200 <= resp.status < 300:
                raise RuntimeError(f"GET {path}: HTTP {resp.status}")
            return json.loads(body)
        raise AssertionError("unreachable")

    def close(self):
        self._drop()
