"""
tests/test_ui_pipeline.py

The guidance loop overlaps its vision calls (Q1). These tests drive the
real page in a real browser against a real threaded stub, because the
things worth pinning here are *timing* properties, and none of them is
visible to a DOM assertion:

  1. calls actually overlap, and no more than the cap allows;
  2. an answer that arrives after a newer one is discarded rather than
     drawn.

A third property -- drive-via-brain staying serial -- is deliberately not
covered; see the note further down for what that would cost and what it
leaves open.

**The stub has to be a real threaded server, not `page.route`.** Playwright
runs route handlers on one thread, so a handler that sleeps serializes the
requests it is supposed to be measuring -- the first version of this test
reported "peak concurrency 1" against a loop that was in fact pipelining
correctly, which is an instrument reading its own bottleneck.

Skipped, not failed, without Playwright's browser, exactly like
tests/test_ui.py.
"""

import json
import os
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import httpx
import pytest

from tests.conftest import REPO_ROOT, SERVER_START_TIMEOUT_S, free_port

sync_api = pytest.importorskip(
    "playwright.sync_api",
    reason="needs `pip install pytest-playwright` + `playwright install chromium`",
)

PHONE = {"width": 390, "height": 844}

# Comfortably longer than the 500ms dispatch throttle, so overlap is
# unambiguous rather than a race the test happens to win.
LATENCY_S = 1.2

MODELS = {
    "default": "stub", "default_prompt": "default", "prompts": ["default"],
    "models": [{"id": "stub", "label": "Stub"}],
}


def _nav(action="LEFT"):
    return {
        "target_visible": True, "target_direction": "left",
        "target_reached": False, "obstacle_ahead": False,
        "room_guess": "kitchen", "action": action,
        "reasoning": f"stub says {action}", "model_id": "stub",
    }


class _Stub:
    """A vision service that is slow on purpose and counts overlap.

    `plan` optionally maps a 1-based call ordinal to (delay, action), so a
    test can make an early call land *after* a later one.
    """

    def __init__(self, plan=None):
        self.lock = threading.Lock()
        self.now = self.peak = self.total = 0
        self.plan = plan or {}
        stub = self

        class H(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def _cors(self):
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Headers", "*")
                self.send_header("Access-Control-Allow-Methods", "*")

            def _send(self, obj, status=200):
                raw = json.dumps(obj).encode()
                self.send_response(status)
                self._cors()
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)

            def do_OPTIONS(self):
                self.send_response(204)
                self._cors()
                self.send_header("Content-Length", "0")
                self.end_headers()

            def do_GET(self):
                if "/navigate/models" in self.path:
                    self._send(MODELS)
                else:
                    self._send({"detail": "not found"}, 404)

            def do_POST(self):
                self.rfile.read(int(self.headers.get("Content-Length", 0)))
                with stub.lock:
                    stub.now += 1
                    stub.total += 1
                    stub.peak = max(stub.peak, stub.now)
                    ordinal = stub.total
                delay, action = stub.plan.get(ordinal, (LATENCY_S, "RIGHT"))
                time.sleep(delay)
                with stub.lock:
                    stub.now -= 1
                self._send(_nav(action))

            def log_message(self, *a):
                pass

        self._server = ThreadingHTTPServer(("127.0.0.1", 0), H)
        threading.Thread(target=self._server.serve_forever, daemon=True).start()
        self.url = f"http://127.0.0.1:{self._server.server_address[1]}"

    def close(self):
        self._server.shutdown()


@pytest.fixture(scope="module")
def twin_server():
    port = free_port()
    url = f"http://127.0.0.1:{port}"
    proc = subprocess.Popen(
        [sys.executable, "-m", "uvicorn", "robot.server:app",
         "--host", "127.0.0.1", "--port", str(port), "--log-level", "warning"],
        cwd=REPO_ROOT, env={**os.environ},
    )
    try:
        deadline = time.monotonic() + SERVER_START_TIMEOUT_S
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                pytest.fail(f"twin server exited early ({proc.returncode})")
            try:
                if httpx.get(f"{url}/health", timeout=0.5).status_code == 200:
                    break
            except httpx.HTTPError:
                time.sleep(0.05)
        else:
            pytest.fail("twin server never became healthy")
        yield url
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()


@pytest.fixture(scope="module")
def browser():
    with sync_api.sync_playwright() as p:
        # The fake device is what makes Robot view drivable at all here --
        # without it getUserMedia never attaches a stream and the loop
        # never starts, silently and with no error on the page.
        b = p.chromium.launch(args=["--use-fake-ui-for-media-stream",
                                    "--use-fake-device-for-media-stream"])
        yield b
        b.close()


def _start_robot_view(browser, twin_server, stub):
    """Robot view, running, pointed at the stub.

    `vp_guide_onboarded` matters: without it the Start button opens the
    onboarding sheet instead of starting, and the loop never runs -- which
    looks identical to a broken camera from the outside.
    """
    ctx = browser.new_context(viewport=PHONE, permissions=["camera"])
    seed = {
        "guidanceMode": "robot",
        "vp_guide_onboarded": "1",
        "vp_vision_url": stub.url,
    }
    ctx.add_init_script(
        "(() => { const s = %s;"
        " try { for (const k in s) localStorage.setItem(k, s[k]); } catch (e) {} })();"
        % json.dumps(seed)
    )
    page = ctx.new_page()
    errors = []
    page.on("pageerror", lambda e: errors.append(str(e)))
    page.goto(twin_server, wait_until="networkidle")
    page.wait_for_timeout(600)
    page.fill("#guidance-target", "red backpack")
    page.click("#btn-guidance")
    page.wait_for_timeout(1200)
    assert page.inner_text("#btn-guidance").strip() == "Stop", (
        "the loop never started -- check the camera stub and the onboarding pref"
    )
    return page, errors


def test_vision_calls_actually_overlap(browser, twin_server):
    """The point of the change. Serial, this many seconds at this latency
    would fit roughly `elapsed / (latency + throttle)` calls; overlapping,
    it fits about twice that."""
    stub = _Stub()
    try:
        page, errors = _start_robot_view(browser, twin_server, stub)
        page.wait_for_timeout(7000)
        assert not errors, errors
        assert stub.peak >= 2, (
            f"calls never overlapped (peak={stub.peak}) -- the loop is still "
            "waiting for each answer before dispatching the next"
        )
        serial_ceiling = 7.0 / (LATENCY_S + 0.5)
        assert stub.total > serial_ceiling, (
            f"{stub.total} calls in 7s is within what a serial loop would "
            f"manage (~{serial_ceiling:.0f})"
        )
        page.context.close()
    finally:
        stub.close()


def test_concurrency_is_capped(browser, twin_server):
    """Bedrock quota is account-scoped and shared with production --
    control/walk_replay.py measured 10 of 22 frames throttled at 8 workers.
    The cap is the only thing standing between an experiment and that."""
    stub = _Stub()
    try:
        page, errors = _start_robot_view(browser, twin_server, stub)
        page.wait_for_timeout(7000)
        assert not errors, errors
        assert stub.peak <= 2, f"exceeded the in-flight cap: peak={stub.peak}"
        page.context.close()
    finally:
        stub.close()


# NOT COVERED HERE, and worth knowing about: drive-via-brain staying
# serial. `guidanceStep` pins that mode to one call in flight because
# MissionRunner advances one action at a time and its step budget and
# failsafes are built on that -- a brain deciding from frames it has
# already acted past is worse than a slow brain. Exercising it from the
# browser needs more scaffolding than the guarantee is worth today: a
# mode: teleop robot server for pushTeleopFrame(), a brain stub serving
# /mission/status, and a connected brain, since driveViaBrainActive()
# requires all three. The risk it leaves open is someone raising
# GUIDANCE_MAX_IN_FLIGHT later without noticing the exemption; the guard
# says so at its site, which is the mitigation until this is covered.


def test_an_overtaken_answer_is_not_drawn(browser, twin_server):
    """Latency varies per call, so an early request can land after a later
    one. Without a sequence check the stale answer would repaint the HUD
    and the person would be steered by a frame two decisions old.

    The first call is made deliberately slow and told to answer LEFT; every
    later call is fast and answers RIGHT. The HUD must read RIGHT both
    before and after the straggler lands.
    """
    stub = _Stub(plan={1: (5.0, "LEFT")})
    try:
        page, errors = _start_robot_view(browser, twin_server, stub)
        page.wait_for_timeout(3500)          # fast calls have answered
        before = page.inner_text("#robot-action-verb").strip().upper()
        page.wait_for_timeout(4000)          # the slow first call has landed
        after = page.inner_text("#robot-action-verb").strip().upper()
        assert not errors, errors
        assert before == "RIGHT", f"expected the fast answers, got {before!r}"
        assert after == "RIGHT", (
            f"a stale answer repainted the HUD: {after!r} -- the sequence "
            "check is not holding"
        )
        page.context.close()
    finally:
        stub.close()
