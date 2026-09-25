"""
tests/test_frame_source.py

Phase S2's UI proof, and what is left of `tests/test_renderer_parity.py`
after the ROS alignment deleted the thing it compared against.

**The history matters, because this file is the end of a deliberate
sequence.** S2 ported the twin's JavaScript raycaster into
`sim/renderer.py` so that the simulator could hand a vision policy real
pixels. It could not simply delete the JavaScript one: the picture is the
policy's *input*, so "it looks about the same" was not good enough, and
`PLAN-sim-hardening.md` S2 named a parity test -- wall silhouette compared
column by column, in a real Chromium, against a real server -- as the thing
that would license the deletion. That test was written, it passed, and it
sat there for three weeks saying the deletion was safe.

`PLAN-ros-alignment.md` R0 then made keeping the JavaScript *unsafe* rather
than merely redundant. `renderFPV` read a CELL and a CARDINAL heading off
the frame, which is all a pre-R0 pose could offer; once the pose became
continuous it could only ever draw one of four directions, and at one call
site it fed that picture to the vision model as though it were the camera.
So the raycaster went, and the parity comparison went with it -- there is
nothing left to be in parity with.

What survives is the readout, and it survives because its meaning improved.
It used to answer "is the JS raycaster still being used?", which was a
question about a migration. It now answers "did this frame come from a
camera at all?", which is a question about a robot: a real camera can be
absent, unplugged, or sending bytes that will not decode, and a page that
quietly drew *something* in that case would be the same defect R0 exposed.

`sim/renderer.py` is still pinned against drift -- by the golden image in
`tests/test_renderer.py`, which does not need a browser.

Skipped, not failed, without Playwright's browser, exactly as
`tests/test_ui.py` is -- so `pytest tests/ -q` stays one invocation on a
fresh checkout.
"""

import json
import os
import subprocess
import sys
import time

import httpx
import pytest

from tests.conftest import REPO_ROOT, SERVER_START_TIMEOUT_S, free_port

sync_api = pytest.importorskip(
    "playwright.sync_api",
    reason="needs `pip install pytest-playwright` + `playwright install chromium`",
)

# The phone viewport the rest of the UI suite uses. This file has no pixel
# geometry left to measure, so there is no reason to differ from it.
PHONE = {"width": 390, "height": 844}


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
                pytest.fail(f"twin server exited early with code {proc.returncode}")
            try:
                if httpx.get(f"{url}/health", timeout=0.5).status_code == 200:
                    break
            except httpx.HTTPError:
                time.sleep(0.05)
        else:
            pytest.fail(f"twin server was not healthy within {SERVER_START_TIMEOUT_S}s")
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
        b = p.chromium.launch()
        yield b
        b.close()


def _open_twin(browser, twin_server, *, strip_image):
    """The twin on the Sim tab, connected to a real robot server.

    `strip_image=True` removes `image_base64` from every `/frame` reply,
    which is how a backend with no camera behaves -- `TeleopRobot` before
    its first pushed frame, or a server deployed before S2. That used to
    make the page fall back to its own raycaster; there is no fallback now,
    which is what these tests are about.
    """
    context = browser.new_context(viewport=PHONE)
    page = context.new_page()

    if strip_image:
        def _strip(route):
            resp = route.fetch()
            body = resp.json()
            body.pop("image_base64", None)
            body.pop("media_type", None)
            route.fulfill(
                status=200, content_type="application/json", body=json.dumps(body)
            )

        page.route("**/frame", _strip)

    errors = []
    page.on("pageerror", lambda e: errors.append(str(e)))
    page.goto(twin_server, wait_until="networkidle")

    # The robot server's address lives on the Settings tab; the canvas lives
    # on Sim. Connect first, then switch -- the Sim tab has to be *visible*
    # before the canvas can size itself, because sizeFpvCanvas() bails at
    # zero width while the tab is hidden and waits for the resize observer.
    page.click("#btn-settings")  # the header gear, not a tab-bar button
    page.fill("#cfg-server-url", twin_server)
    page.click("#btn-connect")
    page.click("#btn-settings")  # toggles back out of settings
    page.click('.tab-btn[data-tab="sim"]')
    page.wait_for_function(
        "() => { const c = document.getElementById('fpv-canvas');"
        " return c && c.width > 0 && c.height > 0; }"
    )
    # Connect paints the first frame; a server frame decodes asynchronously,
    # so give the canvas a beat to land before reading the readout.
    page.wait_for_timeout(400)

    assert not errors, f"page errors: {errors}"
    return page


def test_the_readout_says_server_against_a_real_frame(browser, twin_server):
    """S2's UI proof: the picture does not change when the raycaster moves
    into Python, but where it comes from does. This is the line someone
    holding a phone actually reads to know they are looking at
    `sim/renderer.py`'s output and not at something the browser invented."""
    page = _open_twin(browser, twin_server, strip_image=False)
    assert "server" in page.inner_text("#fpv-source")


def test_a_frame_with_no_pixels_says_none_rather_than_drawing_something(
        browser, twin_server):
    """The state that replaced "local raycaster", and the one that matters
    on hardware. A camera can be absent or broken, and the honest answer is
    an empty canvas that says so -- not a plausible picture of a direction
    the robot is not facing, which is exactly what the deleted JS raycaster
    would now produce off a continuous pose (R0).

    Read as the negative too: if this ever says "server", the interception
    has stopped working and the test is passing vacuously.
    """
    page = _open_twin(browser, twin_server, strip_image=True)
    source = page.inner_text("#fpv-source")
    assert "none" in source
    assert "server" not in source


def test_the_page_survives_a_frame_with_no_pixels(browser, twin_server):
    """The regression the deletion could plausibly have caused: `drawFPV`'s
    no-image branch used to call a function that no longer exists. A
    ReferenceError there would leave the last frame on screen and look like
    nothing had gone wrong -- so this asserts on the console, not the canvas.
    """
    context = browser.new_context(viewport=PHONE)
    page = context.new_page()
    errors = []
    page.on("pageerror", lambda e: errors.append(str(e)))

    def _strip(route):
        resp = route.fetch()
        body = resp.json()
        body.pop("image_base64", None)
        route.fulfill(status=200, content_type="application/json",
                      body=json.dumps(body))

    page.route("**/frame", _strip)
    page.goto(twin_server, wait_until="networkidle")
    page.click("#btn-settings")
    page.fill("#cfg-server-url", twin_server)
    page.click("#btn-connect")
    page.click("#btn-settings")
    page.click('.tab-btn[data-tab="sim"]')
    page.click("#btn-forward")
    page.wait_for_timeout(500)
    assert not errors, f"page errors while drawing a frame with no pixels: {errors}"
