"""
tests/test_renderer_parity.py

Phase S2's parity check: the Python renderer and the JavaScript raycaster
it was ported from must draw the same view for the same pose.

**This is the test that licenses deleting the JS one.** Until the two are
shown to agree, `web-twin/app.js`'s `renderFPV` cannot be retired without
someone taking it on faith that the picture did not change -- and the
picture is the vision policy's input, so "it looks about the same" is not
good enough. `PLAN-sim-hardening.md` S2 names this explicitly as the test
that makes the deletion safe.

## What it compares, and why not pixels

Not bytes. `canvas.fillRect` antialiases a fractional-height wall column
and PIL does not, so the two rasterisations differ by a pixel along every
wall edge no matter how correct the geometry is. Demanding identical
bytes would fail for a reason nobody cares about while still missing a
genuinely wrong wall.

What it compares instead is the **wall silhouette**: for each screen
column, the row where the wall band starts. That is the geometry --
`cast_ray` -> `perp` -> `wallHeight` -> where the column is painted -- and
it is what "the same view" actually means. Object billboards are excluded
by column, since a billboard covers the wall behind it and its own edges
are a separate (and much smaller) piece of drawing.

## How it gets the JavaScript's answer

By running it. A real `uvicorn robot.server:app`, the real `index.html`
and `app.js` in a real Chromium, connected to the real server -- the same
subprocess-plus-Playwright pattern `tests/test_ui.py` established, and
for the same reason: a reimplementation of the raycaster inside the test
would be a third copy to keep in sync, which is the problem this phase
exists to remove.

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

from sim import renderer
from sim.maps.starter_house import build_starter_world
from tests.conftest import REPO_ROOT, SERVER_START_TIMEOUT_S, free_port

sync_api = pytest.importorskip(
    "playwright.sync_api",
    reason="parity needs `pip install pytest-playwright` + `playwright install chromium`",
)

# Wide enough that the twin's own sizing logic (sizeFpvCanvas clamps to
# 200..480) lands on a real width rather than the minimum, so the
# comparison runs over a few hundred columns rather than a handful.
VIEWPORT = {"width": 900, "height": 900}

# A wall edge may land on either side of a pixel boundary depending on how
# each renderer rounds a fractional height. More than this is a geometry
# difference, not a rounding one.
EDGE_TOLERANCE_PX = 2

# The starter world's opening pose -- a fresh server puts the robot in the
# living room facing the doorway. Asserted below rather than assumed,
# because the whole comparison is meaningless against the wrong pose.
EXPECTED_POSE = {"position": [2, 2], "facing": "E"}


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


def _open_twin(browser, twin_server, *, strip_image):
    """The twin, on the Sim tab, connected and showing a rendered frame.

    `strip_image=True` removes `image_base64` from every `/frame` reply, so
    the page falls back to its own raycaster -- **which is the entire point
    of the comparison.** Since S2 the twin prefers the server's pixels, so
    without this the canvas would be showing `sim/renderer.py`'s output and
    the parity test would be comparing Python against Python and passing
    for the wrong reason. Stripping it makes the page behave like a pre-S2
    server, which is the only way left to get the JavaScript to draw.

    deviceScaleFactor is pinned to 1 so the canvas's backing store is the
    same size as its logical drawing surface -- otherwise getImageData
    returns device pixels and every column index would be off by the
    display's DPR.
    """
    context = browser.new_context(viewport=VIEWPORT, device_scale_factor=1)
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

    # The robot server's address lives on the Settings tab; the canvas
    # lives on Sim. Connect first, then switch -- the Sim tab has to be
    # *visible* before the canvas can size itself, because sizeFpvCanvas()
    # bails at zero width while the tab is hidden and waits for the resize
    # observer to fire when it is shown.
    page.click("#btn-settings")  # the header gear, not a tab-bar button
    page.fill("#cfg-server-url", twin_server)
    page.click("#btn-connect")
    page.click("#btn-settings")  # toggles back out of settings
    page.click('.tab-btn[data-tab="sim"]')
    page.wait_for_function(
        "() => { const c = document.getElementById('fpv-canvas');"
        " return c && c.width > 0 && c.height > 0; }"
    )
    # Connect paints the first frame; give the canvas a beat to land before
    # reading it back (a server frame decodes asynchronously).
    page.wait_for_timeout(400)

    assert not errors, f"page errors: {errors}"
    return page


@pytest.fixture(scope="module")
def browser():
    with sync_api.sync_playwright() as p:
        b = p.chromium.launch()
        yield b
        b.close()


@pytest.fixture(scope="module")
def connected_page(browser, twin_server):
    """Drawn by the JavaScript raycaster -- see `_open_twin`."""
    return _open_twin(browser, twin_server, strip_image=True)


def _canvas_wall_tops(page):
    """One entry per column: the first row that is not sky, or None.

    Read out of the live canvas. The top half of the frame is painted with
    the sky colour and a wall column is painted over it from
    `(H - wallHeight) / 2` downward, so the first non-sky row *is* the top
    of the wall band.
    """
    return page.evaluate(
        """(sky) => {
          const c = document.getElementById('fpv-canvas');
          const d = c.getContext('2d').getImageData(0, 0, c.width, c.height).data;
          const tops = [];
          for (let x = 0; x < c.width; x++) {
            let top = null;
            for (let y = 0; y < c.height; y++) {
              const i = (y * c.width + x) * 4;
              if (Math.abs(d[i] - sky[0]) > 8 || Math.abs(d[i+1] - sky[1]) > 8
                  || Math.abs(d[i+2] - sky[2]) > 8) { top = y; break; }
            }
            tops.push(top);
          }
          return { width: c.width, height: c.height, tops: tops };
        }""",
        list(renderer.COLOR_SKY),
    )


def _object_columns(world, width, base_angle):
    """Screen columns covered by an object billboard, which hides the wall
    behind it and so cannot be compared as wall silhouette."""
    covered = set()
    for obj in renderer._visible_objects(
        world.layout, world.objects, world.robot_x + 0.5, world.robot_y + 0.5, base_angle
    ):
        import math

        t = (obj["rel_angle"] + renderer.FPV_FOV / 2) / renderer.FPV_FOV
        screen_x = t * width
        perp = max(0.3, obj["dist"] * math.cos(obj["rel_angle"]))
        # Mirrors render()'s own billboard sizing; the +2 is slack for the
        # edge pixels either renderer may round outward.
        h = min(width * 0.55, (width * 0.65) / perp)
        w = h * 0.7
        covered.update(range(int(screen_x - w / 2) - 2, int(screen_x + w / 2) + 3))
    return covered


def test_the_server_is_at_the_pose_this_comparison_assumes(twin_server):
    frame = httpx.get(f"{twin_server}/frame", timeout=5).json()
    assert list(frame["position"]) == EXPECTED_POSE["position"]
    assert frame["facing"] == EXPECTED_POSE["facing"]


def test_python_and_javascript_agree_on_the_wall_silhouette(connected_page):
    canvas = _canvas_wall_tops(connected_page)
    width, height = canvas["width"], canvas["height"]

    world = build_starter_world()
    base_angle = renderer.HEADING_ANGLE[EXPECTED_POSE["facing"]]
    profile = renderer.wall_profile(
        world.layout, world.robot_x + 0.5, world.robot_y + 0.5,
        base_angle, width, height,
    )
    skip = _object_columns(world, width, base_angle)

    mismatches = []
    compared = 0
    for x, (wall_height, _shade) in enumerate(profile):
        if x in skip or canvas["tops"][x] is None:
            continue
        compared += 1
        expected_top = (height - wall_height) / 2
        if abs(expected_top - canvas["tops"][x]) > EDGE_TOLERANCE_PX:
            mismatches.append((x, round(expected_top, 1), canvas["tops"][x]))

    assert compared > width * 0.4, (
        f"only {compared} of {width} columns were comparable -- the canvas is "
        "probably not showing what this test thinks it is"
    )
    assert not mismatches, (
        f"{len(mismatches)} of {compared} columns disagree "
        f"(column, python_top, js_top): {mismatches[:8]}"
    )


def test_the_readout_says_local_when_the_server_sends_no_pixels(connected_page):
    """Guards the test above from passing for the wrong reason. If this
    ever says "server", `_open_twin`'s interception has stopped working and
    the parity comparison is quietly Python-against-Python."""
    assert "local raycaster" in connected_page.inner_text("#fpv-source")


def test_the_readout_says_server_against_a_real_frame(browser, twin_server):
    """Phase S2's UI proof: the picture does not change, where it comes
    from does. This is the readout someone holding a phone actually looks
    at to tell whether the JS raycaster has been retired -- and the
    condition under which `renderFPV` becomes safe to delete."""
    page = _open_twin(browser, twin_server, strip_image=False)
    assert "server" in page.inner_text("#fpv-source")


def test_the_two_renderers_use_the_same_field_of_view(connected_page):
    """A field-of-view mismatch is the one geometry error the silhouette
    check above can miss: both would still be internally consistent, and
    the walls would simply be in different places. Compared here as the
    constants themselves, read out of the page's own source rather than
    restated."""
    js_source = connected_page.evaluate(
        "async () => (await fetch('app.js')).text()"
    )
    for name, value in (
        ("FPV_FOV", "Math.PI / 3"),
        ("FPV_MAX_DIST", "14"),
        ("FPV_STEP", "0.05"),
    ):
        assert f"const {name} = {value}" in js_source, (
            f"{name} changed in app.js -- sim/renderer.py must change with it"
        )

    assert renderer.FPV_FOV == pytest.approx(3.141592653589793 / 3)
    assert renderer.FPV_MAX_DIST == 14
    assert renderer.FPV_STEP == 0.05
