"""3.50.D4b: the twin's "Re-map now" control (the user's option 3, 2026-10-23).

A person asks the bridge to map a reloaded map again from where the robot
is: `POST /world/remap` on the robot server (3.50.D4), named `twin-dpad` as
the map tap is. The robot server does not pass the bridge's `slam.state` on,
so the page cannot know in advance that a re-map would be taken: the button
shows on a SLAM map only, and the page says what the server answered -- 200
re-mapping, 409 its reason (not on a saved map, already re-mapping), 501 no
SLAM here (the button goes). Written before the button existed and seen to
fail (docs/plans/ros-alignment/3.50-map-persistence.md, D4b).

`UI_SHOTS=<dir>` keeps the phone-size screenshot.
"""

import json
import os

import pytest

pytest.importorskip("playwright")

from playwright import sync_api  # noqa: E402

from tests.test_ui import (PHONE, _SLAM_ERROR, _SLAM_MAP, _SLAM_POSE,  # noqa: F401,E402
                           _stub_world, browser, open_twin, twin_server)


def _open(browser, twin_server, grid=_SLAM_MAP, pose=_SLAM_POSE):
    page, errors = open_twin(browser, twin_server)
    _stub_world(page, grid, pose, dict(_SLAM_ERROR, source=grid["map_id"]))
    page.route("**/world/goal", lambda r: r.fulfill(
        status=200, content_type="application/json", body=json.dumps({"goal": None, "plan": []})))
    page.click("#btn-settings")
    page.fill("#cfg-server-url", twin_server)
    page.click("#btn-connect")
    page.click('.tab-btn[data-tab="sim"]')
    page.wait_for_function(
        "() => (document.getElementById('map-readout') || {}).textContent.includes('cells seen')",
        timeout=8000)
    return page, errors


def _answer(page, status, body, seen=None, delay_ms=0):
    def handler(route):
        if seen is not None:
            seen.append({"method": route.request.method,
                         "driver": route.request.headers.get("x-driver")})
        if delay_ms:
            page.wait_for_timeout(delay_ms)
        route.fulfill(status=status, content_type="application/json", body=json.dumps(body))
    page.route("**/world/remap", handler)


def test_the_button_shows_on_a_slam_map_only(browser, twin_server):
    page, errors = _open(browser, twin_server)
    sync_api.expect(page.locator("#btn-remap")).to_be_visible(timeout=5000)
    page.context.close()
    sim = dict(_SLAM_MAP, map_id="sim-grid")
    page, errors = _open(browser, twin_server, grid=sim, pose=dict(_SLAM_POSE, map_id="sim-grid"))
    page.wait_for_timeout(500)
    sync_api.expect(page.locator("#btn-remap")).to_be_hidden()
    assert not errors, errors
    page.context.close()


def test_a_tap_asks_the_robot_server_as_a_person(browser, twin_server):
    page, errors = _open(browser, twin_server)
    seen = []
    _answer(page, 200, {"remapping": True, "trigger": "request", "map_id": "home", "version": 2},
            seen)
    page.click("#btn-remap")
    sync_api.expect(page.locator(".toast").last).to_contain_text("Re-mapping", timeout=3000)
    assert seen == [{"method": "POST", "driver": "twin-dpad"}], seen
    assert not errors, errors
    page.context.close()


def test_a_refusal_says_the_servers_reason(browser, twin_server):
    page, errors = _open(browser, twin_server)
    _answer(page, 409, {"detail": "re-map only from a reloaded map being localised on "
                                  "(slam ready, mode mapping)"})
    page.click("#btn-remap")
    toast = page.locator(".toast").last
    sync_api.expect(toast).to_contain_text("Not re-mapped", timeout=3000)
    sync_api.expect(toast).to_contain_text("mode mapping")
    # Still offered: a refusal is about now, and the next reload may take it.
    sync_api.expect(page.locator("#btn-remap")).to_be_enabled()
    assert not errors, errors
    page.context.close()


def test_a_world_without_slam_takes_the_button_away(browser, twin_server):
    page, errors = _open(browser, twin_server)
    _answer(page, 501, {"detail": "this world keeps no saved map -- re-mapping needs SLAM"})
    page.click("#btn-remap")
    sync_api.expect(page.locator(".toast").last).to_contain_text("no saved map", timeout=3000)
    sync_api.expect(page.locator("#btn-remap")).to_be_hidden(timeout=3000)
    assert not errors, errors
    page.context.close()


def test_a_double_tap_asks_once(browser, twin_server):
    page, errors = _open(browser, twin_server)
    seen = []
    _answer(page, 200, {"remapping": True}, seen, delay_ms=600)
    page.click("#btn-remap")
    page.click("#btn-remap", force=True, timeout=1000, no_wait_after=True)
    page.wait_for_timeout(1200)
    assert len(seen) == 1, seen
    sync_api.expect(page.locator("#btn-remap")).to_be_enabled()
    assert not errors, errors
    page.context.close()


def test_the_phone_screenshot_of_the_control(browser, twin_server):
    """Evidence (CLAUDE.md section 7): the Sim tab at 390 px on a SLAM map,
    the control under the map, no sideways scroll."""
    page, errors = _open(browser, twin_server)
    assert page.viewport_size == PHONE
    sync_api.expect(page.locator("#btn-remap")).to_be_visible(timeout=5000)
    width = page.evaluate("document.documentElement.scrollWidth")
    assert width <= PHONE["width"], f"the page scrolls sideways ({width} px)"
    box = page.locator("#btn-remap").bounding_box()
    assert box["height"] >= 32, f"the control is too small to tap ({box})"
    out = os.environ.get("UI_SHOTS")
    if out:
        page.locator("#btn-remap").scroll_into_view_if_needed()
        sync_api.expect(page.locator(".toast")).to_have_count(0, timeout=6000)
        page.screenshot(path=os.path.join(out, "3.50-D4b-remap-390.png"))
    assert not errors, errors
    page.context.close()
