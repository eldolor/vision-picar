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
                           _stub_world, browser, open_twin, private_twin_server, twin_server)


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


def _answer(page, status, body, seen=None):
    def handler(route):
        if seen is not None:
            seen.append({"method": route.request.method,
                         "driver": route.request.headers.get("x-driver")})
        route.fulfill(status=status, content_type="application/json", body=json.dumps(body))
    page.route("**/world/remap", handler)


def _hold(page):
    """Requests to /world/remap held unanswered; the test fulfils them."""
    held = []
    page.route("**/world/remap", lambda route: held.append(route))
    return held


def _remap(page):
    """The two taps a re-map takes: arm, then (past the double-tap gap) ask."""
    page.click("#btn-remap")
    sync_api.expect(page.locator("#btn-remap")).to_have_text("Tap again to re-map")
    page.wait_for_timeout(500)
    page.click("#btn-remap")


def _reconnect(page, url):
    page.click("#btn-settings")
    page.fill("#cfg-server-url", url)
    page.click("#btn-connect")
    page.click('.tab-btn[data-tab="sim"]')


def test_the_button_shows_on_a_slam_map_only(browser, twin_server):
    page, errors = _open(browser, twin_server)
    sync_api.expect(page.locator("#btn-remap")).to_be_visible(timeout=5000)
    assert not errors, errors
    page.context.close()
    sim = dict(_SLAM_MAP, map_id="sim-grid")
    page, errors = _open(browser, twin_server, grid=sim, pose=dict(_SLAM_POSE, map_id="sim-grid"))
    page.wait_for_timeout(500)
    sync_api.expect(page.locator("#btn-remap")).to_be_hidden()
    assert not errors, errors
    page.context.close()


def test_one_tap_only_arms_it(browser, twin_server):
    """Review round 1: a re-map can cancel a live goal, the brain's too, so a
    stray tap must send nothing."""
    page, errors = _open(browser, twin_server)
    seen = []
    _answer(page, 200, {"remapping": True}, seen)
    page.click("#btn-remap")
    sync_api.expect(page.locator("#btn-remap")).to_have_text("Tap again to re-map")
    sync_api.expect(page.locator("#btn-remap")).to_have_text("Re-map now", timeout=8000)
    assert seen == [], seen
    assert not errors, errors
    page.context.close()


def test_a_double_tap_only_arms_it(browser, twin_server):
    """Review round 2: two taps a few ms apart (a double-tap, a bounce) passed
    the arm and sent the re-map."""
    page, errors = _open(browser, twin_server)
    seen = []
    _answer(page, 200, {"remapping": True}, seen)
    page.dblclick("#btn-remap")
    page.wait_for_timeout(500)
    assert seen == [], seen
    sync_api.expect(page.locator("#btn-remap")).to_have_text("Tap again to re-map")
    assert not errors, errors
    page.context.close()


def test_two_taps_ask_the_robot_server_as_a_person(browser, twin_server):
    page, errors = _open(browser, twin_server)
    seen = []
    _answer(page, 200, {"remapping": True, "trigger": "request", "map_id": "home", "version": 2},
            seen)
    _remap(page)
    toast = page.locator(".toast").last
    sync_api.expect(toast).to_contain_text("Re-mapping", timeout=3000)
    sync_api.expect(toast).to_contain_text("may be cancelled")
    assert seen == [{"method": "POST", "driver": "twin-dpad"}], seen
    assert not errors, errors
    page.context.close()


def test_a_refusal_says_the_servers_reason(browser, twin_server):
    page, errors = _open(browser, twin_server)
    _answer(page, 409, {"detail": "re-map only from a reloaded map being localised on "
                                  "(slam ready, mode mapping)"})
    _remap(page)
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
    _remap(page)
    sync_api.expect(page.locator(".toast").last).to_contain_text("no saved map", timeout=3000)
    sync_api.expect(page.locator("#btn-remap")).to_be_hidden(timeout=3000)
    assert not errors, errors
    page.context.close()


def test_one_request_at_a_time(browser, twin_server):
    page, errors = _open(browser, twin_server)
    held = _hold(page)
    _remap(page)
    btn = page.locator("#btn-remap")
    sync_api.expect(btn).to_have_text("Asking\u2026")
    sync_api.expect(btn).to_be_disabled()
    page.wait_for_timeout(300)
    assert len(held) == 1, held
    held[0].fulfill(status=200, content_type="application/json",
                    body=json.dumps({"remapping": True}))
    sync_api.expect(btn).to_have_text("Re-map now", timeout=3000)
    sync_api.expect(btn).to_be_enabled()
    assert len(held) == 1
    assert not errors, errors
    page.context.close()


def test_a_lost_connection_takes_the_button_away(browser, twin_server):
    """Review round 1: the button was set only when the map was drawn, so a
    failed reconnect left it live, pointed at a dead server."""
    page, errors = _open(browser, twin_server)
    sync_api.expect(page.locator("#btn-remap")).to_be_visible(timeout=5000)
    _reconnect(page, "http://127.0.0.1:9")
    sync_api.expect(page.locator("#btn-remap")).to_be_hidden(timeout=8000)
    assert not errors, errors
    page.context.close()


def test_an_answer_from_another_server_is_dropped(browser, twin_server, private_twin_server):
    """Review round 1: a 501 from the last server, landing after a reconnect
    to another, hid the button for the new one."""
    page, errors = _open(browser, twin_server)
    held = _hold(page)
    _remap(page)
    page.wait_for_timeout(300)
    assert len(held) == 1
    _reconnect(page, private_twin_server)
    sync_api.expect(page.locator("#btn-remap")).to_have_text("Re-map now", timeout=8000)
    held[0].fulfill(status=501, content_type="application/json",
                    body=json.dumps({"detail": "this world keeps no saved map"}))
    page.wait_for_timeout(500)
    sync_api.expect(page.locator("#btn-remap")).to_be_visible()
    sync_api.expect(page.locator("#btn-remap")).to_be_enabled()
    assert not errors, errors
    page.context.close()


def test_a_reconnect_to_the_same_server_keeps_its_answer(browser, twin_server):
    """Review round 2: a reconnect to the same server dropped the answer of a
    re-map that server took, and let a second one be asked meanwhile."""
    page, errors = _open(browser, twin_server)
    held = _hold(page)
    _remap(page)
    page.wait_for_timeout(300)
    _reconnect(page, twin_server)
    page.wait_for_timeout(1000)
    sync_api.expect(page.locator("#btn-remap")).to_have_text("Asking\u2026")
    held[0].fulfill(status=200, content_type="application/json",
                    body=json.dumps({"remapping": True}))
    sync_api.expect(page.locator(".toast").last).to_contain_text("Re-mapping", timeout=3000)
    sync_api.expect(page.locator("#btn-remap")).to_have_text("Re-map now")
    assert len(held) == 1
    assert not errors, errors
    page.context.close()


def test_away_and_back_does_not_put_two_requests_out(browser, twin_server, private_twin_server):
    """Review round 3: a request was known only by its URL, so after A, B and
    back to A the first request's end freed the button while the second was
    still out, and showed the first one's answer."""
    page, errors = _open(browser, twin_server)
    held = _hold(page)
    _remap(page)
    page.wait_for_timeout(300)
    _reconnect(page, private_twin_server)
    sync_api.expect(page.locator("#btn-remap")).to_have_text("Re-map now", timeout=8000)
    _reconnect(page, twin_server)
    sync_api.expect(page.locator("#btn-remap")).to_have_text("Re-map now", timeout=8000)
    _remap(page)
    page.wait_for_timeout(300)
    assert len(held) == 2, held
    held[0].fulfill(status=409, content_type="application/json",
                    body=json.dumps({"detail": "first request's answer"}))
    page.wait_for_timeout(500)
    btn = page.locator("#btn-remap")
    sync_api.expect(btn).to_have_text("Asking\u2026")
    sync_api.expect(btn).to_be_disabled()
    assert "first request" not in (page.locator("#toast-stack").text_content() or "")
    held[1].fulfill(status=200, content_type="application/json",
                    body=json.dumps({"remapping": True}))
    sync_api.expect(btn).to_have_text("Re-map now", timeout=3000)
    assert not errors, errors
    page.context.close()


def test_a_hung_request_is_given_up(browser, twin_server):
    """Review round 3: with nothing to end it, a request that never answered
    held the button at "Asking..." until the page was reloaded."""
    page, errors = _open(browser, twin_server)
    # The limits are `var`s in the page's closure: the clock is moved instead,
    # installed before the taps so their timers are the fake clock's.
    page.clock.install()
    held = _hold(page)
    _remap(page)
    page.wait_for_timeout(300)
    assert len(held) == 1
    page.clock.run_for(21000)
    sync_api.expect(page.locator(".toast").last).to_contain_text("may have started")
    sync_api.expect(page.locator("#btn-remap")).to_have_text("Asking\u2026")
    page.clock.run_for(40000)
    sync_api.expect(page.locator(".toast").last).to_contain_text("stopped waiting", timeout=3000)
    sync_api.expect(page.locator("#btn-remap")).to_have_text("Re-map now")
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
        page.click("#btn-remap")
        sync_api.expect(page.locator("#btn-remap")).to_have_text("Tap again to re-map")
        assert page.evaluate("document.documentElement.scrollWidth") <= PHONE["width"]
        page.screenshot(path=os.path.join(out, "3.50-D4b-remap-armed-390.png"))
    assert not errors, errors
    page.context.close()
