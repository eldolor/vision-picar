"""
tests/test_ui_review_376d.py

`PLAN-ros-alignment.md` 3.76, batch D: the twin bugs the whole-file review
of `web-twin/app.js` confirmed. Each test drives the real page in a real
browser at a phone's width, against a live robot server (`tests/test_ui.py`
fixtures), and was written before its fix and seen to fail on `dev`.
"""

import json

import pytest

pytest.importorskip("playwright")

from tests.test_ui import PHONE, browser, twin_server  # noqa: F401,E402 -- fixtures


def _page(browser, url, seed):
    context = browser.new_context(viewport=PHONE, permissions=["camera"])
    context.add_init_script(
        "(() => { const s = %s;"
        " try { for (const k in s) localStorage.setItem(k, s[k]); } catch (e) {} })();"
        % json.dumps(seed))
    page = context.new_page()
    errors = []
    page.on("pageerror", lambda e: errors.append(str(e)))
    return page, errors


# ---------- a setup link cannot send the saved secrets elsewhere ----------


@pytest.mark.parametrize("param,secret_key,header_host", [
    ("serverUrl", "vp_cfg_server_secret", "robot"),
    ("brainUrl", "vp_cfg_brain_secret", "brain"),
])
def test_a_link_that_changes_a_service_address_does_not_carry_its_saved_secret(
        browser, twin_server, param, secret_key, header_host):
    """The setup link's URL parameters were stored and applied with no
    secret in the link, and the secret already saved on the phone stayed:
    the page then auto-connected and sent it to whatever host the link
    named. A link that moves a service elsewhere must bring that service's
    secret, or the page asks for it."""
    seed = {"vp_server_url": twin_server, "vp_brain_url": twin_server,
            secret_key: "the-real-secret"}
    page, errors = _page(browser, twin_server, seed)
    seen = []

    def evil(route):
        seen.append(route.request.headers.get("x-app-secret"))
        route.fulfill(status=200, content_type="application/json", body="{}")

    page.route("http://evil.test/**", evil)
    page.goto(f"{twin_server}/?{param}=http://evil.test", wait_until="networkidle")
    page.wait_for_timeout(1500)
    assert "the-real-secret" not in seen, f"the saved {header_host} secret went to evil.test"
    assert not errors, errors
    page.context.close()


def test_a_link_to_the_same_address_keeps_the_saved_secret(browser, twin_server):
    """The setup QR's own case: same host, no secret in the link."""
    seed = {"vp_server_url": twin_server, "vp_cfg_server_secret": "kept"}
    page, _ = _page(browser, twin_server, seed)
    page.goto(f"{twin_server}/?serverUrl={twin_server}", wait_until="networkidle")
    assert page.input_value("#cfg-server-secret") == "kept"
    page.context.close()


# ---------- Close while Start is still awaiting ----------


def test_closing_the_camera_while_start_awaits_starts_nothing(browser, twin_server):
    """The fullscreen's Close is live from the moment the camera opens, but
    Start still awaited the brain's /health (recording) -- and video
    metadata, /mission/start -- and then carried on regardless: a run with
    no camera, out of sight, making calls no Stop could reach."""
    page, errors = _page(browser, twin_server, {
        "guidanceMode": "robot", "vp_vision_url": twin_server,
        "vp_brain_url": "http://brain.test", "vp_record_walk": "1",
        "vp_guide_onboarded": "1"})
    held, navigate = [], []
    armed = {"on": False}

    def brain(route):
        if route.request.url.endswith("/health") and armed["on"]:
            held.append(route)                     # the start's recording check, held
            return
        route.fulfill(status=200, content_type="application/json",
                      body=json.dumps({"ok": True, "recording_allowed": True,
                                       "running": False, "outcome": "idle"}))

    page.route("http://brain.test/**", brain)
    page.route("**/navigate", lambda route: (navigate.append(1), route.fulfill(
        status=200, content_type="application/json", body=json.dumps({"action": "WAIT"})))[1])
    page.goto(twin_server, wait_until="networkidle")
    page.click("#btn-settings")
    page.click("#btn-brain-connect")
    page.wait_for_timeout(500)
    armed["on"] = True
    page.evaluate("document.getElementById('btn-guidance').click()")
    page.wait_for_function("document.getElementById('guide-fullscreen')"
                           " && document.getElementById('guide-fullscreen').classList.contains('active')",
                           timeout=5000)
    for _ in range(50):
        if held:
            break
        page.wait_for_timeout(100)
    assert held, "the start never reached its recording check -- the test proves nothing"
    page.evaluate("document.getElementById('btn-guidance-close').click()")
    for r in held:
        r.fulfill(status=200, content_type="application/json",
                  body=json.dumps({"ok": True, "recording_allowed": True}))
    page.wait_for_timeout(1500)
    assert not navigate, f"{len(navigate)} paid calls after Close"
    assert page.inner_text("#btn-guidance").strip() == "Start"
    assert not errors, errors
    page.context.close()


# ---------- an older call's failure does not overwrite a newer answer ----------


def test_an_older_failure_landing_after_a_newer_answer_is_not_drawn(browser, twin_server):
    """Robot view keeps two calls in flight. The success path drops an answer
    older than the one already drawn; the failure path did not, so a slow
    failure from call 1 landing after call 2's answer put the error over
    the fresh decision and started a backoff."""
    page, errors = _page(browser, twin_server, {
        "guidanceMode": "robot", "vp_vision_url": twin_server, "vp_guide_onboarded": "1"})
    held, calls = [], []

    def navigate(route):
        calls.append(route)
        if len(calls) == 2:
            route.fulfill(status=200, content_type="application/json", body=json.dumps(
                {"action": "FORWARD", "reasoning": "clear path ahead", "confidence": 0.9}))
        else:
            held.append(route)                         # call 1, and any later ones

    page.route("**/navigate", navigate)
    page.goto(twin_server, wait_until="networkidle")
    page.evaluate("document.getElementById('btn-guidance').click()")
    for _ in range(60):
        if len(calls) >= 3:
            break
        page.wait_for_timeout(100)
    assert len(calls) >= 3, f"only {len(calls)} calls -- the test proves nothing"
    held[0].fulfill(status=500, content_type="application/json",
                    body=json.dumps({"detail": "boom-old"}))
    page.wait_for_timeout(400)
    caption = page.inner_text("#guide-caption-text")
    for r in held[1:]:
        r.fulfill(status=200, content_type="application/json", body=json.dumps({"action": "WAIT"}))
    page.evaluate("document.getElementById('btn-guidance-close').click()")
    assert "boom-old" not in caption, caption
    assert not errors, errors
    page.context.close()


# ---------- the Sim tab's brain panel keeps watching a running mission ----------


def _brain_running(page, statuses, stop_status=500):
    """A brain whose mission is running; `statuses` lists the HTTP codes the
    next /mission/status calls answer with (200 = running)."""
    seen = {"status": 0}

    def brain(route):
        url = route.request.url
        if url.endswith("/health"):
            return route.fulfill(status=200, content_type="application/json",
                                 body=json.dumps({"ok": True, "robot_url": "x"}))
        if url.endswith("/mission/stop"):
            return route.fulfill(status=stop_status, content_type="application/json",
                                 body=json.dumps({"detail": "stop failed"}))
        if "/mission/status" in url:
            seen["status"] += 1
            code = statuses.pop(0) if statuses else 200
            return route.fulfill(status=code, content_type="application/json", body=json.dumps(
                {"running": True, "outcome": "running", "step": seen["status"], "log_tail": []}
                if code == 200 else {"detail": "boom"}))
        route.fulfill(status=200, content_type="application/json", body="{}")

    page.route("http://brain.test/**", brain)
    return seen


def test_one_brain_error_does_not_stop_the_panel_watching(browser, twin_server):
    """Any status error other than "the brain is gone" stopped polling for
    good, with the mission still shown running: the panel froze while the
    robot drove on."""
    page, errors = _page(browser, twin_server, {"vp_brain_url": "http://brain.test"})
    seen = _brain_running(page, [200, 200, 200, 500])   # the page connects on its own at load
    page.goto(twin_server, wait_until="networkidle")
    page.wait_for_timeout(2500)
    after_error = seen["status"]
    page.wait_for_timeout(2000)
    assert seen["status"] > after_error, "the panel stopped polling after one error"
    assert not errors, errors
    page.context.close()


def test_a_failed_stop_keeps_the_panel_watching(browser, twin_server):
    """A failed Stop also stopped the poll, in its `finally`."""
    page, errors = _page(browser, twin_server, {"vp_brain_url": "http://brain.test"})
    seen = _brain_running(page, [], stop_status=500)
    page.goto(twin_server, wait_until="networkidle")
    page.wait_for_timeout(1500)
    page.evaluate("document.getElementById('btn-brain-mission').click()")
    page.wait_for_timeout(1000)
    after_stop = seen["status"]
    page.wait_for_timeout(2000)
    assert seen["status"] > after_stop, "a failed Stop stopped the panel watching"
    assert not errors, errors
    page.context.close()


# ---------- a recorded walk's name is one the brain accepts ----------


def test_a_long_model_and_prompt_still_give_a_walk_name_the_brain_accepts(browser, twin_server):
    """The name's parts can add to 82 characters; the brain refuses any walk
    name over 64 (`WALK_NAME`, control/recording_routes.py) on every frame,
    so the whole walk was lost behind one toast."""
    import re
    from control.recording_routes import WALK_NAME
    page, errors = _page(browser, twin_server, {
        "guidanceMode": "robot", "vp_vision_url": twin_server, "vp_guide_onboarded": "1",
        "vp_brain_url": "http://brain.test", "vp_record_walk": "1",
        "vp_navigate_model_id": "qwen.qwen3-vl-235b-a22b",
        "vp_navigate_prompt_variant": "default-with-distance"})
    page.route("**/navigate/models", lambda route: route.fulfill(
        status=200, content_type="application/json", body=json.dumps({
            "default": "qwen.qwen3-vl-235b-a22b", "default_prompt": "default",
            "prompts": ["default", "default-with-distance"],
            "models": [{"id": "qwen.qwen3-vl-235b-a22b", "label": "Qwen3-VL"}]})))
    page.route("**/navigate", lambda route: route.fulfill(
        status=200, content_type="application/json", body=json.dumps({"action": "WAIT"})))
    walks = []

    def brain(route):
        if route.request.url.endswith("/recording/frame"):
            walks.append(json.loads(route.request.post_data)["walk"])
        route.fulfill(status=200, content_type="application/json",
                      body=json.dumps({"ok": True, "recording_allowed": True, "running": False}))

    page.route("http://brain.test/**", brain)
    page.goto(twin_server, wait_until="networkidle")
    page.wait_for_timeout(800)
    page.evaluate("document.getElementById('btn-guidance').click()")
    for _ in range(50):
        if walks:
            break
        page.wait_for_timeout(100)
    page.evaluate("document.getElementById('btn-guidance-close').click()")
    assert walks, "no frame was recorded -- the test proves nothing"
    assert WALK_NAME.match(walks[0]), f"{walks[0]!r} ({len(walks[0])} chars) is refused by the brain"
    assert "qwen3-vl" in walks[0] and re.search(r"-\d{8}-\d{6}$", walks[0]), walks[0]
    assert not errors, errors
    page.context.close()


def test_a_walk_whose_saves_are_still_in_flight_at_stop_is_still_finished(browser, twin_server):
    """`/recording/finish` was sent only if a save had already landed by
    Stop: a short walk on a slow link was never finished, and otherwise the
    finish could land before the walk's last frames."""
    page, errors = _page(browser, twin_server, {
        "guidanceMode": "robot", "vp_vision_url": twin_server, "vp_guide_onboarded": "1",
        "vp_brain_url": "http://brain.test", "vp_record_walk": "1"})
    page.route("**/navigate", lambda route: route.fulfill(
        status=200, content_type="application/json", body=json.dumps({"action": "WAIT"})))
    held, order = [], []

    def brain(route):
        url = route.request.url
        if url.endswith("/recording/frame"):
            held.append(route)                         # a slow link: every save in flight
            return
        if url.endswith("/recording/finish"):
            order.append("finish")
        route.fulfill(status=200, content_type="application/json",
                      body=json.dumps({"ok": True, "recording_allowed": True, "running": False}))

    page.route("http://brain.test/**", brain)
    page.goto(twin_server, wait_until="networkidle")
    page.wait_for_timeout(800)
    page.evaluate("document.getElementById('btn-guidance').click()")
    for _ in range(50):
        if held:
            break
        page.wait_for_timeout(100)
    assert held, "no frame was recorded -- the test proves nothing"
    page.evaluate("document.getElementById('btn-guidance-close').click()")
    page.wait_for_timeout(200)
    assert not order, "the walk was finished before its frames landed"
    for r in held:
        order.append("frame")
        r.fulfill(status=200, content_type="application/json", body="{}")
    page.wait_for_timeout(800)
    assert order and order[-1] == "finish", order
    assert not errors, errors
    page.context.close()


# ---------- a D-pad press that never reached the robot says so ----------


def test_a_stop_that_never_reached_the_robot_says_so(browser, twin_server):
    """The D-pad's handlers had no catch: a STOP lost to a dropped tunnel
    showed nothing -- no toast, no log line, "Safety: OK" -- so a person
    could not tell a stop from one that never arrived."""
    page, errors = _page(browser, twin_server, {})
    page.goto(twin_server, wait_until="networkidle")
    page.wait_for_timeout(800)
    page.route(f"{twin_server}/stop", lambda route: route.abort())
    page.click('button.tab-btn[data-tab="sim"]')
    page.evaluate("document.getElementById('btn-stop').click()")
    toast = page.locator(".toast", has_text="STOP did not reach the robot")
    toast.first.wait_for(timeout=3000)
    log = page.inner_text("#log")
    assert "NOT SENT" in log and "VETOED" not in log, log
    page.context.close()
