"""
tests/test_ui.py

The twin's client, in a real browser. Until this file existed the UI was
the one component of this project with no automated coverage at all --
which is an awkward gap given CLAUDE.md section 7's rule that a phase is
not done until someone holding a phone can watch it work. The twin is the
designated proof surface for everything, and nothing proved the twin.

**Written against two bugs that actually shipped**, both on 2026-08-29,
both found by a person looking at a phone rather than by the 283 passing
Python tests:

  1. The /navigate model picker never populated. fetchNavigateModels() ran
     during init, before #cfg-url had been restored from localStorage, so
     it bailed on an empty URL -- and it set its "already loaded" flag
     BEFORE the request, so the one early failure latched and it never
     retried. The page looked fine and the list was permanently empty.
  2. The picker rendered as a ~40px chevron with no readable text.
     .switch-row is flex with space-between, built for rows whose control
     is a fixed-width toggle; the model picker is the only control in the
     app that carries text, and the description squeezed it to nothing.

Note what each needs. (1) is a load-order bug a jsdom test could catch.
(2) is invisible to any DOM-only test -- it needs layout, at a phone
viewport, which is why this is Playwright and not jsdom.

Driven from pytest so `pytest tests/ -q` stays the single invocation
CLAUDE.md documents. Skipped cleanly (not failed) when Playwright or its
browser isn't installed, so a checkout without `playwright install
chromium` still runs the rest of the suite:

    pip install pytest-playwright && python -m playwright install chromium

The vision service is never contacted: GET /navigate/models is fulfilled
by Playwright request interception, so these tests exercise the client's
own logic without a second live service or a paid call.
"""

import os
import re
import subprocess
import sys
import time
from pathlib import Path

import httpx
import pytest

from tests.conftest import REPO_ROOT, SERVER_START_TIMEOUT_S, free_port

sync_api = pytest.importorskip(
    "playwright.sync_api",
    reason="UI tests need `pip install pytest-playwright` + `playwright install chromium`",
)

# The phone this project is actually used from. The squeeze bug only
# appears at a narrow width -- at desktop width the picker had room and
# looked fine, which is exactly why it reached a phone.
PHONE = {"width": 390, "height": 844}

MODELS_REPLY = {
    "default": "us.anthropic.claude-opus-4-5-20251101-v1:0",
    "default_prompt": "default",
    "prompts": ["default", "next-step-obstacle"],
    "models": [
        {"id": "us.anthropic.claude-opus-4-5-20251101-v1:0", "label": "Claude Opus 4.5 (best judgement)"},
        {"id": "us.anthropic.claude-sonnet-4-5-20250929-v1:0", "label": "Claude Sonnet 4.5 (cautious)"},
        {"id": "qwen.qwen3-vl-235b-a22b", "label": "Qwen3-VL (non-Anthropic)"},
        {"id": "amazon.nova-lite-v1:0", "label": "Nova Lite (cheap baseline)"},
    ],
}


@pytest.fixture(scope="module")
def twin_server():
    """A live `uvicorn robot.server:app`, same subprocess pattern as
    tests/test_watchdog_integration.py -- a real server serving the real
    index.html and app.js, not a fixture copy of either."""
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
        b = p.chromium.launch(
            # Lets a test load the page from a host that is neither
            # localhost nor the service host, which is the only shape
            # in which the endpoint notice is supposed to appear.
            args=[
                # Lets a test load the page from a host that is neither
                # localhost nor the service host, which is the only shape
                # in which the endpoint notice is supposed to appear.
                "--host-resolver-rules=MAP vision-picar.test 127.0.0.1",
                # Without a fake camera getUserMedia never attaches a
                # stream, Robot view never starts, and anything downstream
                # of pressing Start is untestable -- silently, since the
                # page reports no error of its own.
                "--use-fake-ui-for-media-stream",
                "--use-fake-device-for-media-stream",
            ])
        yield b
        b.close()


def open_twin(browser, twin_server, *, saved_vision_url=True, mode="robot", models=None):
    """A phone-sized page with the twin loaded.

    `saved_vision_url` seeds localStorage the way a returning user's browser
    would -- which is the state bug (1) needed: the URL exists in storage
    but is not yet in the DOM when init runs.
    """
    context = browser.new_context(viewport=PHONE)
    seed = {"guidanceMode": mode}
    if saved_vision_url:
        seed["vp_vision_url"] = twin_server
    context.add_init_script(
        "(() => { const s = %s;"
        " try { for (const k in s) localStorage.setItem(k, s[k]); } catch (e) {} })();"
        % _json(seed)
    )
    page = context.new_page()
    page.route("**/navigate/models", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json(models or MODELS_REPLY)))
    errors = []
    page.on("pageerror", lambda e: errors.append(str(e)))
    page.goto(twin_server, wait_until="networkidle")
    return page, errors


def _json(obj):
    import json
    return json.dumps(obj)


def model_select(page):
    return page.locator("#cfg-navigate-model")


# ---------- the page works at all ----------


def test_the_page_loads_without_a_script_error(browser, twin_server):
    """app.js is external now; a broken route or a parse error would leave
    HTML that renders and does nothing. The Python side asserts the route
    serves the file -- this asserts the browser can actually run it."""
    page, errors = open_twin(browser, twin_server)
    assert errors == [], f"uncaught page errors: {errors}"
    assert page.locator("#btn-mode-robot").is_visible()
    page.close()


# ---------- regression: the picker never populated ----------


def test_the_model_picker_populates_on_a_normal_page_load(browser, twin_server):
    """Bug (1). The saved vision URL is in localStorage but not yet in the
    DOM when setGuidanceMode() runs during init, so the picker's own early
    fetch attempt finds nothing -- something later has to actually fill it.

    Asserting on option COUNT, not just "no error": the broken build also
    threw nothing. It just silently offered "Service default" forever.
    """
    page, _ = open_twin(browser, twin_server)
    select = model_select(page)
    sync_api.expect(select).to_be_visible()
    # 1 page-owned "Service default" + the service's 4
    sync_api.expect(select.locator("option")).to_have_count(len(MODELS_REPLY["models"]) + 1)
    assert "Claude Opus 4.5" in select.locator("option").nth(1).inner_text()
    page.close()


def test_a_failed_model_fetch_does_not_latch_the_picker_shut(browser, twin_server):
    """The precise mechanism of bug (1): the "already loaded" flag was set
    before the request, so one failure was permanent. Here the first fetch
    500s and a later one succeeds -- the picker must recover."""
    context = browser.new_context(viewport=PHONE)
    context.add_init_script(
        '(() => { try { localStorage.setItem("guidanceMode","robot"); } catch(e){} })();')
    page = context.new_page()

    calls = {"n": 0}

    def flaky(route):
        calls["n"] += 1
        if calls["n"] == 1:
            route.fulfill(status=500, body="nope")
        else:
            route.fulfill(status=200, content_type="application/json", body=_json(MODELS_REPLY))

    page.route("**/navigate/models", flaky)
    page.goto(twin_server, wait_until="networkidle")

    # No saved URL, so init's attempt is a no-op; setting one must retry.
    # The field lives in a collapsed Settings panel, and getting there is
    # not what this test is about -- set the value and fire the same change
    # event the panel would, which is the handler under test.
    def set_vision_url(value):
        page.evaluate(
            "(v) => { const el = document.getElementById('cfg-url'); el.value = v;"
            " el.dispatchEvent(new Event('change')); }", value)

    set_vision_url(twin_server)          # first fetch: 500, must not latch
    set_vision_url(twin_server + "/")    # second: succeeds

    sync_api.expect(model_select(page).locator("option")).to_have_count(
        len(MODELS_REPLY["models"]) + 1)
    page.close()


# ---------- regression: the picker was unreadable ----------


def test_the_model_picker_is_readable_on_a_phone(browser, twin_server):
    """Bug (2), and the reason this file uses a real browser: the select
    collapsed to about 40px beside its own description, so the chosen model
    had nowhere to render. Nothing about the DOM was wrong -- only layout.
    """
    page, _ = open_twin(browser, twin_server)
    select = model_select(page)
    sync_api.expect(select).to_be_visible()
    box = select.bounding_box()
    assert box is not None
    assert box["width"] >= 150, (
        f"the model picker is {box['width']:.0f}px wide at {PHONE['width']}px -- "
        "too narrow to show a model name (it regressed to ~40px once)"
    )
    page.close()


def test_the_selected_model_is_visible_after_choosing_one(browser, twin_server):
    """The user-visible symptom, asserted end to end: pick a model, and the
    control shows which one."""
    page, _ = open_twin(browser, twin_server)
    select = model_select(page)
    select.select_option("qwen.qwen3-vl-235b-a22b")
    assert select.input_value() == "qwen.qwen3-vl-235b-a22b"
    chosen = page.evaluate(
        "() => { const s = document.getElementById('cfg-navigate-model');"
        " return s.options[s.selectedIndex].textContent.trim(); }")
    assert "Qwen3-VL" in chosen
    page.close()


def test_the_choice_survives_a_reload(browser, twin_server):
    """It is persisted to localStorage and re-applied only if the service
    still offers it -- so a walk started after a reload runs the model the
    picker is showing, not silently the default."""
    page, _ = open_twin(browser, twin_server)
    model_select(page).select_option("amazon.nova-lite-v1:0")
    page.reload(wait_until="networkidle")
    sync_api.expect(model_select(page)).to_have_value("amazon.nova-lite-v1:0")
    page.close()


# ---------- mode switching ----------


def test_robot_only_rows_are_hidden_in_guide_mode(browser, twin_server):
    """The picker, recording and drive-via-brain belong to Robot view; Guide
    steers a person and has no model to choose."""
    page, _ = open_twin(browser, twin_server, mode="guide")
    sync_api.expect(page.locator("#navigate-model-row")).to_be_hidden()

    page.click("#btn-mode-robot")
    sync_api.expect(page.locator("#navigate-model-row")).to_be_visible()
    sync_api.expect(page.locator("#record-walk-row")).to_be_visible()
    page.close()


# ---------- connection status ----------


def test_connection_status_reads_as_a_state_not_a_sentence(browser, twin_server):
    """These two readouts are how you find out, standing in a room holding a
    phone, whether the thing you are about to drive is reachable. They used
    to be dim grey body text that looked identical connected or dead, with a
    long URL burying the one word that mattered.

    Asserts the structure that makes the state legible: a dot, a label
    carrying the state on its own, and the URL demoted to a detail line.
    """
    page, _ = open_twin(browser, twin_server)
    for eid in ("connection-status", "brain-connection-status"):
        el = page.locator("#" + eid)
        assert el.locator(".conn-status-dot").count() == 1, eid
        label = el.locator(".conn-status-label")
        # State-agnostic: the fixture's twin is live, so the robot panel may
        # legitimately have auto-connected by now. What matters is that the
        # label carries a state on its own, without a URL in it.
        text = label.inner_text().strip()
        assert text in {"Connected", "Not connected", "Connecting\u2026",
                        "Reconnecting\u2026", "No URL yet"}, f"{eid}: {text!r}"
        assert "http" not in text, f"{eid} label should not carry a URL: {text!r}"
        # The label must be visually stronger than the surrounding hint text,
        # or this is the same unreadable line with extra markup.
        weight = page.evaluate(
            "(id) => getComputedStyle(document.querySelector('#'+id+' .conn-status-label'))"
            ".fontWeight", eid)
        assert int(weight) >= 600, f"{eid} label is not bold ({weight})"
    page.close()


def test_a_live_connection_is_visually_distinct_from_a_dead_one(browser, twin_server):
    """The original bug in miniature: connected and not-connected rendered in
    exactly the same colour, so the state could only be read by parsing the
    sentence."""
    page, _ = open_twin(browser, twin_server)
    el = page.locator("#connection-status")

    def label_colour():
        return page.evaluate(
            "() => getComputedStyle(document.querySelector("
            "'#connection-status .conn-status-label')).color")

    def set_state(cls):
        page.evaluate(
            "(c) => { const el = document.getElementById('connection-status');"
            " el.classList.remove('is-ok','is-err','is-busy'); el.classList.add(c); }", cls)

    set_state("is-err")
    dead = label_colour()
    set_state("is-ok")
    alive = label_colour()

    assert dead != alive, "connected and disconnected look identical"
    assert el.evaluate("el => el.classList.contains('is-ok')")
    page.close()


def test_the_prompt_picker_appears_when_the_service_offers_a_choice(browser, twin_server):
    """The other axis. The stall this project spent days on was a wording
    problem, so being able to pick the wording matters at least as much as
    picking the model."""
    page, _ = open_twin(browser, twin_server)
    select = page.locator("#cfg-navigate-prompt")
    sync_api.expect(select).to_be_visible()
    sync_api.expect(select.locator("option")).to_have_count(
        len(MODELS_REPLY["prompts"]) + 1)  # + "Service default"

    select.select_option("next-step-obstacle")
    assert select.input_value() == "next-step-obstacle"
    page.close()


def test_the_prompt_picker_hides_when_there_is_only_one_wording(browser, twin_server):
    """A single variant is not a choice, and a control that never does
    anything is worse than no control."""
    single = dict(MODELS_REPLY, prompts=["default"])
    page, _ = open_twin(browser, twin_server, models=single)
    sync_api.expect(page.locator("#navigate-prompt-row")).to_be_hidden()
    page.close()


def test_service_default_says_what_it_resolves_to(browser, twin_server):
    """"Service default" and an explicit pick of the same model or prompt
    produce identical walks -- the only difference is whether the choice is
    sent and recorded. Two options that silently mean the same thing is a
    trap, so each names what it currently resolves to."""
    page, _ = open_twin(browser, twin_server)

    model_placeholder = page.locator("#cfg-navigate-model option").first.inner_text()
    assert "Service default" in model_placeholder
    assert "opus-4-5" in model_placeholder, model_placeholder

    prompt_placeholder = page.locator("#cfg-navigate-prompt option").first.inner_text()
    assert "Service default" in prompt_placeholder
    assert MODELS_REPLY["default_prompt"] in prompt_placeholder, prompt_placeholder
    page.close()


# ---------- the endpoint notice ----------
#
# A saved vision URL outlives the page that saved it. Opening a second
# deployment on a phone that has used the first leaves the new page calling
# the old service, and the only symptom is a 401 -- which reads as a wrong
# secret rather than a call to the wrong place. This cost a real debugging
# session before the notice existed.


def _open_at_alias_host(browser, twin_server, vision_url):
    """The twin loaded from a host that is neither localhost nor the vision
    service, with `vision_url` already saved the way a previous deployment
    would have left it."""
    port = twin_server.rsplit(":", 1)[1]
    page_url = f"http://vision-picar.test:{port}/"
    context = browser.new_context(viewport=PHONE)
    context.add_init_script(
        "(() => { try { localStorage.setItem('vp_vision_url', %s); } catch (e) {} })();"
        % _json(vision_url)
    )
    page = context.new_page()
    errors = []
    page.on("pageerror", lambda e: errors.append(str(e)))
    page.goto(page_url, wait_until="networkidle")
    page.wait_for_timeout(800)
    page.click("#btn-settings")
    page.wait_for_timeout(300)
    return page, errors, f"vision-picar.test:{port}"


def test_no_endpoint_notice_when_the_service_is_this_page(browser, twin_server):
    """The common case, and the reason this is a notice rather than a
    warning: it must be silent when nothing is unusual."""
    port = twin_server.rsplit(":", 1)[1]
    page, errors, _ = _open_at_alias_host(
        browser, twin_server, f"http://vision-picar.test:{port}")
    assert not errors
    assert page.locator("#cfg-url-mismatch").is_hidden()
    page.context.close()


def test_the_notice_names_both_hosts_when_they_differ(browser, twin_server):
    """Both halves matter. "Calls go elsewhere" is not actionable without
    saying where, and "this page is X" is what makes the mismatch legible."""
    page, errors, page_host = _open_at_alias_host(
        browser, twin_server, "https://example-vision.test")
    assert not errors
    assert page.locator("#cfg-url-mismatch").is_visible()
    text = " ".join(page.inner_text("#cfg-url-mismatch").split())
    assert "example-vision.test" in text, text
    assert page_host in text, text
    page.context.close()


def test_the_notice_does_not_fire_for_local_development(browser, twin_server):
    """localhost routinely splits the twin (:8000) from the vision service
    (:8080). Flagging that would make the notice noise on the one setup
    every developer uses."""
    context = browser.new_context(viewport=PHONE)
    context.add_init_script(
        "(() => { try { localStorage.setItem('vp_vision_url',"
        " 'http://127.0.0.1:8080'); } catch (e) {} })();")
    page = context.new_page()
    page.goto(twin_server, wait_until="networkidle")
    page.wait_for_timeout(800)
    page.click("#btn-settings")
    page.wait_for_timeout(300)
    assert page.locator("#cfg-url-mismatch").is_hidden()
    context.close()



# ---------- connection failures say which kind they are ----------
#
# Three situations were reported with one message: the request never
# arrived, the secret was rejected, and the server answered that it had no
# frame yet. Only the first is about networks and origins. Repeating that
# advice for the other two sends someone to check three things that are
# already fine -- which is exactly what happened against a healthy
# mode: teleop server that simply had no walk in progress.


def _connect_against(browser, twin_server, status, detail):
    """The twin, connecting to a robot server that answers with a chosen
    status. Routed rather than really deployed: what is under test is the
    client's wording, not the server's behaviour."""
    page, errors = open_twin(browser, twin_server)

    def handler(route):
        route.fulfill(status=status, content_type="application/json",
                      body=_json({"detail": detail}))

    page.route("**/frame", handler)
    page.click("#btn-settings")
    page.fill("#cfg-server-url", twin_server)
    page.click("#btn-connect")
    page.wait_for_timeout(900)
    return page, errors, " ".join(page.inner_text("#connection-status").split())


def test_a_stalled_teleop_server_is_not_reported_as_unreachable(browser, twin_server):
    """The message that started this: a healthy server answering 503 was
    presented as possibly-down, possibly-off-network, possibly-CORS."""
    page, errors, text = _connect_against(
        browser, twin_server, 503,
        "no new frame in 83553.5s (limit 15.0s) -- is the phone still capturing?")
    assert not errors
    assert "CORS" not in text, "still blaming CORS for a server that answered"
    assert "Waiting for a walk" in text, text
    assert "Nothing to do" in text, "says what happened but not what to do"
    # Not styled as a fault: nothing is wrong and nothing needs doing.
    assert not page.locator("#connection-status.is-err").count(), (
        "a state that needs no action is still presented as an error"
    )
    page.context.close()


def test_a_rejected_secret_says_the_server_was_reachable(browser, twin_server):
    """A 401 is never a network problem, and saying so points at the one
    thing that is actually wrong -- which deployment the secret belongs to."""
    page, errors, text = _connect_against(
        browser, twin_server, 401, "Missing or invalid x-app-secret header.")
    assert not errors
    assert "reachable" in text, text
    assert "CORS" not in text, text
    assert page.locator("#connection-status.is-err").count(), (
        "a wrong secret does need action and should look like it"
    )
    page.context.close()


def test_an_unreachable_server_still_gets_the_network_advice(browser, twin_server):
    """The advice is right for the case it was written for, and must not be
    lost while narrowing the cases it applies to."""
    page, errors = open_twin(browser, twin_server)
    page.route("**/frame", lambda route: route.abort())
    page.click("#btn-settings")
    page.fill("#cfg-server-url", twin_server)
    page.click("#btn-connect")
    page.wait_for_timeout(900)
    text = " ".join(page.inner_text("#connection-status").split())
    assert "Could not reach" in text, text
    assert "CORS" in text, text
    page.context.close()


def test_the_auto_reconnect_also_reports_what_the_server_said(browser, twin_server):
    """The reconnect that runs on page load had its own wording -- "didn't
    respond. Tap Connect to retry." -- which is the CORS mistake in a
    shorter sentence: a server that answered 503 did respond. This was
    missed the first time because only the manual Connect path was fixed,
    and the page-load path is the one most people actually see."""
    context = browser.new_context(viewport=PHONE)
    context.add_init_script(
        "(() => { try { localStorage.setItem('vp_cfg_server_url', %s); }"
        " catch (e) {} })();" % _json(twin_server))
    page = context.new_page()
    page.route("**/frame", lambda route: route.fulfill(
        status=503, content_type="application/json",
        body=_json({"detail": "no new frame in 83553.5s (limit 15.0s) -- "
                              "is the phone still capturing?"})))
    page.goto(twin_server, wait_until="networkidle")
    page.wait_for_timeout(1500)
    page.click("#btn-settings")
    page.wait_for_timeout(300)
    text = " ".join(page.inner_text("#connection-status").split())
    assert "didn't respond" not in text, f"still claiming no response: {text}"
    assert "Waiting for a walk" in text, text
    context.close()


def test_a_state_that_needs_no_action_does_not_raise_a_toast(browser, twin_server):
    """A toast reading "Couldn't reach the robot server" was once shown
    directly above a panel explaining the server was running. Interrupting
    someone is for things they have to act on."""
    page, errors, _ = _connect_against(
        browser, twin_server, 503,
        "no new frame in 85456.8s (limit 15.0s) -- is the phone still capturing?")
    assert not errors
    body = page.inner_text("body")
    assert "Couldn't reach the robot server" not in body, (
        "toast contradicts the panel it sits on top of"
    )
    page.context.close()



def test_recording_says_so_when_it_cannot_start(browser, twin_server):
    """A walk recorded with no brain connected is silently dropped frame by
    frame, and the first sign of it is an empty admin console after the
    walk. The Settings row explains the requirement, but nobody is reading
    Settings at the moment they press Start."""
    context = browser.new_context(viewport=PHONE, permissions=["camera"])
    context.add_init_script(
        "(() => { try {"
        " localStorage.setItem('guidanceMode','robot');"
        " localStorage.setItem('vp_guide_onboarded','1');"
        " localStorage.setItem('vp_record_walk','1');"
        " } catch (e) {} })();")
    page = context.new_page()
    errors = []
    page.on("pageerror", lambda e: errors.append(str(e)))
    page.goto(twin_server, wait_until="networkidle")
    page.wait_for_timeout(800)
    page.fill("#guidance-target", "red backpack")
    page.click("#btn-guidance")
    page.wait_for_timeout(1500)
    assert not errors
    body = page.inner_text("body")
    assert "Not recording" in body, (
        "recording was silently skipped with no brain connected"
    )
    context.close()
def test_double_tapping_start_does_not_begin_two_walks(browser, twin_server):
    """The attribution bug of 2026-09-02, as a test.

    `startGuidance()` sets `guidanceRunning` only after several awaits
    (motion permission, getUserMedia, video metadata, and -- when recording
    -- a brain health check), while the Start button's guard reads it
    synchronously. Both taps of a double-tap therefore passed it and two
    loops ran. They share one `guidanceEpoch`, so neither orphans the
    other's calls, and the second `beginWalkRecording()` renames the walk
    under the first loop's feet. A frame dispatched under the old walk then
    lands in the new walk's directory carrying the OLD wording and the OLD
    seq -- which is how bottle-opus-4-5-center-third-path-20260902-163923
    came to hold sixteen center-third-path frames and one bearing-only frame
    at seq 37, and why no walk recorded straight after another was safely
    attributable.

    Counted off the toast stack because app.js is one IIFE with no test
    hooks: `beginWalkRecording()` announces every walk it names, so two
    toasts means two walks. The clicks are dispatched in ONE JS task, since
    two Playwright clicks give the first enough time to finish and the race
    window is the whole point.
    """
    context = browser.new_context(viewport=PHONE, permissions=["camera"])
    context.add_init_script(
        "(() => { try {"
        " localStorage.setItem('guidanceMode','robot');"
        " localStorage.setItem('vp_guide_onboarded','1');"
        " localStorage.setItem('vp_record_walk','1');"
        " } catch (e) {} })();")
    page = context.new_page()
    errors = []
    page.on("pageerror", lambda e: errors.append(str(e)))
    page.route("**/brain-stub/health", lambda route: route.fulfill(
        status=200, content_type="application/json",
        body=_json({"status": "ok", "mission_running": False,
                    "recording_allowed": True, "drills_allowed": True,
                    "tick_timeout_s": 30.0,
                    "identity": {"git_revision": "abc1234"}})))
    page.goto(twin_server, wait_until="networkidle")
    page.wait_for_timeout(500)

    # Recording needs a connected brain, or beginWalkRecording() takes its
    # early return and never names a walk at all -- which would make this
    # test pass against the very defect it is written for.
    page.click("#btn-settings")
    page.fill("#cfg-server-url", twin_server)
    page.fill("#cfg-brain-url", twin_server + "/brain-stub")
    page.click("#btn-brain-connect")
    page.wait_for_timeout(600)
    page.click('.tab-btn[data-tab="guide"]')
    page.wait_for_timeout(300)
    page.fill("#guidance-target", "red backpack")

    page.evaluate(
        "() => { const b = document.getElementById('btn-guidance');"
        " b.click(); b.click(); }")
    page.wait_for_timeout(2500)

    assert not errors, errors
    named = page.evaluate(
        "() => Array.from(document.querySelectorAll('#toast-stack .toast'))"
        ".filter(t => t.textContent.includes('Recording this walk as')).length")
    assert named == 1, (
        f"{named} walks were started by one double-tap -- a second walk "
        "renames the first under its own loop, which is what makes frames "
        "unattributable")
    context.close()


# ---------- the environment banner ----------


def _open_with_health(browser, twin_server, env_label):
    """The twin, with /health's env_label forced to a given value.

    Intercepted rather than served by a second uvicorn with ENV_LABEL set:
    what is under test here is the client's reaction to the field, and the
    server side of it is already pinned in tests/test_server.py.
    """
    page, errors = open_twin(browser, twin_server)

    def handler(route):
        resp = route.fetch()
        body = resp.json()
        body["env_label"] = env_label
        route.fulfill(status=200, content_type="application/json", body=_json(body))

    page.route("**/health", handler)
    page.reload(wait_until="networkidle")
    page.wait_for_timeout(600)
    return page, errors


def test_no_banner_when_the_server_reports_no_environment(browser, twin_server):
    """Production's state. The banner must be invisible by default, or it
    is worse than useless -- a warning that is always on teaches people to
    stop seeing it."""
    page, errors = _open_with_health(browser, twin_server, "")
    assert not errors
    assert page.evaluate(
        "() => getComputedStyle(document.getElementById('env-banner')).display"
    ) == "none"
    assert not page.evaluate("() => document.body.classList.contains('env-flagged')")


def test_the_banner_names_the_environment_when_the_server_does(browser, twin_server):
    """The visibility assertion is not redundant with the text one: this
    test was first written without it, and passed against a banner whose
    reveal had been deliberately removed -- inner_text still returned the
    text of a display:none element. Same trap as the 40px model picker."""
    page, errors = _open_with_health(browser, twin_server, "Lab")
    assert not errors
    assert page.evaluate(
        "() => getComputedStyle(document.getElementById('env-banner')).display"
    ) != "none", "banner has the text but was never revealed"
    assert "LAB ENVIRONMENT" in page.inner_text("#env-banner").upper()
    assert page.evaluate("() => document.body.classList.contains('env-flagged')")


def test_the_banner_stays_in_view_when_the_page_scrolls(browser, twin_server):
    """Sticky, not fixed: the tab bar is the page's one fixed element, and
    a second competing for the viewport is how the bottom bar started
    drifting on iOS. Sticky keeps it visible without leaving the flow."""
    page, _ = _open_with_health(browser, twin_server, "Lab")
    page.evaluate("window.scrollTo(0, document.body.scrollHeight)")
    page.wait_for_timeout(400)
    box = page.locator("#env-banner").bounding_box()
    assert box is not None and box["y"] < 60, f"banner scrolled out of view: {box}"


def test_the_banner_is_readable_on_a_phone(browser, twin_server):
    """Same lesson as the model picker: a control that renders but is
    unreadable at a phone width has not shipped. Full bleed, real height."""
    page, _ = _open_with_health(browser, twin_server, "Lab")
    box = page.locator("#env-banner").bounding_box()
    assert box["width"] >= PHONE["width"] - 4, f"banner not full width: {box}"
    assert box["height"] >= 20, f"banner too short to read: {box}"



# ---------- the remote brain's policy picker (M1) ----------
#
# `policy: "vision"` has existed on control/brain_server.py since S2b, and
# until now the twin had no way to ask for it -- the Remote brain panel
# always started a frontier mission. That made the vision policy the one
# thing in this project you could not watch from a phone, which
# CLAUDE.md section 7 says is the same as not shipped.
#
# The picker also carries the model and the wording, because a paid mission
# whose result nobody can attribute is not a measurement.


def open_sim_tab(browser, twin_server, **kwargs):
    page, errors = open_twin(browser, twin_server, **kwargs)
    page.click('.tab-btn[data-tab="sim"]')
    return page, errors


def test_the_remote_brain_can_be_set_to_the_vision_policy(browser, twin_server):
    page, _ = open_sim_tab(browser, twin_server, mode="guide")
    select = page.locator("#brain-policy")
    sync_api.expect(select).to_be_visible()

    select.select_option("vision")
    assert select.input_value() == "vision"
    page.close()


def test_choosing_the_vision_policy_says_what_it_would_ask_and_what_it_costs(browser, twin_server):
    """A paid mission that does not say which model and which wording it
    would run is not a measurement -- it is the NavigateModelId trap again,
    where a week of walks was attributed to a model that was never running."""
    page, _ = open_sim_tab(browser, twin_server, mode="guide")
    hint = page.locator("#brain-policy-hint")
    sync_api.expect(hint).to_be_hidden()

    page.locator("#brain-policy").select_option("vision")
    sync_api.expect(hint).to_be_visible()
    sync_api.expect(hint).to_contain_text("paid")
    # Nothing picked yet, so both resolve to the service's own defaults --
    # and the hint has to name those rather than going quiet. They arrive
    # with GET /navigate/models, so the readout has to be redrawn when it
    # lands; expect() polls, which is what makes that assertable.
    sync_api.expect(hint).to_contain_text("Claude Opus 4.5")
    sync_api.expect(hint).to_contain_text("service default")
    sync_api.expect(hint).to_contain_text(MODELS_REPLY["default_prompt"])
    page.close()


def test_the_vision_policy_brings_the_model_and_prompt_pickers_with_it(browser, twin_server):
    """The two pickers were Robot-view-only. The vision policy is their
    second consumer, so they have to appear for it too -- otherwise the panel
    names a model and a wording the operator has no way to change."""
    page, _ = open_sim_tab(browser, twin_server, mode="guide")
    page.locator("#brain-policy").select_option("vision")

    # They live on the Guide tab -- see the button test below for why -- so
    # "visible" is only a fair question once that tab is on screen.
    page.click('.tab-btn[data-tab="guide"]')
    sync_api.expect(page.locator("#navigate-model-row")).to_be_visible()
    sync_api.expect(page.locator("#navigate-prompt-row")).to_be_visible()

    # And they go away again with the policy that wanted them: Guide mode
    # steers a person and has no model to choose.
    page.click('.tab-btn[data-tab="sim"]')
    page.locator("#brain-policy").select_option("frontier")
    page.click('.tab-btn[data-tab="guide"]')
    sync_api.expect(page.locator("#navigate-model-row")).to_be_hidden()
    page.close()


def test_the_hint_follows_the_pickers(browser, twin_server):
    """Pick a wording on the Guide tab, and the Sim tab's panel says so. The
    whole point is that the operator reads what will run instead of inferring
    it from a log afterwards."""
    page, _ = open_sim_tab(browser, twin_server, mode="guide")
    page.locator("#brain-policy").select_option("vision")
    page.click("#btn-brain-pickers")
    page.locator("#cfg-navigate-prompt").select_option("next-step-obstacle")

    page.click('.tab-btn[data-tab="sim"]')
    hint = page.locator("#brain-policy-hint")
    sync_api.expect(hint).to_contain_text("next-step-obstacle")
    sync_api.expect(hint).to_contain_text("your pick")
    page.close()


def test_the_pickers_are_one_tap_away_from_the_panel(browser, twin_server):
    """They live on the Guide tab, beside Robot view -- the flow they were
    built for. Duplicating the selects in the Sim tab would give two copies
    that can disagree about what is selected, so the panel navigates to them
    instead."""
    page, _ = open_sim_tab(browser, twin_server, mode="guide")
    sync_api.expect(page.locator("#brain-policy-pickers-row")).to_be_hidden()

    page.locator("#brain-policy").select_option("vision")
    sync_api.expect(page.locator("#brain-policy-pickers-row")).to_be_visible()

    page.click("#btn-brain-pickers")
    sync_api.expect(page.locator('.tab-page[data-tab="guide"]')).to_have_class(
        re.compile(r"\bactive\b"))
    sync_api.expect(page.locator("#cfg-navigate-model")).to_be_visible()
    page.close()


def test_the_policy_choice_survives_a_reload(browser, twin_server):
    """A standing choice that silently resets to the free policy would be a
    mission you paid for by accident, or one you did not get."""
    page, _ = open_sim_tab(browser, twin_server, mode="guide")
    page.locator("#brain-policy").select_option("vision")
    page.reload(wait_until="load")

    page.click('.tab-btn[data-tab="sim"]')
    assert page.locator("#brain-policy").input_value() == "vision"
    sync_api.expect(page.locator("#brain-policy-hint")).to_be_visible()
    page.close()


# ---------- the depth strip (phase M2) ----------
#
# M2's UI proof, and the reason it is a browser test rather than a DOM one:
# the strip has to line up with the camera view above it and survive a phone
# viewport. A strip that renders but sits 40px wide, or drifts out of
# alignment with the picture it describes, is worse than no strip -- it still
# looks authoritative. That is the same class of defect as the 40px model
# picker this file was written for.


def _connect_for_depth(browser, twin_server):
    """The twin connected to the real robot server, which under mode: sim
    publishes a real grid from sim/renderer.py's raycaster. Not routed:
    what is under test here is that the two ends agree, and a stubbed grid
    would pin the stub."""
    page, errors = open_twin(browser, twin_server)
    page.click("#btn-settings")
    page.fill("#cfg-server-url", twin_server)
    page.click("#btn-connect")
    # The strip lives on the Sim tab, beneath the FPV canvas. Without this
    # the zones exist in the DOM and are not visible -- which is the first
    # thing these tests caught, and a reminder that "the element is there"
    # and "someone can see it" are different assertions.
    page.click('.tab-btn[data-tab="sim"]')
    page.wait_for_selector("#depth-strip .depth-zone", state="visible", timeout=5000)
    return page, errors


def test_the_depth_strip_shows_one_zone_per_column_the_robot_reports(browser, twin_server):
    """The grid declares its own shape (`rows`/`cols` travel in the data),
    so the strip must be drawn from that rather than from a hardcoded eight
    -- otherwise a real sensor with a different geometry would be drawn
    wrong and still look right."""
    page, errors = _connect_for_depth(browser, twin_server)
    reported = page.evaluate(
        "async () => (await (await fetch(document.getElementById('cfg-server-url').value"
        " + '/depth')).json()).cols")
    assert page.locator("#depth-strip .depth-zone").count() == reported
    assert not errors, errors
    page.close()


def test_the_depth_strip_is_visible_and_full_width_on_a_phone(browser, twin_server):
    """Bug (2)'s shape, one feature later: the zones live in a flex row, and
    a flex row inside a centring wrapper is exactly what collapsed the model
    picker to 40px."""
    page, _ = _connect_for_depth(browser, twin_server)
    strip = page.locator("#depth-strip")
    sync_api.expect(strip).to_be_visible()
    box = strip.bounding_box()
    assert box is not None
    assert box["width"] >= 200, (
        f"the depth strip is {box['width']:.0f}px wide at {PHONE['width']}px -- "
        "too narrow to read a zone off"
    )
    assert box["height"] >= 8, "a zero-height strip renders as nothing"
    page.close()


def test_the_strip_lines_up_with_the_camera_view_it_measures(browser, twin_server):
    """Left-to-right on the strip is left-to-right in the picture. A strip
    wider or narrower than the canvas would still show the right numbers
    while pointing at the wrong part of the room."""
    page, _ = _connect_for_depth(browser, twin_server)
    canvas = page.locator("#fpv-canvas").bounding_box()
    strip = page.locator("#depth-strip").bounding_box()
    assert canvas and strip
    assert abs(canvas["x"] - strip["x"]) <= 2, (
        f"strip starts at {strip['x']:.0f}, canvas at {canvas['x']:.0f}")
    assert abs(canvas["width"] - strip["width"]) <= 2, (
        f"strip is {strip['width']:.0f}px, canvas {canvas['width']:.0f}px")
    page.close()


def test_the_readout_names_the_clearance_the_veto_actually_reads(browser, twin_server):
    """The number a person standing next to the robot wants is the one the
    safety layer is about to compare against a threshold -- not the nearest
    zone anywhere in the grid, which in a corridor is a side wall the robot
    is supposed to drive past. Phase M3 changed this readout for exactly
    that reason."""
    page, _ = _connect_for_depth(browser, twin_server)
    text = page.inner_text("#depth-readout")
    assert "path" in text and "cm" in text, text
    assert "1×8" in text, f"the readout should name the grid's own shape: {text}"
    page.close()


# ---------- the veto's own zones (phase M3) ----------


def test_the_zones_the_veto_reads_are_marked_and_come_from_the_server(browser, twin_server):
    """Which zones are "the path" is robot-runtime safety logic, so the page
    must render the server's answer rather than work it out. Marking them at
    all is the phase's point: "the collar fired" and "the collar fired on
    THAT" are different things to be able to see."""
    page, errors = _connect_for_depth(browser, twin_server)
    served = page.evaluate(
        "async () => (await (await fetch(document.getElementById('cfg-server-url').value"
        " + '/depth')).json()).path.indices")
    marked = page.evaluate(
        "() => Array.from(document.querySelectorAll('#depth-strip .depth-zone'))"
        ".map((el, i) => el.classList.contains('path') ? i : -1).filter(i => i >= 0)")
    assert marked == served, f"page marked {marked}, server said {served}"
    assert 0 < len(marked) < 8, (
        f"{len(marked)} of 8 zones marked -- all of them is the whole-grid veto "
        "that refuses every corridor, one of them is the single beam again")
    assert not errors, errors
    page.close()


def test_a_blocked_path_says_the_move_would_be_vetoed(browser, twin_server):
    """The collar firing has to be visible on the strip that explains it,
    not only as a log line after the fact."""
    page, errors = open_twin(browser, twin_server)
    page.route("**/depth", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json({
            "rows": 1, "cols": 4,
            "zones": [{"status": "range", "distance_cm": 90.0},
                      {"status": "range", "distance_cm": 8.0},
                      {"status": "range", "distance_cm": 9.0},
                      {"status": "range", "distance_cm": 90.0}],
            "path": {"indices": [1, 2], "clearance_cm": 8.0, "source": "depth_grid",
                     "blocked": True, "min_distance_cm": 20},
        })))
    page.click("#btn-settings")
    page.fill("#cfg-server-url", twin_server)
    page.click("#btn-connect")
    page.click('.tab-btn[data-tab="sim"]')
    page.wait_for_selector("#depth-strip .depth-zone.path", state="visible", timeout=5000)
    assert "vetoed" in page.inner_text("#depth-readout")
    assert page.locator("#depth-strip.blocked").count() == 1
    assert not errors, errors
    page.close()


def test_a_blind_path_says_it_fell_back_to_the_single_beam(browser, twin_server):
    """The same centimetres from a different sensor is a different
    situation. "Every path zone was unusable, so this is the old one beam"
    must not read like "the grid answered" -- that is the distinction the
    tri-state exists to preserve, and erasing it in the readout would undo
    the phase in the one place a person looks."""
    page, errors = open_twin(browser, twin_server)
    page.route("**/depth", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json({
            "rows": 1, "cols": 4,
            "zones": [{"status": "range", "distance_cm": 90.0},
                      {"status": "unusable", "distance_cm": None},
                      {"status": "unusable", "distance_cm": None},
                      {"status": "range", "distance_cm": 90.0}],
            "path": {"indices": [1, 2], "clearance_cm": 45.0,
                     "source": "distance_sensor", "blocked": False,
                     "min_distance_cm": 20},
        })))
    page.click("#btn-settings")
    page.fill("#cfg-server-url", twin_server)
    page.click("#btn-connect")
    page.click('.tab-btn[data-tab="sim"]')
    page.wait_for_selector("#depth-strip .depth-zone", state="visible", timeout=5000)
    text = page.inner_text("#depth-readout")
    assert "single beam" in text, text
    assert "2 unusable" in text, text
    assert not errors, errors
    page.close()


def test_nothing_within_range_is_not_reported_as_a_distance(browser, twin_server):
    """`no_target` carries no number, and the readout must not invent one.
    Printing "path 0cm" for "nothing is there" would be the exact
    two-outcome collapse M3 argues against, surfacing in the UI."""
    page, errors = open_twin(browser, twin_server)
    page.route("**/depth", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json({
            "rows": 1, "cols": 2,
            "zones": [{"status": "no_target", "distance_cm": None},
                      {"status": "no_target", "distance_cm": None}],
            "path": {"indices": [0, 1], "clearance_cm": None,
                     "source": "depth_grid_no_target", "blocked": False,
                     "min_distance_cm": 20},
        })))
    page.click("#btn-settings")
    page.fill("#cfg-server-url", twin_server)
    page.click("#btn-connect")
    page.click('.tab-btn[data-tab="sim"]')
    page.wait_for_selector("#depth-strip .depth-zone", state="visible", timeout=5000)
    text = page.inner_text("#depth-readout")
    assert "clear beyond range" in text, text
    assert "0cm" not in text, text
    assert "vetoed" not in text, text
    assert not errors, errors
    page.close()


def test_a_server_with_no_depth_route_says_so_instead_of_going_blank(browser, twin_server):
    """The stacks are redeployed one at a time, so a twin talking to a
    pre-M2 robot server is a real state and not an error. It must be
    legible as 'this server does not report depth' rather than as an empty
    space that could equally mean 'no obstacles'."""
    page, errors = open_twin(browser, twin_server)
    page.route("**/depth", lambda route: route.fulfill(
        status=404, content_type="application/json", body=_json({"detail": "Not Found"})))
    page.click("#btn-settings")
    page.fill("#cfg-server-url", twin_server)
    page.click("#btn-connect")
    page.click('.tab-btn[data-tab="sim"]')
    page.wait_for_timeout(900)
    text = page.inner_text("#depth-readout")
    assert "not reported" in text, text
    assert page.locator("#depth-strip .depth-zone").count() == 0
    assert not errors, errors
    page.close()


def test_an_unmeasurable_zone_is_not_drawn_as_a_distance(browser, twin_server):
    """The tri-state's whole point, made visible: 'I could not tell' must
    not be paintable as 'clear' or as any range. This is what stops a
    reader -- and later M3's veto -- treating a failed zone as a number."""
    page, errors = open_twin(browser, twin_server)
    page.route("**/depth", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json({
            "rows": 1, "cols": 2,
            "zones": [{"status": "range", "distance_cm": 40.0},
                      {"status": "unusable", "distance_cm": None}],
        })))
    page.click("#btn-settings")
    page.fill("#cfg-server-url", twin_server)
    page.click("#btn-connect")
    page.click('.tab-btn[data-tab="sim"]')
    page.wait_for_selector("#depth-strip .depth-zone", state="visible", timeout=5000)
    zones = page.locator("#depth-strip .depth-zone")
    assert zones.count() == 2
    assert "unusable" in (zones.nth(1).get_attribute("class") or "")
    assert "unusable" not in (zones.nth(0).get_attribute("class") or "")
    assert "1 unusable" in page.inner_text("#depth-readout")
    assert not errors, errors
    page.close()


# ---------- who is driving, and why a move was refused (phase M4) ----------
#
# Both readouts answer the question a person standing next to the robot
# actually asks: it is not moving, what stopped it? Before M4 the page
# could not tell "someone else took the robot" from "it is about to hit a
# wall" -- every refusal rendered as the words SAFETY VETO.


def _open_sim_tab(browser, twin_server):
    page, errors = open_twin(browser, twin_server)
    page.click("#btn-settings")
    page.fill("#cfg-server-url", twin_server)
    page.click("#btn-connect")
    page.click('.tab-btn[data-tab="sim"]')
    return page, errors


def test_the_driver_readout_starts_at_nobody_and_names_the_pad(browser, twin_server):
    """"Nobody has driven this robot yet" is a real state and must not read
    as an anonymous driver who has since gone quiet -- the two look the
    same on the wire if you check the value instead of the key."""
    page, errors = _open_sim_tab(browser, twin_server)
    sync_api.expect(page.locator("#brain-tel-driver")).to_have_text("nobody yet", timeout=5000)

    page.click("#btn-look-left")
    sync_api.expect(page.locator("#brain-tel-driver")).to_have_text("twin-dpad", timeout=5000)
    assert not errors, errors
    page.close()


def test_a_preemption_does_not_read_as_a_safety_veto_in_the_log(browser, twin_server):
    """The bug this phase is about, at the surface a person reads. Two
    refusals with opposite correct responses -- retry later, versus stop,
    you are not driving -- rendered as the same three words."""
    page, errors = open_twin(browser, twin_server)
    page.route("**/action", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json({
            "executed": False, "reason": "preempted",
            "detail": "twin-dpad is driving -- brain is lower priority and was refused",
        })))
    page.click("#btn-settings")
    page.fill("#cfg-server-url", twin_server)
    page.click("#btn-connect")
    page.click('.tab-btn[data-tab="sim"]')
    page.click("#btn-forward")
    log = page.locator("#log")
    sync_api.expect(log).to_contain_text("PREEMPTED", timeout=5000)
    assert "SAFETY VETO" not in log.inner_text()
    assert not errors, errors
    page.close()


def test_the_refusal_readout_names_the_reason_and_who_was_refused(browser, twin_server):
    """Which party was refused matters as much as why: "the brain was
    preempted" and "the pad was preempted" are different stories about the
    same robot."""
    page, errors = open_twin(browser, twin_server)

    def health(route):
        route.fulfill(status=200, content_type="application/json", body=_json({
            "status": "ok", "seconds_since_last_command": 0.1,
            "watchdog_timeout_s": 1.0, "min_distance_cm": 20, "mode": "sim",
            "env_label": "", "driver": "twin-dpad", "authority_holder": "twin-dpad",
            "last_refusal": {"reason": "preempted", "detail": "outranked",
                             "driver": "brain", "at": 1.0, "seconds_ago": 2.5},
        }))

    page.route("**/health", health)
    page.click("#btn-settings")
    page.fill("#cfg-server-url", twin_server)
    page.click("#btn-connect")
    page.click('.tab-btn[data-tab="sim"]')
    readout = page.locator("#brain-tel-refusal")
    sync_api.expect(readout).to_contain_text("PREEMPTED", timeout=5000)
    sync_api.expect(readout).to_contain_text("brain")
    assert not errors, errors
    page.close()


def test_a_server_older_than_m4_leaves_both_readouts_blank(browser, twin_server):
    """Same choice the depth strip makes about a server with no /depth: say
    nothing rather than invent a driver. The stacks are redeployed one at a
    time, so this is a state the twin really meets."""
    page, errors = open_twin(browser, twin_server)
    page.route("**/health", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json({
            "status": "ok", "seconds_since_last_command": 0.1,
            "watchdog_timeout_s": 1.0, "mode": "sim",
        })))
    page.click("#btn-settings")
    page.fill("#cfg-server-url", twin_server)
    page.click("#btn-connect")
    page.click('.tab-btn[data-tab="sim"]')
    page.wait_for_timeout(900)
    assert page.inner_text("#brain-tel-driver").strip() in ("–", "-", ""), \
        page.inner_text("#brain-tel-driver")
    assert not errors, errors
    page.close()


# ---------- one health answer, from the page (phase M5) ----------


def _settings_health(browser, twin_server, *, brain_url=None, brain_health=None,
                     robot_health=None):
    page, errors = open_twin(browser, twin_server)
    if robot_health is not None:
        page.route(re.compile(r".*:%s/health" % twin_server.rsplit(":", 1)[1]),
                   lambda route: route.fulfill(status=200,
                                               content_type="application/json",
                                               body=_json(robot_health)))
    if brain_health is not None:
        page.route("**/brain-stub/health", lambda route: route.fulfill(
            status=200, content_type="application/json", body=_json(brain_health)))
    page.click("#btn-settings")
    page.fill("#cfg-server-url", twin_server)
    if brain_url is not None:
        page.fill("#cfg-brain-url", brain_url)
    page.click("#btn-health-check")
    return page, errors


HEALTHY_BRAIN_BODY = {
    "status": "ok", "mission_running": False, "tick_timeout_s": 30.0,
    "identity": {"git_revision": "abc1234"},
}


def test_the_health_line_names_the_half_that_failed(browser, twin_server):
    """The press-this for M5: kill the brain and the page says which half
    went, rather than a bare red light. Two health routes each reporting
    themselves fine is what this replaces."""
    page, errors = _settings_health(
        browser, twin_server, brain_url="http://127.0.0.1:9/unreachable")
    verdict = page.locator("#system-health-verdict")
    sync_api.expect(verdict).to_contain_text("UNHEALTHY", timeout=8000)
    sync_api.expect(verdict).to_contain_text("brain")
    sync_api.expect(page.locator("#system-health-detail")).to_contain_text("robot: ok")
    assert not errors, errors
    page.close()


def test_a_healthy_pair_reads_ok_and_names_the_build(browser, twin_server):
    """M11 rolls a release back on this verdict, so "which build said it
    was fine" has to be on the line that said it."""
    page, errors = _settings_health(
        browser, twin_server, brain_url=twin_server + "/brain-stub",
        brain_health=HEALTHY_BRAIN_BODY)
    verdict = page.locator("#system-health-verdict")
    sync_api.expect(verdict).to_have_text("OK", timeout=8000)
    sync_api.expect(page.locator("#system-health-detail")).to_contain_text("abc1234")
    assert not errors, errors
    page.close()


def test_a_robot_against_a_wall_does_not_read_as_unhealthy(browser, twin_server):
    """The rule, at the surface people actually look at: only conditions a
    release can be blamed for reach the verdict. A page that went red
    because the robot was parked is one everybody learns to ignore -- and
    it would disagree with the command that gates the rollback."""
    page, errors = _settings_health(
        browser, twin_server, brain_url=twin_server + "/brain-stub",
        brain_health=HEALTHY_BRAIN_BODY,
        robot_health={
            "status": "ok", "mode": "sim", "seconds_since_last_command": 3600.0,
            "watchdog_timeout_s": 1.0, "seconds_since_watchdog_poll": 0.05,
            "watchdog_poll_interval_s": 0.1,
            "last_refusal": {"reason": "safety_distance", "detail": "Blocked FORWARD",
                             "driver": "twin-dpad", "seconds_ago": 0.2},
            "identity": {"git_revision": "abc1234"},
        })
    sync_api.expect(page.locator("#system-health-verdict")).to_have_text("OK", timeout=8000)
    assert not errors, errors
    page.close()


def test_a_stalled_watchdog_loop_reads_as_unhealthy(browser, twin_server):
    """The failure nothing else here can see: the server answers every
    request while the guard that stops the motors is gone."""
    page, errors = _settings_health(
        browser, twin_server, brain_url=twin_server + "/brain-stub",
        brain_health=HEALTHY_BRAIN_BODY,
        robot_health={
            "status": "ok", "mode": "sim", "seconds_since_last_command": 0.2,
            "watchdog_timeout_s": 1.0, "seconds_since_watchdog_poll": 5.0,
            "watchdog_poll_interval_s": 0.1,
            "identity": {"git_revision": "abc1234"},
        })
    verdict = page.locator("#system-health-verdict")
    sync_api.expect(verdict).to_contain_text("UNHEALTHY", timeout=8000)
    sync_api.expect(page.locator("#system-health-detail")).to_contain_text("watchdog loop")
    assert not errors, errors
    page.close()


# ---------- the tiered policy, and what makes it watchable (phase P2) ----------
#
# `PLAN-onboard-perception.md` 6.3 is the specification these cover, and its
# own sentence is the reason they are UI tests at all: *"a deliberation-call
# counter that visibly does not climb every step. That single number makes
# the whole architecture watchable."* A tiered mission whose saving can only
# be read out of a JSON status is, by CLAUDE.md section 7, not shipped.
#
# The brain is stubbed by request interception, exactly as the M4 and M5
# tests stub it: nothing here starts a mission, loads a model or spends
# anything. What is under test is whether a person holding a phone can see
# the three tiers doing their separate jobs.

TIERED_BRAIN_HEALTH = {
    "status": "ok", "mission_running": False, "tick_timeout_s": 30.0,
    "drills_allowed": True, "recording_allowed": True,
    "identity": {"git_revision": "abc1234"},
    "perception_available": True,
    "perception_detector": "yolo11s.pt",
    "perception_clip_model": "RN50",
    "tier_consecutive_frames": 2,
    "tier_cold_search_after": 6,
}


def tiered_status(*, frames=12, cloud_calls=3, status="detected", margin=0.21,
                  detector="yolo11s.pt", running=True, verdict="corroborated",
                  local_p=0.69, claims=3, corroborated=2):
    """A /mission/status body shaped as control/mission_runner.py emits one
    under `policy: "tiered"`."""
    return {
        "running": running, "outcome": "running" if running else "found",
        "policy": "tiered", "mission": "Find the red backpack.",
        "target_object": "red backpack", "step": frames, "max_steps": 120,
        "found": False, "room_reached": False, "complete": False,
        "last_action": "FORWARD", "last_reasoning": "target ahead -- because",
        "rooms_visited": [], "rooms_searched": [], "vision_failures": 0,
        "ticks": frames, "seconds_since_last_tick": 0.2, "tick_rate_hz": 1.0,
        "sighting": None, "log_tail": ["step 1: FORWARD (ok)"],
        "perception": {
            "status": status, "crop_source": "label_gate", "reason": "scripted",
            "synthesised": False, "pan_deg": 0.0, "tilt_deg": 0.0,
            "bearing_deg": 4.2, "match_margin": margin, "similarity": 0.31,
            "label": "backpack", "candidates": 2,
        },
        "tier": {
            "cloud_called": True, "trigger": "candidate_sighting",
            "models": {"detector": detector, "scorer": "RN50",
                       "target": "red backpack", "crop_source": "label_gate"},
            "stats": {"frames": frames, "cloud_calls": cloud_calls,
                      "frames_per_call": round(frames / cloud_calls, 2),
                      "triggers": {"mission_start": 1, "candidate_sighting": 2},
                      "perception": {"detected": 4, "absent": 8},
                      "claims": claims, "corroborated": corroborated,
                      "verdicts": {verdict: 1}},
            # 1.11a, reported and not enforced.
            "corroboration": (None if verdict is None else {
                "verdict": verdict, "claimed": verdict != "no_claim",
                "bar": 0.5, "local_probability": local_p, "enforced": False}),
        },
    }


def frontier_status():
    """The same panel under a policy with no perception tier -- both keys
    null, which is what the runner really sends."""
    return {
        "running": True, "outcome": "running", "policy": "frontier",
        "step": 4, "max_steps": 120, "last_action": "FORWARD",
        "last_reasoning": "free space clear", "vision_failures": 0,
        "rooms_searched": [], "log_tail": [], "tier": None, "perception": None,
    }


def open_with_brain(browser, twin_server, *, status=None, health=None,
                    mode="guide"):
    """The Sim tab with a stubbed brain connected. Connecting polls
    /mission/status once, which is what draws the readouts.

    `mode` seeds the Guide tab's own sub-mode: the Robot-view switches
    ("Record this walk", "Drive via brain") are display:none outside it, so a
    test about them has to ask for "robot"."""
    page, errors = open_twin(browser, twin_server, mode=mode)
    page.route("**/brain-stub/health", lambda route: route.fulfill(
        status=200, content_type="application/json",
        body=_json(health if health is not None else TIERED_BRAIN_HEALTH)))
    if status is not None:
        page.route("**/brain-stub/mission/status", lambda route: route.fulfill(
            status=200, content_type="application/json", body=_json(status)))
    page.click("#btn-settings")
    page.fill("#cfg-brain-url", twin_server + "/brain-stub")
    page.click("#btn-brain-connect")
    page.wait_for_timeout(600)
    page.click('.tab-btn[data-tab="sim"]')
    return page, errors


def test_the_remote_brain_can_be_set_to_the_tiered_policy(browser, twin_server):
    """P1 and P2 have existed since 2026-09-06 with no way to ask for them
    from a phone -- the same state `policy: "vision"` was in before M1, and
    the same verdict applies."""
    page, _ = open_sim_tab(browser, twin_server, mode="guide")
    select = page.locator("#brain-policy")
    select.select_option("tiered")
    assert select.input_value() == "tiered"
    page.close()


def test_the_tiered_hint_says_the_models_are_local_and_the_cloud_is_on_a_trigger(browser, twin_server):
    """The cost story is the opposite of the vision policy's and has to read
    that way before anything is spent: perception every frame for free, the
    paid call only on an event."""
    page, _ = open_with_brain(browser, twin_server)
    page.locator("#brain-policy").select_option("tiered")
    hint = page.locator("#brain-policy-hint")
    sync_api.expect(hint).to_be_visible()
    sync_api.expect(hint).to_contain_text("yolo11s.pt")
    sync_api.expect(hint).to_contain_text("RN50")
    sync_api.expect(hint).to_contain_text("trigger")
    page.close()


def test_the_tiered_policy_brings_the_model_and_prompt_pickers_with_it(browser, twin_server):
    """It still makes a paid /navigate call -- fewer of them, not none -- so
    a tiered walk has to be as attributable as a vision one."""
    page, _ = open_sim_tab(browser, twin_server, mode="guide")
    page.locator("#brain-policy").select_option("tiered")
    sync_api.expect(page.locator("#brain-policy-pickers-row")).to_be_visible()
    page.click('.tab-btn[data-tab="guide"]')
    sync_api.expect(page.locator("#navigate-model-row")).to_be_visible()
    sync_api.expect(page.locator("#navigate-prompt-row")).to_be_visible()
    page.close()


def test_the_deliberation_counter_is_on_the_panel(browser, twin_server):
    """6.3's *"single number"*. It has to show BOTH terms -- calls and
    frames -- because a bare call count climbing by one is indistinguishable
    from a call on every step, which is the thing this architecture claims
    not to do."""
    page, errors = open_with_brain(
        browser, twin_server, status=tiered_status(frames=12, cloud_calls=3))
    readout = page.locator("#brain-tel-calls")
    sync_api.expect(readout).to_be_visible(timeout=5000)
    text = readout.inner_text()
    assert "3" in text and "12" in text, text
    # 6.1 measured 4-6x, and this number is directly comparable to it.
    assert "4" in text, f"the saving is not shown: {text!r}"
    assert not errors, errors
    page.close()


def test_the_perception_tri_state_is_shown_and_unavailable_is_not_absent(browser, twin_server):
    """1.12's whole reason for a three-way output, at the surface: *"a wedged
    capture never looks like a missing target."* Two words that mean opposite
    things must not render the same way."""
    page, _ = open_with_brain(browser, twin_server, status=tiered_status(status="absent"))
    readout = page.locator("#brain-tel-perception")
    sync_api.expect(readout).to_contain_text("absent", timeout=5000)
    absent_class = readout.get_attribute("class") or ""
    page.close()

    page2, _ = open_with_brain(browser, twin_server,
                               status=tiered_status(status="unavailable"))
    readout2 = page2.locator("#brain-tel-perception")
    sync_api.expect(readout2).to_contain_text("unavailable", timeout=5000)
    assert (readout2.get_attribute("class") or "") != absent_class, (
        "`absent` and `unavailable` render identically -- which is the one "
        "thing 1.12's tri-state exists to prevent")
    page2.close()


def test_the_clip_margin_and_the_detector_name_are_on_the_panel(browser, twin_server):
    """The margin, not the similarity: CLIP returns a similarity rather than
    a probability, so the margin over the distractors is the number that
    means anything (brain/perceive.py's DEFAULT_MATCH_MARGIN). And the
    detector's own name, because swapping it is the experiment loop."""
    page, _ = open_with_brain(browser, twin_server,
                              status=tiered_status(margin=0.21, detector="yolo11n.pt"))
    sync_api.expect(page.locator("#brain-tel-margin")).to_contain_text("0.21", timeout=5000)
    sync_api.expect(page.locator("#brain-tel-detector")).to_contain_text("yolo11n.pt")
    page.close()


def test_the_tier_readouts_stay_hidden_under_a_policy_that_has_no_tier(browser, twin_server):
    """Same choice the depth strip makes about a server with no /depth: say
    nothing rather than draw a zero. A counter reading 0 calls over 0 frames
    would look like a tiered mission that had stopped deliberating."""
    page, _ = open_with_brain(browser, twin_server, status=frontier_status())
    page.wait_for_timeout(600)
    rows = page.locator("#brain-tier-rows")
    # Present-and-hidden, not absent: a missing element is also "hidden" to
    # Playwright, and this test would then pass against a build with no tier
    # readouts at all -- which is the trap CLAUDE.md records two tests in
    # this project falling into.
    assert rows.count() == 1, "the tier readouts are not in the page at all"
    sync_api.expect(rows).to_be_hidden()
    page.close()


def test_a_brain_with_no_perception_models_says_so_before_you_start(browser, twin_server):
    """`ultralytics`/`torch` are an optional install, so this is a normal
    state for a fresh checkout. The mission does refuse with a usable
    message -- but reading it requires having already pressed Start, and the
    panel can say it first."""
    health = dict(TIERED_BRAIN_HEALTH, perception_available=False)
    page, _ = open_with_brain(browser, twin_server, health=health)
    page.locator("#brain-policy").select_option("tiered")
    hint = page.locator("#brain-policy-hint")
    sync_api.expect(hint).to_contain_text("requirements-perception.txt", timeout=5000)
    page.close()


def test_the_tier_readouts_are_readable_on_a_phone(browser, twin_server):
    """The lesson of the 40px model picker, applied to the readouts this
    phase adds: they are only proof if they can be read at 390px without the
    panel scrolling sideways."""
    page, _ = open_with_brain(browser, twin_server, status=tiered_status())
    sync_api.expect(page.locator("#brain-tel-calls")).to_be_visible(timeout=5000)
    for eid in ("brain-tel-perception", "brain-tel-margin",
                "brain-tel-detector", "brain-tel-calls"):
        box = page.locator("#" + eid).bounding_box()
        assert box is not None, eid
        assert box["width"] >= 40, f"{eid} is {box['width']:.0f}px wide"
        assert box["x"] + box["width"] <= PHONE["width"] + 1, (
            f"{eid} runs off the right edge of a {PHONE['width']}px screen: {box}")
    overflow = page.evaluate(
        "() => document.documentElement.scrollWidth - document.documentElement.clientWidth")
    assert overflow <= 0, f"the page scrolls sideways by {overflow}px"
    page.close()


# ---------- the tiered policy on the phone path (4.10's actual goal) ----------
#
# The Remote brain panel above drives the GRID WORLD, and
# PLAN-onboard-perception.md 1.12 is explicit that a COCO detector finds
# nothing in a raycaster render -- so a tiered mission started there
# exercises the loop and never the detector. The walk that can actually
# test YOLO + CLIP is the Robot-view one: real phone frames pushed into
# TeleopRobot, perception running in the brain process beside it. 4.10
# calls that "the highest-fidelity pre-hardware test available", and it is
# the thing the policy was built for.
#
# "Drive via brain" hardcoded `policy: "vision"` until 2026-09-07, so the
# one path where these models see real pixels could not ask for them.


def open_robot_view(browser, twin_server, *, health=None):
    page, errors = open_twin(browser, twin_server, mode="robot")
    page.route("**/brain-stub/health", lambda route: route.fulfill(
        status=200, content_type="application/json",
        body=_json(health if health is not None else TIERED_BRAIN_HEALTH)))
    page.click("#btn-settings")
    page.fill("#cfg-brain-url", twin_server + "/brain-stub")
    page.click("#btn-brain-connect")
    page.wait_for_timeout(600)
    page.click('.tab-btn[data-tab="guide"]')
    return page, errors


def test_a_walk_can_choose_the_tiered_policy(browser, twin_server):
    """The picker appears with the toggle it belongs to, and offers no
    "frontier": a photograph carries no grid coordinates and TeleopRobot has
    no distance sensor, so the rule-based policy is a blind wall-follower
    here rather than a cheaper option."""
    page, errors = open_robot_view(browser, twin_server)
    sync_api.expect(page.locator("#drive-policy-row")).to_be_hidden()

    page.check("#cfg-drive-via-brain")
    sync_api.expect(page.locator("#drive-policy-row")).to_be_visible()
    values = page.eval_on_selector_all(
        "#cfg-drive-policy option", "els => els.map(e => e.value)")
    assert values == ["vision", "tiered"], values

    page.locator("#cfg-drive-policy").select_option("tiered")
    assert page.locator("#cfg-drive-policy").input_value() == "tiered"
    assert not errors, errors
    page.close()


def test_the_walk_says_the_perception_models_run_on_its_own_frames(browser, twin_server):
    """The whole point of doing this on a phone rather than in the twin, said
    where the walk is started: these are the frames the detector will
    actually see."""
    page, _ = open_robot_view(browser, twin_server)
    page.check("#cfg-drive-via-brain")
    page.locator("#cfg-drive-policy").select_option("tiered")
    sub = page.locator("#drive-via-brain-sub")
    sync_api.expect(sub).to_contain_text("yolo11s.pt")
    sync_api.expect(sub).to_contain_text("your frames")
    page.close()


def test_a_walk_warns_before_you_start_when_the_models_are_missing(browser, twin_server):
    """Standing in a room holding a phone is the worst moment to discover an
    optional dependency. The mission does refuse with the pip command, but
    the row can say it first."""
    page, _ = open_robot_view(
        browser, twin_server,
        health=dict(TIERED_BRAIN_HEALTH, perception_available=False))
    page.check("#cfg-drive-via-brain")
    page.locator("#cfg-drive-policy").select_option("tiered")
    sync_api.expect(page.locator("#drive-via-brain-sub")).to_contain_text(
        "requirements-perception.txt")
    page.close()


def test_the_walk_sends_the_policy_it_was_set_to(browser, twin_server):
    """The bug this closes: "Drive via brain" sent `policy: "vision"` no
    matter what, so the one path where YOLO and CLIP get real pixels could
    not ask for them. Asserted on the request body, because every other
    symptom of getting this wrong is invisible -- a vision walk and a tiered
    walk look identical from the phone until the bill arrives."""
    context = browser.new_context(viewport=PHONE, permissions=["camera"])
    context.add_init_script(
        "(() => { try {"
        " localStorage.setItem('guidanceMode','robot');"
        " localStorage.setItem('vp_guide_onboarded','1');"
        " localStorage.setItem('vp_drive_via_brain','1');"
        " localStorage.setItem('vp_drive_policy','tiered');"
        " } catch (e) {} })();")
    page = context.new_page()
    errors = []
    page.on("pageerror", lambda e: errors.append(str(e)))

    started = []
    page.route("**/brain-stub/health", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json(TIERED_BRAIN_HEALTH)))
    # "Drive via brain" refuses unless the robot server reports mode: teleop,
    # and it primes one frame before the mission exists.
    page.route("**/health", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json({
            "status": "ok", "mode": "teleop", "seconds_since_last_command": 0.1,
            "watchdog_timeout_s": 1.0})))
    page.route("**/teleop/frame", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json({"ok": True})))

    def capture_start(route):
        started.append(route.request.post_data_json)
        route.fulfill(status=200, content_type="application/json",
                      body=_json({"started": True, "status": {"running": True}}))

    page.route("**/brain-stub/mission/start", capture_start)
    page.route("**/brain-stub/mission/status", lambda route: route.fulfill(
        status=200, content_type="application/json",
        body=_json(tiered_status())))

    page.goto(twin_server, wait_until="networkidle")
    page.wait_for_timeout(500)
    page.click("#btn-settings")
    page.fill("#cfg-server-url", twin_server)
    page.fill("#cfg-brain-url", twin_server + "/brain-stub")
    page.click("#btn-brain-connect")
    page.wait_for_timeout(600)
    page.click('.tab-btn[data-tab="guide"]')
    page.wait_for_timeout(300)
    page.fill("#guidance-target", "red backpack")
    page.click("#btn-guidance")
    page.wait_for_timeout(3000)

    assert started, "no mission was started -- drive via brain never fired"
    assert started[0]["policy"] == "tiered", started[0]
    assert started[0]["target_object"] == "red backpack"
    assert not errors, errors
    context.close()


def test_a_mission_that_ENDS_says_so_instead_of_sitting_on_deciding(browser, twin_server):
    """Observed on a real rig walk, 2026-09-12: "it eventually got stuck at
    deciding".

    `driveViaBrainStep()` pauses the run when it sees the mission has
    ended -- and the caller then dropped that answer because the run was
    paused, by the very pause the answer had just caused. The caption
    stayed on the "Deciding..." set before dispatch, so the one fact a
    person needs -- the mission ended, and why -- was the one thing the
    HUD would not show. A mission ending on max_steps is the common case,
    and it looked identical to a hang.

    Built on the same harness as
    `test_the_walk_sends_the_policy_it_was_set_to`, and everything in it
    is load bearing: without `permissions=["camera"]` getUserMedia never
    attaches, and without `/health` reporting `mode: teleop` drive-via-
    brain refuses and the page quietly runs the plain Guide loop instead
    -- where the caption reads "Analyzing..." and this bug does not live.
    """
    context = browser.new_context(viewport=PHONE, permissions=["camera"])
    context.add_init_script(
        "(() => { try {"
        " localStorage.setItem('guidanceMode','robot');"
        " localStorage.setItem('vp_guide_onboarded','1');"
        " localStorage.setItem('vp_drive_via_brain','1');"
        " localStorage.setItem('vp_drive_policy','tiered');"
        " } catch (e) {} })();")
    page = context.new_page()
    errors = []
    page.on("pageerror", lambda e: errors.append(str(e)))

    ended = dict(tiered_status(running=False))
    ended["outcome"] = "max_steps"
    ended["error"] = None

    page.route("**/brain-stub/health", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json(TIERED_BRAIN_HEALTH)))
    page.route("**/health", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json({
            "status": "ok", "mode": "teleop", "seconds_since_last_command": 0.1,
            "watchdog_timeout_s": 1.0})))
    page.route("**/teleop/frame", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json({"ok": True})))
    page.route("**/brain-stub/mission/start", lambda route: route.fulfill(
        status=200, content_type="application/json",
        body=_json({"started": True, "status": {"running": True}})))
    page.route("**/brain-stub/mission/status", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json(ended)))

    page.goto(twin_server, wait_until="networkidle")
    page.wait_for_timeout(500)
    page.click("#btn-settings")
    page.fill("#cfg-server-url", twin_server)
    page.fill("#cfg-brain-url", twin_server + "/brain-stub")
    page.click("#btn-brain-connect")
    page.wait_for_timeout(600)
    page.click('.tab-btn[data-tab="guide"]')
    page.wait_for_timeout(300)
    page.fill("#guidance-target", "woven laundry basket")
    page.click("#btn-guidance")

    sync_api.expect(page.locator("#guide-caption-text")).to_contain_text(
        "max_steps", timeout=15000)
    text = page.inner_text("#guide-caption-text")
    assert "Deciding" not in text, text
    assert not errors, errors
    context.close()


# ---------- reaching a tunnelled robot or brain (ngrok) ----------
#
# The brain cannot be deployed -- policy "tiered" loads YOLO and CLIP into
# that process -- so the deployed twin reaches it through a tunnel. ngrok's
# free tier answers any request carrying a browser User-Agent with an HTML
# interstitial instead of proxying it, which means every fetch() from this
# page comes back as markup and res.json() throws on a `<`. The documented
# escape is a request header, and only this page can send it.
#
# Scoped to ngrok hosts because it is a custom header: it forces a CORS
# preflight on requests that would otherwise be simple, one of which is the
# twice-a-second /health poll behind the watchdog readout.


def _headers_for(page, url_state):
    """What the page would send to a given robot/brain URL. Read through a
    real request rather than by calling internals -- app.js is one IIFE with
    no test hooks, and the header only matters if it reaches the wire."""
    seen = {}

    def capture(route):
        seen.update({k.lower(): v for k, v in route.request.headers.items()})
        route.fulfill(status=200, content_type="application/json",
                      body=_json({"status": "ok", "mode": "sim",
                                  "seconds_since_last_command": 0.1,
                                  "watchdog_timeout_s": 1.0}))

    page.route("**/health", capture)
    page.evaluate(url_state)
    page.wait_for_timeout(900)
    return seen


def test_a_tunnelled_robot_gets_the_interstitial_bypass(browser, twin_server):
    """Without this the twin looks broken in a way that says nothing useful:
    the request succeeds, the body is an HTML page, and the only symptom is
    a JSON parse error."""
    page, errors = open_twin(browser, twin_server)
    page.click("#btn-settings")
    page.fill("#cfg-server-url", "https://salami-turbulent-engorge.ngrok-free.dev")
    page.click("#btn-connect")
    page.click('.tab-btn[data-tab="sim"]')
    headers = _headers_for(page, "() => {}")

    assert headers.get("ngrok-skip-browser-warning") == "1", sorted(headers)
    assert not errors, errors
    page.close()


def test_a_plain_host_is_left_alone(browser, twin_server):
    """The header is a workaround for someone else's free tier, not a
    protocol -- and adding it everywhere would put a CORS preflight on the
    twice-a-second health poll of every LAN and localhost setup."""
    page, errors = open_twin(browser, twin_server)
    page.click("#btn-settings")
    page.fill("#cfg-server-url", twin_server)
    page.click("#btn-connect")
    page.click('.tab-btn[data-tab="sim"]')
    headers = _headers_for(page, "() => {}")

    assert "ngrok-skip-browser-warning" not in headers, sorted(headers)
    assert not errors, errors
    page.close()


def test_the_brain_gets_it_too_and_a_path_prefixed_url_still_works(browser, twin_server):
    """One ngrok free domain serves both services by path
    (`https://host/` robot, `https://host/brain` brain), so the brain URL
    carries a path -- which the header check must not be confused by, and
    which brainApi's plain string concatenation has to keep handling."""
    page, errors = open_twin(browser, twin_server)
    seen = {}

    def capture(route):
        seen.update({k.lower(): v for k, v in route.request.headers.items()})
        seen["url"] = route.request.url
        route.fulfill(status=200, content_type="application/json",
                      body=_json(TIERED_BRAIN_HEALTH))

    page.route("**/brain/health", capture)
    page.click("#btn-settings")
    page.fill("#cfg-brain-url", "https://salami-turbulent-engorge.ngrok-free.dev/brain")
    page.click("#btn-brain-connect")
    page.wait_for_timeout(900)

    assert seen.get("ngrok-skip-browser-warning") == "1", sorted(seen)
    assert seen.get("url", "").endswith("/brain/health"), seen.get("url")
    assert not errors, errors
    page.close()


# ---------- 1.11a on the panel: measured, and visibly not enforced ----------
#
# The amendment is proposed and undecided, and CLAUDE.md section 7 is why
# these are UI tests: *"a phase is not done when its tests pass. It is done
# when someone holding a phone can watch the thing it built do its job."*
# The job here is unusual -- the thing being watched is a measurement, not a
# behaviour -- so the row has one extra duty no other readout has: it must
# make it impossible to mistake the verdict for a decision the robot acted
# on. That is what the "not enforced" assertions below are protecting.


def test_the_corroboration_verdict_is_on_the_panel(browser, twin_server):
    """1.11a's falsifier is a walk that does not exist yet, so the verdict
    has to be visible on the walks that are about to be recorded -- by
    replay it can only ever be re-derived from frames already collected."""
    page, errors = open_with_brain(
        browser, twin_server,
        status=tiered_status(verdict="unclear", local_p=0.14))
    readout = page.locator("#brain-tel-corroboration")
    sync_api.expect(readout).to_contain_text("unclear", timeout=5000)
    text = readout.inner_text()
    # The local number AND the bar it was read against: a verdict without
    # its threshold is not readable, and 1.11a's whole finding is that the
    # bar for corroborating is not the bar for detecting.
    assert "0.14" in text and "0.5" in text, text
    assert not errors, errors
    page.close()


def test_the_panel_says_the_verdict_is_not_enforced(browser, twin_server):
    """The one thing this row must never do is look like a decision. Under
    the shipped rule the mission still believes the cloud on every one of
    these frames -- including the `unclear` ones."""
    page, _ = open_with_brain(browser, twin_server,
                              status=tiered_status(verdict="unclear"))
    sync_api.expect(page.locator("#brain-tel-corroboration")).to_contain_text(
        "not enforced", timeout=5000)
    page.close()


def test_the_corroborated_of_claimed_tally_is_shown(browser, twin_server):
    """1.11a's own list of what it still needs: *"the counter in 6.3 should
    show corroborated-versus-claimed, or the twin cannot show this
    working."* Two counts rather than a rate, for the same reason the
    deliberation counter is calls and frames."""
    page, _ = open_with_brain(
        browser, twin_server,
        status=tiered_status(claims=17, corroborated=4))
    text = page.locator("#brain-tel-corroboration").inner_text()
    assert "4/17" in text, text
    page.close()


def test_a_free_step_says_no_claim_rather_than_holding_the_last_verdict(browser, twin_server):
    """Most steps under this policy cost nothing, so most steps have no
    claim to corroborate. A verdict held over from the last paid call would
    be read as this step's -- the same staleness the deliberation counter
    avoids by showing both terms."""
    page, _ = open_with_brain(browser, twin_server,
                              status=tiered_status(verdict=None))
    sync_api.expect(page.locator("#brain-tel-corroboration")).to_contain_text(
        "no claim", timeout=5000)
    page.close()


def test_the_corroboration_row_hides_under_a_policy_with_no_tier(browser, twin_server):
    """Same rule as every other readout in this group: say nothing rather
    than draw a zero. "0/0 claims corroborated" would read as a tier that
    had stopped agreeing with anything."""
    page, _ = open_with_brain(browser, twin_server, status=frontier_status())
    page.wait_for_timeout(600)
    row = page.locator("#brain-tel-corroboration")
    assert row.count() == 1, "the corroboration readout is not in the page at all"
    sync_api.expect(page.locator("#brain-tier-rows")).to_be_hidden()
    page.close()


def test_the_corroboration_row_is_readable_on_a_phone(browser, twin_server):
    """The 40px model picker again. A fifth row in this group is the one
    most likely to push the panel past 390px, and `.select-input` already
    did exactly that once."""
    page, _ = open_with_brain(browser, twin_server,
                              status=tiered_status(verdict="unclear", claims=17))
    box = page.locator("#brain-tel-corroboration").bounding_box()
    assert box is not None and box["width"] >= 40, box
    assert box["x"] + box["width"] <= PHONE["width"] + 1, box
    overflow = page.evaluate(
        "() => document.documentElement.scrollWidth - document.documentElement.clientWidth")
    assert overflow <= 0, f"the page scrolls sideways by {overflow}px"
    page.close()


# ---------- the cloud endpoint's own connection check ----------
#
# Added 2026-09-08, after a real rig session was blocked by it. The vision
# service publishes no CORS headers and never needed any: the deployed twin
# is served from the same CloudFront distribution it calls. Serve the page
# from an ngrok tunnel instead and every vision call becomes cross-origin,
# the preflight 404s, and the operator sees "Load failed" on the Robot view
# camera -- with five URL fields on the Settings tab and nothing saying
# which one is wrong. curl says the service is fine, because it is.


def test_the_cloud_endpoint_has_a_connection_check(browser, twin_server):
    """The brain has had one since B4. The half that actually blocks a walk
    did not."""
    page, errors = open_twin(browser, twin_server, mode="guide")
    page.click("#btn-settings")
    assert page.locator("#btn-vision-connect").count() == 1
    sync_api.expect(page.locator("#vision-connection-status")).to_contain_text(
        "Not checked")
    assert not errors, errors
    page.close()


def test_a_healthy_but_cross_origin_vision_service_is_reported_as_the_fault(
        browser, twin_server):
    """The finding this exists for: **reachable and unusable are different
    states.** Retrying, changing the secret and restarting the service all do
    nothing, so the message has to name the origin mismatch rather than say
    "not reachable" about a service that answered."""
    page, errors = open_twin(browser, twin_server, mode="guide")
    # Healthy, and deliberately NOT the origin serving the page.
    page.route("https://vision.example.com/health", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json({"status": "ok"})))
    page.click("#btn-settings")
    page.fill("#cfg-url", "https://vision.example.com")
    page.click("#btn-vision-connect")
    status = page.locator("#vision-connection-status")
    sync_api.expect(status).to_contain_text("blocked by the browser", timeout=5000)
    text = status.inner_text()
    # It must name BOTH origins -- the whole failure is that they differ, and
    # the operator is looking at five URL fields.
    assert "vision.example.com" in text, text
    assert "Load failed" in text, text
    assert not errors, errors
    page.close()


def test_a_same_origin_vision_service_reads_as_connected(browser, twin_server):
    """Served from the origin it calls, which is the deployed arrangement:
    no preflight, so no CORS to get wrong."""
    page, errors = open_twin(browser, twin_server, mode="guide")
    page.route("**/health", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json({"status": "ok"})))
    page.click("#btn-settings")
    page.fill("#cfg-url", twin_server)
    page.click("#btn-vision-connect")
    sync_api.expect(page.locator("#vision-connection-status")).to_contain_text(
        "Connected", timeout=5000)
    assert not errors, errors
    page.close()


def test_an_unreachable_vision_service_says_so_without_blaming_cors(browser, twin_server):
    """A dead service and a blocked one need different fixes, so they must
    not render the same."""
    page, _ = open_twin(browser, twin_server, mode="guide")
    page.route("**/health", lambda route: route.abort())
    page.click("#btn-settings")
    page.fill("#cfg-url", twin_server)
    page.click("#btn-vision-connect")
    status = page.locator("#vision-connection-status")
    sync_api.expect(status).to_contain_text("Not reachable", timeout=5000)
    assert "blocked by the browser" not in status.inner_text()
    page.close()


def test_the_cloud_check_is_readable_on_a_phone(browser, twin_server):
    page, _ = open_twin(browser, twin_server, mode="guide")
    page.click("#btn-settings")
    box = page.locator("#btn-vision-connect").bounding_box()
    assert box is not None and box["width"] >= 40, box
    assert box["x"] + box["width"] <= PHONE["width"] + 1, box
    overflow = page.evaluate(
        "() => document.documentElement.scrollWidth - document.documentElement.clientWidth")
    assert overflow <= 0, f"the page scrolls sideways by {overflow}px"
    page.close()


# ---------- recording a tiered walk (the exclusion that expired) ----------
#
# "Record this walk" and "Drive via brain" were mutually exclusive from T3
# until 2026-09-08, on the grounds that recording saves a /navigate reply per
# frame and driving via brain never produces one. T4 made that false --
# missionStatusToRobotResult() returns the whole mission status, which since
# P2 carries the perception tri-state, the tier counters and 1.11a's
# corroboration verdict, i.e. strictly MORE than a /navigate reply.
#
# The exclusion had therefore become exactly backwards: the tiered walk is the
# only path where YOLO and CLIP ever see real pixels, so it is the single most
# valuable walk to keep, and it was the only one the twin refused to record.
# It cost a rig session. These tests are here so it cannot come back.


def test_turning_on_drive_via_brain_leaves_recording_alone(browser, twin_server):
    page, errors = open_with_brain(browser, twin_server, status=tiered_status(),
                                   mode="robot")
    page.click('.tab-btn[data-tab="guide"]')
    page.locator("#cfg-record-walk").check()
    page.locator("#cfg-drive-via-brain").check()
    assert page.locator("#cfg-record-walk").is_checked(), (
        "enabling Drive via brain switched recording off -- the tiered walk is "
        "the one most worth keeping")
    assert not errors, errors
    page.close()


def test_turning_on_recording_leaves_drive_via_brain_alone(browser, twin_server):
    """The same exclusion, from the other side. Both handlers enforced it."""
    page, errors = open_with_brain(browser, twin_server, status=tiered_status(),
                                   mode="robot")
    page.click('.tab-btn[data-tab="guide"]')
    page.locator("#cfg-drive-via-brain").check()
    page.locator("#cfg-record-walk").check()
    assert page.locator("#cfg-drive-via-brain").is_checked(), (
        "enabling recording switched Drive via brain off")
    assert not errors, errors
    page.close()


def test_both_switches_can_be_on_at_once(browser, twin_server):
    """The state a tiered rig walk actually needs: a real mission driving the
    robot AND every frame kept for the corpus."""
    page, _ = open_with_brain(browser, twin_server, status=tiered_status(),
                              mode="robot")
    page.click('.tab-btn[data-tab="guide"]')
    page.locator("#cfg-record-walk").check()
    page.locator("#cfg-drive-via-brain").check()
    assert page.locator("#cfg-record-walk").is_checked()
    assert page.locator("#cfg-drive-via-brain").is_checked()
    page.close()


# ---------------------------------------------------------------------------
# Phases A-C: the three readouts they owe the twin (section 7)
# ---------------------------------------------------------------------------
#
# In a real browser at a phone viewport, because that is where every UI bug
# in this project has actually been found -- a model picker that rendered
# empty, one that rendered 40px wide, an admin console usable only sideways.
# None of those were visible to a DOM-only check.


def test_the_odometry_line_reports_the_real_robot_and_is_readable_on_a_phone(
        browser, twin_server):
    """Phase B. `mode: sim` has working odometry, so this must show metres
    and a heading -- not the honest no-op, which is what a wrapper falling
    behind the interface would produce."""
    page, errors = _connect_for_depth(browser, twin_server)
    page.wait_for_function(
        "() => !/not connected/.test("
        "document.getElementById('odometry-readout').innerText)", timeout=5000)
    text = page.inner_text("#odometry-readout")
    assert "odometry:" in text
    assert "m travelled" in text, text
    assert "heading" in text, text
    # Path length, not displacement -- said on the line itself, because the
    # difference is the whole reason a robot searching one small room still
    # triggers a distance rule.
    assert "path length" in text, text

    box = page.locator("#odometry-readout").bounding_box()
    assert box and box["width"] > 200, (
        f"odometry line is {box and box['width']}px wide on a 390px phone")
    assert not errors, errors
    page.close()


def test_the_odometry_line_says_NO_ENCODERS_rather_than_zero(browser, twin_server):
    """The state every teleop rig walk is in, and the one that must not be
    mistaken for a robot that has not moved. Routed, because `mode: sim`
    cannot produce it and the point is what the page does with the honest
    no-op."""
    page, errors = open_twin(browser, twin_server)
    page.route("**/odometry", lambda route: route.fulfill(
        status=200, content_type="application/json",
        body='{"usable": false, "distance_m": null, "heading_deg": null}'))
    page.click("#btn-settings")
    page.fill("#cfg-server-url", twin_server)
    page.click("#btn-connect")
    page.click('.tab-btn[data-tab="sim"]')
    page.wait_for_function(
        "() => /no encoders/.test("
        "document.getElementById('odometry-readout').innerText)", timeout=5000)
    text = page.inner_text("#odometry-readout")
    assert "no encoders" in text
    # And it must say what happens as a result, or a reader has to know the
    # pacing rules to interpret it.
    assert "frame count" in text, text
    assert "0.00" not in text, "a no-encoder backend rendered as zero travel"
    assert not errors, errors
    page.close()


def test_a_server_with_no_odometry_route_says_so_rather_than_going_blank(
        browser, twin_server):
    """A 404 means the server predates the route, which is a real state
    while stacks are redeployed one at a time. Blank and "no motion" must
    not look alike -- the same rule the depth strip already follows."""
    page, errors = open_twin(browser, twin_server)
    page.route("**/odometry", lambda route: route.fulfill(status=404, body="{}"))
    page.click("#btn-settings")
    page.fill("#cfg-server-url", twin_server)
    page.click("#btn-connect")
    page.click('.tab-btn[data-tab="sim"]')
    page.wait_for_function(
        "() => /not reported/.test("
        "document.getElementById('odometry-readout').innerText)", timeout=5000)
    assert "not reported by this server" in page.inner_text("#odometry-readout")
    assert not errors, errors
    page.close()


def _tier_status_with(**tier_extra):
    """`tiered_status()` with the Phase A/C keys layered on, so these tests
    share the shape every other brain-panel test already asserts against
    rather than inventing a second one that can drift from the runner."""
    status = tiered_status()
    status["tier"].update(tier_extra)
    return status


def test_the_pacing_row_names_the_rule_in_force(browser, twin_server):
    """Phase C. A silent fallback from distance to frame count is how a
    walk becomes unattributable -- the same failure `crop_source` is
    reported for, one tier up. So the row says which."""
    page, errors = open_with_brain(browser, twin_server, status=_tier_status_with(
        pacing={"rule": "distance", "frames_absent": 2, "frames_bar": 6,
                "cm_since_call": 12.0, "cm_bar": 40.0,
                "odometry_usable": True, "reason": None}))
    sync_api.expect(page.locator("#brain-tel-pacing")).to_contain_text(
        "by distance", timeout=5000)
    text = page.inner_text("#brain-tel-pacing")
    assert "12/40cm" in text, text
    # The number a person actually wants while watching: how much further
    # before anything looks again.
    assert "next look in 28cm" in text, text
    assert not errors, errors
    page.close()


def test_the_pacing_row_says_WHY_it_fell_back_to_frames(browser, twin_server):
    """The state every teleop rig walk is in -- a phone has no encoders --
    so this is the row a reader sees on the walks that matter."""
    page, errors = open_with_brain(browser, twin_server, status=_tier_status_with(
        pacing={"rule": "frames", "frames_absent": 3, "frames_bar": 6,
                "cm_since_call": None, "cm_bar": 40.0,
                "odometry_usable": False,
                "reason": "this backend reports no odometry"}))
    sync_api.expect(page.locator("#brain-tel-pacing")).to_contain_text(
        "by frame count", timeout=5000)
    text = page.inner_text("#brain-tel-pacing")
    assert "3/6" in text, text
    assert "no odometry" in text, text
    assert not errors, errors
    page.close()


def test_a_held_goal_never_looks_like_a_fresh_answer(browser, twin_server):
    """Phase A. Three states that must not look alike: waiting while
    driving on the last goal, waiting with no goal yet, and not waiting."""
    page, errors = open_with_brain(browser, twin_server, status=_tier_status_with(
        in_flight="cold_search", holding="FORWARD"))
    sync_api.expect(page.locator("#brain-tel-inflight")).to_contain_text(
        "in flight", timeout=5000)
    text = page.inner_text("#brain-tel-inflight")
    assert "cold_search call in flight" in text
    assert "holding goal FORWARD" in text, text
    assert not errors, errors
    page.close()


def test_waiting_with_no_goal_yet_is_distinguished_from_holding_one(
        browser, twin_server):
    """The opening call has nothing to hold, so the robot scans. A
    different state from driving on a confirmed goal, and it reads
    differently."""
    page, errors = open_with_brain(browser, twin_server, status=_tier_status_with(
        in_flight="mission_start", holding=None))
    sync_api.expect(page.locator("#brain-tel-inflight")).to_contain_text(
        "in flight", timeout=5000)
    text = page.inner_text("#brain-tel-inflight")
    assert "mission_start call in flight" in text
    assert "scanning, no goal yet" in text, text
    assert "holding goal" not in text
    assert not errors, errors
    page.close()


def test_no_call_outstanding_reads_as_idle_rather_than_blank(browser, twin_server):
    """Blank and "nothing is happening" must not look alike -- the rule the
    depth strip already follows, applied to the deliberation row."""
    page, errors = open_with_brain(browser, twin_server,
                                   status=_tier_status_with(in_flight=None,
                                                            holding=None))
    sync_api.expect(page.locator("#brain-tel-inflight")).to_contain_text(
        "idle", timeout=5000)
    assert "no call outstanding" in page.inner_text("#brain-tel-inflight")
    assert not errors, errors
    page.close()


def test_a_late_landed_verdict_says_it_arrived_from_an_earlier_call(
        browser, twin_server):
    """Phase A moves the verdict off the frame it is about. Unlabelled it
    reads as a verdict about the picture on screen now -- the stale-readout
    failure this row already guards against from the other direction."""
    status = tiered_status()
    status["tier"]["corroboration"].update(
        {"landed_late": True, "for_trigger": "cold_search"})
    page, errors = open_with_brain(browser, twin_server, status=status)
    sync_api.expect(page.locator("#brain-tel-corroboration")).to_contain_text(
        "landed from the cold_search call", timeout=5000)
    text = page.inner_text("#brain-tel-corroboration")
    assert "corroborated" in text
    assert "not enforced" in text, text
    assert not errors, errors
    page.close()


# ---------- N1: the map (PLAN-mapping.md) ----------
#
# The map is this phase's whole UI proof, and section 7's rule is that a
# phase is done when someone holding a phone can watch it work. These go
# through the real server and the real MockWorld rather than a stubbed
# payload: what is under test is that the two ends agree about a house.


def _connect_for_map(browser, twin_server):
    page, errors = open_twin(browser, twin_server)
    page.click("#btn-settings")
    page.fill("#cfg-server-url", twin_server)
    page.click("#btn-connect")
    # Same lesson as the depth strip: the canvas lives on the Sim tab, and
    # "the element exists" is not "someone can see it".
    page.click('.tab-btn[data-tab="sim"]')
    page.wait_for_function(
        "() => { const c = document.getElementById('world-map');"
        " return c && c.width > 0; }", timeout=5000)
    return page, errors


def test_the_map_is_drawn_at_the_size_the_server_reports(browser, twin_server):
    """The grid declares its own shape, exactly as the depth grid does, so
    the canvas must come from the data and not from a hardcoded house."""
    page, errors = _connect_for_map(browser, twin_server)
    reported = page.evaluate(
        "async () => (await (await fetch(document.getElementById('cfg-server-url').value"
        " + '/world/map')).json())")
    canvas = page.evaluate(
        "() => { const c = document.getElementById('world-map');"
        " return {w: c.width, h: c.height}; }")
    assert canvas["w"] % reported["width"] == 0
    assert canvas["h"] % reported["height"] == 0
    assert canvas["w"] // reported["width"] == canvas["h"] // reported["height"]
    assert not errors, errors
    page.close()


def test_the_map_is_visible_on_a_phone(browser, twin_server):
    """Bug (2)'s shape again. A map collapsed to nothing is worse than no
    map, because the readout beside it still says how much was seen."""
    page, _ = _connect_for_map(browser, twin_server)
    box = page.locator("#world-map").bounding_box()
    assert box is not None and box["width"] >= 100 and box["height"] >= 60, box
    assert page.locator("#world-map").is_visible()
    page.close()


def test_the_map_the_page_draws_is_the_map_the_server_reports(browser, twin_server):
    """The single property worth pinning in a browser, and the one a
    stubbed payload could not pin: the page draws what the SERVER said
    about the house, not a floor plan it worked out for itself.

    Note this suite shares one server across the module and MockRobot's
    world persists -- there is no reset endpoint, on purpose, because real
    hardware has none either. So this drives and then checks agreement,
    rather than assuming a start pose or a particular amount of house.
    """
    page, errors = _connect_for_map(browser, twin_server)

    for _ in range(3):
        page.click("#btn-forward")
        page.wait_for_timeout(200)

    # The readout carries "<seen>/<total> cells seen", which is the page's
    # own count off the payload it drew.
    page.wait_for_function(
        "async () => {"
        " const u = document.getElementById('cfg-server-url').value;"
        " const m = await (await fetch(u + '/world/map')).json();"
        " const truth = m.cells.filter(c => c !== -1).length;"
        " const el = document.getElementById('map-readout');"
        " const shown = el && el.textContent.match(/(\\d+)\\/(\\d+) cells/);"
        " return !!shown && Number(shown[1]) === truth"
        "        && Number(shown[2]) === m.cells.length; }",
        timeout=8000)
    assert not errors, errors
    page.close()


def test_part_of_the_house_is_still_unknown(browser, twin_server):
    """A map is not a copy of the floor plan. If every cell were known the
    instant the page connected, the tri-state would be decorative and the
    twin would be showing something no mapper produces -- so the drawn
    map must contain cells in the never-seen colour."""
    page, errors = _connect_for_map(browser, twin_server)
    unknown = page.evaluate(
        "async () => { const u = document.getElementById('cfg-server-url').value;"
        " const m = await (await fetch(u + '/world/map')).json();"
        " return m.cells.filter(c => c === -1).length; }")
    assert unknown > 0, "the whole house was known at connect -- is it being copied?"
    assert not errors, errors
    page.close()


def test_a_server_with_no_world_routes_says_so_instead_of_going_blank(browser, twin_server):
    """A pre-N1 server is a real state while deployments are redeployed one
    at a time. Blank and 'no map' must not look alike -- the same rule the
    depth strip follows, and the reason it says 'not reported by this
    server' rather than drawing nothing."""
    page, errors = open_twin(browser, twin_server)
    page.route("**/world/pose", lambda route: route.fulfill(status=404, body="{}"))
    page.route("**/world/map", lambda route: route.fulfill(status=404, body="{}"))
    page.click("#btn-settings")
    page.fill("#cfg-server-url", twin_server)
    page.click("#btn-connect")
    page.click('.tab-btn[data-tab="sim"]')
    page.wait_for_function(
        "() => (document.getElementById('map-readout') || {}).textContent"
        "      === 'map: not reported by this server'", timeout=8000)
    assert page.evaluate("() => document.getElementById('world-map').width") == 0
    assert not errors, errors
    page.close()
