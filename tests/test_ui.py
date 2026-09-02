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
