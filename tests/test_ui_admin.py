"""
tests/test_ui_admin.py

The admin console, in a real browser. tests/test_ui.py covers the twin; this
covers the other page, which had accumulated real client logic (auto-scoring,
replay, sorting, score rendering) and no coverage at all.

**Written after the same bug shipped twice.** The twin's model picker
rendered empty because its fetch ran before the vision URL was restored; the
console's "Replay with..." dropdown rendered empty because the walk list was
rendered before the model list was fetched. Same shape, different page, and
the second one reached a phone because only the first page had tests.

Frames and models are stubbed via Playwright request interception, so no
recordings volume and no vision service are involved.
"""

import json
import os
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

PHONE = {"width": 390, "height": 844}
DESKTOP = {"width": 1280, "height": 900}

MODELS = {
    "default": "us.anthropic.claude-opus-4-5-20251101-v1:0",
    "models": [
        {"id": "us.anthropic.claude-opus-4-5-20251101-v1:0", "label": "Claude Opus 4.5"},
        {"id": "qwen.qwen3-vl-235b-a22b", "label": "Qwen3-VL"},
        {"id": "amazon.nova-lite-v1:0", "label": "Nova Lite"},
    ],
}

WALKS = {
    "recording_dir": "/tmp/recordings",
    "walks": [
        {"walk": "red-backpack-opus-4-5-20260830-104705", "frames": 14, "bytes": 984_000,
         "label": None, "model_id": "us.anthropic.claude-opus-4-5-20251101-v1:0",
         "finished": True, "recorded_at": 1788000000.0,
         "eval": {"score": 72, "verdict": "good", "flags": ["oscillating"],
                  "basis": "judge+metrics", "model_id": "us.anthropic.claude-opus-4-5-20251101-v1:0"},
         "replays": []},
        {"walk": "red-backpack-qwen3-vl-235b-a22b-20260830-104120", "frames": 39,
         "bytes": 2_400_000, "label": None, "model_id": "qwen.qwen3-vl-235b-a22b",
         "finished": True, "recorded_at": 1787999000.0,
         "eval": {"score": 56, "verdict": "mixed", "flags": ["oscillating"],
                  "basis": "judge+metrics", "model_id": "qwen.qwen3-vl-235b-a22b"},
         "replays": []},
    ],
}


@pytest.fixture(scope="module")
def admin_server(tmp_path_factory):
    root = tmp_path_factory.mktemp("recordings")
    config = root / "robot.yaml"
    config.write_text(f"brain:\n  recording_dir: {root / 'walks'}\n")
    (root / "walks").mkdir()
    port = free_port()
    url = f"http://127.0.0.1:{port}"
    proc = subprocess.Popen(
        [sys.executable, "-m", "uvicorn", "control.admin_server:app",
         "--host", "127.0.0.1", "--port", str(port), "--log-level", "warning"],
        cwd=REPO_ROOT, env={**os.environ, "BRAIN_CONFIG_PATH": str(config)},
    )
    try:
        deadline = time.monotonic() + SERVER_START_TIMEOUT_S
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                pytest.fail(f"admin server exited early with code {proc.returncode}")
            try:
                if httpx.get(f"{url}/health", timeout=0.5).status_code == 200:
                    break
            except httpx.HTTPError:
                time.sleep(0.05)
        else:
            pytest.fail("admin server was not healthy in time")
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


def open_console(browser, admin_server, *, models=MODELS, walks=WALKS,
                 viewport=DESKTOP, models_delay_ms=0, summary=None):
    context = browser.new_context(viewport=viewport)
    context.add_init_script(
        '(() => { try { localStorage.setItem("vp_admin_secret", "test"); } catch (e) {} })();')
    page = context.new_page()

    def models_route(route):
        if models_delay_ms:
            time.sleep(models_delay_ms / 1000)
        route.fulfill(status=200, content_type="application/json", body=json.dumps(models))

    page.route("**/recording/models", models_route)
    page.route("**/recording/walks", lambda r: r.fulfill(
        status=200, content_type="application/json", body=json.dumps(walks)))
    page.route("**/recording/summary", lambda r: r.fulfill(
        status=200, content_type="application/json",
        body=json.dumps(summary if summary is not None else {"rows": []})))
    page.route("**/stats", lambda r: r.fulfill(
        status=200, content_type="application/json",
        body=json.dumps({"walks": len(walks["walks"]), "frames": 53, "bytes": 3_384_000})))
    # Auto-scoring would otherwise fire for every finished walk.
    page.route("**/evaluation", lambda r: r.fulfill(
        status=200, content_type="application/json", body=json.dumps({"score": 0})))

    errors = []
    page.on("pageerror", lambda e: errors.append(str(e)))
    page.goto(admin_server + "/admin", wait_until="networkidle")
    # NOT clicking Connect: admin.js auto-connects when a secret is stored,
    # and clicking as well fires a SECOND load -- by which point the model
    # list is already cached, which masked exactly the bug these tests exist
    # for. This is the flow a real page load takes.
    page.wait_for_selector(".walk", timeout=10_000)
    return page, errors


def replay_options(page, index=0):
    return page.locator(".replay-select").nth(index).locator("option")


# ---------- the bug that shipped ----------


def test_the_replay_dropdown_is_populated(browser, admin_server):
    """It shipped with only its placeholder in it: the walk list was rendered
    before the model list had been fetched, so every dropdown was built from
    an empty array. Asserting on option COUNT, because the broken version
    threw nothing and looked fine."""
    page, errors = open_console(browser, admin_server)
    assert errors == [], f"uncaught page errors: {errors}"

    options = replay_options(page)
    sync_api.expect(options).to_have_count(len(MODELS["models"]) + 1)  # + "Replay with…"
    labels = [options.nth(i).inner_text() for i in range(options.count())]
    assert any("opus" in x.lower() for x in labels), labels
    assert any("qwen" in x.lower() for x in labels), labels
    page.close()


def test_the_dropdown_fills_in_even_when_the_model_list_is_slow(browser, admin_server):
    """The fix renders the walks first and repaints when the models land, so
    a slow model fetch must not leave the dropdowns permanently empty."""
    page, _ = open_console(browser, admin_server, models_delay_ms=600)
    sync_api.expect(replay_options(page)).to_have_count(len(MODELS["models"]) + 1)
    page.close()


def test_the_console_still_lists_walks_when_models_are_unavailable(browser, admin_server):
    """A vision service that is down must cost the replay control, not the
    whole page -- browsing and deleting recordings has nothing to do with it."""
    page, errors = open_console(
        browser, admin_server, models={"models": [], "default": None})
    sync_api.expect(page.locator(".walk")).to_have_count(len(WALKS["walks"]))
    sync_api.expect(replay_options(page)).to_have_count(1)  # placeholder only
    assert errors == []
    page.close()


# ---------- the rest of the console ----------


def test_walks_are_listed_newest_first_by_recording_time(browser, admin_server):
    """Not by name: the walk name carries the model now, so sorting on it
    groups by model and buries walks recorded between two runs on another --
    which read in the console as the walks having gone missing."""
    page, _ = open_console(browser, admin_server)
    names = page.locator(".walk-name").all_inner_texts()
    # The opus walk is newer (recorded_at) though its name sorts earlier.
    assert "opus" in names[0], names
    page.close()


def test_the_score_and_the_operator_label_are_separate_controls(browser, admin_server):
    """The machine score is advisory and must never look like the operator's
    own label -- they are deliberately different shapes."""
    page, _ = open_console(browser, admin_server)
    walk = page.locator(".walk").first
    sync_api.expect(walk.locator(".score-badge")).to_have_count(1)
    assert "72" in walk.locator(".score-badge").inner_text()
    sync_api.expect(walk.locator(".label-select")).to_have_count(1)
    assert walk.locator(".label-select").input_value() == ""
    page.close()


# ---------- narrow screens ----------


def test_the_console_does_not_scroll_sideways_on_a_phone(browser, admin_server):
    """The page was built at desktop width: one row per walk, with the name,
    a label picker, a replay picker and four buttons side by side. On a phone
    that row could not fit, so the buttons ran off the right edge and the
    name column collapsed to about 130px -- wrapping a walk name over six
    lines beside a wide empty gap.

    Horizontal overflow is the symptom worth pinning, because it is what
    makes controls unreachable rather than merely ugly."""
    page, _ = open_console(browser, admin_server, viewport=PHONE)
    overflow = page.evaluate(
        "() => document.documentElement.scrollWidth - document.documentElement.clientWidth")
    assert overflow <= 1, f"page scrolls {overflow}px sideways at {PHONE['width']}px wide"
    page.close()


def test_every_walk_control_is_reachable_on_a_phone(browser, admin_server):
    """Not scrolling sideways is not enough on its own -- the controls have
    to actually be on screen and hittable."""
    page, _ = open_console(browser, admin_server, viewport=PHONE)
    walk = page.locator(".walk").first
    for selector in (".label-select", ".replay-select", ".walk-actions button"):
        el = walk.locator(selector).first
        sync_api.expect(el).to_be_visible()
        box = el.bounding_box()
        assert box is not None, selector
        assert box["x"] >= 0, f"{selector} starts off the left edge"
        assert box["x"] + box["width"] <= PHONE["width"] + 1, (
            f"{selector} runs {box['x'] + box['width'] - PHONE['width']:.0f}px "
            f"past the right edge")
    page.close()


def test_a_walk_name_gets_the_full_width_on_a_phone(browser, admin_server):
    """The visible symptom: the name was squeezed into a narrow column beside
    the controls instead of having the row to itself."""
    page, _ = open_console(browser, admin_server, viewport=PHONE)
    name = page.locator(".walk-name").first.bounding_box()
    assert name is not None
    assert name["width"] > PHONE["width"] * 0.6, (
        f"the walk name only gets {name['width']:.0f}px of {PHONE['width']}px")
    page.close()


def test_the_slideshow_controls_are_thumb_sized_on_a_phone(browser, admin_server):
    """Prev/Next/Play are pressed once per frame -- a 39-frame walk is 39
    taps -- and at desktop sizing they were 34px tall and 52-70px wide, under
    every touch-target guideline going.

    This asserts the thing that actually changed. An earlier version of this
    test asserted the controls were on screen, which passed with the mobile
    CSS removed: at 390x844 the desktop layout does fit, so that test proved
    nothing. The rest of the phone slideshow work (safe-area insets, sticky
    controls, swipe) is precautionary -- no reproduction was found for it.
    """
    walks = json.loads(json.dumps(WALKS))
    page, _ = open_console(browser, admin_server, viewport=PHONE, walks=walks)

    page.route("**/recording/walks/*", lambda r: r.fulfill(
        status=200, content_type="application/json", body=json.dumps({
            "walk": walks["walks"][0]["walk"],
            "frames": [{"file": "frame-0000.jpg", "bytes": 1000}],
            "entries": [{"seq": 0, "file": "frame-0000.jpg",
                         "navigate": {"action": "FORWARD", "reasoning": "r"}}],
            "label": None, "model_id": None, "meta": None, "eval": None})))
    page.route("**/frames/*", lambda r: r.fulfill(
        status=200, content_type="image/jpeg", body=b"\xff\xd8\xff\xd9"))

    page.locator(".walk").first.locator(".walk-actions button", has_text="View").click()
    page.wait_for_selector(".frame", timeout=10_000)
    page.locator(".frame img").first.click()
    sync_api.expect(page.locator("#slideshow")).to_have_class("slideshow show")

    for sel in ("#slideshow-prev", "#slideshow-next", "#slideshow-play"):
        box = page.locator(sel).bounding_box()
        assert box is not None, sel
        assert box["height"] >= 44, f"{sel} is only {box['height']:.0f}px tall"
        assert box["width"] >= 90, f"{sel} is only {box['width']:.0f}px wide"
    page.close()


def test_the_summary_panel_renders_per_model_rows(browser, admin_server):
    """The comparison that used to live in a chat message. It has to be on
    the page, and it has to fit a phone."""
    rows = {"rows": [
        {"model_id": "us.anthropic.claude-opus-4-5-20251101-v1:0", "prompt_variant": "default",
         "source": "recorded", "walks": 5, "mean_score": 74, "median_score": 73,
         "best": 80, "worst": 66, "reach_rate": 1.0, "median_frames": 8,
         "collisions": 0, "flags": {"oscillating": 3}},
        {"model_id": "qwen.qwen3-vl-235b-a22b", "prompt_variant": "default",
         "source": "recorded", "walks": 5, "mean_score": 85, "median_score": 91,
         "best": 96, "worst": 40, "reach_rate": 0.8, "median_frames": 6,
         "collisions": 1, "flags": {"collision": 1}},
    ]}
    page, errors = open_console(browser, admin_server, viewport=PHONE, summary=rows)

    sync_api.expect(page.locator("#summary-panel")).to_be_visible()
    sync_api.expect(page.locator(".summary-row")).to_have_count(2)
    text = page.locator("#summary").inner_text()
    assert "74" in text and "85" in text
    assert "hit something" in text, "a collision must be called out, not buried in a mean"

    overflow = page.evaluate(
        "() => document.documentElement.scrollWidth - document.documentElement.clientWidth")
    assert overflow <= 1, f"summary panel scrolls {overflow}px sideways on a phone"
    assert errors == []
    page.close()


def test_the_summary_panel_stays_hidden_with_nothing_to_compare(browser, admin_server):
    page, _ = open_console(browser, admin_server, summary={"rows": []})
    sync_api.expect(page.locator("#summary-panel")).to_be_hidden()
    page.close()


# ---------- 3.64 C1: a replay that outlives its response ----------


def test_a_timed_out_replay_waits_for_the_new_result_not_the_old_one(browser, admin_server):
    """3.64 C1: the POST outlives API Gateway's 30 s and the console polls
    for the result -- but it took the first stored replay of that model, so
    a re-replay showed the OLD result as the new one. It must wait for a
    record stamped after the request."""
    walks = json.loads(json.dumps(WALKS))
    model = MODELS["models"][1]["id"]
    old = {"model_id": model, "prompt_variant": "default", "score": 81,
           "verdict": "good", "replayed_at": 1788000100.0, "agreement": 0.9}
    new = {**old, "score": 34, "verdict": "poor", "replayed_at": 1788000900.0}
    walks["walks"][0]["replays"] = [old]
    context = browser.new_context(viewport=PHONE)
    context.add_init_script(
        '(() => { try { localStorage.setItem("vp_admin_secret", "test"); } catch (e) {} '
        'window.ADMIN_REPLAY_POLL_MS = 100; })();')
    page = context.new_page()
    page.route("**/recording/models", lambda r: r.fulfill(
        status=200, content_type="application/json", body=json.dumps(MODELS)))
    page.route("**/recording/walks", lambda r: r.fulfill(
        status=200, content_type="application/json", body=json.dumps(walks)))
    page.route("**/recording/summary", lambda r: r.fulfill(
        status=200, content_type="application/json", body=json.dumps({"rows": []})))
    page.route("**/stats", lambda r: r.fulfill(
        status=200, content_type="application/json",
        body=json.dumps({"walks": 2, "frames": 53, "bytes": 3_384_000})))
    page.route("**/evaluation", lambda r: r.fulfill(
        status=200, content_type="application/json", body=json.dumps({"score": 0})))
    page.route("**/replay", lambda r: r.fulfill(
        status=504, content_type="application/json", body='{"message":"Gateway Timeout"}'))
    lists = {"n": 0}

    def replays(route):
        lists["n"] += 1
        # The old record for the snapshot and the first polls; the new one
        # only after the work finishes.
        body = {"replays": [new if lists["n"] > 3 else old]}
        route.fulfill(status=200, content_type="application/json", body=json.dumps(body))
    page.route("**/replays", replays)
    page.goto(admin_server + "/admin", wait_until="networkidle")
    page.wait_for_selector(".walk", timeout=10_000)

    page.locator(".replay-select").first.select_option(model + "|default")
    replay_text = page.locator(".walk").first.locator(".walk-eval").nth(1)
    sync_api.expect(replay_text).to_contain_text("score 34", timeout=10_000)
    assert lists["n"] > 3
    page.close()
    context.close()
