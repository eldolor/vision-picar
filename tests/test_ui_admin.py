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
                 viewport=DESKTOP, models_delay_ms=0):
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
