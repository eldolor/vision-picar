"""
tests/test_review_376o.py

`PLAN-ros-alignment.md` 3.76, step O: the bugs the review of the twin's page
(`web-twin/index.html`, `web-twin/manifest.json`, whole files) confirmed.
Each test was written before its fix and seen to fail on `dev` for the
reason it names. The real twin in a real browser at a phone viewport, as
`tests/test_ui.py` runs it.
"""

from pathlib import Path

from tests.conftest import REPO_ROOT
from tests.test_ui import (  # noqa: F401  (fixtures)
    PHONE, _open_with_health, browser, open_twin, twin_server)

INDEX = Path(REPO_ROOT) / "web-twin" / "index.html"
APP = Path(REPO_ROOT) / "web-twin" / "app.js"


def _fullscreen(page):
    page.evaluate("document.getElementById('guide-fullscreen').classList.add('active')")


# ---------- O.1: toasts under Guide's fullscreen ----------


def test_a_toast_is_seen_over_guides_fullscreen(browser, twin_server):
    """The toast stack sat at z-index 120 under the fullscreen's 200: a walk
    recorded from Robot view warned "Frame not saved" where nobody could
    see it (both reviewers: `guidance-video` was topmost at its centre)."""
    page, _ = open_twin(browser, twin_server)
    _fullscreen(page)
    page.evaluate("""() => {
      const t = document.createElement('div');
      t.className = 'toast'; t.id = 'probe-toast'; t.textContent = 'Frame not saved: x';
      // Hit-testable for the probe only: the stack never takes a tap, and
      // elementFromPoint skips what takes none.
      t.style.pointerEvents = 'auto';
      document.querySelector('.toast-stack').appendChild(t);
    }""")
    top = page.evaluate("""() => {
      const r = document.getElementById('probe-toast').getBoundingClientRect();
      const el = document.elementFromPoint(r.x + r.width / 2, r.y + r.height / 2);
      return el && (el.id || el.className);
    }""")
    assert top == "probe-toast", top
    page.context.close()


# ---------- O.2: the environment banner overflowed ----------


def test_the_banner_does_not_make_the_page_scroll_sideways(browser, twin_server):
    """`margin: 0 -16px` against `.wrap`'s 14 px: every tab 392 px wide on a
    390 px phone (both reviewers)."""
    page, _ = _open_with_health(browser, twin_server, "Lab")
    for tab in ("guide", "sim"):
        page.click(f'.tab-btn[data-tab="{tab}"]')
        page.wait_for_timeout(100)
        width = page.evaluate("[document.documentElement.scrollWidth, document.documentElement.clientWidth]")
        assert width[0] <= width[1], (tab, width)
    page.context.close()


# ---------- O.3: the secret fields ----------


def test_the_url_and_secret_fields_are_typed_as_they_are_sent(browser, twin_server):
    """Plain text inputs with no hints: iOS capitalises a hand-typed secret's
    first letter and autocorrects it, and the twin sends what was typed --
    a 401 that reads as the wrong secret (both reviewers)."""
    page, _ = open_twin(browser, twin_server)
    for field in ("cfg-server-url", "cfg-server-secret", "cfg-brain-url", "cfg-brain-secret",
                  "cfg-url", "cfg-secret"):
        attrs = page.evaluate(
            "(id) => { const e = document.getElementById(id);"
            " return [e.getAttribute('autocapitalize'), e.getAttribute('autocorrect'), e.spellcheck]; }",
            field)
        assert attrs == ["off", "off", False], (field, attrs)
    page.context.close()


# ---------- O.4: the countdown under reduced motion ----------


def test_the_robot_countdown_runs_its_time_under_reduced_motion(browser, twin_server):
    """The reduced-motion rule's `!important` 0.01 ms beat the inline
    duration, so the "time to the next capture" bar filled at once."""
    context = browser.new_context(viewport=PHONE, reduced_motion="reduce")
    page = context.new_page()
    page.goto(twin_server, wait_until="load")
    # The line app.js runs, on the element it runs it on.
    line = next(l.strip() for l in APP.read_text().splitlines()
                if "robotCountdown.firstElementChild.style" in l)
    page.evaluate("(line) => { const ms = 5000;"
                  " const robotCountdown = document.querySelector('.robot-countdown');"
                  " robotCountdown.classList.add('run'); eval(line); }", line)
    duration = page.evaluate(
        "getComputedStyle(document.querySelector('.robot-countdown span')).animationDuration")
    assert duration == "5s", duration
    context.close()


# ---------- O.5: the action badge sat on the telemetry ----------


def test_the_action_badge_clears_the_telemetry_in_portrait(browser, twin_server):
    """At top 18% the badge covered the SEQ, OUT and VF rows of the Robot
    view's telemetry on a 390 x 844 phone (Thermos)."""
    page, _ = open_twin(browser, twin_server)
    _fullscreen(page)
    page.evaluate("""() => {
      document.getElementById('robot-hud').classList.add('visible');
      const tel = document.querySelector('.robot-telemetry');
      tel.style.display = 'block';
      tel.innerHTML = Array.from({length: 8}, (_, i) => '<div><b>K' + i + '</b>v</div>').join('');
      const b = document.querySelector('.robot-action-badge');
      b.classList.add('visible');
      b.querySelector('.robot-action-verb').textContent = 'FORWARD';
    }""")
    page.wait_for_timeout(300)
    tel = page.locator(".robot-telemetry").bounding_box()
    badge = page.locator(".robot-action-badge").bounding_box()
    assert badge["y"] >= tel["y"] + tel["height"], (tel, badge)
    page.context.close()


# ---------- O.6-O.8: the lows taken ----------


def test_resume_searching_stays_readable_under_hover(browser, twin_server):
    """`button:hover` outranked `.guide-resume-btn`: dark grey behind its
    dark text (Thermos: about 1.3:1)."""
    page, _ = open_twin(browser, twin_server)
    _fullscreen(page)
    page.evaluate("document.getElementById('btn-guidance-resume').classList.add('visible')")
    page.hover("#btn-guidance-resume")
    page.wait_for_timeout(200)
    bg, safe = page.evaluate("""() => [
      getComputedStyle(document.getElementById('btn-guidance-resume')).backgroundColor,
      (() => { const d = document.createElement('div'); d.style.background = 'var(--accent-safe)';
               document.body.appendChild(d); const c = getComputedStyle(d).backgroundColor;
               d.remove(); return c; })()]""")
    assert bg == safe, (bg, safe)
    page.context.close()


def test_each_keyframes_name_is_defined_once():
    """A second `@keyframes conn-pulse` replaced the first, so the Settings
    card's halo never ran (both reviewers)."""
    import re
    names = re.findall(r"@keyframes\s+([\w-]+)", INDEX.read_text())
    assert len(names) == len(set(names)), sorted(n for n in names if names.count(n) > 1)


def test_the_safety_readout_is_set_like_the_other_readouts(browser, twin_server):
    """`#tel-safety` sat outside `.telemetry`, so "Safety: OK" rendered in
    the default 16 px sans (Thermos)."""
    page, _ = open_twin(browser, twin_server)
    page.click('.tab-btn[data-tab="sim"]')
    font = page.evaluate("[getComputedStyle(document.getElementById('tel-safety')).fontFamily,"
                         " getComputedStyle(document.getElementById('tel-safety')).fontSize]")
    assert "mono" in font[0].lower() or "menlo" in font[0].lower() or "courier" in font[0].lower(), font
    assert font[1] == "12px", font
    page.context.close()
