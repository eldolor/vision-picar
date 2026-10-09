#!/bin/bash
# SessionStart hook: make the test suite and the merge gate runnable in a
# Claude Code cloud session (CLAUDE.md section 1). Cloud sessions only;
# synchronous, so nothing runs before the dependencies are in place.
set -euo pipefail

if [ "${CLAUDE_CODE_REMOTE:-}" != "true" ]; then
  exit 0
fi

cd "$CLAUDE_PROJECT_DIR"

# Nothing below may stop the steps after it: a failed pip install (PyPI
# unreachable through the proxy) must not also lose the push gate or the
# venv on PATH. Each step warns and carries on.
warn() { echo "session-start: $*" >&2; }

# 1. The repo's machine setup (docs/guides/PARALLEL-SESSIONS.md): the
#    pre-push gate on dev, and rerere for replayed conflict resolutions.
#    First, because it needs nothing from the network.
git config core.hooksPath tools/hooks || warn "could not set core.hooksPath"
git config rerere.enabled true || true
git config rerere.autoupdate true || true

# 2. The venv CLAUDE.md section 1 prescribes ("trust pytest from .venv"),
#    and every command in the session uses it -- only once it exists.
if [ ! -x .venv/bin/python ]; then
  python3 -m venv .venv || { warn "could not create .venv"; exit 0; }
fi
if [ -n "${CLAUDE_ENV_FILE:-}" ]; then
  echo "export VIRTUAL_ENV=\"$CLAUDE_PROJECT_DIR/.venv\"" >> "$CLAUDE_ENV_FILE"
  echo "export PATH=\"$CLAUDE_PROJECT_DIR/.venv/bin:\$PATH\"" >> "$CLAUDE_ENV_FILE"
fi

# 3. The requirements. pip install is idempotent and benefits from the
#    cached container. (The hook runs on startup, resume and clear, not
#    on every compaction: .claude/settings.json's matcher.)
.venv/bin/pip install -q --disable-pip-version-check -r requirements.txt \
  || warn "pip install -r requirements.txt failed; tests may not run"

# 4. The UI tests drive a real Chromium. The cloud image preinstalls one
#    under $PLAYWRIGHT_BROWSERS_PATH and forbids `playwright install`.
#    requirements.txt leaves Playwright unpinned (pytest-playwright pulls
#    the newest), and the newest may expect a newer browser build -- then
#    all ~133 UI tests error and the merge gate can never pass. Keep what
#    pip chose unless it cannot launch the installed browser; only then
#    fall back to the release that matches it (chromium-1194 -> playwright
#    1.56.0). requirements.txt is never changed, and a later
#    `pip install -r` keeps the fallback, since nothing pins a newer one.
can_launch() {
  .venv/bin/python - <<'PY' >/dev/null 2>&1
from playwright.sync_api import sync_playwright
with sync_playwright() as p:
    p.chromium.launch().close()
PY
}
if ! can_launch; then
  .venv/bin/pip install -q --disable-pip-version-check "playwright==1.56.0" \
    || warn "could not install playwright 1.56.0"
  can_launch || warn "no Playwright here can launch the installed Chromium; the UI tests will error"
fi
exit 0
