#!/bin/bash
# SessionStart hook: make the test suite and the merge gate runnable in a
# Claude Code cloud session (CLAUDE.md section 1). Cloud sessions only;
# synchronous, so nothing runs before the dependencies are in place.
set -euo pipefail

if [ "${CLAUDE_CODE_REMOTE:-}" != "true" ]; then
  exit 0
fi

cd "$CLAUDE_PROJECT_DIR"

# 1. The venv CLAUDE.md section 1 prescribes ("trust pytest from .venv").
#    pip install is idempotent and benefits from the cached container.
if [ ! -x .venv/bin/python ]; then
  python3 -m venv .venv
fi
.venv/bin/pip install -q --disable-pip-version-check -r requirements.txt

# 2. The UI tests drive a real Chromium. The cloud image preinstalls one
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
  .venv/bin/pip install -q --disable-pip-version-check "playwright==1.56.0"
  if ! can_launch; then
    echo "session-start: no Playwright here can launch the installed Chromium;" \
         "the UI tests will error" >&2
  fi
fi

# 3. The repo's machine setup (docs/guides/PARALLEL-SESSIONS.md): the
#    pre-push gate on dev, and rerere for replayed conflict resolutions.
git config core.hooksPath tools/hooks
git config rerere.enabled true
git config rerere.autoupdate true

# 4. Every command in the session uses the venv.
if [ -n "${CLAUDE_ENV_FILE:-}" ]; then
  echo "export VIRTUAL_ENV=\"$CLAUDE_PROJECT_DIR/.venv\"" >> "$CLAUDE_ENV_FILE"
  echo "export PATH=\"$CLAUDE_PROJECT_DIR/.venv/bin:\$PATH\"" >> "$CLAUDE_ENV_FILE"
fi
