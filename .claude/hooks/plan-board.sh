#!/bin/bash
# SessionStart hook (local sessions): put the plan's step board into the
# session's context, so it claims a step before working
# (docs/guides/PARALLEL-SESSIONS.md, rule 8). Cloud sessions share no .git
# with the others, so the board would mean nothing there.
[ "${CLAUDE_CODE_REMOTE:-}" = "true" ] && exit 0
cd "$CLAUDE_PROJECT_DIR" 2>/dev/null || exit 0
[ -f tools/plan_claims.py ] && python3 tools/plan_claims.py hook-start 2>/dev/null
exit 0
