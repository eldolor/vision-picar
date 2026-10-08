"""
identity.py

One line, logged at start-up by every service here, at warning level --
phase M5 (PLAN-microduck-transplants.md).

Microduck's updater design keeps one because three of its documented
failures are unanswerable without it: a board that silently came back on a
release nobody asked for, an update killed before its health check ran,
and a journal that was empty after a power cut. The question all three
reduce to is "which build is actually running, from where", and a process
that never says so cannot be asked afterwards.

**The executable path is the load-bearing field**, and the least obvious
one. M11 installs releases into `releases/<git-rev>/` under a `current`
symlink; after a rollback the git revision may be exactly what you
expected while the symlink still points at the build you were trying to
leave. The path is what separates "the update worked" from "the symlink
moved" -- and it is free to log and impossible to reconstruct later.

Warning level on purpose. A service's first line has to survive whatever
log level someone left the deployment on, and this is the line you go
looking for when nothing else makes sense.

**Why this lives in `robot/` and not `control/`.** Both servers log it, and
`robot/server.py` must never import `control/` -- that is the layering
inversion the whole brain/robot split exists to prevent. `robot/` is
already the lower half that `control/` depends on (`interface.py`,
`safety.py`), so the shared thing goes here even though nothing about it is
robot-specific.
"""

import logging
import os
import subprocess
import sys
from pathlib import Path
from typing import Optional

logger = logging.getLogger("identity")

# Set at image build time -- there is no .git in a container, so the
# subprocess fallback below only ever answers on a developer's machine.
GIT_REVISION_ENV = "GIT_REVISION"


def git_revision() -> str:
    revision = os.environ.get(GIT_REVISION_ENV, "").strip()
    if revision:
        return revision
    try:
        out = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            capture_output=True, text=True, timeout=2,
            cwd=Path(__file__).resolve().parent.parent,
        )
        if out.returncode == 0 and out.stdout.strip():
            return out.stdout.strip()
    except Exception:  # noqa: BLE001 -- no git, no repo, no shell: all fine
        pass
    return "unknown"


def identity(service: str, config_path: Optional[str] = None) -> dict:
    return {
        "service": service,
        "git_revision": git_revision(),
        # sys.executable rather than argv[0]: under `python -m uvicorn` the
        # latter is uvicorn's own path, which says nothing about which
        # release directory this interpreter was started from.
        "executable": sys.executable,
        "config_path": str(config_path) if config_path else "(default)",
    }


# PLAN 3.45: the brain's decisions are logged at INFO, and with no handler
# configured Python prints only WARNING and above -- so a live run's brain
# log held nothing but this module's line. Unset, logging is left exactly
# as found (deployments and the suite); `tests/demo_explore.stack()` sets
# it to INFO for every live run.
LOG_LEVEL_ENV = "PICAR_LOG_LEVEL"
_HANDLER_MARK = "_picar_handler"


def configure_logging() -> Optional[int]:
    """One stderr handler on the root logger at `PICAR_LOG_LEVEL`, with
    epoch timestamps so a log lines up with a run's 1 Hz series. Returns
    the level set, or None when the variable is unset. Idempotent: a test
    that builds many apps still gets one handler."""
    name = os.environ.get(LOG_LEVEL_ENV, "").strip().upper()
    if not name:
        return None
    level = logging.getLevelName(name)
    if not isinstance(level, int):
        raise ValueError(f"{LOG_LEVEL_ENV}={name!r} is not a logging level")
    root = logging.getLogger()
    if not any(getattr(h, _HANDLER_MARK, False) for h in root.handlers):
        handler = logging.StreamHandler()
        handler.setFormatter(logging.Formatter("%(created).3f %(levelname)s %(name)s %(message)s"))
        setattr(handler, _HANDLER_MARK, True)
        root.addHandler(handler)
    root.setLevel(level)
    # Every HTTP call the brain makes is an httpx INFO line: 12 800 of a
    # run's 13 000 (3.45's baseline). Its warnings still come through.
    logging.getLogger("httpx").setLevel(max(level, logging.WARNING))
    return level


def log_identity(service: str, config_path: Optional[str] = None) -> dict:
    ident = identity(service, config_path)
    logger.warning(
        "%s starting -- git=%s executable=%s config=%s",
        ident["service"], ident["git_revision"], ident["executable"], ident["config_path"],
    )
    return ident
