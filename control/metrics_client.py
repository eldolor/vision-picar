"""
metrics_client.py

Ships one row per mission to the metrics service. The brain's half of
`control/metrics_routes.py`.

## Why it cannot fail a mission

A metrics POST happens at the end of a run, over the network, to a
service that may be down or misconfigured. **Nothing here may raise into
the mission loop.** A robot that stopped driving because a dashboard was
unreachable would be a worse robot, and this project already applies that
rule to the odometry read in `MissionRunner._guarded_vision()`.

So every failure is logged and swallowed, and the shipper is off unless
`brain.metrics_url` is set -- which is also what keeps every test and
every laptop run from POSTing anywhere.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Optional

logger = logging.getLogger("metrics")

DEFAULT_TIMEOUT_S = 5.0


def ship_run(url: str, row: dict, *, secret: str = "",
             timeout_s: float = DEFAULT_TIMEOUT_S) -> bool:
    """POST one run row. Returns whether it landed; never raises."""
    if not url:
        return False
    try:
        import httpx

        headers = {"x-app-secret": secret} if secret else {}
        r = httpx.post(url.rstrip("/") + "/metrics/runs", json=row,
                       headers=headers, timeout=timeout_s)
        if r.status_code >= 400:
            logger.warning("metrics rejected (%s): %s", r.status_code,
                           r.text[:200])
            return False
        return True
    except Exception as e:  # noqa: BLE001 -- see the module docstring
        logger.warning("metrics not shipped: %s", e)
        return False


def ship_run_async(url: str, row: dict, **kwargs) -> Optional[threading.Thread]:
    """Same, on a daemon thread, so a slow metrics service cannot hold up
    the end of a mission. Returns the thread for tests to join on."""
    if not url:
        return None
    t = threading.Thread(target=ship_run, args=(url, row), kwargs=kwargs,
                         daemon=True, name="metrics-ship")
    t.start()
    return t


def row_for(status: dict, *, git_revision: str = "", config: dict = None,
            walk: str = None) -> dict:
    """Build the row from a MissionRunner status dict.

    Reads only what the status already publishes. The percentiles come
    pre-computed from `brain/tiered.py` and are stored as they arrive --
    see `control/metrics_routes.py` on why they must never be re-derived
    across runs.
    """
    tier = status.get("tier") or {}
    stats = dict(tier.get("stats") or {})
    return {
        "run_id": f"{int(time.time())}-{status.get('policy') or 'none'}",
        "started_at": status.get("started_at"),
        "finished_at": time.time(),
        "git_revision": git_revision,
        "policy": status.get("policy"),
        "config": dict(config or {}),
        "outcome": status.get("outcome"),
        "steps": status.get("step"),
        "target_object": status.get("target_object"),
        "walk": walk,
        "stats": stats,
    }
