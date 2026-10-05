"""
metrics_routes.py

The two routes that store and read back tier metrics, mounted by the
walks service (`control/admin_server.py`, and therefore the walks Lambda).

## Why here and not on the brain

Same argument `control/recording_routes.py` already makes for the write
half of a walk: **the write path follows the storage.** Metrics live in
the recordings bucket, the walks Lambda is the process with credentials
for it, and a robot on someone's floor should not carry AWS keys in order
to file a latency sample. The brain POSTs; this stores.

## Why S3 and not a database

A metrics row is append-only, written once at the end of a mission, and
read back a few times a day by one dashboard. That is the cheapest thing
S3 does and the most expensive thing a table does for no return --
`PLAN-aws-cost-redesign.md` is the standing reason to prefer the former.

One object per run, in a **per-day container**:

    metrics-2026-09-13/<run-id>.json

`WalkStore` refuses a slash in a container name -- a path-traversal guard
worth keeping -- so the day is part of the name rather than a sub-prefix.
The effect is the same and it is the whole query plan. "Show me the last 14 days" is 14
LIST calls and their objects, not a scan of everything ever written, and
it stays that way as the corpus grows.

## What a row is, and what it is NOT

A row is one MISSION's summary: counts, trigger mix, and latency
percentiles for both tiers. It carries the git revision and the config
that produced it, because the entire point is watching a number move as
optimisations land, and a number whose configuration is unknown cannot be
compared with the next one.

It is **not** per-frame data. `walk.jsonl` already holds that and is the
right place for it; duplicating it here would make the cheap query
expensive for no new fact.

**Percentiles arrive pre-computed and are never re-derived across runs.**
Averaging p90s from two missions does not give the p90 of the pair, and a
dashboard that did it would be quietly wrong in the direction that
flatters. Cross-run summaries here report a RANGE and the per-run values,
never a blended percentile.
"""

from __future__ import annotations

import json
import re
import time
from datetime import datetime, timedelta, timezone
from typing import Optional

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel, Field

from control.walk_store import METRICS_PREFIX  # one definition, beside the filter
RUN_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,120}$")
MAX_DAYS = 90


class MetricsRun(BaseModel):
    """One mission's summary. Extra keys are REFUSED rather than dropped:
    a field the server silently ignores is how a dashboard ends up drawing
    a metric nobody is storing."""

    model_config = {"extra": "forbid"}

    run_id: str
    started_at: Optional[float] = None
    finished_at: Optional[float] = None
    # What produced this row. Without these a trend line is a set of
    # numbers with no cause attached.
    git_revision: Optional[str] = None
    policy: Optional[str] = None
    config: dict = Field(default_factory=dict)
    # The mission's own outcome, so a latency improvement that came with a
    # collapse in outcomes cannot read as a win.
    outcome: Optional[str] = None
    steps: Optional[int] = None
    target_object: Optional[str] = None
    walk: Optional[str] = None
    # The tier counters and the two latency distributions.
    stats: dict = Field(default_factory=dict)


def _day_of(ts: Optional[float]) -> str:
    when = datetime.fromtimestamp(ts or time.time(), tz=timezone.utc)
    return when.strftime("%Y-%m-%d")


def register_metrics_routes(app, store, require_secret) -> APIRouter:
    """Mount POST /metrics/runs and GET /metrics/summary on `app`.

    `store` is a `control/walk_store.py` backend -- the same abstraction
    the walks themselves use, so this works against a local directory in
    development and a bucket in Lambda with no branch here.
    """
    router = APIRouter()

    @router.post("/metrics/runs", dependencies=[Depends(require_secret)])
    async def put_run(run: MetricsRun):
        if not RUN_ID_RE.match(run.run_id):
            raise HTTPException(400, "run_id must be a short safe name")
        row = run.model_dump()
        row["stored_at"] = time.time()
        day = _day_of(run.finished_at or run.started_at)
        store.write_json(f"{METRICS_PREFIX}{day}", f"{run.run_id}.json", row)
        return {"stored": run.run_id, "day": day}

    @router.get("/metrics/summary", dependencies=[Depends(require_secret)])
    async def summary(days: int = 14):
        """Every run in the last `days` days, newest first.

        Rows are returned whole rather than aggregated. The dashboard does
        the drawing, and this refuses to blend percentiles across runs --
        see the module docstring for why that would be wrong rather than
        merely approximate.
        """
        days = max(1, min(MAX_DAYS, int(days)))
        today = datetime.now(tz=timezone.utc)
        runs = []
        for back in range(days):
            day = (today - timedelta(days=back)).strftime("%Y-%m-%d")
            container = f"{METRICS_PREFIX}{day}"
            try:
                names = store.list_names(container, prefix="")
            except Exception:
                # A day with no runs. Not an error -- most days have none.
                continue
            for name in names:
                if not name.endswith(".json"):
                    continue
                row = store.read_json(container, name)
                if row:
                    row.setdefault("day", day)
                    runs.append(row)
        runs.sort(key=lambda r: r.get("finished_at") or r.get("stored_at") or 0,
                  reverse=True)
        return {"days": days, "runs": runs, "count": len(runs)}

    app.include_router(router)
    return router
