"""
tests/test_metrics.py

The metrics pipeline: the routes that store a run, the client that ships
one, and the rule both of them exist to protect.

**That rule is the reason most of these tests are here.** A percentile
belongs to the run that produced it. Averaging p90s across runs does not
give the p90 of the pair, and a dashboard that did it would be quietly
wrong in the direction that flatters. Nothing in the pipeline blends.
"""

import json
import tempfile
import time

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from brain.tiered import percentiles
from control.metrics_client import row_for, ship_run
from control.metrics_routes import register_metrics_routes
from control.walk_store import LocalWalkStore


@pytest.fixture
def client():
    app = FastAPI()
    store = LocalWalkStore(tempfile.mkdtemp())
    register_metrics_routes(app, store, lambda: None)
    return TestClient(app), store


def a_run(**over):
    row = {"run_id": "r1", "finished_at": time.time(), "policy": "tiered",
           "outcome": "found", "steps": 40, "git_revision": "abc1234",
           "config": {"tier_cold_search_after": 6},
           "stats": {"frames": 40, "cloud_calls": 8,
                     "cloud_ms": {"n": 8, "p50": 3000, "p90": 3300,
                                  "p99": 3800, "max": 6000, "mean": 3100},
                     "perception_ms": {"n": 40, "p50": 850, "p90": 900,
                                       "p99": 950, "max": 1000, "mean": 860},
                     "perception": {"detected": 10, "absent": 30}}}
    row.update(over)
    return row


# ---------- percentiles ----------


def test_no_samples_is_None_rather_than_a_row_of_zeroes():
    """"No calls were made" and "every call took 0ms" are different facts,
    and a dashboard drawing the second when the first is true is the
    stale-readout failure this project keeps meeting."""
    assert percentiles([]) is None


def test_every_percentile_carries_its_sample_count():
    p = percentiles([1, 2, 3])
    assert p["n"] == 3


def test_percentiles_do_not_interpolate_past_the_data():
    """p99 of three samples is the largest of three, not a curve fit."""
    p = percentiles([10, 20, 30])
    assert p["p99"] == 30 and p["max"] == 30


# ---------- storage ----------


def test_a_run_is_stored_under_its_own_day(client):
    c, store = client
    r = c.post("/metrics/runs", json=a_run())
    assert r.status_code == 200
    day = r.json()["day"]
    assert store.read_json("metrics-" + day, "r1.json")["outcome"] == "found"


def test_an_unknown_field_is_REFUSED_not_dropped(client):
    """A field the server silently ignores is how a dashboard ends up
    drawing a metric nobody is storing."""
    c, _ = client
    assert c.post("/metrics/runs", json=a_run(bogus=1)).status_code == 422


def test_a_dangerous_run_id_is_refused(client):
    c, _ = client
    assert c.post("/metrics/runs", json=a_run(run_id="../../etc/passwd")).status_code in (400, 422)


def test_a_day_with_no_runs_is_not_an_error(client):
    """Most days have none. A summary that 500s on an empty prefix would
    be broken for the common case."""
    c, _ = client
    r = c.get("/metrics/summary?days=30")
    assert r.status_code == 200 and r.json()["count"] == 0


def test_the_summary_returns_runs_whole_and_newest_first(client):
    c, _ = client
    now = time.time()
    c.post("/metrics/runs", json=a_run(run_id="old", finished_at=now - 60))
    c.post("/metrics/runs", json=a_run(run_id="new", finished_at=now))
    runs = c.get("/metrics/summary?days=2").json()["runs"]
    assert [r["run_id"] for r in runs] == ["new", "old"]
    # Whole: the percentiles arrive as stored, un-aggregated.
    assert runs[0]["stats"]["cloud_ms"]["p90"] == 3300


def test_the_summary_never_blends_percentiles_across_runs(client):
    """The rule the whole pipeline exists to protect. Two runs with
    different p90s must come back as two p90s -- the service must not
    offer a combined one, because there is no correct way to compute it
    from what it stores."""
    c, _ = client
    c.post("/metrics/runs", json=a_run(run_id="a"))
    slow = a_run(run_id="b")
    slow["stats"]["cloud_ms"] = {"n": 4, "p50": 9000, "p90": 9500,
                                 "p99": 9900, "max": 10000, "mean": 9200}
    c.post("/metrics/runs", json=slow)
    body = c.get("/metrics/summary?days=2").json()
    assert sorted(r["stats"]["cloud_ms"]["p90"] for r in body["runs"]) == [3300, 9500]
    assert "p90" not in body, "the service offered a blended percentile"


def test_the_day_window_is_bounded(client):
    c, _ = client
    assert c.get("/metrics/summary?days=100000").json()["days"] <= 90


# ---------- the client ----------


def test_shipping_is_off_without_a_url():
    """What keeps every test and every laptop run from POSTing anywhere."""
    assert ship_run("", {"run_id": "x"}) is False


def test_a_dead_metrics_service_can_never_fail_a_mission():
    """A robot that stopped driving because a dashboard was unreachable
    would be a worse robot."""
    assert ship_run("http://127.0.0.1:9", {"run_id": "x"}, timeout_s=0.2) is False


def test_the_row_carries_the_config_that_produced_it():
    """A trend line without the settings behind it is a set of numbers
    with no cause attached."""
    row = row_for({"policy": "tiered", "outcome": "found", "step": 12,
                   "tier": {"stats": {"frames": 12}}},
                  git_revision="deadbee", config={"tier_async_cloud": True})
    assert row["config"]["tier_async_cloud"] is True
    assert row["git_revision"] == "deadbee"
    assert row["stats"]["frames"] == 12


def test_the_row_carries_the_OUTCOME_beside_the_latency():
    """So a latency improvement that came with a collapse in outcomes
    cannot read as a win."""
    row = row_for({"outcome": "failed", "policy": "tiered", "tier": {}})
    assert row["outcome"] == "failed"
