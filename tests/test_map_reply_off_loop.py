"""3.87: `/world/map`'s reply is encoded off the event loop.

The wheel loop (20 Hz) and the watchdog run on the robot server's event
loop. FastAPI encodes a sync route's plain return value there
(`jsonable_encoder`), which for a map of ~10^5 cells took 131-218 ms on the
Jetson -- a late wheel tick on every explore tick (3.80.H, 3.87).
"""
import asyncio
import time

from fastapi import FastAPI
from fastapi.testclient import TestClient

import robot.server as server

CELLS = 300_000          # ~120 ms of jsonable_encoder on the laptop
PERIOD_S = 0.05          # the wheel loop's period (WHEEL_LOOP_INTERVAL_S)


class BigMapWorld:
    """Only the map is big; every other world answer is "no world"."""

    def __init__(self):
        self.map = {"usable": True, "map_id": "big", "map_version": 7,
                    "resolution_m": 0.05, "width": 600, "height": 500,
                    "origin_x_m": -1.25, "origin_y_m": 0.5,
                    "cells": [i % 3 - 1 for i in range(CELLS)]}

    def get_map(self):
        return self.map

    def __getattr__(self, name):          # get_pose & co: not under test
        from world.interface import NullWorld
        return getattr(NullWorld(), name)


def _client(monkeypatch):
    world = BigMapWorld()
    monkeypatch.setattr(server, "get_world", lambda *a, **k: world)
    return TestClient(server.create_app()), world


def test_a_big_map_reply_never_stalls_the_event_loop(monkeypatch):
    client, _ = _client(monkeypatch)
    with client:
        gaps = []

        done = []

        async def probe():
            last = time.monotonic()
            while not done:
                await asyncio.sleep(0.005)
                now = time.monotonic()
                gaps.append(now - last)
                last = now

        client.get("/world/map")            # warm: imports, first-call costs
        fut = client.portal.start_task_soon(probe)
        time.sleep(0.05)                     # the probe is ticking first
        for _ in range(3):
            assert client.get("/world/map").status_code == 200
        done.append(True)                    # the probe spans every request
        fut.result()
    assert gaps and max(gaps) < PERIOD_S, f"event loop stalled {max(gaps) * 1000:.0f} ms"


def test_the_map_reply_is_the_json_fastapi_would_have_sent(monkeypatch):
    client, world = _client(monkeypatch)
    plain = FastAPI()                        # FastAPI's own default encoding

    @plain.get("/m")
    def m():
        return world.map

    with client:
        r = client.get("/world/map")
    want = TestClient(plain).get("/m")
    assert r.headers["content-type"] == want.headers["content-type"]
    assert r.content == want.content
