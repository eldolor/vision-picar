"""3.123 criterion 1: `/world/map` is sent only when it has changed, and is
never stale -- a 304 only for the same `map_id` and `map_version`, a changed
or unusable map always in full, the same bytes as before for a client that
sends no tag, and `RemoteWorld` returning the server's current map."""
import json

from fastapi.testclient import TestClient
from starlette.responses import JSONResponse

import robot.server as server
from control.remote_world import RemoteWorld
from world.interface import unusable_map


class SwitchWorld:
    """A world whose map a test sets, counting the reads."""

    def __init__(self):
        self.map = self.make("house", 1)
        self.reads = 0

    @staticmethod
    def make(map_id, version, cells=(0, 1, -1, 0)):
        return {"usable": True, "map_id": map_id, "map_version": version,
                "resolution_m": 0.05, "width": 2, "height": 2,
                "origin_x_m": 0.0, "origin_y_m": 0.0, "cells": list(cells)}

    def get_map(self):
        self.reads += 1
        return self.map

    def __getattr__(self, name):          # get_pose & co: not under test
        from world.interface import NullWorld
        return getattr(NullWorld(), name)


def _setup(monkeypatch):
    world = SwitchWorld()
    monkeypatch.setattr(server, "get_world", lambda *a, **k: world)
    client = TestClient(server.create_app(), base_url="http://robot.test")
    return client, world, RemoteWorld("http://robot.test", client=client)


def test_a_plain_fetch_gets_the_same_bytes_and_a_tag(monkeypatch):
    client, world, _ = _setup(monkeypatch)
    r = client.get("/world/map")
    assert r.status_code == 200
    assert r.content == JSONResponse(world.map).body
    assert r.headers["etag"] == '"house:1"'
    assert r.headers["cache-control"] == "no-cache"
    r2 = client.get("/world/map")               # a client sending no tag: full again
    assert r2.status_code == 200 and r2.content == r.content


def test_a_304_only_for_the_same_tag(monkeypatch):
    client, world, _ = _setup(monkeypatch)
    tag = client.get("/world/map").headers["etag"]
    for header in (tag, f"W/{tag}", f'"other:1", {tag}'):
        r = client.get("/world/map", headers={"if-none-match": header})
        assert r.status_code == 304 and r.content == b"", header
        assert r.headers["etag"] == tag
    for header in ('"house:2"', '"other:1"', "*", ""):
        r = client.get("/world/map", headers={"if-none-match": header})
        assert r.status_code == 200 and json.loads(r.content) == world.map, header
    assert world.reads == 8                      # the world is asked every time


def test_remote_world_returns_the_current_map_and_never_a_stale_one(monkeypatch):
    client, world, remote = _setup(monkeypatch)
    first = remote.get_map()
    assert first == world.map
    assert remote.get_map() is first             # a 304: the same map, not decoded again
    # A new version, a new map (another house or SLAM session), a map gone
    # unusable, then usable again: each comes back in full, the current one.
    for m in (SwitchWorld.make("house", 2, (1, 1, 1, 1)),
              SwitchWorld.make("other", 2, (0, 0, 0, 0)),
              {**unusable_map(), "retryable": True},
              SwitchWorld.make("other", 2, (0, 0, 0, 0))):
        world.map = m
        got = remote.get_map()
        assert got == m and got is not first
        first = got
    assert world.reads == 6


def test_an_unusable_or_unnamed_map_is_never_tagged(monkeypatch):
    client, world, _ = _setup(monkeypatch)
    for m in ({**unusable_map()},
              {**SwitchWorld.make("house", 1), "map_version": True},
              {**SwitchWorld.make("house", 1), "map_id": None},
              {**SwitchWorld.make('ho"use', 1)}):
        world.map = m
        r = client.get("/world/map", headers={"if-none-match": '"house:1"'})
        assert r.status_code == 200 and "etag" not in r.headers
        assert r.content == JSONResponse(m).body


def test_the_kept_body_follows_the_tag(monkeypatch):
    """The encoded body is kept for its tag only: a new version is encoded
    afresh even with the same size, so a plain fetch never gets old cells."""
    client, world, _ = _setup(monkeypatch)
    client.get("/world/map")
    world.map = SwitchWorld.make("house", 2, (-1, -1, -1, -1))
    assert json.loads(client.get("/world/map").content)["cells"] == [-1, -1, -1, -1]


def test_the_world_contract_over_remote_world_twice(world_over_asgi):
    """The simulator's world through the real app: the second read (a 304
    when nothing moved) answers the same contract."""
    a = world_over_asgi.get_map()
    b = world_over_asgi.get_map()
    assert a["usable"] and b["usable"] and b["map_version"] >= a["map_version"]
    assert len(b["cells"]) == b["width"] * b["height"]
