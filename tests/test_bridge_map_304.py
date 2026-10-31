"""
tests/test_bridge_map_304.py

3.126: the ROS bridge answers 304 for an unchanged map. Since 3.123 the
brain's /world/map read gets a 304, but the robot server's RosWorld still
fetched the whole /slam/map from the bridge on every read, encoded there and
decoded on the robot server's threadpool. Now the bridge tags a map reply
"<session>:<version>" (3.124: versions move only with the grid) and RosWorld
revalidates.
"""

import sys
import threading
import time
import types
from http.server import ThreadingHTTPServer
from pathlib import Path

import httpx
import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "service/slam/src/picar_bridge"))
from picar_bridge import mapversion  # noqa: E402
from tests.test_review_376h1 import _bridge, _Msg, bridge_mod  # noqa: E402,F401
from tests.test_ros_world import FakeTruth  # noqa: E402
from world.ros_world import RosWorld  # noqa: E402


def _map_msg(cells, sec=100):
    return _Msg(header=_Msg(stamp=_Msg(sec=sec, nanosec=0)),
                info=_Msg(resolution=0.05, width=len(cells), height=1,
                          origin=_Msg(position=_Msg(x=0.0, y=0.0),
                                      orientation=_Msg(x=0.0, y=0.0, z=0.0, w=1.0))),
                data=list(cells))


# ---------- the tag helpers ----------

def test_the_tag_names_session_and_version_and_nothing_unusable():
    assert mapversion.map_tag({"session": "ab", "version": 3, "data": [0]}) == '"ab:3"'
    assert mapversion.map_tag(None) is None
    assert mapversion.map_tag({"session": "ab", "version": 0, "usable": False}) is None


@pytest.mark.parametrize("header,hit", [
    ('"ab:3"', True), ('W/"ab:3"', True), ('"x:1", "ab:3"', True),
    ('"ab:4"', False), ('"ab:3', False), ("*", False), ("", False), (None, False)])
def test_if_none_match(header, hit):
    assert mapversion.not_modified(header, '"ab:3"') is hit


# ---------- the bridge's handler, over real HTTP ----------

@pytest.fixture
def served(bridge_mod):
    b = _bridge(bridge_mod)
    b.start_truth, b.start_truth_pending = {"x_m": 0.0, "y_m": 0.0, "heading_deg": 0.0}, False

    def lookup(*a, **kw):
        now = time.time()
        tr = _Msg()
        tr.header.stamp = _Msg(sec=int(now), nanosec=int((now % 1) * 1e9))
        tr.transform.translation = _Msg(x=0.0, y=0.0, z=0.0)
        tr.transform.rotation = _Msg(x=0.0, y=0.0, z=0.0, w=1.0)
        return tr

    b.tf_buffer = types.SimpleNamespace(lookup_transform=lookup)
    server = ThreadingHTTPServer(("127.0.0.1", 0), bridge_mod._handler(b))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        yield b, f"http://127.0.0.1:{server.server_address[1]}"
    finally:
        server.shutdown()
        server.server_close()


def test_the_bridge_answers_304_only_for_the_current_tag(served):
    b, url = served
    b._on_map(_map_msg([0, -1, 100]))
    r = httpx.get(url + "/slam/map")
    tag = r.headers["etag"]
    assert r.status_code == 200 and r.json()["data"] == [0, -1, 100] and tag == f'"{b.session}:1"'
    r = httpx.get(url + "/slam/map", headers={"if-none-match": tag})
    assert r.status_code == 304 and r.content == b"" and r.headers["etag"] == tag
    b._on_map(_map_msg([0, -1, 100]))                    # a repeat: still version 1
    assert httpx.get(url + "/slam/map", headers={"if-none-match": tag}).status_code == 304
    b._on_map(_map_msg([0, 0, 100]))                     # changed: full, a new tag
    r = httpx.get(url + "/slam/map", headers={"if-none-match": tag})
    assert r.status_code == 200 and r.json()["data"] == [0, 0, 100] and r.headers["etag"] != tag
    assert b.health()[1]["map_replies"] == {"full": 2, "not_modified": 2}


def test_the_bridge_never_answers_304_without_its_map(served):
    b, url = served
    b._on_map(_map_msg([0, 0]))
    tag = httpx.get(url + "/slam/map").headers["etag"]
    b.slam_state = "checking"                            # a reload being judged
    r = httpx.get(url + "/slam/map", headers={"if-none-match": tag})
    assert r.status_code == 200 and r.json()["usable"] is False and "etag" not in r.headers
    b.slam_state = "ready"
    with b.lock:
        b.last_map = None                                # cleared by a reload or re-map
    r = httpx.get(url + "/slam/map", headers={"if-none-match": tag})
    assert r.status_code == 200 and r.json()["usable"] is False and "etag" not in r.headers
    b._on_map(_map_msg([0, 0]))                          # the same grid, a new version
    r = httpx.get(url + "/slam/map", headers={"if-none-match": tag})
    assert r.status_code == 200 and r.headers["etag"] == f'"{b.session}:2"'


def test_ros_world_through_the_real_handler(served):
    b, url = served
    world = RosWorld(url, truth=FakeTruth(0.0, 0.0, 0.0))
    b._on_map(_map_msg([0, -1, 100, 0]))
    first = world.get_map()
    assert first["usable"] and first["map_version"] == 1
    again = world.get_map()
    assert again == first
    assert b.health()[1]["map_replies"] == {"full": 1, "not_modified": 1}
    b._on_map(_map_msg([100, -1, 100, 0]))
    changed = world.get_map()
    assert changed["map_version"] == 2 and changed["cells"] != first["cells"]
    assert b.health()[1]["map_replies"] == {"full": 2, "not_modified": 1}


# ---------- RosWorld against a scripted bridge ----------

class Scripted:
    """/slam/pose answers a fixed anchorable pose; /slam/map answers the
    next scripted (status, etag, body) and records each If-None-Match."""

    MAP = {"session": "s1", "version": 1, "resolution_m": 0.05, "width": 2, "height": 1,
           "origin_x_m": 0.0, "origin_y_m": 0.0, "origin_yaw_rad": 0.0, "data": [0, 100],
           "at": 1.0}

    def __init__(self, replies):
        self.replies, self.sent = list(replies), []

    def handler(self, request):
        if request.url.path == "/slam/pose":
            return httpx.Response(200, json={
                "session": "s1", "start_truth": {"x_m": 0.0, "y_m": 0.0, "heading_deg": 0.0},
                "start_truth_pending": False, "at": time.time(),
                "map": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
                "odom": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0}})
        self.sent.append(request.headers.get("if-none-match"))
        status, etag, body = self.replies.pop(0)
        headers = {"etag": etag} if etag else {}
        if status == 304:
            return httpx.Response(304, headers=headers)
        return httpx.Response(status, json=body, headers=headers)


def _world(bridge):
    client = httpx.Client(base_url="http://bridge", transport=httpx.MockTransport(bridge.handler))
    return RosWorld("http://bridge", truth=FakeTruth(0.0, 0.0, 0.0), client=client)


def test_ros_world_sends_its_tag_and_reuses_the_map_on_a_304():
    m2 = {**Scripted.MAP, "version": 2, "data": [100, 100]}
    bridge = Scripted([(200, '"s1:1"', Scripted.MAP), (304, '"s1:1"', None),
                       (200, '"s1:2"', m2), (304, '"s1:2"', None)])
    world = _world(bridge)
    a, b, c, d = (world.get_map() for _ in range(4))
    assert a == b and a["usable"] and c == d and c["map_version"] == 2 and c != a
    assert bridge.sent == [None, '"s1:1"', '"s1:1"', '"s1:2"']


def test_a_304_naming_another_tag_is_followed_by_one_full_read():
    m2 = {**Scripted.MAP, "version": 2, "data": [100, 100]}
    bridge = Scripted([(200, '"s1:1"', Scripted.MAP), (304, '"s1:9"', None),
                       (200, '"s1:2"', m2)])
    world = _world(bridge)
    world.get_map()
    got = world.get_map()
    assert got["usable"] and got["map_version"] == 2
    assert bridge.sent == [None, '"s1:1"', None]


def test_a_304_to_an_untagged_request_is_a_bridge_error():
    bridge = Scripted([(304, '"s1:1"', None)])
    got = _world(bridge).get_map()
    assert got["usable"] is False and got["bridge_status"] == 304 and got["retryable"] is False
    bridge = Scripted([(200, '"s1:1"', Scripted.MAP), (304, '"s1:9"', None), (304, '"s1:9"', None)])
    world = _world(bridge)
    world.get_map()
    got = world.get_map()
    assert got["usable"] is False and got["bridge_status"] == 304


def test_an_untagged_reply_is_not_kept():
    """An old bridge (no ETag) is read in full every time, as before."""
    bridge = Scripted([(200, None, Scripted.MAP), (200, None, Scripted.MAP)])
    world = _world(bridge)
    assert world.get_map() == world.get_map()
    assert bridge.sent == [None, None]
