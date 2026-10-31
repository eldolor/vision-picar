"""3.126 criterion 2: the wall time of an unchanged RosWorld.get_map()
against the bridge's real HTTP handler (stubbed ROS), a 400 x 400 grid.
Run from the checkout under test (dev or the branch):
    python -m pytest evaluations/bridge-map-3126/bench_test.py -s -q
It imports that checkout's bridge and RosWorld, so the same file times both."""

import json
import random
import statistics
import threading
import time
import types
from http.server import ThreadingHTTPServer

from tests.test_review_376h1 import _bridge, _Msg, bridge_mod  # noqa: F401
from tests.test_ros_world import FakeTruth
from world.ros_world import RosWorld

N, READS = 400, 200


def test_bench(bridge_mod):
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
    rng = random.Random(3126)
    cells = [rng.choice([-1, -1, 0, 0, 0, 100]) for _ in range(N * N)]
    b._on_map(_Msg(header=_Msg(stamp=_Msg(sec=100, nanosec=0)),
                   info=_Msg(resolution=0.05, width=N, height=N,
                             origin=_Msg(position=_Msg(x=-10.0, y=-10.0),
                                         orientation=_Msg(x=0.0, y=0.0, z=0.0, w=1.0))),
                   data=cells))
    server = ThreadingHTTPServer(("127.0.0.1", 0), bridge_mod._handler(b))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        world = RosWorld(f"http://127.0.0.1:{server.server_address[1]}", truth=FakeTruth(0.0, 0.0, 0.0))
        first = world.get_map()
        assert first["usable"]
        times = []
        for _ in range(READS):
            t = time.perf_counter()
            m = world.get_map()
            times.append(time.perf_counter() - t)
            assert m is first or m == first
        times.sort()
        out = {"grid": N, "reads": READS, "median_ms": round(statistics.median(times) * 1e3, 3),
               "p90_ms": round(times[int(0.9 * READS)] * 1e3, 3),
               "map_replies": b.health()[1].get("map_replies")}
        print("BENCH", json.dumps(out))
    finally:
        server.shutdown()
        server.server_close()
