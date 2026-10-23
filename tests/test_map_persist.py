"""3.50.D: the saved map -- the store's versions, and the decisions the
launch files pin (criteria 12-13).

The store is plain Python (`picar_bridge/mapstore.py`) so it is tested here
without ROS; the bridge only hands it the files slam_toolbox wrote.
"""
import json
import os
import sys

import pytest
import yaml

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "service/slam/src/picar_bridge"))

from picar_bridge import mapstore, relocalise, slam_process  # noqa: E402
from picar_bridge.mapstore import MapStore  # noqa: E402

BRINGUP = os.path.join(ROOT, "service/slam/src/picar_bringup")


def _fake_serialise(prefix, size=10):
    """What slam_toolbox's serialize_map writes for a filename prefix."""
    for ext in mapstore.GRAPH_FILES:
        with open(prefix + ext, "wb") as f:
            f.write(b"x" * size)


def _save(store, map_id, start_truth=None, session="s1"):
    pending = store.begin(map_id)
    _fake_serialise(pending.prefix)
    return store.commit(pending, session=session, start_truth=start_truth)


def test_empty_store_has_no_current_map(tmp_path):
    assert MapStore(str(tmp_path)).current() is None


def test_new_map_ids_are_distinct_and_safe():
    ids = {mapstore.new_map_id() for _ in range(50)}
    assert len(ids) == 50
    assert all(mapstore.MAP_ID_RE.fullmatch(i) for i in ids)


def test_a_save_is_the_current_map_with_its_meta(tmp_path):
    store = MapStore(str(tmp_path))
    meta = _save(store, "map-0a1b2c3d", start_truth={"x_m": 1.0, "y_m": 2.0, "heading_deg": 90})
    cur = store.current()
    assert cur is not None
    assert cur.map_id == "map-0a1b2c3d" and cur.version == 1
    assert cur.meta == meta
    assert meta["start_truth"] == {"x_m": 1.0, "y_m": 2.0, "heading_deg": 90}
    # The dock is the first node's pose: where session 1 began, the map's origin.
    assert meta["dock_pose"] == {"x": 0.0, "y": 0.0, "theta": 0.0}
    for ext in mapstore.GRAPH_FILES:
        assert os.path.isfile(cur.prefix + ext)
    with open(os.path.join(os.path.dirname(cur.prefix), "meta.json")) as f:
        assert json.load(f) == meta


def test_versions_count_up_and_only_three_are_kept(tmp_path):
    store = MapStore(str(tmp_path))
    for _ in range(5):
        _save(store, "map-0a1b2c3d")
    assert store.current().version == 5
    assert store.versions("map-0a1b2c3d") == [3, 4, 5]


def test_start_truth_and_map_id_carry_from_the_first_save(tmp_path):
    """A later session's save keeps session 1's frame facts: SLAM's map frame
    is still where session 1 began, whatever this session's start was."""
    store = MapStore(str(tmp_path))
    first = {"x_m": 1.0, "y_m": 2.0, "heading_deg": 0}
    _save(store, "map-0a1b2c3d", start_truth=first, session="s1")
    meta = _save(store, "map-0a1b2c3d", start_truth={"x_m": 9, "y_m": 9, "heading_deg": 9},
                 session="s2")
    assert meta["start_truth"] == first
    assert meta["first_session"] == "s1" and meta["session"] == "s2"


def test_a_save_with_missing_files_is_refused_and_leaves_current(tmp_path):
    store = MapStore(str(tmp_path))
    _save(store, "map-0a1b2c3d")
    pending = store.begin("map-0a1b2c3d")      # slam_toolbox wrote nothing
    with pytest.raises(mapstore.SaveFailed):
        store.commit(pending, session="s2", start_truth=None)
    assert store.current().version == 1
    assert store.versions("map-0a1b2c3d") == [1]
    assert not [d for d in os.listdir(tmp_path / "map-0a1b2c3d") if d.startswith(".")]


def test_an_unfinished_version_is_never_current(tmp_path):
    """A crash between slam_toolbox's write and the commit leaves only a
    hidden staging directory, which current() never offers."""
    store = MapStore(str(tmp_path))
    pending = store.begin("map-0a1b2c3d")
    _fake_serialise(pending.prefix)
    assert store.current() is None
    assert MapStore(str(tmp_path)).current() is None


def test_current_ignores_a_pointer_to_something_not_saved(tmp_path):
    """Criterion 12: the only map ever offered for loading is one the robot
    saved -- a pointer edited by hand to another path, or to a version whose
    meta does not match, is refused."""
    store = MapStore(str(tmp_path))
    _save(store, "map-0a1b2c3d")
    pointer = tmp_path / mapstore.CURRENT
    for bad in ({"map_id": "../../etc", "version": 1},
                {"map_id": "map-0a1b2c3d", "version": 7},
                {"map_id": "map-ffffffff", "version": 1},
                {"map_id": "map-0a1b2c3d"},
                "not json"):
        pointer.write_text(bad if isinstance(bad, str) else json.dumps(bad))
        assert store.current() is None, bad


def test_a_crashed_save_is_pruned_at_the_next_commit(tmp_path):
    """Staging from an earlier process (a container restarted mid-save) is
    removed; this store's own save in flight is not."""
    crashed = MapStore(str(tmp_path)).begin("map-0a1b2c3d")     # the crashed process
    os.utime(crashed.staging, (1, 1))                           # long ago
    store = MapStore(str(tmp_path))
    _save(store, "map-0a1b2c3d")
    left = [n for n in os.listdir(tmp_path / "map-0a1b2c3d") if n.startswith(".")]
    assert left == []
    live = store.begin("map-0a1b2c3d")
    _save(store, "map-0a1b2c3d")
    assert os.path.isdir(live.staging)


def test_bad_map_ids_are_refused(tmp_path):
    store = MapStore(str(tmp_path))
    for bad in ("", "../x", "map-0a1b2c3d/..", "map 1", "x" * 80):
        with pytest.raises(ValueError):
            store.begin(bad)


def test_the_pointer_is_replaced_atomically(tmp_path, monkeypatch):
    """os.replace, never an in-place write: a reader sees the old pointer or
    the new one, never half of one."""
    store = MapStore(str(tmp_path))
    _save(store, "map-0a1b2c3d")
    calls = []
    real = os.replace
    monkeypatch.setattr(mapstore.os, "replace", lambda a, b: (calls.append(b), real(a, b)))
    _save(store, "map-0a1b2c3d")
    assert str(tmp_path / mapstore.CURRENT) in calls


# ---- criteria 12-13: the launch files ----

def _slam_params():
    with open(os.path.join(BRINGUP, "config/slam.yaml")) as f:
        return yaml.safe_load(f)["slam_toolbox"]["ros__parameters"]


def test_slam_yaml_loads_no_map_by_itself():
    """Criterion 12: no map is named in the config. A reload goes through
    the bridge, from the store, and nothing else."""
    p = _slam_params()
    for key in ("map_file_name", "map_start_pose", "map_start_at_dock"):
        assert key not in p, key


def test_no_amcl_and_no_map_server():
    """Criterion 13 (the user, 2026-10-11): slam_toolbox maps and localises;
    nav2's static layer is its /map."""
    texts = []
    for rel in ("launch/picar.launch.py", "config/nav2.yaml"):
        with open(os.path.join(BRINGUP, rel)) as f:
            texts.append(f.read())
    for t in texts:
        assert "amcl" not in t.lower()
        assert "map_server" not in t
        assert "nav2_map_server" not in t


def test_slam_maps_with_the_async_node():
    """3.50.D2-b: map_and_localization_slam_toolbox_node mapped
    home_first_floor worse than the async node (east max 0.142/0.228 m vs
    0.089/0.071 m), so mapping stays on the async node (the user, 2026-10-23).
    Since 3.50.D3 the bridge starts slam_toolbox, and the launch file starts
    none: one owner, or a reload would run beside a mapping node."""
    assert slam_process.EXECUTABLES == {"mapping": "async_slam_toolbox_node",
                                        "localisation": "localization_slam_toolbox_node"}
    with open(os.path.join(BRINGUP, "launch/picar.launch.py")) as f:
        launch = f.read()
    assert 'package="slam_toolbox"' not in launch
    assert "slam_toolbox_node" not in launch.replace("slam_toolbox, which", "")


def test_only_the_newest_maps_are_kept(tmp_path):
    """3.50.D2 review: each container start without a reload is a new
    map_id; old maps beyond KEEP_MAPS are removed, the current one never."""
    store = MapStore(str(tmp_path))
    ids = [f"map-0000000{i}" for i in range(mapstore.KEEP_MAPS + 2)]
    for i, map_id in enumerate(ids):
        _save(store, map_id)
        os.utime(tmp_path / map_id, (1000 + i, 1000 + i))
    _save(store, ids[-1])
    kept = sorted(n for n in os.listdir(tmp_path) if mapstore.MAP_ID_RE.fullmatch(n))
    assert kept == sorted(ids[-mapstore.KEEP_MAPS:])
    assert store.current().map_id == ids[-1]


def test_a_missing_start_truth_is_filled_only_by_its_own_session(tmp_path):
    """3.50.D2 review: a first save before the start truth arrived must not
    pin None for the map's life -- but only the first session's start is the
    map frame's; a later session's start truth is another place."""
    store = MapStore(str(tmp_path))
    _save(store, "map-0a1b2c3d", start_truth=None, session="s1")
    later = {"x_m": 1.0, "y_m": 2.0, "heading_deg": 0}
    assert _save(store, "map-0a1b2c3d", start_truth=later, session="s1")["start_truth"] == later
    other = MapStore(str(tmp_path / "b"))
    _save(other, "map-0a1b2c3d", start_truth=None, session="s1")
    assert _save(other, "map-0a1b2c3d", start_truth=later, session="s2")["start_truth"] is None


def test_a_malformed_meta_does_not_block_later_saves(tmp_path):
    """3.50.D2 review: one truncated or foreign meta.json is skipped."""
    store = MapStore(str(tmp_path))
    _save(store, "map-0a1b2c3d", start_truth={"x_m": 1.0, "y_m": 0.0, "heading_deg": 0})
    for bad in ("{}", "[1, 2]", '{"map_id": "map-0a1b2c3d"}'):
        (tmp_path / "map-0a1b2c3d" / "v1" / mapstore.META).write_text(bad)
        meta = _save(store, "map-0a1b2c3d", session="s2")
        assert meta["map_id"] == "map-0a1b2c3d"


def test_a_failed_pointer_is_a_failed_save_that_names_the_version(tmp_path, monkeypatch):
    store = MapStore(str(tmp_path))
    _save(store, "map-0a1b2c3d")

    def boom(*a):
        raise OSError("no space")
    monkeypatch.setattr(store, "_point_at", boom)
    with pytest.raises(mapstore.SaveFailed, match="v2 is on disk"):
        _save(store, "map-0a1b2c3d")
    monkeypatch.undo()
    assert store.current().version == 1


def test_a_timed_out_save_leaves_its_staging_for_the_next_prune(tmp_path):
    store = MapStore(str(tmp_path))
    pending = store.begin("map-0a1b2c3d")
    store.abandon(pending, remove=False)
    assert os.path.isdir(pending.staging)
    # Review round 3: slam_toolbox may still be writing there -- the next
    # save, seconds later, leaves it; once stale, a save removes it.
    _save(store, "map-0a1b2c3d")
    assert os.path.isdir(pending.staging)
    os.utime(pending.staging, (1, 1))
    _save(store, "map-0a1b2c3d")
    assert not os.path.isdir(pending.staging)


def test_a_start_truth_filled_later_is_carried_past_the_first_version(tmp_path):
    """Review round 3: v1 saved before the start truth arrived, v2 filled it;
    a later session's save carries v2's, not v1's None."""
    store = MapStore(str(tmp_path))
    truth = {"x_m": 1.0, "y_m": 2.0, "heading_deg": 0}
    _save(store, "map-0a1b2c3d", start_truth=None, session="s1")
    _save(store, "map-0a1b2c3d", start_truth=truth, session="s1")
    assert _save(store, "map-0a1b2c3d", start_truth=None, session="s2")["start_truth"] == truth


def test_a_failed_pointer_write_leaves_no_temp_file(tmp_path, monkeypatch):
    store = MapStore(str(tmp_path))
    _save(store, "map-0a1b2c3d")

    def boom(a, b):
        raise OSError("no space")
    monkeypatch.setattr(mapstore.os, "replace", boom)
    with pytest.raises(mapstore.SaveFailed):
        _save(store, "map-0a1b2c3d")
    monkeypatch.undo()
    assert not [n for n in os.listdir(tmp_path) if n.endswith(".tmp")]
    assert store.current().version == 1


def test_a_pointer_that_moved_is_a_save_even_if_the_dir_sync_fails(tmp_path, monkeypatch):
    store = MapStore(str(tmp_path))
    real = mapstore._fsync_dir

    def flaky(path):
        if path == str(tmp_path):
            raise OSError("EIO")
        real(path)
    monkeypatch.setattr(mapstore, "_fsync_dir", flaky)
    _save(store, "map-0a1b2c3d")
    assert store.current().version == 1


# ---- 3.50.D3: the reload, its check and the swap to a fresh map ----

import math  # noqa: E402
import time  # noqa: E402
import types  # noqa: E402

from tests.test_review_376h1 import _Msg, bridge_mod  # noqa: E402,F401 -- the stubbed bridge

RES = 0.05


def _room_grid():
    """A 5 x 3 m room, walls one cell thick, the map's origin at (-1, -1):
    an OccupancyGrid record as the bridge keeps it."""
    w, h = int(5 / RES), int(3 / RES)
    data = [0] * (w * h)
    for c in range(w):
        data[c] = data[(h - 1) * w + c] = 100
    for r in range(h):
        data[r * w] = data[r * w + w - 1] = 100
    return {"resolution_m": RES, "width": w, "height": h, "origin_x_m": -1.0,
            "origin_y_m": -1.0, "origin_yaw_rad": 0.0, "data": data}


def _room_scan(x, y, yaw, n=360):
    """Exact ranges from (x, y, yaw) to the room's wall centre lines."""
    xmin, xmax, ymin, ymax = -1 + RES / 2, 4 - RES / 2, -1 + RES / 2, 2 - RES / 2
    ranges = []
    for i in range(n):
        a = yaw - math.pi + i * 2 * math.pi / n
        c, s = math.cos(a), math.sin(a)
        ts = [t for t in ((xmax - x) / c if c > 1e-9 else None, (xmin - x) / c if c < -1e-9 else None,
                          (ymax - y) / s if s > 1e-9 else None, (ymin - y) / s if s < -1e-9 else None)
              if t is not None and t > 0]
        ranges.append(min(ts))
    return {"angle_min_rad": -math.pi, "angle_increment_rad": 2 * math.pi / n, "ranges_m": ranges}


def test_scans_agree_with_the_map_where_the_robot_really_is():
    grid, scan = _room_grid(), _room_scan(0.5, 0.4, 0.2)
    assert relocalise.scan_agreement(scan, (0.5, 0.4, 0.2), grid) > 0.95


def test_scans_disagree_when_the_robot_is_placed_a_metre_and_30_deg_off():
    """Criterion 8's start: the robot is 1 m and 30 deg from the dock but
    localisation puts it ON the dock, so its scans are laid there."""
    grid = _room_grid()
    scan = _room_scan(1.5, 0.4, math.radians(30))         # where it really is
    assert relocalise.scan_agreement(scan, (0.5, 0.4, 0.0), grid) < relocalise.AGREE_MIN


def test_no_returns_is_no_agreement():
    scan = {"angle_min_rad": 0.0, "angle_increment_rad": 0.1, "ranges_m": [None, math.inf, 0.0]}
    assert relocalise.scan_agreement(scan, (0, 0, 0), _room_grid()) is None


def test_the_verdict_needs_both_agreement_and_the_dock():
    ok = relocalise.judge([0.9] * 5, (0.02, -0.01, math.radians(1)))
    assert ok["ok"] and ok["reason"] == "matched"
    assert not relocalise.judge([0.9, 0.9, 0.3, 0.2, 0.1], (0, 0, 0))["ok"]   # median 0.3
    far = relocalise.judge([0.9] * 5, (0.6, 0.0, 0.0))
    assert not far["ok"] and "from the dock" in far["reason"]
    turned = relocalise.judge([0.9] * 5, (0.0, 0.0, math.radians(20)))
    assert not turned["ok"]
    assert not relocalise.judge([0.9] * 5, None)["ok"]
    assert not relocalise.judge([None] * 5, (0, 0, 0))["ok"]


def test_the_localisation_node_is_given_the_saved_graph_and_the_dock():
    argv = slam_process.command("/x/lib", "/p/slam.yaml", "localisation", "/maps/m/v2/graph",
                                (0.0, 0.0, 0.0))
    assert argv[0] == "/x/lib/localization_slam_toolbox_node"
    assert "__node:=slam_toolbox" in argv and "/p/slam.yaml" in argv
    assert "map_file_name:=/maps/m/v2/graph" in argv
    assert "map_start_pose:=[0.0, 0.0, 0.0]" in argv
    assert "mode:=localization" in argv


def test_a_mapping_node_loads_no_map():
    """Criterion 12: a fresh map loads no file."""
    argv = slam_process.command("/x/lib", "/p/slam.yaml", "mapping")
    assert argv[0] == "/x/lib/async_slam_toolbox_node"
    assert not any("map_file_name" in a or "map_start" in a for a in argv)
    with pytest.raises(ValueError):
        slam_process.command("/x/lib", "/p/slam.yaml", "mapping", "/maps/m/v1/graph")
    with pytest.raises(ValueError):
        slam_process.command("/x/lib", "/p/slam.yaml", "lifelong")


class _Proc:
    def __init__(self, argv, **_kw):
        self.argv, self.signals, self.code = argv, [], None

    def poll(self):
        return self.code

    def send_signal(self, sig):
        self.signals.append(sig)
        self.code = 0

    def wait(self, timeout=None):
        return self.code

    def kill(self):
        self.code = -9


def test_one_slam_toolbox_at_a_time():
    procs = []
    sp = slam_process.SlamProcess("/x/lib", "/p/slam.yaml",
                                  spawn=lambda argv, **kw: procs.append(_Proc(argv)) or procs[-1])
    sp.start("localisation", "/maps/m/v1/graph", (0, 0, 0))
    sp.start("mapping")
    assert len(procs) == 2 and procs[0].signals and procs[0].poll() is not None
    assert sp.mode == "mapping" and sp.alive() and sp.starts == 2
    sp.stop()
    assert not sp.alive() and sp.mode is None


class _FakeSlam:
    def __init__(self):
        self.calls, self.mode, self.starts = [], None, 0

    def start(self, mode, map_prefix=None, start_pose=None):
        self.calls.append((mode, map_prefix, start_pose))
        self.mode = mode
        self.starts += 1

    def stop(self):
        self.mode = None


def _reload_bridge(bridge_mod, monkeypatch, tmp_path, verdict):
    pkgs = types.ModuleType("ament_index_python.packages")
    pkgs.get_package_prefix = lambda name: "/opt/" + name
    pkgs.get_package_share_directory = lambda name: "/share/" + name
    monkeypatch.setitem(sys.modules, "ament_index_python", types.ModuleType("ament_index_python"))
    monkeypatch.setitem(sys.modules, "ament_index_python.packages", pkgs)
    fake = _FakeSlam()
    monkeypatch.setattr(bridge_mod.slam_process, "SlamProcess", lambda *a, **kw: fake)
    store = MapStore(str(tmp_path))
    _save(store, "map-0a1b2c3d", start_truth={"x_m": 1.0, "y_m": 2.0, "heading_deg": 90.0})
    b = bridge_mod.Bridge()
    b.map_store = store
    b.last_map = {"session": b.session, "version": 1, "data": [0]}
    b.start_truth = {"x_m": 3.0, "y_m": 3.0, "heading_deg": 0.0}     # this session's own
    b.start_truth_pending = False
    monkeypatch.setattr(b, "check_reload", lambda deadline, dock=None: verdict)
    return b, fake, store


def test_a_reload_that_fits_localises_on_the_saved_map(bridge_mod, monkeypatch, tmp_path):
    b, fake, store = _reload_bridge(bridge_mod, monkeypatch, tmp_path,
                                    relocalise.judge([0.9] * 5, (0.0, 0.0, 0.0)))
    b.start_slam()
    assert fake.calls == [("localisation", store.current().prefix, (0.0, 0.0, 0.0))]
    assert b.map_id == "map-0a1b2c3d" and b.slam_state == "ready"
    assert b.reload["result"] == "matched" and b.reload["version"] == 1
    reply = b.slam_pose()[1]
    # The saved map's frame is session 1's: its start truth, not this session's.
    assert reply["start_truth"] == {"x_m": 1.0, "y_m": 2.0, "heading_deg": 90.0}
    assert reply["start_truth_pending"] is False
    assert b.health()[1]["slam"]["reload"]["result"] == "matched"


def test_a_reload_that_does_not_fit_falls_back_to_a_fresh_map(bridge_mod, monkeypatch, tmp_path):
    """Criterion 8: a bad match is a fresh map with a new map_id, never a
    mission on the mismatched frame."""
    b, fake, store = _reload_bridge(bridge_mod, monkeypatch, tmp_path,
                                    relocalise.judge([0.2] * 5, (0.0, 0.0, 0.0)))
    b.start_slam()
    assert [c[0] for c in fake.calls] == ["localisation", "mapping"]
    assert fake.calls[1] == ("mapping", None, None)
    assert b.map_id != "map-0a1b2c3d" and mapstore.MAP_ID_RE.fullmatch(b.map_id)
    assert b.last_map is None                      # the saved map's /map is not this one
    assert b.reload["result"] == "fell_back" and "agree" in b.reload["reason"]
    assert b.slam_pose()[1]["start_truth"] == {"x_m": 3.0, "y_m": 3.0, "heading_deg": 0.0}


def test_no_saved_map_or_reload_off_maps_from_empty(bridge_mod, monkeypatch, tmp_path):
    b, fake, _ = _reload_bridge(bridge_mod, monkeypatch, tmp_path, None)
    monkeypatch.setattr(bridge_mod, "MAP_RELOAD", False)
    b.start_slam()
    assert fake.calls == [("mapping", None, None)] and b.reload["result"] == "off"


def test_nav2_is_held_and_no_pose_is_published_while_the_reload_is_judged(bridge_mod):
    from tests.test_review_376h1 import _bridge
    b = _bridge(bridge_mod)
    b.slam_state = "checking"
    b.start_truth, b.start_truth_pending = {"x_m": 1.0, "y_m": 1.0, "heading_deg": 0.0}, False
    code, body = b.send_goal({"x_m": 1.0, "y_m": 0.0})
    assert code == 503 and body["slam"] == "checking"
    assert b.nav.futures == []                     # nothing reached nav2
    reply = b.slam_pose()[1]
    assert reply["map"] is None and reply["start_truth_pending"] is True


def test_a_mission_end_while_localising_saves_nothing_and_fails_nothing(bridge_mod, tmp_path):
    from tests.test_review_376h1 import _bridge
    b = _bridge(bridge_mod)
    store = MapStore(str(tmp_path))
    _save(store, "map-0a1b2c3d")
    b.map_store, b.map_id = store, "map-0a1b2c3d"
    b.slam = _FakeSlam()
    b.slam.mode = "localisation"
    b._save_map("mission_end")
    assert b.map_saves["skipped"] == 1 and b.map_saves["failed"] == 0
    assert store.versions("map-0a1b2c3d") == [1]


def test_a_map_from_a_stopped_node_is_dropped(bridge_mod):
    from tests.test_review_376h1 import _bridge
    b = _bridge(bridge_mod)
    b.slam_started_at = time.time()
    old = int(b.slam_started_at - 2)
    msg = _Msg(header=_Msg(stamp=_Msg(sec=old, nanosec=0)),
               info=_Msg(resolution=0.05, width=1, height=1,
                         origin=_Msg(position=_Msg(x=0.0, y=0.0),
                                     orientation=_Msg(x=0.0, y=0.0, z=0.0, w=1.0))),
               data=[0])
    b._on_map(msg)
    assert b.last_map is None
    msg.header.stamp.sec = int(b.slam_started_at + 1)
    b._on_map(msg)
    assert b.last_map is not None


def test_no_pose_within_the_window_is_a_bad_match(bridge_mod):
    from tests.test_review_376h1 import _bridge
    b = _bridge(bridge_mod)
    v = b.check_reload(time.monotonic() + 0.2)
    assert not v["ok"] and "no localised pose" in v["reason"]


def test_a_start_failure_is_shown_not_a_silent_dead_thread(bridge_mod, monkeypatch, tmp_path):
    """Thermos: before 3.50.D3 the launch file always started slam_toolbox;
    a raise on the start thread must leave a reason on /health."""
    b, fake, _ = _reload_bridge(bridge_mod, monkeypatch, tmp_path, None)
    monkeypatch.setattr(fake, "start", lambda *a, **kw: (_ for _ in ()).throw(OSError("no exe")))
    b.start_slam()
    h = b.health()[1]["slam"]
    assert h["state"] == "failed" and "no exe" in h["reload"]["error"]


def test_the_dock_check_is_against_the_dock_it_started_at():
    assert relocalise.judge([0.9] * 5, (1.0, 2.0, 0.5), dock=(1.0, 2.0, 0.5))["ok"]
    assert not relocalise.judge([0.9] * 5, (0.0, 0.0, 0.0), dock=(1.0, 2.0, 0.5))["ok"]


def test_no_pdeathsig_from_a_short_lived_thread():
    """Both reviewers: PR_SET_PDEATHSIG fires when the forking THREAD exits."""
    seen = {}
    sp = slam_process.SlamProcess("/x/lib", "/p/slam.yaml",
                                  spawn=lambda argv, **kw: seen.update(kw) or _Proc(argv))
    sp.start("mapping")
    assert "preexec_fn" not in seen


def test_a_reload_without_a_saved_start_truth_waits_for_its_own(bridge_mod, monkeypatch, tmp_path):
    b, fake, store = _reload_bridge(bridge_mod, monkeypatch, tmp_path,
                                    relocalise.judge([0.9] * 5, (0.0, 0.0, 0.0)))
    b.start_slam()
    b.reload_start_truth, b.start_truth, b.start_truth_pending = None, None, True
    assert b.slam_pose()[1]["start_truth_pending"] is True


def test_a_failed_start_is_not_ok_and_names_the_map_tried(bridge_mod, monkeypatch, tmp_path):
    b, fake, _ = _reload_bridge(bridge_mod, monkeypatch, tmp_path, None)
    monkeypatch.setattr(b, "check_reload", lambda *a: (_ for _ in ()).throw(ValueError("odd map")))
    b.start_slam()
    h = b.health()[1]
    assert h["ok"] is False and h["slam"]["state"] == "failed"
    assert h["slam"]["reload"]["map_id"] == "map-0a1b2c3d" and "odd map" in h["slam"]["reload"]["error"]
    assert fake.mode is None                       # the half-judged node was stopped


def test_a_corrupt_dock_pose_starts_at_the_origin(bridge_mod, monkeypatch, tmp_path):
    b, fake, store = _reload_bridge(bridge_mod, monkeypatch, tmp_path,
                                    relocalise.judge([0.9] * 5, (0.0, 0.0, 0.0)))
    meta_path = os.path.join(os.path.dirname(store.current().prefix), "meta.json")
    meta = json.load(open(meta_path))
    meta["dock_pose"] = {"x": float("nan"), "y": True, "theta": 0.0}
    json.dump(meta, open(meta_path, "w"))
    b.start_slam()
    assert fake.calls[0][2] == (0.0, 0.0, 0.0)


def test_the_bridge_ends_the_launch_if_it_dies():
    with open(os.path.join(BRINGUP, "launch/picar.launch.py")) as f:
        launch = f.read()
    assert 'executable="bridge", output="both",\n             on_exit=Shutdown(' in launch
