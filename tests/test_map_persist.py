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


def test_a_reload_that_raises_falls_back_to_a_fresh_map(bridge_mod, monkeypatch, tmp_path):
    """Review round 3: a reload that cannot be judged does not fit -- a fresh
    map, not a bridge with no SLAM."""
    b, fake, _ = _reload_bridge(bridge_mod, monkeypatch, tmp_path, None)
    monkeypatch.setattr(b, "check_reload", lambda *a: (_ for _ in ()).throw(ValueError("odd map")))
    b.start_slam()
    h = b.health()[1]
    assert h["ok"] is True and h["slam"]["state"] == "ready" and fake.mode == "mapping"
    assert h["slam"]["reload"]["result"] == "fell_back" and "odd map" in h["slam"]["reload"]["reason"]
    assert h["slam"]["reload"]["map_id"] == "map-0a1b2c3d" and b.map_id != "map-0a1b2c3d"


def test_a_failed_fresh_start_is_not_ok(bridge_mod, monkeypatch, tmp_path):
    b, fake, _ = _reload_bridge(bridge_mod, monkeypatch, tmp_path, None)
    monkeypatch.setattr(bridge_mod, "MAP_RELOAD", False)
    monkeypatch.setattr(fake, "start", lambda *a, **kw: (_ for _ in ()).throw(OSError("no exe")))
    b.start_slam()
    h = b.health()[1]
    assert h["ok"] is False and h["slam"]["state"] == "failed"
    assert h["slam"]["reload"]["result"] == "failed"


def test_no_save_before_slam_is_ready(bridge_mod, tmp_path):
    """Review round 3: a fresh node's empty graph must not become current."""
    from tests.test_review_376h1 import _bridge
    b = _bridge(bridge_mod)
    store = MapStore(str(tmp_path))
    b.map_store, b.slam_state = store, "starting"
    b.slam = _FakeSlam()
    b.slam.mode = "mapping"
    b._save_map("mission_end")
    assert b.map_saves["skipped"] == 1 and b.map_saves["failed"] == 0 and store.current() is None


def test_a_corrupt_dock_pose_starts_at_the_origin(bridge_mod, monkeypatch, tmp_path):
    b, fake, store = _reload_bridge(bridge_mod, monkeypatch, tmp_path,
                                    relocalise.judge([0.9] * 5, (0.0, 0.0, 0.0)))
    meta_path = os.path.join(os.path.dirname(store.current().prefix), "meta.json")
    with open(meta_path) as f:
        meta = json.load(f)
    meta["dock_pose"] = {"x": float("nan"), "y": True, "theta": 0.0}
    with open(meta_path, "w") as f:
        json.dump(meta, f)
    b.start_slam()
    assert fake.calls[0][2] == (0.0, 0.0, 0.0)


def test_the_bridge_ends_the_launch_if_it_dies():
    with open(os.path.join(BRINGUP, "launch/picar.launch.py")) as f:
        launch = f.read()
    import re
    assert re.search(r'executable="bridge"[^)]*on_exit=Shutdown\(', launch)


def test_a_rejected_reload_survives_a_failed_fresh_spawn(bridge_mod, monkeypatch, tmp_path):
    b, fake, _ = _reload_bridge(bridge_mod, monkeypatch, tmp_path,
                                relocalise.judge([0.2] * 5, (0.0, 0.0, 0.0)))
    real = fake.start

    def start(mode, *a, **kw):
        if mode == "mapping":
            raise OSError("no exe")
        return real(mode, *a, **kw)
    monkeypatch.setattr(fake, "start", start)
    b.start_slam()
    r = b.health()[1]["slam"]["reload"]
    assert r["result"] == "failed" and r["before"] == "fell_back" and "agree" in r["reason"]


# ---- 3.50.D4: the re-map triggers (picar_bridge/remap.py) ----

from picar_bridge import remap  # noqa: E402

ROOM = (-1 + RES / 2, 4 - RES / 2, -1 + RES / 2, 2 - RES / 2)    # wall centre lines


def _ray_box(x, y, c, s, box):
    """Distance along (c, s) from (x, y) to an axis-aligned box, or None."""
    x0, x1, y0, y1 = box
    tmin, tmax = -math.inf, math.inf
    for o, dv, lo, hi in ((x, c, x0, x1), (y, s, y0, y1)):
        if abs(dv) < 1e-12:
            if not lo <= o <= hi:
                return None
            continue
        t0, t1 = (lo - o) / dv, (hi - o) / dv
        tmin, tmax = max(tmin, min(t0, t1)), min(tmax, max(t0, t1))
    return tmin if tmax >= tmin > 0 else None


def _scan(x, y, yaw, boxes=(), n=360):
    """_room_scan with solid boxes (x0, x1, y0, y1) in the room."""
    base = _room_scan(x, y, yaw, n)
    ranges = []
    for i, r in enumerate(base["ranges_m"]):
        a = yaw - math.pi + i * 2 * math.pi / n
        for b in boxes:
            t = _ray_box(x, y, math.cos(a), math.sin(a), b)
            if t is not None:
                r = min(r, t)
        ranges.append(r)
    return {**base, "ranges_m": ranges}


def _grid_with(boxes=(), unknown=()):
    g = _room_grid()
    data = list(g["data"])
    for (x0, x1, y0, y1), v in [(b, 100) for b in boxes] + [(b, -1) for b in unknown]:
        for r in range(g["height"]):
            for c in range(g["width"]):
                cx, cy = -1 + (c + 0.5) * RES, -1 + (r + 0.5) * RES
                if x0 <= cx <= x1 and y0 <= cy <= y1:
                    data[r * g["width"] + c] = v
    return {**g, "data": data}


def _run(saved_grid, scans_at, seconds=12.0, hz=10.0):
    """Feed scans; the first trigger record (or None) and the tracker."""
    tr = remap.Contradictions(remap.SavedMap(saved_grid))
    for k in range(int(seconds * hz)):
        t = k / hz
        scan, pose = scans_at(t)
        fired = tr.add(scan, pose, t)
        if fired:
            return fired, tr, t
    return None, tr, None


BOX = (1.0, 1.6, 0.6, 1.0)          # a 60 x 40 cm object in the room


def test_an_unchanged_room_never_fires():
    pose = (0.5, 0.4, 0.2)
    fired, tr, _ = _run(_room_grid(), lambda t: (_scan(*pose), pose), seconds=20)
    assert fired is None and tr.lost == 0 and tr.scans == 200
    assert remap.judge_scan(remap.SavedMap(_room_grid()), _scan(*pose), pose)["contra"].size <= 2


def test_something_new_fires_after_it_persists():
    pose = (0.0, 0.0, 0.0)
    fired, _, t = _run(_room_grid(), lambda t: (_scan(*pose, boxes=[BOX]), pose))
    assert fired is not None and fired["trigger"] == "contradiction"
    assert fired["cells"] >= remap.MIN_GROUP
    # Not before MIN_LOOKS scans could have looked.
    assert t >= (remap.MIN_LOOKS - 1) / 10.0
    cx, cy = fired["centre_m"]
    assert 0.9 <= cx <= 1.7 and 0.5 <= cy <= 1.1


def test_something_gone_fires():
    """The saved map has the object; the room no longer does."""
    pose = (0.0, 0.0, 0.0)
    fired, _, _ = _run(_grid_with(boxes=[BOX]), lambda t: (_scan(*pose), pose))
    assert fired is not None and fired["trigger"] == "contradiction"


def test_a_walker_passing_does_not_fire():
    """SIM_MOVERS: a 30 cm thing hopping 30 cm a second across the room."""
    pose = (0.0, 0.0, 0.0)

    def at(t):
        x = 0.6 + 0.3 * (int(t) % 10)
        return _scan(*pose, boxes=[(x, x + 0.3, 1.1, 1.4)]), pose
    fired, _, _ = _run(_room_grid(), at, seconds=30)
    assert fired is None


def test_a_slipped_frame_is_lost_and_never_fires():
    """D4-g: scans laid a metre and 30 deg off are a lost robot, not a house
    that changed."""
    real, believed = (1.5, 0.4, math.radians(30)), (0.5, 0.4, 0.0)
    fired, tr, _ = _run(_room_grid(), lambda t: (_scan(*real), believed), seconds=15)
    assert fired is None and tr.lost == tr.scans > 0


def test_a_small_pose_error_beside_a_wall_does_not_fire():
    """Grazing beams along a wall, with the localised pose 2 cm and 1 deg out."""
    real = (3.6, 1.75, 0.0)
    believed = (3.62, 1.77, math.radians(1))
    fired, tr, _ = _run(_room_grid(), lambda t: (_scan(*real), believed), seconds=20)
    assert fired is None and tr.lost == 0


def test_returns_into_unknown_are_not_new():
    """An unmapped part of the saved map is trigger 1's, not trigger 2's."""
    pose = (0.0, 0.0, 0.0)
    unknown = (2.5, 4.0, -1.0, 2.0)
    fired, _, _ = _run(_grid_with(unknown=[unknown]),
                       lambda t: (_scan(*pose, boxes=[(3.0, 3.4, 0.0, 0.6)]), pose))
    assert fired is None


def test_entered_unknown_is_the_saved_maps_unknown_or_off_its_edge():
    saved = remap.SavedMap(_grid_with(unknown=[(2.5, 4.0, -1.0, 2.0)]))
    assert not remap.entered_unknown(saved, 0.5, 0.5)
    assert remap.entered_unknown(saved, 3.0, 0.5)
    assert remap.entered_unknown(saved, 10.0, 0.5)


# ---- 3.50.D4: the bridge's re-map ----

from tests.test_review_376h1 import _Future  # noqa: E402


class _LiveSlam(_FakeSlam):
    def alive(self):
        return self.mode is not None


class _Serialise:
    """slam_toolbox's serialize_map: writes the graph files, answers 0."""

    def wait_for_service(self, timeout_sec):
        return True

    def call_async(self, req):
        _fake_serialise(req.filename)
        f = _Future()
        f.complete(types.SimpleNamespace(result=0))
        return f


def _matched(bridge_mod, monkeypatch, tmp_path, here=(1.0, 0.5, 0.3)):
    """A bridge localising on a matched reload of the room, at `here`."""
    b, fake, store = _reload_bridge(bridge_mod, monkeypatch, tmp_path,
                                    relocalise.judge([0.9] * 5, (0.0, 0.0, 0.0)))
    live = _LiveSlam()
    monkeypatch.setattr(bridge_mod.slam_process, "SlamProcess", lambda *a, **kw: live)
    verdict = relocalise.judge([0.9] * 5, (0.0, 0.0, 0.0))

    def check(deadline, dock=None):
        # The localisation node's /map arrives while the reload is judged.
        b.last_map = {**_room_grid(), "session": b.session, "version": 1}
        return verdict
    monkeypatch.setattr(b, "check_reload", check)
    monkeypatch.setattr(b, "_fresh_pose",
                        lambda: {"x_m": here[0], "y_m": here[1], "yaw_rad": here[2], "age_s": 0.0})
    monkeypatch.setattr(b, "_pose_now", lambda mo=None: tuple(here))
    monkeypatch.setattr(b, "_map_to_odom", lambda: (0.0, 0.0, 0.0))
    b._rest_since = time.monotonic() - 1.0              # standing still
    b.last_odom = {"at": time.time() + 3600}            # and odometry that stays fresh
    b.start_slam()
    assert b.slam_state == "ready" and live.mode == "localisation"
    return b, live, store


def _settle(b, seconds=10.0):
    end = time.monotonic() + seconds
    while b.slam_state == "remapping" and time.monotonic() < end:
        time.sleep(0.01)


def test_a_matched_reload_freezes_its_map_for_the_watch(bridge_mod, monkeypatch, tmp_path):
    b, _, _ = _matched(bridge_mod, monkeypatch, tmp_path)
    assert b._remap_saved is not None and b._remap_saved.w == _room_grid()["width"]
    b.last_map = {**_room_grid(), "data": [0] * len(_room_grid()["data"])}    # /map fills in
    assert b._remap_saved.state.max() == remap.OCC                           # the frozen copy does not
    assert b.health()[1]["slam"]["watch"] == {"scans": 0, "lost": 0}


def test_a_person_s_request_re_maps_from_the_current_pose(bridge_mod, monkeypatch, tmp_path):
    """D4-e: the mapping node on the version localised on, from where the
    robot is; the same map_id, and the next save is its next version."""
    b, live, store = _matched(bridge_mod, monkeypatch, tmp_path)
    code, body = b.remap("request", "twin")
    assert code == 200 and body["remapping"] is True and body["trigger"] == "request"
    _settle(b)
    assert b.health()[1]["slam"]["remap"]["from_pose"] == [1.0, 0.5, 0.3]
    assert live.calls[-1] == ("mapping", store.current().prefix, (1.0, 0.5, 0.3))
    assert b.slam_state == "ready" and live.mode == "mapping"
    assert b.map_id == "map-0a1b2c3d"
    r = b.health()[1]["slam"]["remap"]
    assert r["result"] == "mapping" and r["version"] == 1 and r["detail"] == "twin"
    assert b._remap_tracker is None                     # the watch is done
    b.serialize_cli = _Serialise()
    monkeypatch.setattr(bridge_mod, "SerializePoseGraph", types.SimpleNamespace(
        Request=types.SimpleNamespace, Response=types.SimpleNamespace(RESULT_SUCCESS=0)))
    b._save_map("mission_end")
    assert b.map_saves["saved"] == 1 and store.versions("map-0a1b2c3d") == [1, 2]
    assert store.current().meta["start_truth"] == {"x_m": 1.0, "y_m": 2.0, "heading_deg": 90.0}


def test_a_re_map_is_refused_unless_localising_and_ready(bridge_mod, monkeypatch, tmp_path):
    b, live, _ = _matched(bridge_mod, monkeypatch, tmp_path)
    b.slam_state = "checking"
    assert b.remap("request")[0] == 409
    b.slam_state = "ready"
    assert b.remap("request")[0] == 200
    _settle(b)
    code, body = b.remap("request")                     # already mapping
    assert code == 409 and body["mode"] == "mapping"
    fresh, fake, _ = _reload_bridge(bridge_mod, monkeypatch, tmp_path / "f", None)
    monkeypatch.setattr(bridge_mod, "MAP_RELOAD", False)
    fresh.slam_state = "ready"
    assert fresh.remap("request")[0] == 409             # a fresh map has nothing to re-map


def test_goals_are_held_while_re_mapping(bridge_mod, monkeypatch, tmp_path):
    b, _, _ = _matched(bridge_mod, monkeypatch, tmp_path)
    b.nav = types.SimpleNamespace(futures=[])
    b.slam_state = "remapping"
    code, body = b.send_goal({"x_m": 1.0, "y_m": 0.0})
    assert code == 503 and body["slam"] == "remapping" and b.nav.futures == []


def test_a_mapping_node_that_dies_localises_again(bridge_mod, monkeypatch, tmp_path):
    """Both reviewers: a failed re-map leaves the robot localising on the
    same version, from where it is, not with no SLAM."""
    monkeypatch.setitem(bridge_mod._test_ok, "value", True)   # the waits run
    b, live, store = _matched(bridge_mod, monkeypatch, tmp_path)
    monkeypatch.setattr(b, "_fresh_pose", lambda: None if live.mode == "mapping" else
                        {"x_m": 1.0, "y_m": 0.5, "yaw_rad": 0.3, "age_s": 0.0})
    monkeypatch.setattr(live, "alive", lambda: live.mode != "mapping")
    assert b.remap("request")[0] == 200
    _settle(b)
    h = b.health()[1]
    assert h["ok"] is True and h["slam"]["state"] == "ready" and h["slam"]["mode"] == "localisation"
    assert live.calls[-1] == ("localisation", store.current().prefix, (1.0, 0.5, 0.3))
    assert h["slam"]["remap"]["result"] == "failed_localising" and "exited" in h["slam"]["remap"]["error"]
    assert b._remap_tracker is not None and b._remap_tracker.scans == 0     # the watch starts over


def test_a_re_map_that_cannot_localise_again_is_failed(bridge_mod, monkeypatch, tmp_path):
    monkeypatch.setitem(bridge_mod._test_ok, "value", True)   # the waits run
    b, live, _ = _matched(bridge_mod, monkeypatch, tmp_path)
    monkeypatch.setattr(b, "_fresh_pose", lambda: None if live.calls[-1][0] != "localisation"
                        or len(live.calls) > 1 else
                        {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0, "age_s": 0.0})
    monkeypatch.setattr(live, "alive", lambda: False)
    assert b.remap("request")[0] == 200
    _settle(b)
    h = b.health()[1]
    assert h["ok"] is False and h["slam"]["state"] == "failed"
    assert h["slam"]["remap"]["result"] == "failed" and "relocalise_error" in h["slam"]["remap"]


def test_the_swap_waits_for_rest_and_no_live_goal(bridge_mod, monkeypatch, tmp_path):
    """Amendment 2: goals held at once; the swap only once the robot is still
    and no goal runs -- a live one is cancelled after REMAP_DEFER_MAX_S."""
    monkeypatch.setattr(bridge_mod, "REMAP_DEFER_MAX_S", 0.3)
    monkeypatch.setattr(bridge_mod, "REMAP_REST_WAIT_S", 0.3)
    b, live, _ = _matched(bridge_mod, monkeypatch, tmp_path)
    b.goal = {"state": "active", "x_m": 1.0, "y_m": 0.0}
    cancels = []
    def cancel(why="cancelled"):        # nav2 ends it, as its result would
        cancels.append(why)
        b.goal = {**b.goal, "state": "cancelled"}
        return True
    monkeypatch.setattr(b, "cancel_goal", cancel)
    assert b.remap("contradiction", {"cells": 12})[0] == 200
    time.sleep(0.15)
    assert live.mode == "localisation" and b.slam_state == "remapping"      # not yet
    assert b.send_goal({"x_m": 2.0, "y_m": 0.0})[0] == 503                 # but goals held
    _settle(b)
    assert cancels == ["re-map"] and live.mode == "mapping"
    r = b.health()[1]["slam"]["remap"]
    assert r["cancelled_goal"] is True and r["waited_s"] >= 0.3 and r["from_pose"] == [1.0, 0.5, 0.3]


def test_at_rest_with_no_goal_the_swap_is_at_once(bridge_mod, monkeypatch, tmp_path):
    b, live, _ = _matched(bridge_mod, monkeypatch, tmp_path)
    t0 = time.monotonic()
    assert b.remap("request")[0] == 200
    _settle(b)
    assert live.mode == "mapping" and time.monotonic() - t0 < 1.0
    assert b.health()[1]["slam"]["remap"]["cancelled_goal"] is False


def test_odometry_sets_rest(bridge_mod):
    from tests.test_review_376h1 import _bridge
    b = _bridge(bridge_mod)

    def odom(v, w):
        return _Msg(pose=_Msg(pose=_Msg(position=_Msg(x=0.0, y=0.0),
                                        orientation=_Msg(x=0.0, y=0.0, z=0.0, w=1.0))),
                    twist=_Msg(twist=_Msg(linear=_Msg(x=v), angular=_Msg(z=w))))
    b._on_odom(odom(0.2, 0.0))
    assert b._rest_since is None
    b._on_odom(odom(0.0, 0.001))
    first = b._rest_since
    b._on_odom(odom(0.0, 0.0))
    assert first is not None and b._rest_since == first
    b._on_odom(odom(0.0, 0.5))
    assert b._rest_since is None


def test_the_frozen_map_is_the_localisation_node_s_first(bridge_mod):
    """Review: the verdict's scans fill unknown cells near the dock; the
    map frozen for trigger 1 is the first one the node published."""
    from tests.test_review_376h1 import _bridge
    b = _bridge(bridge_mod)
    b.slam_state, b.slam_started_at = "checking", time.time() - 5

    def grid(v):
        return _Msg(header=_Msg(stamp=_Msg(sec=int(time.time()), nanosec=0)),
                    info=_Msg(resolution=0.05, width=1, height=1,
                              origin=_Msg(position=_Msg(x=0.0, y=0.0),
                                          orientation=_Msg(x=0.0, y=0.0, z=0.0, w=1.0))),
                    data=[v])
    b._on_map(grid(-1))
    b._on_map(grid(0))
    assert b._reload_map["data"] == [-1] and b.last_map["data"] == [0]


def test_a_body_that_is_not_an_object_is_refused():
    with open(os.path.join(ROOT, "service/slam/src/picar_bridge/picar_bridge/bridge.py")) as f:
        src = f.read()
    i = src.index('path == "/slam/remap"')
    assert "isinstance(body, dict)" in src[i:i + 300]


def test_partial_scans_do_not_wrap_their_ends():
    """Review: the incidence of a partial scan's end beams has one neighbour."""
    grid = _room_grid()
    full = _scan(0.5, 0.4, 0.0)
    half = {**full, "ranges_m": full["ranges_m"][:180]}
    j = remap.judge_scan(remap.SavedMap(grid), half, (0.5, 0.4, 0.0))
    assert j["agreement"] > 0.9 and j["contra"].size <= 2


def test_the_watch_re_maps_on_entering_the_saved_map_s_unknown(bridge_mod, monkeypatch, tmp_path):
    b, _, _ = _matched(bridge_mod, monkeypatch, tmp_path, here=(0.5, 0.5, 0.0))
    b._remap_saved = remap.SavedMap(_grid_with(unknown=[(2.5, 4.0, -1.0, 2.0)]))
    asked = []
    monkeypatch.setattr(b, "remap", lambda trigger, detail=None: asked.append(trigger) or (200, {}))
    b.remap_step(None)
    assert asked == []
    monkeypatch.setattr(b, "_fresh_pose",
                        lambda: {"x_m": 3.0, "y_m": 0.5, "yaw_rad": 0.0, "age_s": 0.0})
    b.remap_step(None)
    assert asked == ["entered_unknown"]


def test_the_watch_re_maps_when_the_scans_contradict(bridge_mod, monkeypatch, tmp_path):
    """Each scan laid at the laser's pose at its own capture stamp."""
    pose = (0.0, 0.0, 0.0)
    b, _, _ = _matched(bridge_mod, monkeypatch, tmp_path, here=pose)
    asked, stamps = [], []
    monkeypatch.setattr(b, "remap", lambda trigger, detail=None: asked.append((trigger, detail)) or (200, {}))

    def laser(at=None):
        stamps.append(at.nanoseconds)
        return pose
    monkeypatch.setattr(b, "_laser_in_map", laser)
    seen = None
    for k in range(120):
        b.last_scan = {**_scan(*pose, boxes=[BOX]), "stamp_ns": k * 10**8, "at": time.time()}
        seen = b.remap_step(seen)
        if asked:
            break
    assert asked and asked[0][0] == "contradiction" and asked[0][1]["cells"] >= remap.MIN_GROUP
    assert stamps[:3] == [0, 10**8, 2 * 10**8]
    assert b.remap_step(seen) == seen                   # the same scan is not judged twice


def test_a_scan_whose_tf_is_late_is_retried_then_dropped(bridge_mod, monkeypatch, tmp_path):
    b, _, _ = _matched(bridge_mod, monkeypatch, tmp_path, here=(0.0, 0.0, 0.0))
    monkeypatch.setattr(b, "_laser_in_map", lambda at=None: None)
    b.last_scan = {**_scan(0.0, 0.0, 0.0), "stamp_ns": 5, "at": time.time()}
    assert b.remap_step(None) is None                   # not yet: try again
    b.last_scan["at"] -= 1.0
    assert b.remap_step(None) == 5                      # too old: given up


def test_no_watch_without_a_matched_reload(bridge_mod, monkeypatch, tmp_path):
    b, fake, _ = _reload_bridge(bridge_mod, monkeypatch, tmp_path, None)
    monkeypatch.setattr(bridge_mod, "MAP_RELOAD", False)
    b.start_slam()
    assert b._remap_saved is None and b.health()[1]["slam"]["watch"] is None
    assert b.remap_step("x") == "x"


def test_the_detector_has_a_hold_time():
    """Amendment 1: a contradiction must last HOLD_S, not only MIN_LOOKS scans."""
    pose = (0.0, 0.0, 0.0)
    fired, _, t = _run(_room_grid(), lambda t: (_scan(*pose, boxes=[BOX]), pose))
    assert fired is not None and t >= remap.HOLD_S


# ---- 3.50.D4: a person's request, through the robot server ----

import httpx  # noqa: E402


def _ros_world(handler):
    from world.ros_world import RosWorld
    return RosWorld("http://bridge", client=httpx.Client(base_url="http://bridge",
                                                         transport=httpx.MockTransport(handler)))


def test_the_world_asks_the_bridge_and_reads_its_refusal_as_an_answer():
    seen = []

    def handler(request):
        seen.append((request.method, request.url.path, json.loads(request.content)))
        if len(seen) == 1:
            return httpx.Response(200, json={"remapping": True, "trigger": "request"})
        return httpx.Response(409, json={"error": "re-map only from a reloaded map",
                                         "slam": "ready", "mode": "mapping"})
    w = _ros_world(handler)
    assert w.remap(by="twin-dpad") == {"remapping": True, "trigger": "request"}
    assert seen[0] == ("POST", "/slam/remap", {"by": "twin-dpad"})
    no = w.remap()
    assert no["remapping"] is False and no["mode"] == "mapping" and "reloaded" in no["reason"]


def test_a_bridge_error_is_not_read_as_a_refusal():
    w = _ros_world(lambda request: httpx.Response(401, json={"error": "secret"}))
    with pytest.raises(httpx.HTTPStatusError):
        w.remap()


def _server_with(monkeypatch, world):
    import robot.server as server
    from fastapi.testclient import TestClient
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    monkeypatch.setattr(server, "get_world", lambda *a, **kw: world)
    return TestClient(server.create_app())


class _RemapWorld:
    def __init__(self, reply=None, error=None):
        self.reply, self.error, self.calls = reply, error, []

    def remap(self, by=""):
        self.calls.append(by)
        if self.error:
            raise self.error
        return self.reply

    def __getattr__(self, name):          # the rest of a world, unused here
        raise AttributeError(name)


def test_the_robot_server_route(monkeypatch):
    w = _RemapWorld({"remapping": True, "trigger": "request"})
    client = _server_with(monkeypatch, w)
    r = client.post("/world/remap", headers={"x-driver": "twin-dpad"})
    assert r.status_code == 200 and r.json()["remapping"] is True and w.calls == ["twin-dpad"]
    w.reply = {"remapping": False, "reason": "not localising"}
    r = client.post("/world/remap")
    assert r.status_code == 409 and "not localising" in r.json()["detail"] and w.calls[-1] == "unnamed"
    w.error = httpx.ConnectError("down")
    assert client.post("/world/remap").status_code == 503


def test_a_world_without_slam_cannot_re_map(monkeypatch):
    import robot.server as server
    from fastapi.testclient import TestClient
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    r = TestClient(server.create_app()).post("/world/remap")       # the plain simulator
    assert r.status_code == 501


def test_a_trigger_that_cannot_start_a_re_map_backs_off(bridge_mod, monkeypatch, tmp_path):
    """Thermos (low): a refused start must not be retried on every look."""
    b, _, _ = _matched(bridge_mod, monkeypatch, tmp_path, here=(3.0, 0.5, 0.0))
    b._remap_saved = remap.SavedMap(_grid_with(unknown=[(2.5, 4.0, -1.0, 2.0)]))
    asked = []
    monkeypatch.setattr(b, "remap", lambda trigger, detail=None: asked.append(trigger) or (503, {}))
    for _ in range(5):
        b.remap_step(None)
    assert asked == ["entered_unknown"]
    b._remap_retry_at -= bridge_mod.REMAP_RETRY_S
    b.remap_step(None)
    assert len(asked) == 2


def test_a_mission_end_during_the_swap_saves_once_it_is_mapping(bridge_mod, monkeypatch, tmp_path):
    """Thermos (low): the re-map session's cells are not lost to a skip."""
    b, live, store = _matched(bridge_mod, monkeypatch, tmp_path)
    b.serialize_cli = _Serialise()
    monkeypatch.setattr(bridge_mod, "SerializePoseGraph", types.SimpleNamespace(
        Request=types.SimpleNamespace, Response=types.SimpleNamespace(RESULT_SUCCESS=0)))
    gate = __import__("threading").Event()
    real_start = live.start

    def slow_start(*a, **kw):
        gate.wait(2.0)
        return real_start(*a, **kw)
    monkeypatch.setattr(live, "start", slow_start)
    assert b.remap("request")[0] == 200
    b._save_map("mission_end")                          # during the swap
    assert store.versions("map-0a1b2c3d") == [1]
    gate.set()
    _settle(b)
    end = time.monotonic() + 2.0
    while store.versions("map-0a1b2c3d") == [1] and time.monotonic() < end:
        time.sleep(0.01)
    assert store.versions("map-0a1b2c3d") == [1, 2]


# ---- 3.50.D4 review round 2 ----

def test_lost_is_judged_as_relocalise_judges_a_reload():
    """Review: the same share on the same scans, so a reload that matched is
    not read as lost by a stricter measure."""
    g = _room_grid()
    pad = 20                                   # 1 m of free cells round the room:
    w, h = g["width"] + 2 * pad, g["height"] + 2 * pad   # every return lands on the grid
    data = [0] * (w * h)
    for r in range(g["height"]):
        data[(r + pad) * w + pad:(r + pad) * w + pad + g["width"]] = g["data"][r * g["width"]:(r + 1) * g["width"]]
    grid = {**g, "width": w, "height": h, "data": data,
            "origin_x_m": g["origin_x_m"] - pad * RES, "origin_y_m": g["origin_y_m"] - pad * RES}
    saved = remap.SavedMap(grid)
    for real, believed in (((0.5, 0.4, 0.2), (0.5, 0.4, 0.2)),
                           ((0.5, 0.4, 0.2), (0.57, 0.46, 0.23))):
        scan = _scan(*real)
        a = remap.judge_scan(saved, scan, believed)["agreement"]
        b = relocalise.scan_agreement(scan, believed, grid)
        assert a == pytest.approx(b, abs=1e-9), (real, believed)
    # Slipped, some returns land off even the padded grid (unknown to the
    # watch, a miss to relocalise): the shares differ a little, the verdict not.
    scan = _scan(1.5, 0.4, math.radians(30))
    a = remap.judge_scan(saved, scan, (0.5, 0.4, 0.0))["agreement"]
    b = relocalise.scan_agreement(scan, (0.5, 0.4, 0.0), grid)
    assert a < remap.LOST_BELOW and b < relocalise.AGREE_MIN


def test_a_goal_registered_after_a_re_map_began_is_refused(bridge_mod, monkeypatch, tmp_path):
    b, _, _ = _matched(bridge_mod, monkeypatch, tmp_path)
    from tests.test_review_376h1 import _Nav
    b.nav = _Nav()

    def wait(timeout_sec):              # the re-map begins while nav2 is awaited
        b.slam_state = "remapping"
        return True
    b.nav.wait_for_server = wait
    code, body = b.send_goal({"x_m": 1.0, "y_m": 0.0})
    assert code == 503 and body["slam"] == "remapping" and b.nav.futures == []


def test_a_robot_that_never_rests_calls_the_swap_off(bridge_mod, monkeypatch, tmp_path):
    monkeypatch.setattr(bridge_mod, "REMAP_DEFER_MAX_S", 0.2)
    monkeypatch.setattr(bridge_mod, "REMAP_REST_WAIT_S", 0.2)
    b, live, _ = _matched(bridge_mod, monkeypatch, tmp_path)
    b._rest_since = None                                   # a person driving
    monkeypatch.setattr(b, "cancel_goal", lambda why="cancelled": False)
    assert b.remap("contradiction", {"cells": 12})[0] == 200
    _settle(b)
    assert live.mode == "localisation" and b.slam_state == "ready"
    assert b.health()[1]["slam"]["remap"]["result"] == "deferred"
    assert b._remap_retry_at > time.monotonic()


def test_rest_needs_fresh_odometry_and_no_goal_still_cancelling(bridge_mod, monkeypatch, tmp_path):
    b, _, _ = _matched(bridge_mod, monkeypatch, tmp_path)
    b.last_odom = {"at": time.time()}
    now = time.monotonic()
    assert b._at_rest(now)
    b.last_odom = {"at": time.time() - 2.0}                # ros2_control hung
    assert not b._at_rest(now)
    b.last_odom = {"at": time.time()}
    other = {"state": "cancelled"}
    b.goal = {"state": "cancelled"}
    b.cancelling = {1: {"record": other, "handle": None, "since": 0, "last": 0}}
    assert not b._at_rest(now)                             # superseded, still driving


def test_a_failed_re_map_latches_the_automatic_triggers(bridge_mod, monkeypatch, tmp_path):
    """Both reviewers: a graph the mapping node cannot load must not loop."""
    monkeypatch.setitem(bridge_mod._test_ok, "value", True)
    b, live, _ = _matched(bridge_mod, monkeypatch, tmp_path, here=(3.0, 0.5, 0.0))
    b.last_odom = {"at": time.time()}
    monkeypatch.setattr(b, "_fresh_pose", lambda: None if live.mode == "mapping" else
                        {"x_m": 3.0, "y_m": 0.5, "yaw_rad": 0.0, "age_s": 0.0})
    monkeypatch.setattr(live, "alive", lambda: live.mode != "mapping")
    b._remap_saved = remap.SavedMap(_grid_with(unknown=[(2.5, 4.0, -1.0, 2.0)]))
    b.remap_step(None)
    _settle(b)
    assert b.health()[1]["slam"]["remap"]["result"] == "failed_localising"
    starts = len(live.calls)
    for _ in range(5):
        b.remap_step(None)                                  # still in unknown
    assert len(live.calls) == starts and b.slam_state == "ready"
    assert b.remap("request")[0] == 200                     # a person may still ask


def test_a_save_lost_to_a_failed_re_map_is_counted(bridge_mod, monkeypatch, tmp_path):
    monkeypatch.setitem(bridge_mod._test_ok, "value", True)
    b, live, _ = _matched(bridge_mod, monkeypatch, tmp_path)
    b.last_odom = {"at": time.time()}
    monkeypatch.setattr(b, "_fresh_pose", lambda: None)
    monkeypatch.setattr(live, "alive", lambda: False)
    monkeypatch.setattr(b, "_await_rest", lambda: (b._save_map("mission_end"), {"at_rest": True})[1])
    assert b.remap("request")[0] == 200
    _settle(b)
    assert b.slam_state == "failed" and b.map_saves["skipped"] == 1 and b._save_after_remap is None


# ---- 3.50.D4 review round 3 ----

def test_a_deferral_backs_off_longer_and_is_counted(bridge_mod, monkeypatch, tmp_path):
    monkeypatch.setattr(bridge_mod, "REMAP_DEFER_MAX_S", 0.1)
    monkeypatch.setattr(bridge_mod, "REMAP_REST_WAIT_S", 0.1)
    b, live, _ = _matched(bridge_mod, monkeypatch, tmp_path)
    b._rest_since = None
    monkeypatch.setattr(b, "cancel_goal", lambda why="cancelled": False)
    assert b.remap("contradiction", {"cells": 12})[0] == 200
    _settle(b)
    h = b.health()[1]["slam"]
    assert h["remap"]["result"] == "deferred" and h["remap_deferrals"] == 1
    assert b._remap_retry_at - time.monotonic() > bridge_mod.REMAP_RETRY_S + 1


def test_rest_reached_in_the_wait_is_rest(bridge_mod, monkeypatch, tmp_path):
    """Review: the check that ended the wait decides, not a later one."""
    monkeypatch.setattr(bridge_mod, "REMAP_DEFER_MAX_S", 0.05)
    b, _, _ = _matched(bridge_mod, monkeypatch, tmp_path)
    calls = iter([False] * 3 + [True] + [False] * 100)
    monkeypatch.setattr(b, "_at_rest", lambda now: next(calls))
    monkeypatch.setattr(b, "cancel_goal", lambda why="cancelled": True)
    assert b._await_rest()["at_rest"] is True


def test_the_latch_shows_on_health(bridge_mod, monkeypatch, tmp_path):
    b, _, _ = _matched(bridge_mod, monkeypatch, tmp_path)
    assert b.health()[1]["slam"]["remap_latched"] is False
    b._remap_latched = True
    assert b.health()[1]["slam"]["remap_latched"] is True


def test_the_fall_back_starts_from_the_localisation_node_s_frame(bridge_mod, monkeypatch, tmp_path):
    """Review: not from a map -> odom the failed mapping node published."""
    monkeypatch.setitem(bridge_mod._test_ok, "value", True)
    b, live, store = _matched(bridge_mod, monkeypatch, tmp_path)
    frames = iter([(1.0, 2.0, 0.0), (9.0, 9.0, 0.0)])          # localisation's, then mapping's
    monkeypatch.setattr(b, "_map_to_odom", lambda: next(frames))
    seen = []
    monkeypatch.setattr(b, "_pose_now", lambda mo=None: seen.append(mo) or (mo[0], mo[1], 0.0))
    monkeypatch.setattr(b, "_fresh_pose", lambda: None if live.mode == "mapping" else
                        {"x_m": 1.0, "y_m": 2.0, "yaw_rad": 0.0, "age_s": 0.0})
    monkeypatch.setattr(live, "alive", lambda: live.mode != "mapping")
    assert b.remap("request")[0] == 200
    _settle(b)
    assert live.calls[-1] == ("localisation", store.current().prefix, (1.0, 2.0, 0.0))
    assert all(mo == (1.0, 2.0, 0.0) for mo in seen)


def test_a_fall_back_frees_the_reload_map(bridge_mod, monkeypatch, tmp_path):
    b, fake, _ = _reload_bridge(bridge_mod, monkeypatch, tmp_path,
                                relocalise.judge([0.2] * 5, (0.0, 0.0, 0.0)))
    b._reload_map = {"data": [0]}
    b.start_slam()
    assert b._reload_map is None


# ---- 3.50.D4 review round 5 ----

def test_the_re_map_record_says_when_mapping_began(bridge_mod, monkeypatch, tmp_path):
    b, _, _ = _matched(bridge_mod, monkeypatch, tmp_path)
    t0 = time.time()
    assert b.remap("request")[0] == 200
    _settle(b)
    r = b.health()[1]["slam"]["remap"]
    assert r["result"] == "mapping" and t0 <= r["mapping_at"] <= time.time()


def test_the_bridge_can_bind_loopback_only():
    """Review: under host networking a laptop run must not be on the LAN."""
    with open(os.path.join(ROOT, "service/slam/src/picar_bridge/picar_bridge/bridge.py")) as f:
        src = f.read()
    assert 'os.environ.get("BRIDGE_HOST", "0.0.0.0")' in src
    assert "ThreadingHTTPServer((host, port)" in src
