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

from picar_bridge import mapstore  # noqa: E402
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
    MapStore(str(tmp_path)).begin("map-0a1b2c3d")          # the crashed process
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


def test_slam_runs_the_node_that_switches_modes():
    """3.50.D1: D localises by default and maps for a re-map or a save, on
    one graph, without a restart -- only map_and_localization does that."""
    with open(os.path.join(BRINGUP, "launch/picar.launch.py")) as f:
        launch = f.read()
    assert 'executable="map_and_localization_slam_toolbox_node"' in launch
    assert "async_slam_toolbox_node" not in launch


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
    _save(store, "map-0a1b2c3d")
    assert not os.path.isdir(pending.staging)
