"""mapstore -- the robot's saved maps on disk, as plain Python (3.50.D).

slam_toolbox's serialize_map writes `<prefix>.posegraph` and `<prefix>.data`
for a filename prefix; this keeps them as numbered versions of one map:

    <root>/current.json                    {"map_id", "version"}: what a reload loads
    <root>/<map_id>/v<N>/graph.posegraph   slam_toolbox's files
    <root>/<map_id>/v<N>/graph.data
    <root>/<map_id>/v<N>/meta.json         map_id, version, dock_pose, start_truth, ...
    <root>/<map_id>/.staging-<N>-<id>/     a save in progress; never offered

A version becomes visible only by a rename of its whole directory, and the
pointer only by os.replace, so a crash mid-save leaves the previous version
current (3.50 "versioned saves that can be rolled back"). Three versions
are kept (3.50's default), and the three newest maps. `current()` offers only a version this store
committed: criterion 12's "map_file_name is only ever a file the robot
saved". Kept out of bridge.py, which imports rclpy, so it is tested on a
laptop (tests/test_map_persist.py).
"""

import json
import os
import re
import shutil
import time
import uuid
from dataclasses import dataclass
from typing import Optional

GRAPH_FILES = (".posegraph", ".data")
GRAPH_NAME = "graph"
META = "meta.json"
CURRENT = "current.json"
KEEP = 3
# Whole maps kept (3.50.D2 review): every container start that does not
# reload is a new map_id, ~7 MB a version; the current one is never removed.
KEEP_MAPS = 3
# Older than any serialize (the bridge gives up after 30 s): a staging no
# save of this process owns is only removed once a writer cannot be in it.
STALE_STAGING_S = 600
MAP_ID_RE = re.compile(r"map-[0-9a-f]{8}")
# The dock is the pose of the graph's first node: session 1 began there,
# so it is the map frame's origin. Localisation mode cannot start "at the
# first node" (3.50.D1), so a reload asks for this pose by value.
DOCK_POSE = {"x": 0.0, "y": 0.0, "theta": 0.0}


class SaveFailed(Exception):
    pass


def new_map_id() -> str:
    return f"map-{uuid.uuid4().hex[:8]}"


@dataclass(frozen=True)
class Pending:
    map_id: str
    version: int
    staging: str

    @property
    def prefix(self) -> str:
        """The filename to hand slam_toolbox's serialize_map."""
        return os.path.join(self.staging, GRAPH_NAME)


@dataclass(frozen=True)
class Saved:
    map_id: str
    version: int
    meta: dict
    prefix: str        # for deserialize_map


def _fsync_write(path: str, data: str) -> None:
    with open(path, "w") as f:
        f.write(data)
        f.flush()
        os.fsync(f.fileno())


def _age(path: str) -> float:
    try:
        return time.time() - os.path.getmtime(path)
    except OSError:
        return 0.0


def _fsync_dir(path: str) -> None:
    fd = os.open(path, os.O_RDONLY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


class MapStore:
    def __init__(self, root: str, keep: int = KEEP, keep_maps: int = KEEP_MAPS):
        self.root = root
        self.keep = keep
        self.keep_maps = keep_maps
        # Staging this store has begun and not yet finished. Any other one is
        # a crashed save's, and pruned -- by name, not by pid: a container's
        # pids repeat at every start.
        self._live = set()

    def _map_dir(self, map_id: str) -> str:
        if not isinstance(map_id, str) or not MAP_ID_RE.fullmatch(map_id):
            raise ValueError(f"bad map_id {map_id!r}")
        return os.path.join(self.root, map_id)

    def versions(self, map_id: str) -> list:
        d = self._map_dir(map_id)
        if not os.path.isdir(d):
            return []
        return sorted(int(n[1:]) for n in os.listdir(d) if re.fullmatch(r"v\d+", n))

    def begin(self, map_id: str) -> Pending:
        """A staging directory for the next version; slam_toolbox writes
        into `pending.prefix`, then `commit()`."""
        d = self._map_dir(map_id)
        os.makedirs(d, exist_ok=True)
        have = self.versions(map_id)
        version = (have[-1] if have else 0) + 1
        staging = os.path.join(d, f".staging-{version}-{uuid.uuid4().hex[:8]}")
        os.makedirs(staging)
        self._live.add(staging)
        return Pending(map_id, version, staging)

    def commit(self, pending: Pending, *, session: str, start_truth: Optional[dict]) -> dict:
        """Make the staged files version N and the current map. Raises
        SaveFailed (and removes the staging) if slam_toolbox's files are
        missing or empty. `start_truth` is kept from the map's first save:
        SLAM's frame is where session 1 began, whatever this session's was."""
        try:
            sizes = {}
            for ext in GRAPH_FILES:
                p = pending.prefix + ext
                if not os.path.isfile(p) or os.path.getsize(p) == 0:
                    raise SaveFailed(f"slam_toolbox wrote no {GRAPH_NAME}{ext}")
                sizes[GRAPH_NAME + ext] = os.path.getsize(p)
            earlier = self._meta_of(pending.map_id, self.versions(pending.map_id))
            first_session = earlier["first_session"] if earlier else session
            kept_start = earlier["start_truth"] if earlier else start_truth
            if kept_start is None and session == first_session:
                # A first save before the start truth arrived: the same
                # session's start is the map frame's, so it fills it in. A
                # later session's start is another place and never does.
                kept_start = start_truth
            meta = {
                "map_id": pending.map_id, "version": pending.version,
                "saved_at": time.time(), "session": session,
                "first_session": first_session,
                "dock_pose": dict(DOCK_POSE),
                # Evaluation only, as the bridge's own start_truth: read by
                # /world/error's anchor, never by the estimate (criterion 12).
                "start_truth": kept_start,
                "files": sizes,
            }
            _fsync_write(os.path.join(pending.staging, META), json.dumps(meta, indent=1))
            for ext in GRAPH_FILES:
                with open(pending.prefix + ext, "rb") as f:
                    os.fsync(f.fileno())
            final = os.path.join(self._map_dir(pending.map_id), f"v{pending.version}")
            if os.path.exists(final):
                raise SaveFailed(f"{final} already exists (two saves at once?)")
            _fsync_dir(pending.staging)          # its entries, before it is named v<N>
            os.rename(pending.staging, final)
            _fsync_dir(os.path.dirname(final))
        except BaseException:
            self.abandon(pending)
            raise
        self._live.discard(pending.staging)
        try:
            self._point_at(pending.map_id, pending.version)
        except OSError as e:
            raise SaveFailed(f"v{pending.version} is on disk but current.json was not "
                             f"moved to it: {e}") from e
        try:
            self._prune(pending.map_id)
        except OSError:
            pass                 # old versions left: the next save prunes again
        return meta

    def abandon(self, pending: Pending, remove: bool = True) -> None:
        """A save that never reached commit(): drop its staging -- or, with
        `remove=False` (a writer may still be in it), leave it for the next
        commit's prune."""
        if remove:
            shutil.rmtree(pending.staging, ignore_errors=True)
        self._live.discard(pending.staging)

    def _meta_of(self, map_id: str, versions: list) -> Optional[dict]:
        """The oldest kept version's meta: the facts fixed at the first save.
        Its start_truth may have been None (saved before it arrived) and
        filled by a later version of the same first session, so the first
        non-None start_truth of any kept version is carried."""
        oldest = None
        for v in versions:
            try:
                with open(os.path.join(self._map_dir(map_id), f"v{v}", META)) as f:
                    meta = json.load(f)
            except (OSError, ValueError):
                continue
            # A meta this store did not write whole is skipped, never trusted:
            # one bad file must not fail every later save of the map.
            if (isinstance(meta, dict) and meta.get("map_id") == map_id
                    and isinstance(meta.get("first_session"), str) and "start_truth" in meta):
                if oldest is None:
                    oldest = dict(meta)
                if oldest["start_truth"] is None and meta["start_truth"] is not None \
                        and meta["first_session"] == oldest["first_session"]:
                    oldest["start_truth"] = meta["start_truth"]
        return oldest

    def _point_at(self, map_id: str, version: int) -> None:
        # Not by pid: a container's pids repeat, and two stacks may share a volume.
        tmp = os.path.join(self.root, f".{CURRENT}.{uuid.uuid4().hex[:8]}.tmp")
        try:
            _fsync_write(tmp, json.dumps({"map_id": map_id, "version": version}))
            os.replace(tmp, os.path.join(self.root, CURRENT))
        except BaseException:
            try:
                os.remove(tmp)
            except OSError:
                pass
            raise
        try:
            _fsync_dir(self.root)
        except OSError:
            pass             # the pointer has moved; only its durability is weaker

    def _prune(self, map_id: str) -> None:
        have = self.versions(map_id)
        for v in have[:-self.keep]:
            shutil.rmtree(os.path.join(self._map_dir(map_id), f"v{v}"), ignore_errors=True)
        d = self._map_dir(map_id)
        for n in os.listdir(d):
            p = os.path.join(d, n)
            # A staging a timed-out save left may still have slam_toolbox
            # writing in it: removed only once it is STALE_STAGING_S old.
            if n.startswith(".staging-") and p not in self._live and _age(p) > STALE_STAGING_S:
                shutil.rmtree(p, ignore_errors=True)
        maps = sorted((n for n in os.listdir(self.root) if MAP_ID_RE.fullmatch(n) and n != map_id),
                      key=lambda n: os.path.getmtime(os.path.join(self.root, n)), reverse=True)
        for n in maps[self.keep_maps - 1:]:
            shutil.rmtree(os.path.join(self.root, n), ignore_errors=True)

    def current(self) -> Optional[Saved]:
        """The map a reload loads, or None. Only a version this store
        committed, with its files and a meta that names it, is offered."""
        try:
            with open(os.path.join(self.root, CURRENT)) as f:
                ptr = json.load(f)
            map_id, version = ptr["map_id"], ptr["version"]
            if not isinstance(version, int) or isinstance(version, bool):
                return None
            vdir = os.path.join(self._map_dir(map_id), f"v{version}")
            with open(os.path.join(vdir, META)) as f:
                meta = json.load(f)
        except (OSError, ValueError, KeyError, TypeError):
            return None
        if meta.get("map_id") != map_id or meta.get("version") != version:
            return None
        prefix = os.path.join(vdir, GRAPH_NAME)
        if not all(os.path.isfile(prefix + ext) for ext in GRAPH_FILES):
            return None
        return Saved(map_id, version, meta, prefix)
