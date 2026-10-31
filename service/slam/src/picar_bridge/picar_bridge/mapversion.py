"""mapversion -- when the bridge's map_version moves (3.124).

slam_toolbox publishes /map every `map_update_interval` whether or not a
cell changed; 44% of a live mission's messages repeated the previous grid
exactly. `map_version` promises to bump whenever the cells change
(world/interface.py), and the robot server's ETag (3.123) and every cache
keyed on it are only as good as that promise, so a repeat keeps the version.

Plain Python, kept out of bridge.py (which imports rclpy), so it is tested
on a laptop (tests/test_bridge_map_version.py).
"""

# Everything a map reply carries that places or fills its cells. `version`
# and `at` are not the grid.
GRID_FIELDS = ("session", "resolution_m", "width", "height",
               "origin_x_m", "origin_y_m", "origin_yaw_rad", "data")


def next_version(held, version: int, new: dict) -> int:
    """The version `new` is published under. The same as `version` only if
    a map is held and `new` equals it field for field (exact: a float that
    moved by an ulp is a change); otherwise one more. No held map -- the
    bridge clears it at every reload, fresh start and re-map -- always
    bumps, so nothing cached across the swap is read as current."""
    if held is not None and all(held.get(k) == new.get(k) for k in GRID_FIELDS):
        return version
    return version + 1


def map_tag(m) -> str | None:
    """3.126: the ETag of a map reply, `"<session>:<version>"`, or None for
    no usable map. Within one bridge the pair names one grid (a session is
    a fresh id each start; versions only rise within it, and only when the
    grid changes)."""
    if not m or not m.get("data") or m.get("session") is None or m.get("version") is None:
        return None
    return f'"{m["session"]}:{m["version"]}"'


def not_modified(header, tag) -> bool:
    """Whether a request's If-None-Match names `tag` (weak tags as strong,
    lists) or is `*`, which any current map matches -- the robot server's
    `_not_modified` rules (robot/server.py, 3.123), pinned together by
    tests/test_bridge_map_304.py."""
    if not header or not tag:
        return False
    tags = {t.strip().removeprefix("W/") for t in header.split(",")}
    return tag in tags or "*" in tags
