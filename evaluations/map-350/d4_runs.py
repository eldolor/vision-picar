"""3.50.D4: the re-map triggers, measured on the criteria written in the plan
("3.50.D4 criteria", amendments 1-3) before this ran.

    eval "$(python tools/plan_section.py env)"
    python evaluations/map-350/d4_runs.py [a b c d] [scaled_house home_first_floor]

  a  a closed room added (criterion 11): run 1 maps with the room's only
     door closed; run 2 reloads with it open and drives a goal inside; the
     re-map's start against the robot's centre entering the saved unknown;
     the room's floor known free on /map and in run 2's save (a reload);
     other rooms' walls; jumps
  b  a moved object heals (criterion 3): one object in a doorway's middle in
     run 1; run 2 waits in view of it for the contradiction trigger, then
     three passes through the door
  c  no permanent wrong closure (criterion 5): five sessions in
     home_first_floor, each re-mapped at its start by a person's request,
     an explore mission each
  d  no false contradiction triggers: three no-change reloads per house,
     600 s watched each (explore, then a room tour; amendment 3)

Every run records contacts (3.18's truth, as tests/demo_explore.run_mission)
and every re-map record the bridge kept (D4-f). The maps are a floor plan of
the SIM house, kept under EXPLORE_LOGDIR, never committed (only numbers are).
Raw output: evaluations/map-350/D4/.
"""
import importlib.util
import json
import math
import os
import shutil
import sys
import threading
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, ROOT)

import httpx  # noqa: E402

from sim.grid_world import CELL_WALL  # noqa: E402
from sim.maps import build_world  # noqa: E402
from tests import demo_explore as de  # noqa: E402
from tests.demo_slam_home import score_map  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("d3_runs", os.path.join(HERE, "d3_runs.py"))
d3 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(d3)

OUT = os.path.join(HERE, "D4")
CELL = 0.3
ABSENT = d3.ABSENT
MAP_RUN_S = d3.MAP_RUN_S
JUMP_M, JUMP_WINDOW_S = 0.5, 2.0          # 3.38's metric at criterion 5's 0.5 m
WATCH_S = 600                             # D4-d
REMAP_WITHIN_S = 10.0                     # D4-a
VIEW_S = 30.0                             # D4-a/b: at rest in view of the door (amendments 3-4)
MOVERS = {"scaled_house": "hallway_crossing", "home_first_floor": ""}
# Amendment 3's doors (cells): the room closed for D4-a, the door for D4-b.
CLOSED_ROOM = {"scaled_house": ("study", [(16, 9), (16, 10), (16, 11)]),
               "home_first_floor": ("den", [(10, 16), (11, 16)])}
DOOR_B = {"scaled_house": (16, 4)}        # home_first_floor's: found from the layout


def _env_movers(movers):
    os.environ["SIM_MOVERS"] = movers


def servers(house, movers=""):
    _env_movers(movers)
    try:
        return d3.servers(house)
    finally:
        _env_movers("")


def health(bridge):
    return bridge.get("/health").json()


def remap_rec(bridge):
    return health(bridge)["slam"].get("remap") or {}


# ---------- sampling through a session: truth error, contacts, re-maps ----------

class Sampler:
    """1 Hz: /world/error (jumps), the robot's truth against walls and
    objects (contacts, as run_mission), the bridge's re-map record."""

    def __init__(self, robot, bridge, house):
        self.robot, self.bridge = robot, bridge
        w = build_world(house)
        self.walls = {(x, y) for y, row in enumerate(w.layout) for x, c in enumerate(row)
                      if c == CELL_WALL}
        self.series, self.remaps, self.contacts, self.min_gap = [], [], 0, math.inf
        self.states = []
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)

    def __enter__(self):
        self._t.start()
        return self

    def __exit__(self, *exc):
        self._stop.set()
        self._t.join(5)

    def _run(self):
        robot = httpx.Client(base_url=str(self.robot.base_url), timeout=10)
        bridge = httpx.Client(base_url=str(self.bridge.base_url), timeout=10)
        while not self._stop.is_set():
            try:
                t = time.time()
                e = robot.get("/world/error").json()
                if e.get("usable"):
                    self.series.append((t, e["position_error_m"], e["heading_error_deg"]))
                tr = robot.get("/world/truth").json()
                objs = {(o["x"], o["y"]) for o in robot.get("/sim/objects").json()["objects"]}
                x, y = tr["x_m"] / CELL, tr["y_m"] / CELL
                A = de.chassis(x, y, math.radians(tr["heading_deg"]) - math.pi / 2)
                for c in self.walls | objs:
                    if abs(c[0] + 0.5 - x) > 1.6 or abs(c[1] + 0.5 - y) > 1.6:
                        continue
                    g = de.gap(A, de.square(*c)) * de.CELL_CM
                    self.min_gap = min(self.min_gap, g)
                    self.contacts += g < 1.0
                h = bridge.get("/health").json()["slam"]
                t = time.time()                  # the state as of now, not the loop's start
                r = h.get("remap") or {}
                if r.get("at") and (not self.remaps or self.remaps[-1].get("at") != r["at"]
                                    or self.remaps[-1].get("result") != r.get("result")):
                    self.remaps.append(dict(r))
                if not self.states or self.states[-1][0] != (h["state"], h["mode"]):
                    self.states.append(((h["state"], h["mode"]), round(t, 2)))
            except Exception:  # noqa: BLE001 -- a restart in progress
                pass
            self._stop.wait(1.0)

    def jumps(self):
        s = self.series
        return [(round(b[0] - s[0][0]), round(b[1] - a[1], 3)) for a, b in zip(s, s[1:])
                if b[0] - a[0] <= JUMP_WINDOW_S and b[1] - a[1] > JUMP_M]

    def summary(self):
        return {"contacts": self.contacts, "min_gap_cm": round(self.min_gap, 1),
                "jumps": self.jumps(), "error_samples": len(self.series),
                "max_error_m": round(max((x[1] for x in self.series), default=0), 3),
                "remaps": self.remaps, "states": self.states}


# ---------- goals ----------

def goal(robot, x_m, y_m, timeout_s=120.0, retry_s=30.0):
    """A nav2 goal in the house frame; refusals (a re-map holds goals) are
    retried for `retry_s`, and so is nav2's own rejection: right after a
    reload the bridge is `ready` before nav2 is active, and nav2 rejects the
    goal at once (the cloud run, 2026-10-24: both D4-b view goals rejected
    at 0.0 s, so the robot never stopped in view). Returns the terminal
    state."""
    t0 = time.time()
    while True:
        r = robot.post("/world/goal", json={"x_m": x_m, "y_m": y_m})
        ok = r.status_code == 200 and r.json().get("accepted")
        if ok:
            t1, state = time.time(), "pending"
            while time.time() - t1 < timeout_s:
                g = robot.get("/world/goal").json()
                state = (g.get("goal") or {}).get("state", "none")
                if state in ("succeeded", "aborted", "canceled", "rejected"):
                    break
                time.sleep(0.3)
            else:
                robot.delete("/world/goal")
            if state != "rejected" or time.time() - t0 > retry_s:
                return {"state": state, "seconds": round(time.time() - t1, 1)}
        elif time.time() - t0 > retry_s:
            return {"state": "not_sent", "reply": r.text[:200]}
        time.sleep(1.0)


def cell_m(c):
    return ((c[0] + 0.5) * CELL, (c[1] + 0.5) * CELL)


def end_mission(brain, bridge):
    """A save needs a mission's end: a short frontier mission, stopped."""
    before = health(bridge)["map"]["saved"]
    brain.post("/mission/start", json={"target_object": ABSENT, "policy": "frontier"})
    time.sleep(1.5)
    brain.post("/mission/stop")
    t = time.time()
    while time.time() - t < 30 and health(bridge)["map"]["saved"] <= before:
        time.sleep(0.3)
    return health(bridge)["map"]


def map_run(house, maps, moves=(), movers=""):
    """Explore for an absent target from an empty store; v1 saved at its end."""
    os.makedirs(maps, exist_ok=True)
    bridge = httpx.Client(base_url=f"http://127.0.0.1:{de.BRIDGE}", timeout=30)
    robot, brain = servers(house, movers)
    for src, dst in moves:
        r = robot.post("/sim/objects/move", json={"src": list(src), "dst": list(dst)})
        assert r.status_code == 200, f"move {src} -> {dst}: {r.status_code} {r.text[:200]}"
    d3.container(maps, bridge)
    de.MISSION_LIMIT_S = MAP_RUN_S[house]
    with Sampler(robot, bridge, house) as s:
        mission = de.run_mission(robot, house, policy="explore", target=ABSENT, max_steps=100000)
    t = time.time()
    while time.time() - t < 30 and health(bridge)["map"]["saved"] < 1:
        time.sleep(0.3)
    h = health(bridge)["map"]
    assert h["saved"] >= 1, f"the map run saved nothing: {h}"
    return {"mission": mission, "saved": h, "sampler": s.summary()}, robot, bridge


def reload(house, maps, movers=""):
    robot, brain = servers(house, movers)
    bridge = httpx.Client(base_url=f"http://127.0.0.1:{de.BRIDGE}", timeout=30)
    w = d3.container(maps, bridge)
    rel = w["health"]["slam"]["reload"]
    return robot, brain, bridge, {"reload": rel, "watch": {k: v for k, v in w.items() if k != "health"}}


# ---------- maps in the house frame ----------

def known_free_share(robot, cells, whole=False):
    """Share of `cells` free (0) on the robot's /world/map: judged at each
    cell's centre, or with `whole` at a 5 x 5 grid of points over the middle
    0.18 m of the 0.3 m cell (the object's footprint less 6 cm at each edge,
    which a wall cell beside a jamb may reach)."""
    m = robot.get("/world/map").json()
    if not m.get("usable"):
        return None
    res, w, h = m["resolution_m"], m["width"], m["height"]
    # Inset from the cell's edges (review): a wall cell beside a jamb must
    # not fail a door cell that is clear.
    offs = [0.2, 0.35, 0.5, 0.65, 0.8] if whole else [0.5]
    free = 0
    for c in cells:
        ok = True
        for fx in offs:
            for fy in offs:
                x, y = (c[0] + fx) * CELL, (c[1] + fy) * CELL
                i = math.floor((x - m["origin_x_m"]) / res)
                j = math.floor((y - m["origin_y_m"]) / res)
                ok &= 0 <= i < w and 0 <= j < h and m["cells"][j * w + i] == 0
        free += ok
    return round(free / max(len(cells), 1), 4)


def house_map_unknown(m, x_m, y_m):
    """Is a house-frame point unknown (or off) on a /world/map record?"""
    res = m["resolution_m"]
    i = math.floor((x_m - m["origin_x_m"]) / res)
    j = math.floor((y_m - m["origin_y_m"]) / res)
    if not (0 <= i < m["width"] and 0 <= j < m["height"]):
        return True
    return m["cells"][j * m["width"] + i] == -1


def walls_true(robot, house, skip=()):
    sm = score_map(robot, house)
    if sm is None:
        return None, None
    rooms = {r: v for r, v in sm["rooms"].items() if r not in skip and r != "?"}
    return all(v[1] >= 1.0 for v in rooms.values()), rooms


# ---------- D4-a ----------

def close_moves(house, door, room):
    """Moves putting the house's own objects, nearest first, on every one of
    `door`'s cells: none from inside the room being closed or already on the
    door, so each source is free floor once moved. map_run asserts each move;
    too few objects is an error."""
    w = build_world(house)
    inside = set(w.rooms[room]) | set(door)
    objs = sorted((o for o in w.objects if o not in inside),
                  key=lambda o: min(math.dist(o, d) for d in door))
    if len(objs) < len(door):
        raise RuntimeError(f"{house}: {len(objs)} objects cannot close {len(door)} door cells")
    return list(zip(objs[:len(door)], door))


def outside(house, door, room):
    """A free cell two steps out of `room` from its door's middle cell."""
    w = build_world(house)
    free, inside = de.free_cells(w), set(w.rooms[room])
    mid = door[len(door) // 2]
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        c1, c2 = (mid[0] + dx, mid[1] + dy), (mid[0] + 2 * dx, mid[1] + 2 * dy)
        # Across the door, never along its own line (review).
        if c1 in free and c2 in free and not {c1, c2} & (inside | set(door)):
            return c2
    raise RuntimeError(f"no free cell outside {room}'s door {door}")


def run_a(house):
    room, door = CLOSED_ROOM[house]
    maps = os.path.join(de.RUN_DIR, f"maps-a-{house}")
    out = {"house": house, "room": room, "door": door}
    moves = close_moves(house, door, room)
    out["moves"] = moves
    out["map_run"], robot, bridge = map_run(house, maps, moves)
    world = build_world(house)
    reach = de.reachable_cells(world)
    room_cells = [c for c in world.rooms[room] if c in reach]
    out["room_known_free_run1"] = known_free_share(robot, room_cells)

    robot, brain, bridge, out["session2"] = reload(house, maps)   # the door open again
    if out["session2"]["reload"].get("result") != "matched":
        out["verdict"] = {"D4-a": "FAILED (reload did not match)"}
        return out
    # The map as reloaded, in the house frame; entry is judged on the truth
    # against it, on this host's clock (review: the bridge's clock and its
    # null pose during a swap are not used).
    saved_first = None
    t = time.time()
    while time.time() - t < 15 and saved_first is None:
        m = robot.get("/world/map").json()
        saved_first = m if m.get("usable") else None
        time.sleep(0.2)
    if saved_first is None:
        out["verdict"] = {"D4-a": "FAILED (no reloaded map to judge entry against)"}
        return out
    spot = de.room_spot(world, room, reach) or room_cells[len(room_cells) // 2]
    entered_at, watch_errors = [None], [0]
    stop = threading.Event()

    def watch_entry():
        with httpx.Client(base_url=str(robot.base_url), timeout=5) as c:
            while not stop.is_set() and entered_at[0] is None:
                try:
                    tr = c.get("/world/truth").json()
                    if house_map_unknown(saved_first, tr["x_m"], tr["y_m"]):
                        entered_at[0] = time.time()
                except Exception:  # noqa: BLE001 -- counted
                    watch_errors[0] += 1
                time.sleep(0.1)
    th = threading.Thread(target=watch_entry, daemon=True)
    th.start()
    with Sampler(robot, bridge, house) as s:
        # Amendment 4: the contradiction trigger judges scans taken at rest
        # within MAX_RANGE_M, and a door closed in the saved map holds nav2
        # out of the room (item 2's limit): first VIEW_S at rest outside it.
        out["to_view"] = goal(robot, *cell_m(outside(house, door, room)), timeout_s=180)
        t0 = time.time()
        while time.time() - t0 < VIEW_S and not any(r.get("result") == "mapping" for r in s.remaps):
            time.sleep(0.5)
        out["goal_in"] = goal(robot, *cell_m(spot), timeout_s=180)
        # Look round the room from its middle, then two more spots in it.
        extra = sorted(room_cells, key=lambda c: -math.dist(c, spot))[:2]
        out["goals_room"] = [goal(robot, *cell_m(c), timeout_s=120) for c in extra]
        out["room_known_free_live"] = known_free_share(robot, room_cells)
        out["saved"] = end_mission(brain, bridge)
    stop.set()
    th.join(5)
    out["sampler"] = s.summary()
    out["watch_errors"] = watch_errors[0]
    first = s.remaps[0] if s.remaps else {}
    out["first_remap"] = first
    # Entry on the truth only (review, CLAUDE.md section 6): the bridge's own
    # entered-unknown report is the trigger under test, never its judge.
    entry = entered_at[0]
    mapped = next((r for r in s.remaps if r.get("result") == "mapping"
                   and r.get("trigger") in ("entered_unknown", "contradiction")), {})
    sampled = next((t for st, t in s.states if st == ("ready", "mapping")), None)
    from_bridge = mapped.get("mapping_at") if sys.platform.startswith("linux") else None
    mapping_at = from_bridge or sampled
    out["entry_from"] = "truth" if entry else None
    out["mapping_at_from"] = "bridge" if from_bridge else ("sampler" if sampled else None)
    out["saved_first_known_cells"] = sum(1 for v in saved_first["cells"] if v != -1)
    out["entered_unknown_at"], out["mapping_at"] = entry, mapping_at
    out["remap_after_entry_s"] = (round(mapping_at - entry, 2) if mapping_at and entry else None)
    # A label only: `timely` already accepts a re-map mapping before entry.
    out["already_mapping_at_entry"] = bool(mapping_at and entry and mapping_at <= entry)
    # Run 2's save, read back through a reload.
    robot, brain, bridge, out["session3"] = reload(house, maps)
    out["room_known_free_saved"] = known_free_share(robot, room_cells)
    ok_walls, rooms = walls_true(robot, house, skip=(room,))
    out["other_walls_true"], out["walls_by_room"] = ok_walls, rooms
    # Timely: mapping within 10 s of entering, or already mapping when it
    # entered (a contradiction first: "either counts"). No entry seen is not met.
    timely = (entry is not None and mapping_at is not None
              and mapping_at - entry <= REMAP_WITHIN_S)
    out["criteria"] = {
        "remapped": any(r.get("result") == "mapping"
                        and r.get("trigger") in ("entered_unknown", "contradiction")
                        for r in s.remaps),
        "remap_within_10s": timely,
        "room_90_live": (out["room_known_free_live"] or 0) >= 0.90,
        "room_90_saved": (out["room_known_free_saved"] or 0) >= 0.90,
        "other_walls_100": bool(ok_walls),
        "no_jumps": not out["sampler"]["jumps"],
        "new_version": (out["saved"].get("last_version") or 0) >= 2,
        "contacts_0": out["sampler"]["contacts"] == 0}
    out["verdict"] = {"D4-a": "MET" if all(out["criteria"].values()) else "FAILED"}
    if entry is None and watch_errors[0]:
        out["verdict"] = {"D4-a": f"INVALID (the entry watch erred {watch_errors[0]} times "
                                  "and saw no entry)"}
    if out["to_view"].get("state") != "succeeded":
        # Amendment 4's setup never stood in view of the door (review rounds
        # 1-2): whatever happened after, it is not the run the plan wrote.
        out["verdict"] = {"D4-a": f"INVALID (the view goal ended {out['to_view'].get('state')})"}
    return out


# ---------- D4-b ----------

def door_b(house):
    if house in DOOR_B:
        return DOOR_B[house]
    w = build_world(house)
    room_of = {c: n for n, cs in w.rooms.items() for c in cs}
    edge = sorted(c for c in w.rooms["dining room"]
                  if any(room_of.get((c[0] + dx, c[1] + dy)) == "hall"
                         for dx, dy in ((1, 0), (0, 1), (-1, 0), (0, -1))))
    return edge[len(edge) // 2]


def sides(house, door):
    """Free cells two steps either side of a door cell, across it."""
    w = build_world(house)
    free = de.free_cells(w)
    for dx, dy in ((1, 0), (0, 1)):
        a, b = (door[0] - 2 * dx, door[1] - 2 * dy), (door[0] + 2 * dx, door[1] + 2 * dy)
        if a in free and b in free and (door[0] - dx, door[1] - dy) in free \
                and (door[0] + dx, door[1] + dy) in free:
            return a, b
    raise RuntimeError(f"no free sides across {door} in {house}")


def run_b(house):
    door = door_b(house)
    a_side, b_side = sides(house, door)
    maps = os.path.join(de.RUN_DIR, f"maps-b-{house}")
    w = build_world(house)
    obj = min(w.objects, key=lambda o: math.dist(o, door))
    out = {"house": house, "door": door, "object": [obj, w.objects[obj]], "sides": [a_side, b_side]}
    out["map_run"], robot, bridge = map_run(house, maps, [(obj, door)])
    robot, brain, bridge, out["session2"] = reload(house, maps)    # the object back
    if out["session2"]["reload"].get("result") != "matched":
        out["verdict"] = {"D4-b": "FAILED (reload did not match)"}
        return out
    door_cells = [door]
    with Sampler(robot, bridge, house) as s:
        out["to_view"] = goal(robot, *cell_m(a_side), timeout_s=180)
        t0 = time.time()                        # up to 30 s at rest, in view
        while time.time() - t0 < VIEW_S and not any(r.get("result") == "mapping" for r in s.remaps):
            time.sleep(0.5)
        out["remap_before_passes"] = dict(remap_rec(bridge))
        passes = []
        for k in range(3):
            target = b_side if k % 2 == 0 else a_side
            g = goal(robot, *cell_m(target), timeout_s=120)
            passes.append({"goal": g, "door_free": known_free_share(robot, door_cells, whole=True)})
        out["passes"] = passes
        out["saved"] = end_mission(brain, bridge)
    out["sampler"] = s.summary()
    first = s.remaps[0] if s.remaps else {}
    out["criteria"] = {
        "remap_by_contradiction": first.get("trigger") == "contradiction",
        "door_free_within_3": any(p["door_free"] == 1.0 for p in passes),
        "goals_3_of_3": all(p["goal"]["state"] == "succeeded" for p in passes),
        "contacts_0": out["sampler"]["contacts"] == 0}
    out["verdict"] = {"D4-b": "MET" if all(out["criteria"].values()) else "FAILED"}
    if out["to_view"].get("state") != "succeeded":
        # As D4-a: never at rest in view, the trigger under test saw nothing.
        out["verdict"] = {"D4-b": f"INVALID (the view goal ended {out['to_view'].get('state')})"}
    return out


# ---------- D4-c ----------

def run_c(house="home_first_floor", base_maps=None):
    maps = os.path.join(de.RUN_DIR, "maps-c")
    out = {"house": house, "sessions": []}
    if base_maps:
        shutil.copytree(base_maps, maps, dirs_exist_ok=True)
    else:
        out["map_run"], _, _ = map_run(house, maps)
    for i in range(5):
        robot, brain, bridge, sess = reload(house, maps)
        if sess["reload"].get("result") != "matched":
            sess["error"] = "reload did not match"
            out["sessions"].append(sess)
            continue
        with Sampler(robot, bridge, house) as s:
            r = robot.post("/world/remap", headers={"x-driver": "twin-dpad"})
            sess["remap_request"] = {"code": r.status_code, "body": r.json()}
            t = time.time()
            while time.time() - t < 20 and remap_rec(bridge).get("result") not in ("mapping", "failed",
                                                                                    "failed_localising",
                                                                                    "deferred"):
                time.sleep(0.3)
            sess["remap"] = remap_rec(bridge)
            # Counted once the re-map has settled (its own swap's saves are in).
            m0 = health(bridge)["map"]
            ends_before = m0["saved"] + m0["skipped"] + m0["failed"]
            de.MISSION_LIMIT_S = WATCH_S
            sess["mission"] = de.run_mission(robot, house, policy="explore", target=ABSENT,
                                             max_steps=100000)
        t = time.time()                         # this session's own save
        while time.time() - t < 30:             # any outcome of this session's save
            m1 = health(bridge)["map"]
            if m1["saved"] + m1["skipped"] + m1["failed"] > ends_before:
                break
            time.sleep(0.3)
        sess["saved"] = health(bridge)["map"]
        sess["sampler"] = s.summary()
        if i == 4:
            ok, rooms = walls_true(robot, house)
            sess["walls_100"], sess["walls_by_room"] = ok, rooms
        out["sessions"].append(sess)
    ss = out["sessions"]
    out["criteria"] = {
        "all_remapped": all((x.get("remap") or {}).get("result") == "mapping" for x in ss),
        "no_jumps": all(x.get("sampler") and not x["sampler"]["jumps"] for x in ss),
        "session5_walls_100": bool(ss and ss[-1].get("walls_100")),
        "contacts_0": all(x.get("sampler") and x["sampler"]["contacts"] == 0 for x in ss),
        "no_failed_mission": all((x.get("mission") or {}).get("outcome") != "failed" for x in ss)}
    out["verdict"] = {"D4-c": "MET" if all(out["criteria"].values()) else "FAILED"}
    return out


# ---------- D4-d ----------

def run_d(house):
    base = os.path.join(de.RUN_DIR, f"maps-d-{house}")
    out = {"house": house, "movers": MOVERS[house], "sessions": []}
    out["map_run"], _, _ = map_run(house, base)
    world = build_world(house)
    reach = de.reachable_cells(world)
    spots = [s for s in (de.room_spot(world, r, reach) for r in sorted(world.rooms)) if s]
    for i in range(3):
        maps = os.path.join(de.RUN_DIR, f"maps-d-{house}-{i}")
        shutil.rmtree(maps, ignore_errors=True)
        shutil.copytree(base, maps)
        robot, brain, bridge, sess = reload(house, maps, MOVERS[house])
        if sess["reload"].get("result") != "matched":
            sess["error"] = "reload did not match"
            out["sessions"].append(sess)
            continue
        t0 = time.time()
        with Sampler(robot, bridge, house) as s:
            de.MISSION_LIMIT_S = WATCH_S
            sess["mission"] = de.run_mission(robot, house, policy="explore", target=ABSENT,
                                             max_steps=100000)
            tour, k = [], 0
            while time.time() - t0 < WATCH_S and spots:
                tour.append(goal(robot, *cell_m(spots[k % len(spots)]),
                                 timeout_s=min(120, max(5, WATCH_S - (time.time() - t0)))))
                k += 1
            sess["tour"] = tour
            sess["watch"] = health(bridge)["slam"].get("watch")
        sess["seconds"] = round(time.time() - t0)
        sess["sampler"] = s.summary()
        # Watched time: localising and ready.
        st = s.states + [(None, time.time())]
        sess["watched_s"] = round(sum(b[1] - a[1] for a, b in zip(st, st[1:])
                                      if a[0] == ("ready", "localisation")), 1)
        sess["triggers"] = sorted({r.get("trigger") for r in s.remaps if r.get("trigger")})
        out["sessions"].append(sess)
    ss = out["sessions"]
    out["criteria"] = {
        "no_contradiction": all("contradiction" not in x.get("triggers", []) for x in ss)
        and all(x.get("sampler") for x in ss),
        "contacts_0": all(x.get("sampler") and x["sampler"]["contacts"] == 0 for x in ss),
        "no_failed_mission": all((x.get("mission") or {}).get("outcome") != "failed" for x in ss)}
    out["verdict"] = {"D4-d": "MET" if all(out["criteria"].values()) else "FAILED"}
    return out


def check_image():
    """The image must carry this checkout's bridge (review): an older one has
    no `mapping_at` (D4-a would read FAILED) and ignores BRIDGE_HOST."""
    import hashlib
    import subprocess
    pkg = os.path.join(ROOT, "service/slam/src/picar_bridge/picar_bridge")
    here = {}
    for name in sorted(os.listdir(pkg)):
        if name.endswith(".py"):
            with open(os.path.join(pkg, name), "rb") as f:
                here[name] = hashlib.sha256(f.read()).hexdigest()
    try:
        r = subprocess.run(["docker", "run", "--rm", "--entrypoint", "sh", d3.IMAGE, "-c",
                            "cd /ws/src/picar_bridge/picar_bridge && sha256sum *.py"],
                           capture_output=True, text=True, timeout=60)
    except (OSError, subprocess.TimeoutExpired) as e:
        raise SystemExit(f"cannot read {d3.IMAGE}: {e}")
    if r.returncode != 0:
        raise SystemExit(f"cannot read {d3.IMAGE} (exit {r.returncode}): {r.stderr.strip()[:300]}")
    there = {p[1]: p[0] for p in (ln.split(None, 1) for ln in r.stdout.splitlines()) if len(p) == 2}
    if there != here:
        raise SystemExit(f"{d3.IMAGE} does not carry this checkout's picar_bridge: rebuild it "
                         "(docker build -t $ROS_IMAGE service/slam)")


def main(argv):
    check_image()
    kinds = [a for a in argv if a in ("a", "b", "c", "d")] or ["a", "b", "c", "d"]
    houses = [a for a in argv if a in MOVERS] or list(MOVERS)
    os.makedirs(OUT, exist_ok=True)
    de.heavy_lock()
    de._kill_ports()
    de._new_run_dir()
    print(f"run dir {de.RUN_DIR}", flush=True)
    runs = {"a": run_a, "b": run_b, "d": run_d}
    for kind in kinds:
        for house in (["home_first_floor"] if kind == "c" else houses):
            name = f"{kind}-{house}"
            print(f"--- {name} {time.strftime('%H:%M:%S')}", flush=True)
            try:
                res = run_c(house) if kind == "c" else runs[kind](house)
            except Exception as e:  # noqa: BLE001 -- recorded, the next run goes on
                res = {"house": house, "error": f"{type(e).__name__}: {e}"}
            res["image"] = d3.IMAGE
            with open(os.path.join(OUT, f"{name}.json"), "w") as f:
                json.dump(res, f, indent=1, default=str)
            print(f"    {res.get('verdict') or res.get('error')}", flush=True)
    subprocess_cleanup()


def subprocess_cleanup():
    import subprocess
    try:
        with open(f"{de.RUN_DIR}/ros-final.log", "w") as f:
            subprocess.run(["docker", "logs", de.CONTAINER], stdout=f, stderr=subprocess.STDOUT)
    except OSError:
        pass                                    # never blocks the clean-up below
    subprocess.run(["docker", "rm", "-f", de.CONTAINER], capture_output=True)
    de._kill_ports()


if __name__ == "__main__":
    main(sys.argv[1:])
