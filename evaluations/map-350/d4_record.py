"""3.50.D4 amendment 4's data: recorded reload sessions for testing candidate
contradiction rules offline (`d4_replay.py`), before any criterion run.

    eval "$(python tools/plan_section.py env)"
    python evaluations/map-350/d4_record.py OUTDIR [scaled_house home_first_floor] [plain changed]

  plain    a map run of the house as it is, then one no-change reload watched
           600 s (explore for an absent target, then a room tour), with D4-d's
           movers: the negatives
  changed  a map run with D4-a's room closed and D4-b's door blocked (the
           house's own objects, d4_runs.close_moves / door_b), then a reload
           with every object back: the robot stops 30 s in view of each
           door, then three passes through D4-b's door: the positives (the
           doors, and the objects' old places) and more negatives elsewhere

Both automatic triggers are switched off inside the container for these
sessions (a patched copy of picar_bridge mounted over the image's), so each
session stays localising -- the regime the trigger watches -- to its end.
In the container a recorder (d4_rec_node.py) keeps every /map, each /scan with
map -> laser at its stamp, and the odometry; on the host the robot's truth
at 10 Hz. One pickle per session in OUTDIR, with the truth and the moves.
The maps are a floor plan of the SIM house; OUTDIR is never committed.
"""
import importlib.util
import json
import math
import os
import pickle
import shutil
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, ROOT)

import httpx  # noqa: E402

from sim.maps import build_world  # noqa: E402

_spec = importlib.util.spec_from_file_location("d4_runs", os.path.join(HERE, "d4_runs.py"))
d4 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(d4)
de, d3 = d4.de, d4.d3

WATCH_S = d4.WATCH_S
VIEW_S = d4.VIEW_S
PKG = os.path.join(ROOT, "service/slam/src/picar_bridge/picar_bridge")
OFF = "self._ask_remap("


def triggers_off(dst):
    """A copy of picar_bridge whose watch judges nothing into a re-map."""
    shutil.rmtree(dst, ignore_errors=True)
    shutil.copytree(PKG, dst, ignore=shutil.ignore_patterns("__pycache__"))
    p = os.path.join(dst, "bridge.py")
    s = open(p).read()
    assert s.count(OFF) == 2, "remap_step's two _ask_remap calls moved: update d4_record.py"
    s = s.replace(OFF, "(lambda *a, **k: None)(")
    open(p, "w").write(s)
    return dst


def mount_patched(pkg):
    net = d3._network

    def network():
        return [*net(), "-v", f"{pkg}:/ws/src/picar_bridge/picar_bridge:ro"]
    d3._network = network


class Truth:
    """The robot's truth at 10 Hz (host clock)."""

    def __init__(self, robot):
        self.url, self.rows = str(robot.base_url), []
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)

    def __enter__(self):
        self._t.start()
        return self

    def __exit__(self, *exc):
        self._stop.set()
        self._t.join(5)

    def _run(self):
        with httpx.Client(base_url=self.url, timeout=5) as c:
            while not self._stop.is_set():
                try:
                    t = c.get("/world/truth").json()
                    self.rows.append((time.time(), t["x_m"], t["y_m"], t["heading_deg"]))
                except Exception:  # noqa: BLE001 -- a gap, not a stop
                    pass
                self._stop.wait(0.1)


MOUNTED = [None]


def mounted_ok():
    """The running container's bridge.py is the patched copy."""
    import hashlib
    want = hashlib.sha256(open(os.path.join(MOUNTED[0], "bridge.py"), "rb").read()).hexdigest()
    r = subprocess.run(["docker", "exec", de.CONTAINER, "sha256sum",
                        "/ws/src/picar_bridge/picar_bridge/bridge.py"], capture_output=True, text=True)
    got = r.stdout.split()[0] if r.stdout else r.stderr.strip()
    if got != want:
        subprocess.run(["docker", "rm", "-f", de.CONTAINER], capture_output=True)
        raise SystemExit(f"the container's bridge.py is not the patched copy ({got[:80]})")


def start_recorder(secs, out):
    """Wait for the container, then run the recorder in it (a thread)."""
    def run():
        while True:
            st = subprocess.run(["docker", "inspect", "-f", "{{.State.Running}}", de.CONTAINER],
                                capture_output=True, text=True).stdout.strip()
            if st == "true":
                break
            time.sleep(0.05)
        mounted_ok()
        subprocess.run(["docker", "cp", os.path.join(HERE, "d4_rec_node.py"),
                        f"{de.CONTAINER}:/tmp/d4_rec_node.py"], check=True)
        r = subprocess.run(["docker", "exec", de.CONTAINER, "bash", "-c",
                            "source /opt/ros/humble/setup.bash; source /underlay/install/setup.bash; "
                            f"source /ws/install/setup.bash; export ROS_DOMAIN_ID={de.ROS_DOMAIN}; "
                            f"python3 /tmp/d4_rec_node.py {secs} /tmp/rec.pkl"],
                           capture_output=True, text=True)
        print("recorder:", r.stdout.strip(), r.stderr.strip()[-1500:], flush=True)
        subprocess.run(["docker", "cp", f"{de.CONTAINER}:/tmp/rec.pkl", out], check=True)
    subprocess.run(["docker", "rm", "-f", de.CONTAINER], capture_output=True)
    th = threading.Thread(target=run)
    th.start()
    return th


def changes(house):
    room, door = d4.CLOSED_ROOM[house]
    moves = d4.close_moves(house, door, room)
    bdoor = d4.door_b(house)
    w = build_world(house)
    used = {s for s, _ in moves} | set(door)
    obj = min((o for o in w.objects if o not in used), key=lambda o: math.dist(o, bdoor))
    return room, door, bdoor, moves + [(obj, bdoor)]


def session(house, kind, base, out_dir, pkg):
    maps = os.path.join(de.RUN_DIR, f"maps-rec-{kind}-{house}")
    shutil.rmtree(maps, ignore_errors=True)
    shutil.copytree(base, maps)
    pkl = os.path.join(out_dir, f"{kind}-{house}.pkl")
    secs = WATCH_S + 120 if kind == "plain" else 420
    rec = start_recorder(secs, pkl + ".ros")
    info = {"house": house, "kind": kind, "events": []}
    movers = d4.MOVERS[house] if kind == "plain" else ""
    robot, brain, bridge, sess = d4.reload(house, maps, movers)
    info["reload"] = sess["reload"]
    t0 = time.time()
    with Truth(robot) as tr:
        if kind == "plain":
            world = build_world(house)
            reach = de.reachable_cells(world)
            spots = [s for s in (de.room_spot(world, r, reach) for r in sorted(world.rooms)) if s]
            de.MISSION_LIMIT_S = WATCH_S
            info["mission"] = de.run_mission(robot, house, policy="explore", target=d4.ABSENT,
                                             max_steps=100000)
            k = 0
            while time.time() - t0 < WATCH_S and spots:
                g = d4.goal(robot, *d4.cell_m(spots[k % len(spots)]),
                            timeout_s=min(120, max(5, WATCH_S - (time.time() - t0))))
                info["events"].append(("tour", spots[k % len(spots)], g, time.time()))
                k += 1
        else:
            room, door, bdoor, _ = changes(house)
            a_side, b_side = d4.sides(house, bdoor)
            for name, cell in (("view_b", a_side), ("view_a", d4.outside(house, door, room))):
                g = d4.goal(robot, *d4.cell_m(cell), timeout_s=180)
                t1 = time.time()
                time.sleep(VIEW_S)
                info["events"].append((name, cell, g, t1, time.time()))
            for k in range(3):
                cell = b_side if k % 2 == 0 else a_side
                g = d4.goal(robot, *d4.cell_m(cell), timeout_s=120)
                info["events"].append(("pass", cell, g, time.time()))
        info["slam"] = d4.health(bridge)["slam"]
    info["truth"] = tr.rows
    info["seconds"] = round(time.time() - t0)
    rec.join(secs + 300)
    with open(pkl + ".ros", "rb") as f:
        ros = pickle.load(f)
    os.remove(pkl + ".ros")
    with open(pkl, "wb") as f:
        pickle.dump({**ros, **info}, f)
    print(f"  {kind}-{house}: {len(ros['scans'])} scans, {info['seconds']} s, "
          f"remap {info['slam'].get('remap')}", flush=True)


def main(argv):
    out_dir = os.path.abspath(argv[0])
    os.makedirs(out_dir, exist_ok=True)
    houses = [a for a in argv[1:] if a in d4.MOVERS] or list(d4.MOVERS)
    kinds = [a for a in argv[1:] if a in ("plain", "changed")] or ["plain", "changed"]
    # No d4.check_image(): this checkout's picar_bridge is mounted over the
    # image's (patched), and each session checks the mount took (mounted_ok).
    de.heavy_lock()
    de._kill_ports()
    de._new_run_dir()
    print(f"run dir {de.RUN_DIR}", flush=True)
    MOUNTED[0] = triggers_off(os.path.join(de.RUN_DIR, "pkg-triggers-off"))
    mount_patched(MOUNTED[0])
    for house in houses:
        for kind in kinds:
            base = os.path.join(de.RUN_DIR, f"maps-base-{kind}-{house}")
            moves = changes(house)[3] if kind == "changed" else ()
            print(f"--- {kind}-{house} {time.strftime('%H:%M:%S')} moves {moves}", flush=True)
            try:
                res, _, _ = d4.map_run(house, base, moves)
                with open(os.path.join(out_dir, f"{kind}-{house}.base.json"), "w") as f:
                    json.dump({"moves": moves, **res}, f, default=str)
                session(house, kind, base, out_dir, None)
            except Exception as e:  # noqa: BLE001 -- recorded, the next goes on
                print(f"  {kind}-{house}: {type(e).__name__}: {e}", flush=True)
    d4.subprocess_cleanup()


if __name__ == "__main__":
    main(sys.argv[1:])
