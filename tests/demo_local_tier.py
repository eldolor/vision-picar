"""
tests/demo_local_tier.py

3.80.C acceptance C8: one tiered mission in the simulator, through the
brain's HTTP API, with the arrival's identity answered by the real local VLM
service (`service/local_vlm`, on tests/fake_edgellm_driver.py told to say
yes) and the cloud unreachable. Expected: `found`, identity from the local
tier, no cloud call at arrival.

    python -m tests.demo_local_tier [--answer yes|no] [--out evaluations/tiered-380/C/C8.json]

Three servers on free ports; nothing on :8000/:8001/:8005 is touched.
"""

import argparse
import json
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

import httpx

ROOT = Path(__file__).resolve().parent.parent
PY = sys.executable
DEAD_URL = "http://127.0.0.1:9"   # the cloud: nothing listens
LIMIT_S = 300

SERVICE = """
import sys, uvicorn
from service.local_vlm.driver import EdgeDriver
from service.local_vlm.server import create_app
fake = [sys.executable, "tests/fake_edgellm_driver.py"]
app = create_app(lambda: EdgeDriver(fake, dict(__import__("os").environ),
                                    {6: "[{", 7: "[]", 9: "x"}.get))
uvicorn.run(app, host="127.0.0.1", port=int(sys.argv[1]), log_level="warning")
"""


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def wait(url, path="/health", ok=lambda r: r.status_code == 200, limit=60):
    end = time.monotonic() + limit
    while time.monotonic() < end:
        try:
            r = httpx.get(url + path, timeout=1)
            if ok(r):
                return
        except httpx.HTTPError:
            pass
        time.sleep(0.2)
    raise RuntimeError(f"{url} never came up")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--answer", default="yes", choices=("yes", "no"))
    ap.add_argument("--house", default="scaled_house")
    ap.add_argument("--out")
    args = ap.parse_args()
    robot_p, brain_p, vlm_p = free_port(), free_port(), free_port()
    env = {k: v for k, v in os.environ.items()
           if k not in ("APP_SHARED_SECRET", "LOCAL_SECRET", "LOCAL_VLM_SECRET")}
    env.update(SIM_MAP=args.house, PICAR_LOG_LEVEL="WARNING", METRICS_URL="",
               INVENTORY_BUCKET="")
    logs = Path(os.environ.get("TMPDIR", "/tmp")) / f"demo_local_tier-{int(time.time())}"
    logs.mkdir(parents=True, exist_ok=True)
    procs = [
        subprocess.Popen([PY, "-c", SERVICE, str(vlm_p)], cwd=ROOT,
                         env=dict(env, FAKE_ANSWER=args.answer),
                         stdout=open(logs / "vlm.log", "w"), stderr=subprocess.STDOUT),
        subprocess.Popen([PY, "-m", "uvicorn", "robot.server:app", "--port", str(robot_p),
                          "--host", "127.0.0.1", "--log-level", "warning"], cwd=ROOT, env=env,
                         stdout=open(logs / "robot.log", "w"), stderr=subprocess.STDOUT),
        subprocess.Popen([PY, "-m", "uvicorn", "control.brain_server:app", "--port",
                          str(brain_p), "--host", "127.0.0.1", "--log-level", "warning"],
                         cwd=ROOT, stdout=open(logs / "brain.log", "w"), stderr=subprocess.STDOUT,
                         env=dict(env, ROBOT_URL=f"http://127.0.0.1:{robot_p}",
                                  VISION_URL=DEAD_URL, PICAR_LOG_LEVEL="INFO",
                                  LOCAL_VLM_URL=f"http://127.0.0.1:{vlm_p}")),
    ]
    try:
        wait(f"http://127.0.0.1:{vlm_p}", ok=lambda r: r.json().get("ready"))
        wait(f"http://127.0.0.1:{robot_p}")
        wait(f"http://127.0.0.1:{brain_p}")
        brain = httpx.Client(base_url=f"http://127.0.0.1:{brain_p}", timeout=60)
        r = brain.post("/mission/start", json={"target_object": "red backpack",
                                               "policy": "tiered",
                                               "mission_time_limit_s": LIMIT_S})
        if r.status_code != 200:
            raise SystemExit(f"start refused: {r.status_code} {r.text[:300]}")
        t0, status = time.time(), {}
        while time.time() - t0 < LIMIT_S + 30:
            status = brain.get("/mission/status").json()
            if not status.get("running") and status.get("outcome") not in (None, "idle",
                                                                           "running"):
                break
            time.sleep(0.5)
        else:
            brain.post("/mission/stop")
        stats = ((status.get("tier") or {}).get("stats") or {})
        served = httpx.get(f"http://127.0.0.1:{vlm_p}/health").json().get("served")
        identity = (status.get("arrival") or {}).get("identity") or {}
        doc = {"outcome": status.get("outcome"), "seconds": round(time.time() - t0),
               "steps": status.get("step"), "arrival": status.get("arrival"),
               "identity_tier": identity.get("tier"),
               "arrival_cloud_calls": (stats.get("triggers") or {}).get("arrival_confirmation", 0),
               "local_calls": stats.get("local_calls"), "local": stats.get("local"),
               "service_served": served, "log_tail": (status.get("log_tail") or [])[-5:],
               "logs": str(logs)}
        want = "found" if args.answer == "yes" else None
        doc["met"] = (doc["outcome"] == "found" and doc["identity_tier"] == "local"
                      and doc["arrival_cloud_calls"] == 0) if want else None
        print(json.dumps(doc, indent=1))
        if args.out:
            Path(args.out).parent.mkdir(parents=True, exist_ok=True)
            Path(args.out).write_text(json.dumps(doc, indent=1))
    finally:
        for p in procs:
            p.terminate()
        for p in procs:
            try:
                p.wait(10)
            except subprocess.TimeoutExpired:
                p.kill()


if __name__ == "__main__":
    main()
