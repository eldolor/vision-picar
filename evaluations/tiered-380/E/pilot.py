"""
evaluations/tiered-380/E/pilot.py

3.80.E pilot (the user, 2026-10-14: "Do a pilot and if the results are still
inconclusive then let's discuss"). Decides whether criteria 1 and 2 need
real Bedrock calls (option B) or can be measured at $0 (option A).

    .venv/bin/python evaluations/tiered-380/E/pilot.py frames  --out ...
    .venv/bin/python evaluations/tiered-380/E/pilot.py mission --cloud real    --out ...
    .venv/bin/python evaluations/tiered-380/E/pilot.py mission --cloud standin --out ...

`frames`: re-asks the deployed /navigate about a few of the 299 pinned
frames and compares with the answer recorded on the walk (3.48's cloud
column): does a fresh call reproduce the record?

`mission`: one tiered mission in the simulator (home_first_floor, the local
tier off: "today's tiered policy"), through the brain's HTTP API, with a
logging proxy between the brain and /navigate. `real` forwards to the
deployed service and records each call's tokens and latency, refusing every
call after BUDGET (429, which the brain reads as the cloud unreachable);
`standin` answers locally ("not visible, RIGHT"), costing nothing. Does the
number of cloud calls depend on what the cloud answers?

Reads VISION_URL and VISION_SECRET from ~/.vision-picar-local-secrets (the
tunnel's file); prints nothing secret.
"""

import argparse
import base64
import json
import os
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import httpx

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from tests.demo_explore import heavy_lock
from tests.demo_local_tier import PY, ROOT, free_port, wait

BUDGET = 60                       # cloud calls a mission may make (~$0.40)
DEFAULT_VISION_URL = "https://d114x92g7i4syl.cloudfront.net"   # service/tunnel/run.sh
# Opus 4.5 on Bedrock, USD per token (the claude-api price list, 2026-10).
PRICE_IN, PRICE_OUT = 5e-6, 25e-6
RECORDINGS = Path(os.environ.get("RECORDINGS", "/Users/anshugaind/vision-picar/recordings"))


def secrets() -> dict:
    out = {}
    for line in Path.home().joinpath(".vision-picar-local-secrets").read_text().splitlines():
        line = line.strip()
        if line and not line.startswith("#") and "=" in line:
            k, v = line.split("=", 1)
            out[k.strip()] = v.strip().strip('"').strip("'")
    return out


def cost(usage: dict) -> float:
    return ((usage.get("input_tokens") or 0) * PRICE_IN
            + (usage.get("output_tokens") or 0) * PRICE_OUT)


# --- frames -------------------------------------------------------------------

def pick_frames(n_each=3):
    """Confabulations and true yeses from the labelled walks, recorded by
    Opus 4.5 (the model id on the record)."""
    conf, true = [], []
    for walk in sorted(p for p in RECORDINGS.iterdir() if (p / "labels.json").is_file()):
        labels = json.loads((walk / "labels.json").read_text())
        target = labels.get("description")
        truth = labels.get("target_visible_labels") or {}
        for line in (walk / "walk.jsonl").read_text().splitlines():
            rec = json.loads(line)
            nav = rec.get("navigate") or {}
            f = rec.get("file")
            if f not in truth or "opus-4-5" not in str(nav.get("model_id")):
                continue
            row = (walk.name, f, target, nav.get("target_visible") is True, truth[f])
            if row[3] and not row[4]:
                conf.append(row)
            elif row[3] and row[4]:
                true.append(row)
    # Spread over walks: every k-th.
    return conf[:: max(1, len(conf) // n_each)][:n_each] + true[:: max(1, len(true) // n_each)][:n_each]


def run_frames(args):
    s = secrets()
    url = (s.get("VISION_URL") or DEFAULT_VISION_URL).rstrip("/")
    rows = []
    for walk, f, target, recorded, label in pick_frames():
        img = base64.b64encode((RECORDINGS / walk / f).read_bytes()).decode()
        t0 = time.monotonic()
        r = httpx.post(f"{url}/navigate", timeout=60,
                       headers={"x-app-secret": s.get("VISION_SECRET", "")},
                       json={"image_base64": img, "media_type": "image/jpeg",
                             "target_object": target, "searched_rooms": []})
        got = r.json() if r.status_code == 200 else {}
        rows.append({"walk": walk, "frame": f, "target": target, "label": label,
                     "recorded": recorded, "fresh": got.get("target_visible"),
                     "status": r.status_code, "model_id": got.get("model_id"),
                     "usage": got.get("usage"), "cost_usd": cost(got.get("usage") or {}),
                     "s": round(time.monotonic() - t0, 2)})
        print(json.dumps(rows[-1]), flush=True)
    doc = {"rows": rows, "calls": len(rows),
           "agree_with_record": sum(r["fresh"] == r["recorded"] for r in rows),
           "cost_usd": round(sum(r["cost_usd"] for r in rows), 4)}
    return doc


# --- mission -----------------------------------------------------------------

class Proxy(BaseHTTPRequestHandler):
    upstream = ""
    secret = ""
    mode = "real"
    calls: list = []
    lock = threading.Lock()

    def log_message(self, *a):
        pass

    def _send(self, code, body: bytes, ctype="application/json"):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.mode == "standin":
            if self.path.startswith("/navigate/models"):
                return self._send(200, json.dumps({"default": "standin", "models": ["standin"],
                                                   "prompt_variants": ["default"]}).encode())
            return self._send(200, b'{"status": "ok"}')
        r = httpx.get(self.upstream + self.path, timeout=30,
                      headers={"x-app-secret": self.secret})
        self._send(r.status_code, r.content, r.headers.get("content-type", "application/json"))

    def do_POST(self):
        body = self.rfile.read(int(self.headers.get("Content-Length") or 0))
        with self.lock:
            n = len(self.calls)
            if self.path == "/navigate" and self.mode == "real" and n >= BUDGET:
                self.calls.append({"refused": True})
                return self._send(429, b'{"detail": "pilot budget spent"}')
        t0 = time.monotonic()
        if self.mode == "standin":
            out = {"target_visible": False, "target_direction": "not_visible",
                   "target_reached": False, "obstacle_ahead": False, "room_guess": "unclear",
                   "action": "RIGHT", "reasoning": "stand-in", "distance_estimate": "unknown"}
            code, content = 200, json.dumps(out).encode()
        else:
            r = httpx.post(self.upstream + self.path, content=body, timeout=60,
                           headers={"x-app-secret": self.secret,
                                    "Content-Type": "application/json"})
            code, content = r.status_code, r.content
            try:
                out = r.json()
            except ValueError:
                out = {}
        if self.path == "/navigate":
            with self.lock:
                self.calls.append({"status": code, "usage": out.get("usage"),
                                   "model_id": out.get("model_id"),
                                   "target_visible": out.get("target_visible"),
                                   "action": out.get("action"),
                                   "s": round(time.monotonic() - t0, 2)})
        self._send(code, content)


def run_mission(args):
    heavy_lock()
    s = secrets()
    Proxy.upstream = (s.get("VISION_URL") or DEFAULT_VISION_URL).rstrip("/")
    Proxy.secret = s.get("VISION_SECRET", "")
    Proxy.mode = args.cloud
    Proxy.calls = []
    proxy_p, robot_p, brain_p = free_port(), free_port(), free_port()
    httpd = ThreadingHTTPServer(("127.0.0.1", proxy_p), Proxy)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    env = {k: v for k, v in os.environ.items()
           if k not in ("APP_SHARED_SECRET", "LOCAL_SECRET", "LOCAL_VLM_URL",
                        "VISION_SHARED_SECRET", "VISION_SECRET")}
    env.update(SIM_MAP=args.house, PICAR_LOG_LEVEL="WARNING", METRICS_URL="",
               INVENTORY_BUCKET="")
    logs = Path(os.environ.get("TMPDIR", "/tmp")) / f"pilot-380E-{args.cloud}-{int(time.time())}"
    logs.mkdir(parents=True, exist_ok=True)
    procs = [
        subprocess.Popen([PY, "-m", "uvicorn", "robot.server:app", "--port", str(robot_p),
                          "--host", "127.0.0.1", "--log-level", "warning"], cwd=ROOT, env=env,
                         stdout=open(logs / "robot.log", "w"), stderr=subprocess.STDOUT),
        subprocess.Popen([PY, "-m", "uvicorn", "control.brain_server:app", "--port",
                          str(brain_p), "--host", "127.0.0.1", "--log-level", "warning"],
                         cwd=ROOT, stdout=open(logs / "brain.log", "w"),
                         stderr=subprocess.STDOUT,
                         env=dict(env, ROBOT_URL=f"http://127.0.0.1:{robot_p}",
                                  VISION_URL=f"http://127.0.0.1:{proxy_p}",
                                  PICAR_LOG_LEVEL="INFO")),
    ]
    t0, status = time.time(), {}
    try:
        wait(f"http://127.0.0.1:{robot_p}")
        wait(f"http://127.0.0.1:{brain_p}")
        brain = httpx.Client(base_url=f"http://127.0.0.1:{brain_p}", timeout=60)
        r = brain.post("/mission/start", json={"target_object": "red backpack",
                                               "policy": "tiered",
                                               "mission_time_limit_s": args.limit})
        if r.status_code != 200:
            raise SystemExit(f"start refused: {r.status_code} {r.text[:300]}")
        while time.time() - t0 < args.limit + 60:
            status = brain.get("/mission/status").json()
            if not status.get("running") and status.get("outcome") not in (None, "idle",
                                                                           "running"):
                break
            time.sleep(1.0)
        else:
            brain.post("/mission/stop")
    finally:
        for p in procs:
            p.terminate()
        for p in procs:
            try:
                p.wait(10)
            except subprocess.TimeoutExpired:
                p.kill()
        httpd.shutdown()
    stats = ((status.get("tier") or {}).get("stats") or {})
    calls = [c for c in Proxy.calls if not c.get("refused")]
    usages = [c["usage"] for c in calls if c.get("usage")]
    return {"cloud": args.cloud, "house": args.house, "outcome": status.get("outcome"),
            "seconds": round(time.time() - t0), "steps": status.get("step"),
            "cloud_calls_proxy": len(calls),
            "refused_over_budget": sum(1 for c in Proxy.calls if c.get("refused")),
            "cloud_calls_brain": stats.get("cloud_calls"), "triggers": stats.get("triggers"),
            "frames": stats.get("frames"),
            "tokens_in_mean": (round(sum(u.get("input_tokens") or 0 for u in usages)
                                     / len(usages)) if usages else None),
            "tokens_out_mean": (round(sum(u.get("output_tokens") or 0 for u in usages)
                                      / len(usages)) if usages else None),
            "cost_usd": round(sum(cost(u) for u in usages), 4),
            "visible_answers": sum(1 for c in calls if c.get("target_visible")),
            "model_ids": sorted({str(c.get("model_id")) for c in calls}),
            "log_tail": (status.get("log_tail") or [])[-4:], "logs": str(logs)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=("frames", "mission"))
    ap.add_argument("--cloud", choices=("real", "standin"), default="standin")
    ap.add_argument("--house", default="home_first_floor")
    ap.add_argument("--limit", type=float, default=1200)
    ap.add_argument("--out")
    args = ap.parse_args()
    doc = run_frames(args) if args.what == "frames" else run_mission(args)
    print(json.dumps(doc, indent=1))
    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(json.dumps(doc, indent=1))


if __name__ == "__main__":
    main()
