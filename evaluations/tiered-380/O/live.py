"""
evaluations/tiered-380/O/live.py

3.80.O criterion O3 (and O2 live): `G/live.py`'s live tiered missions under
`tier_search: explore` on a fresh ROS stack per house, with
* perception blind to the target (`SIM_PERCEPTION_BLIND=1` on the brain);
* a stand-in local VLM on this host (the service's `/identity` route) that
  answers from the simulator's report: the robot server's current `/frame`
  detections -- a yes with a box at the target's bearing, else a no;
* the cloud (the counting proxy, `--cloud standin`) answering from the same
  report: `target_visible` iff the frame reports the target.
Records `G/live.py`'s row per house (contacts on ground truth, arrivals,
found, calls). Free.

    eval "$(python tools/plan_section.py env)"
    .venv/bin/python evaluations/tiered-380/O/live.py --out evaluations/tiered-380/O/live-O3.jsonl --houses 8
"""

import base64
import io
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import httpx
from PIL import Image

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "evaluations/tiered-380/G"))
import live as glive                                         # noqa: E402  (G/live.py)
from brain.local_vlm import model_size                       # noqa: E402
from tests import demo_explore                               # noqa: E402
from tests.demo_local_tier import free_port                  # noqa: E402

TARGET = glive.TARGET
SIM_HFOV_DEG = 60.0
_robot = {"client": None}


def _seen():
    """The target's in-frame bearing in the robot server's current frame, or None."""
    c = _robot["client"]
    if c is None:
        return None
    try:
        frame = c.get("/frame").json()
    except Exception:  # noqa: BLE001 -- no frame: say no
        return None
    for d in frame.get("detections") or ():
        if d.get("label") == TARGET and d.get("bearing_deg") is not None:
            return float(d["bearing_deg"])
    return None


class StandinLocal(BaseHTTPRequestHandler):
    """The local VLM service's `/identity` and `/health`, from the sim's report."""

    def log_message(self, *a):
        pass

    def _send(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        self._send(200, {"service": "local_vlm", "ready": True, "alive": True,
                         "engines": "standin (sim report)"})

    def do_POST(self):
        req = json.loads(self.rfile.read(int(self.headers.get("Content-Length") or 0)) or b"{}")
        t0 = time.monotonic()
        bearing = _seen()
        if self.path.rstrip("/") != "/identity":
            return self._send(200, {"status": "error", "score": None, "reason": "stand-in: identity only"})
        if bearing is None:
            return self._send(200, {"status": "answered", "score": 0.0, "parsed": True, "box": None,
                                    "ms": (time.monotonic() - t0) * 1000})
        w, h = model_size(*Image.open(io.BytesIO(base64.b64decode(req["image_b64"]))).size)
        x = bearing / SIM_HFOV_DEG + 0.5
        box = [max(0.0, (x - 0.02) * w), 0.4 * h, min(w, (x + 0.02) * w), 0.6 * h]
        self._send(200, {"status": "answered", "score": 0.95, "parsed": True,
                         "box": box if req.get("box") else None,
                         "ms": (time.monotonic() - t0) * 1000})


class TruthProxy(glive.Proxy):
    """The counting proxy's stand-in, answering from the sim's report."""

    def do_POST(self):
        if self.mode != "standin" or self.path != "/navigate":
            return super().do_POST()
        self.rfile.read(int(self.headers.get("Content-Length") or 0))
        visible = _seen() is not None
        out = {"target_visible": visible,
               "target_direction": "center" if visible else "not_visible",
               "target_reached": False, "obstacle_ahead": False, "room_guess": "unclear",
               "action": "FORWARD" if visible else "RIGHT", "reasoning": "stand-in (sim report)",
               "distance_estimate": "unknown"}
        with self.lock:
            self.calls.append({"ms": 0, "usage": None, "target_visible": visible})
        self._send(200, json.dumps(out).encode())


def main():
    _stack = demo_explore.stack

    def stack(house, *a, **k):
        client = _stack(house, *a, **k)
        _robot["client"] = client
        return client

    demo_explore.stack = stack
    glive.demo_explore.stack = stack
    glive.Proxy = TruthProxy
    port = free_port()
    httpd = ThreadingHTTPServer(("127.0.0.1", port), StandinLocal)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    os.environ.update(LOCAL_VLM_URL=f"http://127.0.0.1:{port}", SIM_PERCEPTION_BLIND="1")
    if "--cloud" not in sys.argv:
        sys.argv += ["--cloud", "standin"]
    try:
        glive.main()
    finally:
        httpd.shutdown()


if __name__ == "__main__":
    main()
