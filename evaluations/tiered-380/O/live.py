"""
evaluations/tiered-380/O/live.py

3.80.O criterion O3 (and O2 live): `G/live.py`'s live tiered missions under
`tier_search: explore` on a fresh ROS stack per house, with
* perception blind to the target (`SIM_PERCEPTION_BLIND=1` on the brain);
* a stand-in local VLM on this host (the service's `/identity` route) that
  answers from the image it is sent: the renderer draws only the backpack in
  `COLOR_TARGET`, so its pixels give the target's bearing -- a yes with a box
  there, else a no;
* the cloud (the counting proxy, `--cloud standin`) answering the same way:
  `target_visible` iff the image shows the target.
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

from PIL import Image

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "evaluations/tiered-380/G"))
import live as glive                                         # noqa: E402  (G/live.py)
from brain.local_vlm import model_size                       # noqa: E402
from tests.demo_local_tier import free_port                  # noqa: E402

TARGET = glive.TARGET
SIM_HFOV_DEG = 60.0
MIN_PIXELS = 12          # target-coloured pixels that make a sighting
TOLERANCE = 60           # summed |RGB| distance, for JPEG's blur


def _seen_in(jpeg: bytes):
    """The target's in-frame bearing in THIS image (the renderer draws only
    the backpack in COLOR_TARGET), or None -- judged on the frame the brain
    asked about, never on a later one (review)."""
    import numpy as np
    from sim.renderer import COLOR_TARGET
    img = np.asarray(Image.open(io.BytesIO(jpeg)).convert("RGB"), dtype=np.int32)
    hit = np.abs(img - np.array(COLOR_TARGET)).sum(axis=2) < TOLERANCE
    if hit.sum() < MIN_PIXELS:
        return None
    xs = np.nonzero(hit)[1]
    return (float(xs.mean()) / img.shape[1] - 0.5) * SIM_HFOV_DEG


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
        jpeg = base64.b64decode(req.get("image_b64") or "")
        bearing = _seen_in(jpeg) if jpeg else None
        if self.path.rstrip("/") != "/identity":
            return self._send(200, {"status": "error", "score": None, "reason": "stand-in: identity only"})
        if bearing is None:
            return self._send(200, {"status": "answered", "score": 0.0, "parsed": True, "box": None,
                                    "ms": (time.monotonic() - t0) * 1000})
        w, h = model_size(*Image.open(io.BytesIO(jpeg)).size)
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
        body = json.loads(self.rfile.read(int(self.headers.get("Content-Length") or 0)) or b"{}")
        image = base64.b64decode(body.get("image_base64") or "")
        visible = bool(image) and _seen_in(image) is not None
        out = {"target_visible": visible,
               "target_direction": "center" if visible else "not_visible",
               "target_reached": False, "obstacle_ahead": False, "room_guess": "unclear",
               "action": "FORWARD" if visible else "RIGHT", "reasoning": "stand-in (sim report)",
               "distance_estimate": "unknown"}
        with self.lock:
            self.calls.append({"ms": 0, "usage": None, "target_visible": visible})
        self._send(200, json.dumps(out).encode())


def main():
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
