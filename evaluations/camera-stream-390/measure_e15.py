"""3.90 E criterion 15: paid cloud calls per minute do not rise with the
phone's frame rate.

No spend: the brain's cloud is a local stub that counts calls and answers
like /navigate. Everything else is real: a robot server in `mode: teleop`,
the brain (`policy: tiered`, YOLOE + CLIP on this machine), and the twin in a
headless Chromium with a fake camera, streaming under Drive via brain at
each rate for --seconds. Calls per minute are read from the brain's own
`tier.stats.cloud_calls` and checked against the stub's count.

    python evaluations/camera-stream-390/measure_e15.py --seconds 120 \\
        --out evaluations/camera-stream-390/E15-laptop.json

Bar (written before measuring): calls per minute at 8 fps within 10% of
those at 2 fps.
"""

import argparse
import json
import os
import subprocess
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import httpx

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ROBOT, BRAIN = 9900, 9901
NAV = {"target_visible": False, "target_direction": None, "target_reached": False,
       "obstacle_ahead": False, "room_guess": "kitchen", "action": "LEFT",
       "reasoning": "stub", "model_id": "stub", "obstacles_ahead": [],
       "free_space": "clear", "doorway_visible": False, "important_objects": [],
       "safest_direction": "LEFT"}


class Cloud:
    def __init__(self):
        self.calls = 0
        cloud = self

        class H(BaseHTTPRequestHandler):
            def _send(self, obj):
                raw = json.dumps(obj).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)

            def do_GET(self):
                if "models" in self.path:
                    self._send({"default": "stub", "default_prompt": "default",
                                "prompts": ["default"], "models": [{"id": "stub"}]})
                else:
                    self._send({"status": "ok"})

            def do_POST(self):
                self.rfile.read(int(self.headers.get("Content-Length", 0)))
                cloud.calls += 1
                time.sleep(1.0)               # a cloud call's latency, roughly
                self._send(NAV)

            def log_message(self, *a):
                pass

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), H)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.url = f"http://127.0.0.1:{self.server.server_address[1]}"


def _wait(url):
    for _ in range(600):
        try:
            if httpx.get(url, timeout=0.5).status_code == 200:
                return
        except httpx.HTTPError:
            pass
        time.sleep(0.1)
    raise RuntimeError(f"{url} never answered")


def run(fps, seconds, cloud, logdir):
    cfg = os.path.join(logdir, "robot.yaml")
    with open(os.path.join(ROOT, "config", "robot.yaml")) as f:
        text = f.read()
    with open(cfg, "w") as f:
        f.write(text.replace("\nmode: sim", "\nmode: teleop", 1))
    env = {k: v for k, v in os.environ.items() if k not in ("APP_SHARED_SECRET", "LOCAL_SECRET")}
    env.update(ROBOT_CONFIG_PATH=cfg, ROBOT_MODE="teleop", WORLD_MODE="none",
               VISION_URL=cloud.url, ROBOT_URL=f"http://127.0.0.1:{ROBOT}")
    procs = [subprocess.Popen([sys.executable, "-m", "uvicorn", "robot.server:app", "--port",
                               str(ROBOT), "--log-level", "warning"], cwd=ROOT, env=env,
                              stdout=open(f"{logdir}/robot-{fps}.log", "w"), stderr=subprocess.STDOUT),
             subprocess.Popen([sys.executable, "-m", "uvicorn", "control.brain_server:app", "--port",
                               str(BRAIN), "--log-level", "warning"], cwd=ROOT, env=env,
                              stdout=open(f"{logdir}/brain-{fps}.log", "w"), stderr=subprocess.STDOUT)]
    try:
        _wait(f"http://127.0.0.1:{ROBOT}/health")
        _wait(f"http://127.0.0.1:{BRAIN}/health")
        from playwright.sync_api import sync_playwright
        with sync_playwright() as p:
            b = p.chromium.launch(args=["--use-fake-ui-for-media-stream",
                                        "--use-fake-device-for-media-stream"])
            ctx = b.new_context(viewport={"width": 390, "height": 844}, permissions=["camera"])
            seed = {"guidanceMode": "robot", "vp_guide_onboarded": "1",
                    "vp_brain_url": f"http://127.0.0.1:{BRAIN}", "vp_drive_via_brain": "1",
                    "vp_drive_policy": "tiered", "vp_stream_fps": str(fps)}
            ctx.add_init_script("(() => { const s = %s; for (const k in s)"
                                " localStorage.setItem(k, s[k]); })();" % json.dumps(seed))
            page = ctx.new_page()
            page.goto(f"http://127.0.0.1:{ROBOT}/", wait_until="networkidle")
            page.wait_for_timeout(1500)
            page.click('.tab-btn[data-tab="guide"]')
            page.fill("#guidance-target", "red backpack")
            calls0 = cloud.calls
            page.click("#btn-guidance")
            # The first tiered start loads YOLOE + CLIP inside /mission/start.
            brain = httpx.Client(base_url=f"http://127.0.0.1:{BRAIN}", timeout=10)
            for _ in range(240):
                if brain.get("/mission/status").json().get("running"):
                    break
                time.sleep(0.5)
            else:
                raise RuntimeError("the mission never started: " + page.inner_text("#guide-start-error"))
            t0 = time.monotonic()
            seq0 = httpx.get(f"http://127.0.0.1:{ROBOT}/teleop/frames",
                             params={"after": 10**9}).json()["latest_seq"]
            page.wait_for_timeout(seconds * 1000)
            st = brain.get("/mission/status").json()
            elapsed = time.monotonic() - t0
            seq1 = httpx.get(f"http://127.0.0.1:{ROBOT}/teleop/frames",
                             params={"after": 10**9}).json()["latest_seq"]
            page.click("#btn-guidance-close")
            b.close()
        stats = (st.get("tier") or {}).get("stats") or {}
        return {"fps_set": fps, "seconds": round(elapsed, 1),
                "running_at_end": st.get("running"), "outcome": st.get("outcome"),
                "frames_pushed": seq1 - seq0, "fps_delivered": round((seq1 - seq0) / elapsed, 2),
                "perceived": stats.get("frames"), "frames_skipped": stats.get("frames_skipped"),
                "cloud_calls": stats.get("cloud_calls"), "stub_calls": cloud.calls - calls0,
                "calls_per_min": round(60 * (stats.get("cloud_calls") or 0) / elapsed, 2),
                "triggers": stats.get("triggers")}
    finally:
        for proc in procs:
            proc.terminate()
        for proc in procs:
            proc.wait(timeout=15)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=int, default=120)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    cloud = Cloud()
    logdir = tempfile.mkdtemp(prefix="e15-")
    rows = [run(fps, args.seconds, cloud, logdir) for fps in (2, 8)]
    low, high = rows[0]["calls_per_min"], rows[1]["calls_per_min"]
    result = {"rows": rows, "logs": logdir, "bar": "8 fps within 10% of 2 fps",
              "met": bool(low) and abs(high - low) / low <= 0.10}
    with open(args.out, "w") as f:
        json.dump(result, f, indent=2)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
