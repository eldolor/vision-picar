"""Headroom's perception load as a MISSION carries it: one target, so ONE
pipeline -- one detector and one CLIP in memory (pipeline_for loads its own
models per call; the latency bench builds one per target, seven in all) --
on real frames at the tier's budgeted rate (4 Hz = 250 ms a frame), for
`seconds`. One line per frame: time, ms, status."""
import json
import sys
import time
from pathlib import Path

from brain.perceive import pipeline_for
from control.perception_eval import frame_dict
from tools.jetson.bench_perception import FRAMES_FILE

seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 600
frames = json.loads(FRAMES_FILE.read_text())
pipe = pipeline_for("red backpack")
print("loaded 1 pipeline", flush=True)
end, i = time.time() + seconds, 0
while time.time() < end:
    f = frames[i % len(frames)]
    i += 1
    t0 = time.perf_counter()
    r = pipe.perceive(frame_dict(Path("recordings") / f["walk"] / f["file"]))
    ms = (time.perf_counter() - t0) * 1000
    print(f"{time.strftime('%T')} {ms:.0f} {r.status}", flush=True)
    time.sleep(max(0.0, 0.25 - ms / 1000))
print(f"done: {i} frames", flush=True)
