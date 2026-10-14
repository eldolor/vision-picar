"""Headroom's perception load as a MISSION carries it: one target, so ONE
pipeline -- one detector and one CLIP in memory (pipeline_for loads its own
models per call; the latency bench builds one per target, seven in all) --
on real frames at the tier's budgeted rate (4 Hz = 250 ms a frame), for
`seconds`. One line per frame: time, ms, status.

3.81.A: with PERCEPTION_TORCH_LOG set, every 20th frame appends torch's CUDA
allocator counters (MB) to that file as one JSON line, so the memory
accounting can split this process's GPU memory into tensors, the
allocator's cache and the rest (CUDA context, cuDNN/cuBLAS workspaces)."""
import json
import os
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
TORCH_LOG = os.environ.get("PERCEPTION_TORCH_LOG")


def log_torch() -> None:
    import torch
    if not torch.cuda.is_available():
        return
    st = torch.cuda.memory_stats()
    mb = 1 << 20
    rec = {"t": round(time.time(), 1),
           "allocated_mb": round(st.get("allocated_bytes.all.current", 0) / mb, 1),
           "reserved_mb": round(st.get("reserved_bytes.all.current", 0) / mb, 1),
           "peak_allocated_mb": round(st.get("allocated_bytes.all.peak", 0) / mb, 1),
           "peak_reserved_mb": round(st.get("reserved_bytes.all.peak", 0) / mb, 1),
           "alloc_retries": st.get("num_alloc_retries", 0)}
    with open(TORCH_LOG, "a") as f:
        f.write(json.dumps(rec) + "\n")


end, i = time.time() + seconds, 0
while time.time() < end:
    f = frames[i % len(frames)]
    i += 1
    t0 = time.perf_counter()
    r = pipe.perceive(frame_dict(Path("recordings") / f["walk"] / f["file"]))
    ms = (time.perf_counter() - t0) * 1000
    print(f"{time.strftime('%T')} {ms:.0f} {r.status}", flush=True)
    if TORCH_LOG and i % 20 == 1:
        log_torch()
    time.sleep(max(0.0, 0.25 - ms / 1000))
print(f"done: {i} frames", flush=True)
