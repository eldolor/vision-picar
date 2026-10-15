"""Headroom's perception load as a MISSION carries it: one target, so ONE
pipeline -- one detector and one CLIP in memory (pipeline_for loads its own
models per call; the latency bench builds one per target, seven in all) --
on real frames at the tier's budgeted rate (4 Hz = 250 ms a frame), for
`seconds`. One line per frame: time, ms, status.

3.81.A: with PERCEPTION_TORCH_LOG set, every 20th frame appends torch's CUDA
allocator counters (MB) to that file as one JSON line, so the memory
accounting can split this process's GPU memory into tensors, the
allocator's cache and the rest (CUDA context, cuDNN/cuBLAS workspaces).

3.81 amendment 2: with PERCEPTION_SWITCH_FILE set, each line appended to
that file is a new target: the loop builds its pipeline as a mission start
does (`pipeline_for`) while the previous one is still alive, then drops the
previous one, logging "switch start <epoch s> <target>" and "switch done <epoch s> <build
s> <target>" so the harness can read the memory dip around it. With
PERCEPTION_SWITCH_RELEASE=1 (3.81 amendment 3) the switch ends the previous
"mission" first: the pipeline is held by a real `TieredVision`, closed before
`pipeline_for` runs, as `MissionRunner._finish` closes it in the brain. The build
time covers the models only: the target's text features (T1-T2's child) are
encoded on the new pipeline's first frames, inside the harness's 90 s."""
import json
import os
import sys
import time
from pathlib import Path

from brain.perceive import pipeline_for
from brain.tiered import TieredVision
from control.perception_eval import frame_dict
from tools.jetson.bench_perception import FRAMES_FILE

seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 600
frames = json.loads(FRAMES_FILE.read_text())
# The pipeline is held as a mission holds it: by its tiered policy (no cloud).
holder = TieredVision(pipeline_for("red backpack"), lambda frame: {})
print("loaded 1 pipeline", flush=True)
TORCH_LOG = os.environ.get("PERCEPTION_TORCH_LOG")
SWITCH_FILE = os.environ.get("PERCEPTION_SWITCH_FILE")
SWITCH_RELEASE = os.environ.get("PERCEPTION_SWITCH_RELEASE") == "1"
switched = 0


def maybe_switch() -> None:
    global holder, switched
    if not SWITCH_FILE or not os.path.exists(SWITCH_FILE):
        return
    targets = [t.strip() for t in Path(SWITCH_FILE).read_text().splitlines() if t.strip()]
    for target in targets[switched:]:
        switched += 1
        print(f"{time.strftime('%T')} switch start {int(time.time())} {target}", flush=True)
        t0 = time.perf_counter()
        if SWITCH_RELEASE:
            holder.close()  # the previous mission ends before the next starts
        # Without SWITCH_RELEASE the previous pipeline is alive through the
        # build (C2); with it, closing the policy is what frees it (3.81.E).
        holder = TieredVision(pipeline_for(target), lambda frame: {})
        print(f"{time.strftime('%T')} switch done {int(time.time())} "
              f"{time.perf_counter() - t0:.1f} {target}", flush=True)


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
    maybe_switch()
    f = frames[i % len(frames)]
    i += 1
    t0 = time.perf_counter()
    r = holder.pipeline.perceive(frame_dict(Path("recordings") / f["walk"] / f["file"]))
    ms = (time.perf_counter() - t0) * 1000
    print(f"{time.strftime('%T')} {ms:.0f} {r.status}", flush=True)
    if TORCH_LOG and i % 20 == 1:
        log_torch()
    time.sleep(max(0.0, 0.25 - ms / 1000))
print(f"done: {i} frames", flush=True)
