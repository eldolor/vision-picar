"""3.81.A: where perception's host memory goes (one YOLOE + CLIP pipeline,
as tools/jetson/perception_loop.py carries it), on the board.

  .venv/bin/python tools/jetson/perception_memory_probe.py

Prints one JSON line per stage (the process's own smaps_rollup), then the
tensors alive after the first frame, summed by device and by the module that
owns them. Stdout only; nothing is written.
"""
import gc
import json
import time
from pathlib import Path


def own() -> dict:
    out = {}
    for line in Path("/proc/self/smaps_rollup").read_text().splitlines()[1:]:
        k, _, v = line.partition(":")
        if k in ("Rss", "Pss_Anon", "Pss_File"):
            out[k + "_mb"] = round(int(v.split()[0]) / 1024, 1)
    return out


def stage(name: str) -> None:
    gc.collect()
    print(json.dumps({"stage": name, **own()}), flush=True)


def main() -> int:
    stage("python")
    import torch
    stage("import torch")
    conv = torch.nn.Conv2d(3, 16, 3).cuda()
    conv(torch.zeros(1, 3, 64, 64, device="cuda"))
    torch.cuda.synchronize()
    del conv
    stage("torch + CUDA + cuDNN")
    from brain.perceive import pipeline_for  # noqa: F401 (imports ultralytics, open_clip)
    import ultralytics  # noqa: F401
    import open_clip  # noqa: F401
    stage("import brain.perceive, ultralytics, open_clip")
    from control.perception_eval import frame_dict
    from tools.jetson.bench_perception import FRAMES_FILE
    pipe = pipeline_for("red backpack")
    stage("pipeline_for (YOLOE + CLIP loaded, prompt set)")
    frames = json.loads(FRAMES_FILE.read_text())
    for f in frames[:5]:
        pipe.perceive(frame_dict(Path("recordings") / f["walk"] / f["file"]))
    stage("after 5 frames")

    # Tensors alive, by device, and the modules that hold the CPU ones.
    seen, by_dev = set(), {}
    for o in gc.get_objects():
        try:
            if isinstance(o, torch.Tensor):
                st = o.untyped_storage()
                key = (o.device.type, st.data_ptr())
                if key in seen:
                    continue
                seen.add(key)
                by_dev[o.device.type] = by_dev.get(o.device.type, 0) + st.nbytes()
        except Exception:  # noqa: BLE001 (meta or odd tensors)
            continue
    print(json.dumps({"tensors_mb_by_device": {k: round(v / 2**20, 1) for k, v in by_dev.items()}}))
    mods = []
    for o in gc.get_objects():
        try:
            if isinstance(o, (torch.nn.Module, torch.jit.ScriptModule)):
                ps = list(o.parameters()) + list(o.buffers())
                if not ps:
                    continue
                dev = {p.device.type for p in ps}
                mb = sum(p.numel() * p.element_size() for p in ps) / 2**20
                if mb > 20:
                    mods.append((round(mb, 1), type(o).__module__ + "." + type(o).__name__,
                                 sorted(dev)))
        except Exception:  # noqa: BLE001
            continue
    # Exact duplicates dropped only. Nested modules are all listed, and a
    # module's size includes its children's: do not add the rows up.
    for mb, name, dev in sorted(set((m, n, tuple(d)) for m, n, d in mods), reverse=True)[:25]:
        print(json.dumps({"module": name, "mb": mb, "device": list(dev)}))
    time.sleep(0.5)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
