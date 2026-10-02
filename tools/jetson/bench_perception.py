"""
tools/jetson/bench_perception.py -- the perception tier's per-frame time,
split into GPU and CPU work. `PLAN-ros-alignment.md` 3.33 criterion 3, and
P26's baseline (`PLAN-onboard-perception.md`).

Runs the SHIPPED pipeline (`brain.perceive.pipeline_for`: YOLOE crops ->
CLIP) over a PINNED frame set (`bench_frames.json`, written by `--pin` once
and committed), so the laptop and the Jetson time exactly the same frames.
Per frame it records:

* `total_ms`      -- `pipeline.perceive(frame)`, end to end;
* `det_pre/det_infer/det_post_ms` -- Ultralytics' own timings for YOLOE
  (its preprocessing and postprocessing run on the CPU, inference on the
  device);
* `clip_gpu_ms`   -- CLIP's image encoder, synchronised on CUDA;
* `clip_cpu_ms`   -- everything else in CLIP scoring: decode, crop, resize,
  normalise (score() total minus the encoder);
* `crops`         -- how many crops CLIP scored.

GPU work is `det_infer + clip_gpu`; everything else in `total` is the CPU's.
The budget is 250 ms a frame at 15 W (3.33, confirmed by the user).

    python -m tools.jetson.bench_perception --recordings ../vision-picar/recordings
    python -m tools.jetson.bench_perception --pin --recordings ...   # once, to choose frames
"""

import argparse
import json
import platform
import statistics
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
FRAMES_FILE = HERE / "bench_frames.json"
BUDGET_MS = 250.0
DEFAULT_N = 60
WARMUP = 3


def pin_frames(recordings: Path, n: int) -> list:
    """Choose `n` frames spread evenly over every labelled walk, deterministic,
    and write them to `bench_frames.json` -- names only, never images."""
    from control.perception_eval import load_corpus

    walks = load_corpus(recordings)
    pool = [(w.name, p.name, w.target) for w in walks for (p, _vis, _adj) in w.frames]
    step = max(1, len(pool) // n)
    chosen = pool[::step][:n]
    FRAMES_FILE.write_text(json.dumps(
        [{"walk": w, "file": f, "target": t} for w, f, t in chosen], indent=1) + "\n")
    return chosen


def _sync(torch):
    if torch is not None and torch.cuda.is_available():
        torch.cuda.synchronize()


def instrument(pipeline, torch, record: dict) -> None:
    """Wrap the pipeline's models so each frame's stage times land in
    `record`. Instance-level wrappers: nothing in brain/ changes."""
    det = pipeline.detector
    yolo = getattr(getattr(det, "backend", det), "model", None)
    if yolo is not None:
        predict = yolo.predict

        def timed_predict(*a, **kw):
            results = predict(*a, **kw)
            sp = getattr(results[0], "speed", {}) if results else {}
            record["det_pre_ms"] += sp.get("preprocess", 0.0)
            record["det_infer_ms"] += sp.get("inference", 0.0)
            record["det_post_ms"] += sp.get("postprocess", 0.0)
            return results
        yolo.predict = timed_predict

    scorer = pipeline.scorer
    encode = scorer.model.encode_image
    score = scorer.score

    def timed_encode(*a, **kw):
        _sync(torch)
        t = time.perf_counter()
        out = encode(*a, **kw)
        _sync(torch)
        record["clip_gpu_ms"] += (time.perf_counter() - t) * 1000
        return out

    def timed_score(*a, **kw):
        t = time.perf_counter()
        out = score(*a, **kw)
        record["clip_score_ms"] += (time.perf_counter() - t) * 1000
        record["crops"] += 1
        return out
    scorer.model.encode_image = timed_encode
    scorer.score = timed_score


def run(recordings: Path, device=None) -> dict:
    from brain.perceive import pipeline_for
    from control.perception_eval import frame_dict
    try:
        import torch
    except ImportError:  # pragma: no cover
        torch = None

    frames = json.loads(FRAMES_FILE.read_text())
    pipelines, rows = {}, []
    for i, f in enumerate(frames):
        record = {"det_pre_ms": 0.0, "det_infer_ms": 0.0, "det_post_ms": 0.0,
                  "clip_gpu_ms": 0.0, "clip_score_ms": 0.0, "crops": 0}
        frame = frame_dict(recordings / f["walk"] / f["file"])
        if f["target"] not in pipelines:
            p = pipeline_for(f["target"], device=device)
            pipelines[f["target"]] = (p, {})
            instrument(p, torch, pipelines[f["target"]][1])
        pipe, live = pipelines[f["target"]]
        # YOLOE encodes its text prompt when the target changes -- once per
        # MISSION, not per frame -- so each frame first re-arms this
        # target's prompt untimed. Timing it would charge a set-up cost to
        # whichever frame followed a target switch (the first laptop run's
        # p90 of 465 ms was mostly that).
        live.clear()
        live.update(record)
        pipe.perceive(frame)
        live.clear()
        live.update(record)
        _sync(torch)
        t = time.perf_counter()
        perception = pipe.perceive(frame)
        _sync(torch)
        total = (time.perf_counter() - t) * 1000
        r = dict(live)
        r["total_ms"] = total
        r["clip_cpu_ms"] = max(0.0, r["clip_score_ms"] - r["clip_gpu_ms"])
        # "model" is the two networks' own compute; "handling" is everything
        # around them. On the Jetson both networks run on CUDA, so model is
        # the GPU's share and handling the CPU's. On a Mac the detector runs
        # on the CPU and CLIP on MPS -- read the devices line, not the names.
        r["model_ms"] = r["det_infer_ms"] + r["clip_gpu_ms"]
        r["handling_ms"] = max(0.0, total - r["model_ms"])
        r["clip_ms_per_crop"] = (r["clip_score_ms"] / r["crops"]) if r["crops"] else None
        r["status"] = getattr(perception, "status", None)
        r["frame"] = f"{f['walk']}/{f['file']}"
        if i >= WARMUP:
            rows.append(r)

    def vals(k):
        return sorted(x[k] for x in rows if x[k] is not None) or [0.0]

    def med(k):
        return round(statistics.median(vals(k)), 1)

    def p90(k):
        xs = vals(k)
        return round(xs[int(0.9 * (len(xs) - 1))], 1)

    keys = ["total_ms", "model_ms", "handling_ms", "det_pre_ms", "det_infer_ms",
            "det_post_ms", "clip_gpu_ms", "clip_cpu_ms", "clip_ms_per_crop", "crops"]
    cuda = bool(torch is not None and torch.cuda.is_available())
    any_pipe = next(iter(pipelines.values()))[0]
    yolo = getattr(getattr(any_pipe.detector, "backend", any_pipe.detector), "model", None)
    # Ultralytics records the device it ran on in its PREDICTOR; `model.device`
    # keeps reading "cpu" after a run on another device (seen on MPS).
    ran_on = getattr(getattr(yolo, "predictor", None), "device", None) or getattr(yolo, "device", "?")
    det_device = str(getattr(ran_on, "type", ran_on))
    return {
        "host": platform.node(), "machine": platform.machine(),
        "python": platform.python_version(),
        "torch": getattr(torch, "__version__", None), "cuda": cuda,
        "device": (torch.cuda.get_device_name(0) if cuda else "no cuda"),
        "detector_device": det_device, "clip_device": str(any_pipe.scorer.device),
        "frames_with_crops": sum(1 for x in rows if x["crops"]),
        "frames": len(rows), "warmup": WARMUP,
        "median": {k: med(k) for k in keys},
        "p90": {k: p90(k) for k in keys},
        "budget_ms": BUDGET_MS,
        "within_budget": med("total_ms") <= BUDGET_MS,
        "rows": rows,
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--recordings", type=Path, required=True)
    ap.add_argument("--pin", action="store_true", help="choose and write the frame set")
    ap.add_argument("-n", type=int, default=DEFAULT_N)
    ap.add_argument("--device", default=None)
    ap.add_argument("--out", type=Path, default=None, help="write the full result as JSON")
    args = ap.parse_args(argv)
    if args.pin:
        chosen = pin_frames(args.recordings, args.n + WARMUP)
        print(f"pinned {len(chosen)} frames -> {FRAMES_FILE}")
        return 0
    res = run(args.recordings, args.device)
    if args.out:
        args.out.write_text(json.dumps(res, indent=1) + "\n")
    print(f"{res['host']} ({res['machine']}, python {res['python']}, torch {res['torch']}, "
          f"cuda: {res['device']}) -- {res['frames']} frames after {res['warmup']} warm-up")
    print(f"  devices: detector {res['detector_device']}, CLIP {res['clip_device']}; "
          f"{res['frames_with_crops']} of {res['frames']} frames had crops to score")
    for k in ("total_ms", "model_ms", "handling_ms", "det_pre_ms", "det_infer_ms", "det_post_ms",
              "clip_gpu_ms", "clip_cpu_ms", "clip_ms_per_crop", "crops"):
        print(f"  {k:13} median {res['median'][k]:8}   p90 {res['p90'][k]:8}")
    print(f"  budget {BUDGET_MS:.0f} ms: {'WITHIN' if res['within_budget'] else 'OVER'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
