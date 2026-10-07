"""
tools/jetson/bench_trt.py -- `PLAN-ros-alignment.md` 3.41 Part A: the
perception tier on TensorRT, against the shipped torch pipeline, on the
Jetson.

Arms (one per process, the same 63 pinned frames as `bench_perception.py`):

* `A`  -- today: `brain.perceive.pipeline_for` (YOLOE + CLIP RN50, torch).
* `B1` -- the same pipeline with both networks as TensorRT fp16 engines:
          YOLOE with the target baked in, CLIP's image tower. Preprocessing
          unchanged (PIL on the CPU, the JPEG decoded once per crop).
* `B2` -- B1, plus P26: the JPEG decoded once per frame and CLIP's crop
          resized and normalised on the GPU.

Nothing in `brain/` changes: B swaps the two model objects inside a
`PerceptionPipeline`, so the crop gate, the distractors and the match
probability are the shipped code's.

    python -m tools.jetson.bench_trt build --recordings recordings      # engines + build times
    python -m tools.jetson.bench_trt run A  --recordings recordings --out a.json
    python -m tools.jetson.bench_trt run B1 --recordings recordings --out b1.json
    python -m tools.jetson.bench_trt compare a.json b1.json b2.json

The engines live in ~/.cache/vision-picar/trt (never in git). YOLOE's
engine is per TARGET -- its text prompt is a constant in the graph -- so
`build` times a cold build, warm builds (TensorRT timing cache reused) and
a cache hit, which is 3.41 criterion 3.
"""

import argparse
import hashlib
import json
import os
import shutil
import statistics
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
FRAMES_FILE = HERE / "bench_frames.json"
ENGINE_DIR = Path(os.environ.get("PICAR_TRT_DIR", Path.home() / ".cache" / "vision-picar" / "trt"))
TIMING_CACHE = ENGINE_DIR / "timing.cache"
WARMUP = 3
CLIP_MAX_BATCH = 16
# open_clip's OpenAI normalisation (open_clip.constants.OPENAI_DATASET_MEAN/STD).
CLIP_MEAN = (0.48145466, 0.4578275, 0.40821073)
CLIP_STD = (0.26862954, 0.26130258, 0.27577711)


# --------------------------------------------------------------------------
# Building engines
# --------------------------------------------------------------------------

def _slug(target: str) -> str:
    return hashlib.sha1(target.encode()).hexdigest()[:12]


def build_engine(onnx_path: Path, fp16: bool = True, profile: dict = None) -> bytes:
    """ONNX -> serialised TensorRT engine, reusing and refreshing the timing
    cache. The cache is what makes a second YOLOE target cheap: its graph is
    the first one's with different constants, so every tactic is timed once."""
    import tensorrt as trt

    logger = trt.Logger(trt.Logger.WARNING)
    builder = trt.Builder(logger)
    network = builder.create_network(0)
    parser = trt.OnnxParser(network, logger)
    if not parser.parse(onnx_path.read_bytes()):
        raise RuntimeError("; ".join(str(parser.get_error(i)) for i in range(parser.num_errors)))
    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1 << 30)
    if fp16:
        config.set_flag(trt.BuilderFlag.FP16)
    cache = config.create_timing_cache(TIMING_CACHE.read_bytes() if TIMING_CACHE.exists() else b"")
    config.set_timing_cache(cache, ignore_mismatch=False)
    if profile:
        p = builder.create_optimization_profile()
        for name, (lo, opt, hi) in profile.items():
            p.set_shape(name, lo, opt, hi)
        config.add_optimization_profile(p)
    serialised = builder.build_serialized_network(network, config)
    if serialised is None:
        raise RuntimeError(f"TensorRT could not build {onnx_path.name}")
    TIMING_CACHE.write_bytes(memoryview(config.get_timing_cache().serialize()))
    return bytes(serialised)


def yoloe_engine_path(target: str) -> Path:
    return ENGINE_DIR / f"yoloe-{_slug(target)}.engine"


def build_yoloe(target: str, weights: str = None) -> dict:
    """One target's YOLOE engine, in Ultralytics' own .engine layout (a
    4-byte length, its metadata JSON, then the engine) so `YOLO(path)` loads
    it with the shipped pre/postprocessing. Returns the timings."""
    from brain.perceive import DEFAULT_YOLOE
    from ultralytics import YOLOE

    out = yoloe_engine_path(target)
    if out.exists():
        return {"target": target, "cached": True, "onnx_s": 0.0, "build_s": 0.0, "path": str(out)}
    t0 = time.perf_counter()
    work = ENGINE_DIR / f"work-{_slug(target)}"
    work.mkdir(parents=True, exist_ok=True)
    src = Path(weights or DEFAULT_YOLOE)
    local = work / src.name
    if not local.exists():
        shutil.copy(src, local)
    model = YOLOE(str(local))
    model.set_classes([target], model.get_text_pe([target]))
    onnx_path = Path(model.export(format="onnx", imgsz=640, simplify=True, verbose=False))
    import onnx
    props = {p.key: p.value for p in onnx.load(str(onnx_path)).metadata_props}
    onnx_s = time.perf_counter() - t0

    t1 = time.perf_counter()
    engine = build_engine(onnx_path)
    build_s = time.perf_counter() - t1

    # Ultralytics stores its metadata in the ONNX as strings (eval()-able
    # literals for dicts/lists); the .engine header wants the same as JSON.
    import ast
    meta = {}
    for k, v in props.items():
        try:
            meta[k] = ast.literal_eval(v)
        except (ValueError, SyntaxError):
            meta[k] = v
    blob = json.dumps(meta).encode()
    with open(out, "wb") as f:
        f.write(len(blob).to_bytes(4, byteorder="little", signed=True))
        f.write(blob)
        f.write(engine)
    shutil.rmtree(work, ignore_errors=True)
    return {"target": target, "cached": False, "onnx_s": round(onnx_s, 1),
            "build_s": round(build_s, 1), "path": str(out)}


def clip_engine_path(model_name: str) -> Path:
    return ENGINE_DIR / f"clip-{model_name.replace('/', '-')}-visual.engine"


def build_clip(model_name: str = None) -> dict:
    """CLIP's image tower as an engine taking a batch of 1..16 crops. Target-
    independent: built once per checkpoint."""
    import open_clip
    import torch
    from brain.perceive import DEFAULT_CLIP

    model_name = model_name or DEFAULT_CLIP
    out = clip_engine_path(model_name)
    if out.exists():
        return {"model": model_name, "cached": True, "build_s": 0.0, "path": str(out)}
    model, _, _ = open_clip.create_model_and_transforms(model_name, pretrained="openai", device="cpu")
    model.eval()
    onnx_path = ENGINE_DIR / f"clip-{model_name}-visual.onnx"
    t0 = time.perf_counter()
    dummy = torch.randn(1, 3, 224, 224)
    torch.onnx.export(model.visual, dummy, str(onnx_path), input_names=["image"],
                      output_names=["embedding"], opset_version=17,
                      dynamic_axes={"image": {0: "n"}, "embedding": {0: "n"}})
    engine = build_engine(onnx_path, profile={
        "image": ((1, 3, 224, 224), (1, 3, 224, 224), (CLIP_MAX_BATCH, 3, 224, 224))})
    out.write_bytes(engine)
    return {"model": model_name, "cached": False, "build_s": round(time.perf_counter() - t0, 1),
            "path": str(out)}


# --------------------------------------------------------------------------
# Arm B's two model objects
# --------------------------------------------------------------------------

class TrtYoloE:
    """`UltralyticsTextDetector`'s shape over a per-target engine. Ultralytics
    loads the .engine through AutoBackend, so letterboxing, NMS and box
    scaling are the same code as arm A's."""

    def __init__(self, target: str, confidence: float):
        from ultralytics import YOLO

        self.model = YOLO(str(yoloe_engine_path(target)), task="segment")
        self.target = target
        self.confidence = confidence
        self.weights = f"trt:{yoloe_engine_path(target).name}"
        self.model_name = self.weights

    def detect_text(self, image: bytes, text: str):
        import io
        from PIL import Image
        from brain.perceive import Box, Detection

        if text != self.target:
            raise ValueError(f"engine was built for {self.target!r}, asked for {text!r}")
        img = Image.open(io.BytesIO(image)).convert("RGB")
        results = self.model.predict(img, conf=self.confidence, verbose=False,
                                     device=0, imgsz=640)
        out = []
        for r in results:
            for b in getattr(r, "boxes", []):
                x1, y1, x2, y2 = (float(v) for v in b.xyxy[0].tolist())
                out.append(Detection(box=Box(x1, y1, x2, y2), label=text,
                                     confidence=float(b.conf[0])))
        return out


class _TrtRunner:
    """One engine, executed on torch's current CUDA stream with torch tensors
    as its buffers -- no pycuda, no host copies."""

    def __init__(self, path: Path):
        import tensorrt as trt

        self._trt = trt
        self.engine = trt.Runtime(trt.Logger(trt.Logger.WARNING)).deserialize_cuda_engine(
            path.read_bytes())
        self.context = self.engine.create_execution_context()

    def __call__(self, image):
        import torch

        image = image.contiguous().float()
        n = image.shape[0]
        self.context.set_input_shape("image", tuple(image.shape))
        out = torch.empty((n, self.engine.get_tensor_shape("embedding")[-1]),
                          dtype=torch.float32, device=image.device)
        self.context.set_tensor_address("image", image.data_ptr())
        self.context.set_tensor_address("embedding", out.data_ptr())
        self.context.execute_async_v3(torch.cuda.current_stream().cuda_stream)
        return out


def trt_clip_scorer(model_name: str = None, gpu_preprocess: bool = False):
    """A `ClipScorer` whose image tower is the engine. The text tower stays
    torch -- it runs once per mission, not per frame. With `gpu_preprocess`
    the JPEG is decoded once per frame and each crop is resized and
    normalised on the GPU (P26)."""
    import io
    import torch
    import torch.nn.functional as F
    from PIL import Image
    from brain.perceive import ClipScorer, DEFAULT_CLIP

    model_name = model_name or DEFAULT_CLIP

    class TrtClipScorer(ClipScorer):
        def __init__(self):
            super().__init__(model_name, device="cuda")
            self.runner = _TrtRunner(clip_engine_path(model_name))
            # The torch image tower is replaced, so it should not count
            # against this arm's memory: the text tower is all that stays.
            self.model.visual = None
            torch.cuda.empty_cache()
            self.model.encode_image = self.runner  # the bench times this attribute
            self._frame_key = None
            self._frame = None
            self._mean = torch.tensor(CLIP_MEAN, device="cuda").view(1, 3, 1, 1)
            self._std = torch.tensor(CLIP_STD, device="cuda").view(1, 3, 1, 1)

        def _gpu_frame(self, image: bytes):
            key = (id(image), len(image))
            if key != self._frame_key:
                pil = Image.open(io.BytesIO(image)).convert("RGB")
                w, h = pil.size
                arr = torch.frombuffer(bytearray(pil.tobytes()), dtype=torch.uint8)
                self._frame = arr.view(h, w, 3).to("cuda", non_blocking=True).permute(2, 0, 1)
                self._frame_key = key
            return self._frame

        def score(self, image: bytes, box, texts):
            if not gpu_preprocess:
                img = Image.open(io.BytesIO(image)).convert("RGB")
                crop = img.crop((int(box.x1), int(box.y1),
                                 max(int(box.x2), int(box.x1) + 1),
                                 max(int(box.y2), int(box.y1) + 1)))
                tensor = self.preprocess(crop).unsqueeze(0).to("cuda")
            else:
                frame = self._gpu_frame(image)
                x1, y1 = int(box.x1), int(box.y1)
                x2, y2 = max(int(box.x2), x1 + 1), max(int(box.y2), y1 + 1)
                crop = frame[:, y1:y2, x1:x2].unsqueeze(0).float() / 255.0
                h, w = crop.shape[-2:]
                s = 224 / min(h, w)
                nh, nw = max(224, round(h * s)), max(224, round(w * s))
                crop = F.interpolate(crop, size=(nh, nw), mode="bicubic",
                                     align_corners=False, antialias=True).clamp_(0, 1)
                top, left = (nh - 224) // 2, (nw - 224) // 2
                crop = crop[:, :, top:top + 224, left:left + 224]
                tensor = (crop - self._mean) / self._std
            with torch.no_grad():
                feats = self.model.encode_image(tensor)
                feats = feats / feats.norm(dim=-1, keepdim=True)
                sims = (feats @ self._text_features(texts).T.float())[0]
            return [float(v) for v in sims]

    return TrtClipScorer()


def pipeline_for_arm(arm: str, target: str):
    from brain.perceive import (LOW_CONFIDENCE, OpenVocabCropSource,
                                PerceptionPipeline, pipeline_for)

    if arm == "A":
        return pipeline_for(target)   # device chosen as bench_perception's 3.33 runs did
    return PerceptionPipeline(
        detector=OpenVocabCropSource(TrtYoloE(target, LOW_CONFIDENCE), target),
        scorer=trt_clip_scorer(gpu_preprocess=(arm == "B2")),
        target=target)


# --------------------------------------------------------------------------
# Measuring
# --------------------------------------------------------------------------

def _mem_available_mb() -> float:
    with open("/proc/meminfo") as f:
        for line in f:
            if line.startswith("MemAvailable:"):
                return int(line.split()[1]) / 1024.0
    return float("nan")


class MemSampler:
    """The lowest MemAvailable seen since `start`. On a Jetson the GPU's
    memory IS the system's, so this counts model weights, activations,
    the CUDA context and TensorRT's workspace alike."""

    def __init__(self, period_s: float = 0.05):
        self.period_s, self.low, self._stop = period_s, float("inf"), threading.Event()

    def _loop(self):
        while not self._stop.is_set():
            self.low = min(self.low, _mem_available_mb())
            time.sleep(self.period_s)

    def start(self):
        self.low = _mem_available_mb()
        self._t = threading.Thread(target=self._loop, daemon=True)
        self._t.start()
        return self

    def stop(self) -> float:
        self._stop.set()
        self._t.join()
        return self.low


def run_arm(arm: str, recordings: Path, memory_target: str = None) -> dict:
    """Latency and answers over the pinned frames (one pipeline per target,
    as `bench_perception` does), or -- with `memory_target` -- one mission's
    footprint: ONE pipeline loaded once, that target's frames, MemAvailable
    tracked from before the models load."""
    baseline = _mem_available_mb()
    sampler = MemSampler().start()
    import torch
    from control.perception_eval import frame_dict
    from tools.jetson.bench_perception import instrument

    frames = json.loads(FRAMES_FILE.read_text())
    if memory_target:
        frames = [f for f in frames if f["target"] == memory_target]
    pipelines, rows = {}, []
    for i, f in enumerate(frames):
        record = {"det_pre_ms": 0.0, "det_infer_ms": 0.0, "det_post_ms": 0.0,
                  "clip_gpu_ms": 0.0, "clip_score_ms": 0.0, "crops": 0}
        frame = frame_dict(recordings / f["walk"] / f["file"])
        if f["target"] not in pipelines:
            p = pipeline_for_arm(arm, f["target"])
            pipelines[f["target"]] = (p, {})
            instrument(p, torch, pipelines[f["target"]][1])
        pipe, live = pipelines[f["target"]]
        live.clear(); live.update(record)
        pipe.perceive(frame)                      # untimed: re-arms the target, as bench_perception
        live.clear(); live.update(record)
        torch.cuda.synchronize()
        t = time.perf_counter()
        perception = pipe.perceive(frame)
        torch.cuda.synchronize()
        total = (time.perf_counter() - t) * 1000
        r = dict(live)
        r["total_ms"] = total
        r["model_ms"] = r["det_infer_ms"] + r["clip_gpu_ms"]
        r["handling_ms"] = max(0.0, total - r["model_ms"])
        r["status"] = perception.status
        r["max_probability"] = (max(c.probability for c in perception.candidates)
                                if perception.candidates else None)
        r["frame"] = f"{f['walk']}/{f['file']}"
        if i >= WARMUP or memory_target:
            rows.append(r)
    low = sampler.stop()

    def q(k, frac):
        xs = sorted(x[k] for x in rows)
        return round(xs[int(frac * (len(xs) - 1))], 1) if xs else None

    keys = ("total_ms", "model_ms", "handling_ms", "det_pre_ms", "det_infer_ms",
            "det_post_ms", "clip_gpu_ms")
    return {
        "arm": arm, "memory_target": memory_target, "frames": len(rows),
        "pipelines": len(pipelines),
        "median": {k: q(k, 0.5) for k in keys}, "p90": {k: q(k, 0.9) for k in keys},
        "mem_baseline_mb": round(baseline), "mem_low_mb": round(low),
        "mem_used_mb": round(baseline - low),
        "unavailable": sum(1 for x in rows if x["status"] == "unavailable"),
        "rows": rows,
    }


def compare(paths) -> dict:
    """Criterion 1 against the first file (arm A), and the latency/memory
    table for criterion 2."""
    res = [json.loads(Path(p).read_text()) for p in paths]
    ref = {r["frame"]: r for r in res[0]["rows"]}
    out = []
    for r in res:
        agree = sum(1 for x in r["rows"] if ref[x["frame"]]["status"] == x["status"])
        deltas = [abs(x["max_probability"] - ref[x["frame"]]["max_probability"])
                  for x in r["rows"]
                  if x["max_probability"] is not None
                  and ref[x["frame"]]["max_probability"] is not None]
        out.append({"arm": r["arm"], "status_agree": f"{agree}/{len(r['rows'])}",
                    "max_dprob": round(max(deltas), 4) if deltas else None,
                    "scored_frames": len(deltas),
                    "median_ms": r["median"]["total_ms"], "p90_ms": r["p90"]["total_ms"],
                    "model_med_ms": r["median"]["model_ms"],
                    "handling_med_ms": r["median"]["handling_ms"],
                    "handling_p90_ms": r["p90"]["handling_ms"],
                    "mem_used_mb": r.get("mem_used_mb"),
                    "unavailable": r["unavailable"]})
    return {"arms": out}


# 3.41's bars, written before measuring (PLAN-ros-alignment.md 3.41).
PROB_BAR = 0.01            # criterion 1: CLIP probability within this of arm A
P90_GAIN_BAR = 0.30        # criterion 2: p90 at least 30% below the rival...
MEM_GAIN_BAR_MB = 500      # ...or peak memory at least 500 MB below it
REBUILD_BAR_S = 60.0       # criterion 3: a new target ready within this


def verdict(arm: dict, ref: dict, arm_mem: dict, ref_mem: dict) -> dict:
    """Criteria 1 and 2 for one arm against its rival (`ref`): the same
    frames' answers, then p90 latency or a mission's memory. Memory is the
    ONE-pipeline run (`--memory-target`), because a mission loads one
    pipeline; the 63-frame run loads one per target."""
    rows = {r["frame"]: r for r in ref["rows"]}
    status_ok = all(rows[r["frame"]]["status"] == r["status"] for r in arm["rows"])
    deltas = [abs(r["max_probability"] - rows[r["frame"]]["max_probability"])
              for r in arm["rows"]
              if r["max_probability"] is not None
              and rows[r["frame"]]["max_probability"] is not None]
    proposals_ok = all((r["max_probability"] is None) == (rows[r["frame"]]["max_probability"] is None)
                       for r in arm["rows"])
    max_d = max(deltas) if deltas else 0.0
    same_answers = status_ok and proposals_ok and max_d <= PROB_BAR
    p90_gain = 1 - arm["p90"]["total_ms"] / ref["p90"]["total_ms"]
    mem_gain = ref_mem["mem_used_mb"] - arm_mem["mem_used_mb"]
    faster_or_lighter = p90_gain >= P90_GAIN_BAR or mem_gain >= MEM_GAIN_BAR_MB
    return {"arm": arm["arm"], "vs": ref["arm"], "status_agree": status_ok,
            "proposals_agree": proposals_ok, "max_dprob": round(max_d, 4),
            "criterion_1": same_answers, "p90_gain": round(p90_gain, 3),
            "mem_gain_mb": mem_gain, "criterion_2": faster_or_lighter,
            "adopt": same_answers and faster_or_lighter}


def room_for_c(b: dict) -> float:
    """3.41's amendment: the share of B's p90 that is NOT GPU model time --
    the most an Isaac ROS arm on the same engines could remove."""
    return round(1 - b["p90"]["model_ms"] / b["p90"]["total_ms"], 3)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("build")
    b.add_argument("--recordings", type=Path, required=True)
    b.add_argument("--out", type=Path)
    r = sub.add_parser("run")
    r.add_argument("arm", choices=["A", "B1", "B2"])
    r.add_argument("--recordings", type=Path, required=True)
    r.add_argument("--memory-target", default=None)
    r.add_argument("--out", type=Path)
    c = sub.add_parser("compare")
    c.add_argument("files", nargs="+")
    args = ap.parse_args(argv)

    if args.cmd == "build":
        ENGINE_DIR.mkdir(parents=True, exist_ok=True)
        targets = []
        for f in json.loads(FRAMES_FILE.read_text()):
            if f["target"] not in targets:
                targets.append(f["target"])
        report = {"timing_cache_existed": TIMING_CACHE.exists(), "clip": build_clip(),
                  "yoloe": []}
        for t in targets:
            row = build_yoloe(t)
            print(json.dumps(row), flush=True)
            report["yoloe"].append(row)
        # A repeat target is a cache hit: loading the engine, nothing built.
        t0 = time.perf_counter()
        TrtYoloE(targets[0], 0.1)
        report["cache_hit_load_s"] = round(time.perf_counter() - t0, 2)
        if args.out:
            args.out.write_text(json.dumps(report, indent=1) + "\n")
        print(json.dumps({k: v for k, v in report.items() if k != "yoloe"}))
        return 0
    if args.cmd == "run":
        res = run_arm(args.arm, args.recordings, args.memory_target)
        if args.out:
            args.out.write_text(json.dumps(res, indent=1) + "\n")
        print(json.dumps({k: v for k, v in res.items() if k != "rows"}, indent=1))
        return 1 if res["unavailable"] else 0
    print(json.dumps(compare(args.files), indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
