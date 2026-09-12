"""OWLv2's image tower as a TensorRT engine, at fp16 or INT8.

**Why this exists.** Every Orin latency figure in `PLAN-onboard-perception.md`
P7b assumes INT8 -- 36ms of GPU work against fp16's 163ms -- and INT8's
accuracy cost had never been measured. fp16 was measured (it is free) because
fp16 is one PyTorch flag; INT8 is not a flag, it is a different toolchain.

And the toolchain matters: `torch.ao.quantization`, bitsandbytes and
TensorRT all produce different weights, so only a TensorRT measurement
predicts a Jetson. TensorRT is what a Jetson runs.

**The experiment is a comparison, not an absolute.** This runs the SAME ONNX
through TensorRT twice, fp16 and INT8, with identical post-processing:

    TRT fp16  vs  PyTorch fp16   -> validates the path
    TRT INT8  vs  TRT fp16       -> the answer, conventions cancelling

Only the image tower is compiled -- the text tower stays in PyTorch, exactly
as it would on the robot (4.2) -- and `tools/hailo/owlv2_host_head.py` joins
the five per-patch tensors to a text embedding. That split is already
verified against the unmodified model to 1.4e-05.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional, Sequence

import numpy as np

OUT_NAMES = ("image_class_embeds", "logit_shift", "logit_scale",
             "pred_boxes", "objectness_logits")


def _trt():
    try:
        import tensorrt as trt
        return trt
    except ImportError as exc:  # pragma: no cover
        raise RuntimeError("TensorRT is not installed in this interpreter") from exc


def _calibrator(trt, torch, calib_path: Path, batch: int, cache: Path):
    """Entropy calibrator fed from the corpus calibration array.

    The same 128 frames the Hailo attempt used -- stratified across all
    eleven walks and balanced on labels.json, so the quantisation ranges are
    fitted to the viewpoint the robot actually has.
    """

    class _Cal(trt.IInt8EntropyCalibrator2):
        def __init__(self):
            super().__init__()
            self.data = np.load(calib_path, mmap_mode="r")
            self.batch = batch
            self.i = 0
            # NHWC on disk (what the Hailo toolchain wanted); the ONNX takes
            # NCHW, so transpose here rather than storing a second copy.
            self.buf = torch.empty((batch, 3, self.data.shape[1],
                                    self.data.shape[2]),
                                   dtype=torch.float32, device="cuda")
            self.cache = cache

        def get_batch_size(self):
            return self.batch

        def get_batch(self, names):
            if self.i + self.batch > len(self.data):
                return None
            chunk = np.ascontiguousarray(
                self.data[self.i:self.i + self.batch].transpose(0, 3, 1, 2))
            self.buf.copy_(torch.from_numpy(chunk))
            self.i += self.batch
            return [int(self.buf.data_ptr())]

        def read_calibration_cache(self):
            return self.cache.read_bytes() if self.cache.exists() else None

        def write_calibration_cache(self, data):
            self.cache.write_bytes(data)

    return _Cal()


def build_engine(onnx_path: Path, out_path: Path, precision: str = "fp16",
                 calib_path: Optional[Path] = None,
                 workspace_gb: int = 8) -> Path:
    """ONNX -> serialised TensorRT engine."""
    trt = _trt()
    import torch

    logger = trt.Logger(trt.Logger.WARNING)
    builder = trt.Builder(logger)
    network = builder.create_network(
        1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    parser = trt.OnnxParser(network, logger)
    if not parser.parse(onnx_path.read_bytes()):
        msgs = [parser.get_error(i) for i in range(parser.num_errors)]
        raise RuntimeError(f"ONNX parse failed: {msgs}")

    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE,
                                 workspace_gb << 30)
    if precision in ("fp16", "int8"):
        config.set_flag(trt.BuilderFlag.FP16)
    if precision == "int8":
        if calib_path is None:
            raise ValueError("int8 needs a calibration array")
        config.set_flag(trt.BuilderFlag.INT8)
        config.int8_calibrator = _calibrator(
            trt, torch, calib_path, 1, out_path.with_suffix(".calib"))

    print(f"[trt] building {precision} engine from {onnx_path.name} "
          f"-- this takes minutes", flush=True)
    blob = builder.build_serialized_network(network, config)
    if blob is None:
        raise RuntimeError("engine build returned None")
    out_path.write_bytes(blob)
    print(f"[trt] wrote {out_path} ({out_path.stat().st_size/1e6:.0f} MB)")
    return out_path


class TrtOwlv2:
    """OWLv2 with its image tower on TensorRT, behind the Detector Protocol.

    The text tower stays in PyTorch and runs once per target string, which is
    the division the robot would use and the reason only the image side was
    ever exported.
    """

    def __init__(self, engine: str, model: str = "google/owlv2-base-patch16-ensemble",
                 threshold: float = 0.0, device: Optional[str] = None,
                 **_ignored):
        trt = _trt()
        import torch
        from transformers import Owlv2Processor, Owlv2TextModel  # noqa
        from transformers import Owlv2ForObjectDetection

        self._torch = torch
        self._trt = trt
        self.device = device or "cuda"
        self.processor = Owlv2Processor.from_pretrained(model)
        # Only the text half is needed in PyTorch; loading the whole model is
        # simpler than surgery and costs memory we have.
        self._full = Owlv2ForObjectDetection.from_pretrained(model).to(
            self.device).eval()

        logger = trt.Logger(trt.Logger.WARNING)
        self.engine = trt.Runtime(logger).deserialize_cuda_engine(
            Path(engine).read_bytes())
        self.context = self.engine.create_execution_context()
        self.stream = torch.cuda.Stream()

        self._in_name = self.engine.get_tensor_name(0)
        self._bufs = {}
        # Dtypes come from the ENGINE. An fp16 build may declare fp16 IO, and
        # a float32 buffer bound to an fp16 tensor is silently garbage rather
        # than an error.
        _np2torch = {np.float32: torch.float32, np.float16: torch.float16,
                     np.int32: torch.int32, np.int8: torch.int8}
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            shape = tuple(self.engine.get_tensor_shape(name))
            nd = trt.nptype(self.engine.get_tensor_dtype(name))
            self._bufs[name] = torch.empty(
                shape, dtype=_np2torch.get(nd, torch.float32),
                device=self.device)

        self.threshold = threshold
        self.weights = f"trt:{Path(engine).name}"
        self.model_name = self.weights
        self._query_cache: dict = {}

    def _text_embed(self, text: str):
        if text not in self._query_cache:
            torch = self._torch
            # The tokenizer directly: Owlv2Processor(images=None) is not
            # portable across versions, and the text tower is all this needs.
            tok = self.processor.tokenizer([text], return_tensors="pt",
                                           padding=True)
            ids, mask = tok["input_ids"], tok["attention_mask"]
            with torch.no_grad():
                q = self._full.owlv2.get_text_features(
                    input_ids=ids.to(self.device),
                    attention_mask=mask.to(self.device)).pooler_output
            self._query_cache[text] = q.float().cpu().numpy()
        return self._query_cache[text]

    def detect_text(self, image: bytes, text: str):  # pragma: no cover
        import io
        from PIL import Image
        from brain.perceive_lab import Detection, Box
        from tools.hailo.owlv2_host_head import join

        torch = self._torch
        img = Image.open(io.BytesIO(image)).convert("RGB")
        px = self.processor(images=img, return_tensors="pt")["pixel_values"]
        buf = self._bufs[self._in_name]
        buf.copy_(px.to(self.device, dtype=buf.dtype))

        for name, buf in self._bufs.items():
            self.context.set_tensor_address(name, int(buf.data_ptr()))
        self.context.execute_async_v3(self.stream.cuda_stream)
        self.stream.synchronize()

        parts = {n: self._bufs[n].float().cpu().numpy() for n in OUT_NAMES}
        logits, boxes, _ = join(parts, self._text_embed(text))
        scores = (1.0 / (1.0 + np.exp(-logits)))[0].max(axis=-1)
        b = boxes[0]
        w, h = img.size
        keep = scores >= self.threshold
        out = []
        for s, (cx, cy, bw, bh) in zip(scores[keep], b[keep]):
            out.append(Detection(
                box=Box(float((cx - bw / 2) * w), float((cy - bh / 2) * h),
                        float((cx + bw / 2) * w), float((cy + bh / 2) * h)),
                label=text, confidence=float(s)))
        out.sort(key=lambda d: -d.confidence)
        return out[:100]


__all__ = ["build_engine", "TrtOwlv2"]
