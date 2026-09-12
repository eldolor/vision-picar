# TensorRT precision runs, 2026-09-12

One g5.xlarge (A10G), TensorRT **10.13.3** -- JetPack 6.x ships 10.3, same
API, so this is closer to an Orin than TRT 11 would be. Instance torn down.

Scored by `control/perception_eval.py` over all 8 adjudicated walks (610
frames, 159 visible) through `tools/trt/trt_owlv2.py`, which runs OWLv2's
image tower as an engine and keeps the text tower in PyTorch -- the division
the robot would use (4.2).

**Designed as a comparison, not an absolute.** Only the image tower is
compiled, so an absolute number could be contaminated by the host-side join.
Two engines from the same ONNX, identical post-processing, so conventions
cancel:

| record | purpose | result |
|---|---|---|
| `trt-fp16.json` | validates the path against PyTorch fp16 | 12% / 82% / 92% -- matches (19/130/146 TP vs 19/131/146) |
| `trt-int8.json` | **INVALID** -- see below | bit-identical to fp16 |
| `trt-int8-real.json` | the answer | **4% / 7% / 16%** |

## INT8 collapses OWLv2

82% at 3 FP becomes **7%**. Not degradation. OWLv2 does not survive naive
post-training quantisation -- known ViT behaviour, and the reason
quantisation-aware training exists. Nothing here says a QAT'd OWLv2 would
fail; it says the free path does.

**So fp16 is the deployment precision by elimination**, and P7b's INT8 rows
are withdrawn: the real Orin figure is **~205 ms / 4.9 Hz**, not 124 ms.

## `trt-int8.json` is invalid and is kept as the cautionary case

Built with `IInt8EntropyCalibrator2` (deprecated in TRT >= 10.1). It
succeeded, and produced an engine whose scores were **bit-identical to fp16
on all 610 frames** -- which reads as "INT8 is free" and is the measurement
not happening. The build log said so: `Missing scale and zero-point for
tensor layer_norm.bias_output, expect fall back to non-int8 implementation`.

Three checks catch it, all cheap:

- identical-to-three-decimals is **suspicious, not clean**
- latency did not move (150 vs 153 ms)
- **the engine did not shrink** -- INT8 weights are half the size, and the
  "INT8" engine was 14KB *larger* than fp16

The real run inverts all three: 0/610 identical, mean |delta| 0.112, max
0.665, engine **98MB vs 184MB**.

## How to redo it

    python -m modelopt.onnx.quantization --onnx_path <onnx> \
      --quantize_mode int8 --calibration_data calib_nchw.npz \
      --output_path <qdq.onnx>

Then build normally -- the ranges are in the graph. **TensorRT 11 removes
implicit quantisation entirely** (no calibrator classes, no INT8/FP16
builder flags) and `pip install tensorrt` gets 11.x by default, so pin 10.x.
