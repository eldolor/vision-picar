"""Does INT8 preserve the FLOOR MASK? -- the measurement P16 made decisive.

P16's tier table is the reason this exists:

    floor mask ON   fp32 50%  10H 49%  8L 49%
    floor mask OFF  fp32 20%  10H 13%  8L  3%

A detector degraded from 34% to 5% costs the tier one point with the mask
and seventeen without it. So the quantization that actually decides
whether the on-board tier works is SegFormer's, not the detector's, and
nobody has measured it.

The metric is **floor-mask IoU against the same model in fp32**, not
against a human label. That is deliberate and it mirrors
`proposal_agreement.py`'s argument: the mask's job in the tier is to
propose regions for CLIP, ADE20K's own floor labels are not this corpus's
ground truth, and what quantization can break is agreement with the model
we measured the 50% with. A per-frame IoU also degrades gracefully -- it
says *how much* was lost, where a recall number over a gate would only say
whether a threshold moved.

Two traps carried over from P13, both of which silently produce a wrong
answer:

  * **`normalization()` is applied in SDK_QUANTIZED and NOT in
    SDK_NATIVE.** Feed native 0-1 and quantized 0-255. Backwards gives
    correlations around 0.1-0.35 and 30x magnitudes, which reads exactly
    like a wiring bug.
  * **The optimization level must be FORCED.** Without a GPU the DFC drops
    to 0, skips every accuracy pass, and still reports success -- which is
    what P11 measured and had to withdraw.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def floor_mask(logits, floor_ids):
    """(H, W) bool: pixels whose argmax class is one of `floor_ids`.

    The argmax is over ALL classes and then tested for membership, not an
    argmax over the floor classes alone -- `brain/perceive.py` asks "is
    this pixel floor", and a pixel that is most-likely sofa must not
    become floor because floor beat rug.
    """
    import numpy as np
    top = logits.argmax(axis=0)
    return np.isin(top, floor_ids)


def iou(a, b) -> float:
    import numpy as np
    inter = np.logical_and(a, b).sum()
    union = np.logical_or(a, b).sum()
    return float(inter / union) if union else 1.0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--build", type=Path, default=Path("build/segformer"))
    ap.add_argument("--arch", default="hailo10h")
    ap.add_argument("--frames", type=Path, required=True,
                    help="file listing image paths, one per line")
    ap.add_argument("--recordings", type=Path, default=Path("."))
    ap.add_argument("--optimization-level", type=int, default=2)
    ap.add_argument("--calib-limit", type=int, default=None)
    ap.add_argument("--limit", type=int, default=None)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args(argv)

    import numpy as np
    import onnxruntime as ort
    from PIL import Image
    from hailo_sdk_client import ClientRunner, InferenceContext

    meta = json.loads((args.build / "export_meta.json").read_text())
    size = meta["image_size"]
    floor_ids = meta["floor_ids"]
    onnx_path = args.build / meta["onnx"]
    mean = np.array(meta["image_mean"], dtype=np.float32)
    std = np.array(meta["image_std"], dtype=np.float32)

    calib = np.load(args.build / "calib_uint8.npy", mmap_mode="r")
    if args.calib_limit:
        calib = calib[:args.calib_limit]
    calib = np.ascontiguousarray(calib)
    print(f"[seg] calibration: {calib.shape}")

    runner = ClientRunner(hw_arch=args.arch)
    runner.translate_onnx_model(
        str(onnx_path), "segformer_b0_ade",
        net_input_shapes={"pixel_values": [1, 3, size, size]})
    script = (f"segformer_input_norm = normalization("
              f"{[round(float(m) * 255, 4) for m in mean]}, "
              f"{[round(float(s) * 255, 4) for s in std]})\n"
              f"model_optimization_flavor(optimization_level="
              f"{args.optimization_level})\n")
    runner.load_model_script(script)
    runner.optimize(calib)
    print("[seg] optimized")

    sess = ort.InferenceSession(str(onnx_path),
                                providers=["CPUExecutionProvider"])

    paths = [l.strip() for l in args.frames.read_text().splitlines() if l.strip()]
    if args.limit:
        paths = paths[:args.limit]

    rows = []
    with runner.infer_context(InferenceContext.SDK_QUANTIZED) as ctx:
        for rel in paths:
            p = args.recordings / rel
            if not p.exists():
                continue
            img = Image.open(p).convert("RGB").resize((size, size))
            raw = np.asarray(img, dtype=np.uint8)

            # fp32: NCHW, ImageNet-normalised 0-1 domain.
            x = (raw.astype(np.float32) / 255.0 - mean) / std
            ref = sess.run(None, {"pixel_values":
                                  x.transpose(2, 0, 1)[None]})[0][0]

            # quantized: NHWC uint8, because the model script's
            # normalization() runs INSIDE the quantized graph.
            q = runner.infer(ctx, raw[None].astype(np.float32))
            q = np.asarray(q)[0]
            if q.ndim == 3 and q.shape[-1] == ref.shape[0]:
                q = q.transpose(2, 0, 1)   # NHWC -> NCHW

            a, b = floor_mask(ref, floor_ids), floor_mask(q, floor_ids)
            rows.append({"frame": rel, "iou": iou(a, b),
                         "fp32_floor_px": int(a.sum()),
                         "int8_floor_px": int(b.sum())})
            if len(rows) % 25 == 0:
                print(f"[seg] {len(rows)}/{len(paths)}", flush=True)

    ious = [r["iou"] for r in rows]
    summary = {
        "arch": args.arch,
        "optimization_level": args.optimization_level,
        "frames": len(rows),
        "mean_iou": float(np.mean(ious)) if ious else None,
        "median_iou": float(np.median(ious)) if ious else None,
        "frames_below_0.5": sum(1 for v in ious if v < 0.5),
        "frames_below_0.8": sum(1 for v in ious if v < 0.8),
        "mean_fp32_floor_px": float(np.mean([r["fp32_floor_px"] for r in rows])) if rows else None,
        "mean_int8_floor_px": float(np.mean([r["int8_floor_px"] for r in rows])) if rows else None,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps({"summary": summary, "frames": rows}, indent=2))
    print("\n" + json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
