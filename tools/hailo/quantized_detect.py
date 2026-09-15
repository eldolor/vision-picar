"""Run the QUANTIZED YOLO-World body in Hailo emulation, and turn its
activations into detections through the real head.

Why this exists. P10 proved YOLO-World compiles. It did not prove the
compiled model is any good: the HEF is INT8 and P9's 72% is an fp32
PyTorch number. **P7d already measured that INT8 destroys OWLv2**, so
assuming YOLO-World survives quantization is exactly the assumption this
loop exists to stop -- and it is answerable in emulation, with no
hardware, before the part is ordered.

The split is the one P10 ships and it is exact: `yoloworld-body.onnx`
(images -> six Conv tensors) reassembles with `yoloworld-head.onnx`
(those -> output0) into the full model at **max |diff| 0.0**. The head is
an `onnx.utils.extract_model` subgraph rather than a hand-written decode,
deliberately: a reimplementation of DFL plus the text einsum is a second
place for the arithmetic to be wrong, and the whole point is to attribute
a difference to INT8 rather than to me.

Writes one small JSON of boxes and scores per frame. The crops, not the
tensors, are what the next stage needs -- CLIP re-ranks them off-host --
so nothing large has to come back.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def _frames(paths, size=640):
    from PIL import Image
    for p in paths:
        with Image.open(p) as im:
            arr = np.asarray(im.convert("RGB").resize((size, size)),
                             dtype=np.uint8)
        yield p, arr


def _head_session(head_onnx: Path):
    import onnxruntime as ort
    return ort.InferenceSession(str(head_onnx), providers=["CPUExecutionProvider"])


def _to_nchw(x: np.ndarray) -> np.ndarray:
    """Hailo emulation returns NHWC; the head subgraph wants NCHW."""
    return np.ascontiguousarray(np.transpose(x, (0, 3, 1, 2)))


def detections(out: np.ndarray, conf: float, n_classes: int) -> list:
    """output0 is [1, 4+K, 8400] -- xywh then per-class sigmoid scores."""
    boxes = out[0, :4, :].T                      # [8400,4] xywh, pixels
    scores = out[0, 4:4 + n_classes, :]          # [K,8400]
    best_cls = scores.argmax(0)
    best = scores.max(0)
    keep = np.nonzero(best >= conf)[0]
    return [{"box": [round(float(v), 2) for v in boxes[i]],
             "cls": int(best_cls[i]), "score": round(float(best[i]), 5)}
            for i in keep]


def _promotion(runner, mode: str) -> str:
    """`quantization_param` lines promoting the EMBEDDING branch to 16-bit.

    Measured 2026-09-13: at the a8_w8 default, Hailo's quantized activations
    correlate with the fp32 ONNX reference at only +0.61..+0.77 on the three
    512-channel `cv3` tensors and +0.66..+0.83 on the 64-channel `cv2` ones,
    and end-to-end the detector drops from 29,331 detections to 370.

    The 512-channel tensors are the ones to promote, and the reason is the
    head rather than the numbers: those are EMBEDDINGS, consumed by an
    einsum against normalised text vectors -- a cosine similarity. Similarity
    matching depends on the direction of a vector, which is exactly what a
    coarse per-tensor scale scrambles. The box branch survives the same
    treatment because DFL bins are a far blunter target.

    Layers are found by SHAPE, not by name: Hailo renames everything during
    translate (`conv41`, ...), and a hardcoded name list would rot silently
    into promoting nothing -- which would read as "16-bit did not help".
    """
    if not mode:
        return ""
    hn = runner.get_hn()
    if isinstance(hn, str):
        import json as _json
        hn = _json.loads(hn)
    layers = hn.get("layers", {})
    # The CONV that FEEDS each 512-channel output, not the output layer
    # itself. Two wrong turns got here and both are worth not repeating:
    # taking every 512-channel layer picked 16, including `conv22` and
    # `concat3`, which the optimizer FUSES AWAY ("Layer ... not found in
    # model"); and naming the `output_layer*` markers fails differently
    # ("Unsupported value a16_w8"), because a pass-through marker has no
    # weights for a w8 to describe.
    #
    # Read from the HN graph rather than hardcoded: translate renames
    # everything, so conv46/conv59/conv71 are true of this export and must
    # not be assumed of the next one.
    names = []
    for name, spec in layers.items():
        if "output_layer" not in name:
            continue
        shape = spec.get("output_shapes") or []
        flat = shape[0] if shape and isinstance(shape[0], list) else shape
        if not (flat and flat[-1] == 512):
            continue
        for src in spec.get("input") or []:
            if (layers.get(src) or {}).get("type") == "conv":
                names.append(src)
    if not names:
        raise SystemExit("found no 512-channel layers to promote -- the HN "
                         "naming or shape layout changed; fix the selector "
                         "rather than shipping a no-op promotion")
    print(f"promoting {len(names)} embedding layers to {mode}: {names}",
          flush=True)
    return "\n".join(
        f"quantization_param({n}, precision_mode={mode})" for n in names) + "\n"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--har", type=Path, default=None,
                    help="the QUANTIZED har from the optimize stage")
    ap.add_argument("--onnx", type=Path, default=None,
                    help="build the quantized model in-process from this ONNX "
                         "plus --calib, instead of loading a saved HAR. The "
                         "compile sweep does not persist the quantized HAR, "
                         "and translate+optimize is ~75s -- cheaper than "
                         "plumbing one around.")
    ap.add_argument("--calib", type=Path, default=None)
    ap.add_argument("--promote", default="",
                    help="precision_mode for the EMBEDDING convs, e.g. "
                         "a16_w8_a16. NOTE the THREE parts -- "
                         "a<in>_w<weights>_a<out>. Two-part names like "
                         "'a16_w8' do not exist in DFC 3.34 and are rejected "
                         "per-layer with a message that reads like the LAYER "
                         "is wrong. Enumerate with PrecisionMode rather than "
                         "guessing; the accepted set is native, a8_w8_a8, "
                         "a8_w8_a16, a8_w4_*, a16_w16_*, a16_w8_*, a16_w4_*. "
                         "Empty leaves everything at the default.")
    ap.add_argument("--arch", default="hailo8l")
    ap.add_argument("--end-nodes", default=None,
                    help="comma-separated ONNX end nodes to cut at. Defaults "
                         "to YOLO-World's. YOLO11s uses model.23's cv2/cv3.")
    ap.add_argument("--global-precision", default="",
                    help="precision_mode for EVERY layer, e.g. a16_w16_a16 -- "
                         "the highest the 8L supports. Applied with the "
                         "wildcard layer selector rather than per layer, "
                         "because 'the maximum' is a property of the build and "
                         "not of a branch.")
    ap.add_argument("--optimization-level", type=int, default=None,
                    help="FORCE the DFC's optimization level. Without this it "
                         "silently drops to 0 -- 'because there's less data "
                         "than the recommended amount (1024), and there's no "
                         "available GPU' -- which skips bias correction, "
                         "AdaRound and QAT and makes the run a measurement of "
                         "nothing. P11 withdrew a whole result to that "
                         "warning. Level 1 gets bias correction, which is not "
                         "a training loop and runs on CPU; 2+ wants a GPU.")
    ap.add_argument("--body", type=Path, default=None,
                    help="fp32 ONNX body instead of the HAR -- the SAME head "
                         "runs either way, so a difference is INT8 and not a "
                         "different decode")
    ap.add_argument("--head", type=Path,
                    default=Path("build/yoloworld/yoloworld-head.onnx"))
    ap.add_argument("--frames", type=Path, required=True,
                    help="a file listing image paths, one per line")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--conf", type=float, default=0.02)
    ap.add_argument("--classes", type=int, default=32)
    ap.add_argument("--limit", type=int, default=None)
    args = ap.parse_args(argv)

    modes = [args.har is not None, args.body is not None, args.onnx is not None]
    if sum(modes) != 1:
        raise SystemExit("pass exactly one of --har, --onnx (quantized) or "
                         "--body (fp32)")
    head = _head_session(args.head)
    head_inputs = [i.name for i in head.get_inputs()]

    paths = [Path(l.strip()) for l in args.frames.read_text().splitlines()
             if l.strip()]
    if args.limit:
        paths = paths[:args.limit]

    results = {}

    def record(i, path, acts_nchw):
        feed = {n: a.astype(np.float32) for n, a in zip(head_inputs, acts_nchw)}
        out = head.run(["output0"], feed)[0]
        results[path.parent.name + "/" + path.name] = detections(
            out, args.conf, args.classes)
        if i % 25 == 0:
            print(f"  {i+1}/{len(paths)}", flush=True)
            args.out.write_text(json.dumps(results))

    if args.body is not None:
        import onnxruntime as ort
        body = ort.InferenceSession(str(args.body),
                                    providers=["CPUExecutionProvider"])
        names = [o.name for o in body.get_outputs()]
        for i, (path, arr) in enumerate(_frames(paths)):
            x = (arr.astype(np.float32) / 255.0).transpose(2, 0, 1)[None]
            record(i, path, body.run(names, {"images": x}))
    else:
        import hailo_sdk_client
        from hailo_sdk_client import ClientRunner
        if args.onnx is not None:
            from tools.hailo.compile_yoloworld import (SPLIT_END_NODES,
                                                       YOLO_NORM)
            runner = ClientRunner(hw_arch=args.arch)
            ends = (args.end_nodes.split(",") if args.end_nodes
                    else list(SPLIT_END_NODES))
            runner.translate_onnx_model(str(args.onnx), args.onnx.stem,
                                        end_node_names=ends)
            script = YOLO_NORM + "\n"
            if args.global_precision:
                script += (f"quantization_param({{*}}, "
                           f"precision_mode={args.global_precision})\n")
            if args.optimization_level is not None:
                script += (f"model_optimization_flavor("
                           f"optimization_level={args.optimization_level})\n")
            script += _promotion(runner, args.promote)
            print("model script:\n" + script, flush=True)
            runner.load_model_script(script)
            calib = np.ascontiguousarray(np.load(str(args.calib)))
            print(f"optimizing on {len(calib)} calibration frames...", flush=True)
            runner.optimize(calib)
        else:
            runner = ClientRunner(har=str(args.har))
        with runner.infer_context(
                hailo_sdk_client.InferenceContext.SDK_QUANTIZED) as ctx:
            for i, (path, arr) in enumerate(_frames(paths)):
                outs = runner.infer(ctx, arr[None])
                if isinstance(outs, np.ndarray):
                    outs = [outs]
                record(i, path, [_to_nchw(np.asarray(o)) for o in outs])
    args.out.write_text(json.dumps(results))
    print(f"wrote {args.out} ({len(results)} frames)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
