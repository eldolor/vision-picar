"""SegFormer-B0 (ADE20K) -> Hailo, because the floor mask is what the tier
rests on.

P16 measured the tier three ways and the mask dominated all of them:

    floor mask ON   fp32 50%  10H 49%  8L 49%
    floor mask OFF  fp32 20%  10H 13%  8L  3%

A detector degraded from 34% to 5% costs the tier one point WITH the mask
and seventeen points without it. So the model whose quantization actually
decides whether the on-board tier works is this one -- and it has never
been compiled, let alone quantized.

Two questions, and only the first is answerable on a CPU box:

  1. **Allocation.** Does it place on the part at all? OWLv2 did not, on
     either arch and at two input sizes (P17), so this is not a formality.
  2. **Accuracy under INT8.** Needs a GPU: without one the DFC drops to
     optimization level 0 and skips every accuracy pass while still
     reporting success, which is precisely what invalidated P11. This
     script forces the level so that a CPU-box run FAILS rather than
     quietly measuring the crudest possible quantization.

The ONNX carries its weights in an external `.onnx.data` sidecar, so both
files have to travel together or the compile reads a weightless graph.
"""

from __future__ import annotations

import argparse
import json
import time
import traceback
from pathlib import Path

STAGES = ("translate", "optimize", "compile")


def _runner(arch: str):
    try:
        from hailo_sdk_client import ClientRunner
    except ImportError as exc:
        raise SystemExit(
            "The Hailo Dataflow Compiler is not installed in this "
            "interpreter -- see tools/hailo/README.md.") from exc
    return ClientRunner(hw_arch=arch)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--build", type=Path, default=Path("build/segformer"))
    ap.add_argument("--arch", default="hailo10h")
    ap.add_argument("--layout", default="uint8", choices=["uint8", "normalized"])
    ap.add_argument("--optimization-level", type=int, default=None,
                    help="FORCE the level. Without it the DFC drops to 0 on a "
                         "box with no GPU and reports success anyway (P11).")
    ap.add_argument("--end-nodes", default=None,
                    help="comma-separated end node names. SegFormer's decode "
                         "head ends in Reshape/Transpose/bilinear-upsample ops "
                         "the parser will not build a shuffle layer for, and "
                         "the DFC names the cut in its own error. Cutting "
                         "there puts the final upsample on the host -- which "
                         "is cheap, and is the same split YOLO-World's text "
                         "einsum and CLIP's text encoder already use.")
    ap.add_argument("--calib-limit", type=int, default=None)
    ap.add_argument("--report", type=Path, default=None)
    args = ap.parse_args(argv)

    import numpy as np

    meta = json.loads((args.build / "export_meta.json").read_text())
    onnx_path = args.build / meta["onnx"]
    if not onnx_path.exists():
        raise SystemExit(f"missing {onnx_path} -- run export_segformer_onnx first")

    calib_name = ("calib_uint8.npy" if args.layout == "uint8"
                  else "calib_normalized.npy")
    calib_path = args.build / calib_name
    if not calib_path.exists():
        raise SystemExit(f"missing {calib_path} -- run calibration_set "
                         f"--image-size {meta['image_size']} first")
    calib = np.ascontiguousarray(np.load(calib_path, mmap_mode="r")
                                 [:args.calib_limit] if args.calib_limit
                                 else np.load(calib_path))

    record = {"model": "segformer-b0-ade", "arch": args.arch,
              "layout": args.layout, "onnx": onnx_path.name,
              "image_size": meta["image_size"],
              "floor_ids": meta["floor_ids"], "stages": {}, "hef": None}

    runner = _runner(args.arch)

    def run(stage, fn):
        t0 = time.time()
        try:
            v = fn()
            record["stages"][stage] = {"ok": True, "seconds": round(time.time() - t0, 1)}
            print(f"[segformer] {stage}: ok ({record['stages'][stage]['seconds']}s)")
            return v, True
        except Exception as exc:  # noqa: BLE001 -- the exception IS the result
            record["stages"][stage] = {"ok": False,
                                       "seconds": round(time.time() - t0, 1),
                                       "error": str(exc)[:900]}
            print(f"[segformer] {stage}: FAIL -- {str(exc)[:300]}")
            traceback.print_exc()
            return None, False

    end_nodes = ([n.strip() for n in args.end_nodes.split(",") if n.strip()]
                 if args.end_nodes else None)
    record["end_node_names"] = end_nodes
    _, ok = run("translate", lambda: runner.translate_onnx_model(
        str(onnx_path), "segformer_b0_ade",
        net_input_shapes={"pixel_values": [1, 3, meta["image_size"],
                                           meta["image_size"]]},
        **({"end_node_names": end_nodes} if end_nodes else {})))
    if ok:
        lines = []
        if args.layout == "uint8":
            mean = [round(m * 255.0, 4) for m in meta["image_mean"]]
            std = [round(s * 255.0, 4) for s in meta["image_std"]]
            # NOT `normalization1`: the DFC auto-names layers on that pattern
            # while parsing, and reusing one fails the whole optimize stage
            # with a message that reads like a graph problem.
            lines.append(f"segformer_input_norm = normalization({mean}, {std})")
        if args.optimization_level is not None:
            lines.append("model_optimization_flavor(optimization_level="
                         f"{args.optimization_level})")
        if lines:
            runner.load_model_script("\n".join(lines) + "\n")
        _, ok = run("optimize", lambda: runner.optimize(calib))
    if ok:
        hef, ok = run("compile", runner.compile)
        if ok and hef:
            out = args.build / f"segformer_b0_ade-{args.arch}.hef"
            out.write_bytes(hef)
            record["hef"] = {"path": str(out), "bytes": out.stat().st_size}
            print(f"[segformer] HEF written: {out} "
                  f"({out.stat().st_size / 1e6:.1f} MB)")

    reached = [s for s in STAGES if record["stages"].get(s, {}).get("ok")]
    print(f"\n[segformer] VERDICT {args.arch}: "
          f"{'HEF' if record['hef'] else 'no HEF'}; reached {reached}")
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(record, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
