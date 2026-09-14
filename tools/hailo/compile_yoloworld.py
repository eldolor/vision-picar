"""Can YOLO-World compile to a Hailo-8L HEF?

The sibling of `compile_owlv2.py`, pointed at the model the part decision
now rests on. The owner ruled out the Jetson on cost (2026-09-13), so
OWLv2 -- which P6 proved cannot compile -- is out, and within what a
Hailo can take P9 measured YOLO-World + CLIP at 72% recall / 99%
precision against the shipped 45% / 94%. The reactive tier is worth 45%
or 72% on this one answer.

Same three stages, because they fail for different reasons and imply
different decisions:

    translate  ONNX -> HAR              op coverage
    optimize   HAR + calibration -> quantized HAR   numerics, memory
    compile    quantized HAR -> HEF     resource allocation on the part

**What the export already settled, and what it did not.** The graph is
convolutional -- 0 layernorm, 1 softmax (YOLOv8's DFL box decode), 68
conv -- where OWLv2 had 73 and 38 and died at allocation on exactly
those. That is a reason to expect success and NOT a substitute for the
test: OWLv2's row in Hailo's own compatibility table said it should
compile too, and this whole loop exists because that row was wrong.

**The one thing to watch is the seven Einsums.** Four sit in the
vision-language path aggregation neck and three at the contrastive head.
They are folded constants in the export -- the vocabulary is baked -- so
they are weight-multiplies rather than dynamic attention, but the DFC
still has to have a lowering for them. If a stage fails, that is where to
look first, and `--no-einsum-fold` re-exports without simplification so
the failure can be attributed.

Normalization differs from OWLv2's and it matters: YOLO wants /255 with
no mean subtraction, so the uint8 calibration array plus
`normalization([0,0,0],[255,255,255])` is correct here. Feeding
`calib_normalized.npy` would be feeding OWLv2's mean/std to a YOLO graph
and reading the quantization damage as a model result.

    python3 -m tools.hailo.compile_yoloworld --build build/yoloworld \\
        --arch hailo8l --report build/yoloworld/compile_report.json
"""
from __future__ import annotations

import argparse
import json
import time
import traceback
from pathlib import Path

STAGES = ("translate", "optimize", "compile")
YOLO_NORM = "yoloworld_input_norm = normalization([0.0, 0.0, 0.0], " \
            "[255.0, 255.0, 255.0])"


def _runner(arch: str):
    try:
        from hailo_sdk_client import ClientRunner
    except ImportError as exc:
        raise SystemExit(
            "The Hailo Dataflow Compiler is not installed in this "
            "interpreter. It is a Developer Zone download, not on PyPI -- "
            "see tools/hailo/README.md.") from exc
    return ClientRunner(hw_arch=arch)


def _calibration(build: Path, limit=None):
    import numpy as np
    path = build / "calib_uint8.npy"
    if not path.exists():
        raise SystemExit(f"no calibration set at {path} -- run "
                         "tools.hailo.calibration_set --image-size 640 first")
    data = np.load(path, mmap_mode="r")
    if limit:
        data = data[:limit]
    return np.ascontiguousarray(data)


def _model_script(extra: str = "") -> str:
    lines = [YOLO_NORM]
    if extra:
        lines.extend(x.strip() for x in extra.split(";") if x.strip())
    return "\n".join(lines) + "\n"


# The six Conv nodes the DFC's own error message recommends parsing to.
# Cutting here does THREE things at once, which is why it is the default:
#
#   * it drops the DFL box decode (`dfl/Reshape`, `dfl/Transpose`), which
#     the parser cannot build a shuffle layer for -- the standard Hailo
#     YOLO recipe puts that on the host anyway;
#   * it drops the three `bchw,bkc->bkhw` Einsums the parser rejects; and
#   * **it un-bakes the vocabulary.** Those Einsums ARE the text contrast.
#     With them on the CPU the accelerator runs a pure image tower and the
#     target string is a runtime argument again -- the same split CLIP's
#     text encoder already uses (4.2), and the thing the baked-in export
#     appeared to cost us.
#
# So the part runs the CNN and the Pi does the text einsum, the DFL decode
# and NMS. All three are cheap; the einsum is one matmul against a cached
# vector, which 2.8 step 1 already assumes for CLIP.
SPLIT_END_NODES = [
    "/model.22/cv2.0/cv2.0.2/Conv", "/model.22/cv2.1/cv2.1.2/Conv",
    "/model.22/cv2.2/cv2.2.2/Conv", "/model.22/cv3.0/cv3.0.2/Conv",
    "/model.22/cv3.1/cv3.1.2/Conv", "/model.22/cv3.2/cv3.2.2/Conv",
]


def attempt(build: Path, onnx_name: str, arch: str, extra: str,
            calib_limit, label: str, end_nodes=None) -> dict:
    """One variant, all three stages, never letting a failure end the run."""
    record = {"variant": label, "onnx": onnx_name, "arch": arch,
              "model_script_extra": extra, "stages": {}}
    onnx_path = build / onnx_name
    if not onnx_path.exists():
        record["stages"]["translate"] = {"ok": False,
                                         "error": f"missing {onnx_path}"}
        return record

    runner = _runner(arch)

    def run(stage, fn):
        started = time.time()
        try:
            out = fn()
            record["stages"][stage] = {"ok": True,
                                       "seconds": round(time.time()-started, 1)}
            return out, True
        except Exception as exc:                      # noqa: BLE001
            record["stages"][stage] = {
                "ok": False,
                "seconds": round(time.time()-started, 1),
                "error": f"{type(exc).__name__}: {exc}"[:2000],
                "traceback": traceback.format_exc()[-2000:],
            }
            return None, False

    kw = {"end_node_names": list(end_nodes)} if end_nodes else {}
    record["end_node_names"] = kw.get("end_node_names")
    _, ok = run("translate", lambda: runner.translate_onnx_model(
        str(onnx_path), onnx_path.stem, **kw))
    if not ok:
        return record

    try:
        runner.load_model_script(_model_script(extra))
    except Exception as exc:                          # noqa: BLE001
        record["stages"]["translate"]["model_script_error"] = str(exc)[:800]

    _, ok = run("optimize", lambda: runner.optimize(
        _calibration(build, calib_limit)))
    if not ok:
        return record

    hef, ok = run("compile", runner.compile)
    if ok and hef is not None:
        path = build / f"{label}.hef"
        path.write_bytes(hef)
        record["hef"] = {"path": str(path), "bytes": len(hef)}
        print(f"[compile] HEF written: {path} ({len(hef)/1e6:.1f} MB)")
    return record


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--build", type=Path, default=Path("build/yoloworld"))
    ap.add_argument("--arch", default="hailo8l")
    ap.add_argument("--onnx", default=None,
                    help="defaults to the export_meta.json entry")
    ap.add_argument("--calib-limit", type=int, default=None)
    ap.add_argument("--report", type=Path, default=None)
    ap.add_argument("--whole-graph", action="store_true",
                    help="parse the full graph instead of cutting at "
                         "SPLIT_END_NODES. Fails on DFC 3.34 -- kept so the "
                         "failure can be reproduced rather than quoted.")
    ap.add_argument("--extra", action="append", default=[],
                    help="model-script directives, ';'-separated; repeat for "
                         "a sweep. e.g. "
                         "'performance_param(compiler_optimization_level=max)'")
    args = ap.parse_args(argv)

    onnx_name = args.onnx
    if onnx_name is None:
        meta = json.loads((args.build / "export_meta.json").read_text())
        onnx_name = meta["onnx"]

    variants = args.extra or [""]
    report = {"model": "yolo-world", "arch": args.arch, "onnx": onnx_name,
              "dfc": _dfc_version(), "attempts": []}
    for i, extra in enumerate(variants):
        label = f"yoloworld-{args.arch}" + (f"-v{i}" if len(variants) > 1 else "")
        print(f"\n[compile] === {label} === extra={extra!r}")
        rec = attempt(args.build, onnx_name, args.arch, extra,
                      args.calib_limit, label,
                      end_nodes=None if args.whole_graph else SPLIT_END_NODES)
        report["attempts"].append(rec)
        _write(report, args.report or args.build / "compile_report.json")
    _summarize(report)
    _write(report, args.report or args.build / "compile_report.json")
    return 0


def _dfc_version() -> str:
    try:
        import hailo_sdk_client
        return getattr(hailo_sdk_client, "__version__", "unknown")
    except Exception:                                 # noqa: BLE001
        return "not installed"


def _write(report: dict, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=1))


def _summarize(report: dict) -> None:
    print("\n[compile] ---- summary ----")
    for rec in report["attempts"]:
        cells = []
        for s in STAGES:
            st = rec["stages"].get(s)
            cells.append("-" if st is None else ("ok" if st["ok"] else "FAIL"))
        print(f"  {rec['variant']:<28} " + "  ".join(
            f"{s}={c}" for s, c in zip(STAGES, cells)))
        for s in STAGES:
            st = rec["stages"].get(s)
            if st and not st["ok"]:
                print(f"      {s} error: {st['error'][:300]}")
                break


if __name__ == "__main__":
    raise SystemExit(main())
