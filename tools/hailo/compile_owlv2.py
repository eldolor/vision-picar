"""Walk OWLv2's ONNX exports through the Hailo Dataflow Compiler.

Runs on the x86-64 EC2 host, not on a Mac -- the DFC ships as a linux_x86_64
wheel and there is no other path (`PLAN-onboard-perception.md` 1.10 item 1).

**What this is for.** P5 recommends Pi 5 + Hailo-8L on the strength of
OWLv2's accuracy, and rests the whole recommendation on one unrun test:
*can OWLv2 compile to a HEF?* Hailo's own compatibility table in 1.10 lists
OWLv2 under **"does not fit -- anything attention-heavy"**, so a plain
failure is the expected outcome and is a perfectly good answer. What is not
a good answer is a bare "it failed", because three different failures point
at three different decisions:

* an **unsupported op** may be one line moved to the CPU (which is what the
  `minimal` head already tests) or a real architectural wall;
* a **resource/allocation** failure at 3600 tokens may survive at 1600
  (`--image-size 640`), at a cost in accuracy that has to be re-measured;
* a **parser** failure that differs between opset 17 and 14 is a toolchain
  version problem and not a statement about the part at all.

So this walks the whole matrix, records what each variant died of and where,
and never lets one failure end the sweep. The report it writes is the
deliverable.

Three stages per variant, each of which can fail on its own:

    translate  ONNX -> Hailo HAR      (op coverage)
    optimize   HAR + calibration -> quantized HAR   (numerics, memory)
    compile    quantized HAR -> HEF   (resource allocation on the part)

Usage on the host::

    python3 -m tools.hailo.compile_owlv2 --build build/owlv2 \\
        --arch hailo8l --report build/owlv2/compile_report.json
"""

from __future__ import annotations

import argparse
import itertools
import json
import os
import sys
import time
import traceback
from pathlib import Path

# Kept in step with export_owlv2_onnx.py, but imported rather than copied so
# they cannot drift.
try:
    from tools.hailo.export_owlv2_onnx import HEADS, OPSETS, OUT_NAMES, onnx_name
except ImportError:  # running as a loose script on the host
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    from tools.hailo.export_owlv2_onnx import HEADS, OPSETS, OUT_NAMES, onnx_name

STAGES = ("translate", "optimize", "compile")


def _runner(arch: str):
    try:
        from hailo_sdk_client import ClientRunner
    except ImportError as exc:
        raise SystemExit(
            "The Hailo Dataflow Compiler is not installed in this "
            "interpreter. It is a Developer Zone download, not on PyPI -- "
            "see tools/hailo/README.md.") from exc
    return ClientRunner(hw_arch=arch)


def _calibration(build: Path, layout: str, limit: int | None):
    import numpy as np
    name = ("calib_uint8.npy" if layout == "uint8"
            else "calib_normalized.npy")
    path = build / name
    if not path.exists():
        raise SystemExit(f"no calibration set at {path} -- run "
                         f"tools.hailo.calibration_set first")
    data = np.load(path, mmap_mode="r")
    if limit:
        data = data[:limit]
    # The DFC wants it resident; a memmap slice would be re-read per epoch.
    return np.ascontiguousarray(data)


def _model_script(layout: str, meta: dict, extra: str = "") -> str | None:
    """The .alls script, when the graph needs one.

    Only the uint8 layout does: the parsed graph's input is OWLv2's already
    normalised `pixel_values`, so feeding raw camera bytes means asking the
    part to do the mean/std itself. That is the arrangement worth having on
    the robot -- it keeps a per-frame float conversion off the Pi's CPU --
    which is why both layouts are tried rather than just the easy one.
    """
    lines = []
    if extra:
        # Allocator / performance directives, from the command line. The
        # 3.34.0 SDK accepts allocator_param, performance_param,
        # resources_param, model_optimization_flavor, input_conversion,
        # context_switch_param, change_output_activation and nms_postprocess
        # -- enumerated off the installed SDK rather than guessed, because a
        # wrong directive fails optimize and reads like a model result.
        lines.extend(x.strip() for x in extra.split(";") if x.strip())
    if layout != "uint8":
        return ("\n".join(lines) + "\n") if lines else None
    mean = [round(m * 255.0, 4) for m in meta["image_mean"]]
    std = [round(s * 255.0, 4) for s in meta["image_std"]]
    # NOT `normalization1`: the DFC already auto-names layers on that
    # pattern while parsing, and a model script that reuses one fails the
    # whole optimize stage with "Given layer names [...] exist in the model"
    # -- which reads like a graph problem and is really a naming collision.
    lines.append(f"owlv2_input_norm = normalization({mean}, {std})")
    return "\n".join(lines) + "\n"


def attempt(build: Path, arch: str, opset: int, head: str, size: int,
            layout: str, meta: dict, calib, out_dir: Path,
            extra_script: str = "") -> dict:
    """One matrix cell, three stages, never raising."""
    # The export writes `factor_patch` into its metadata and suffixes the
    # filename to match; read it rather than assuming, or a factored build
    # reports "missing <un-suffixed name>" as a TRANSLATE failure -- which
    # is indistinguishable in the report from the model being rejected.
    factored = bool(meta.get("factor_patch", False))
    onnx_path = build / onnx_name(opset, head, size, factored)
    tag = f"{size}_op{opset}_{head}_{layout}" + ("_factored" if factored else "")
    record = {"tag": tag, "onnx": onnx_path.name, "opset": opset,
              "head": head, "image_size": size, "layout": layout,
              "arch": arch, "stages": {}, "reached": None, "hef": None}

    if not onnx_path.exists():
        record["stages"]["translate"] = {"ok": False,
                                         "error": f"missing {onnx_path}"}
        return record

    runner = _runner(arch)
    log_dir = out_dir / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)

    def run(stage: str, fn):
        started = time.time()
        try:
            value = fn()
            record["stages"][stage] = {"ok": True,
                                       "seconds": round(time.time() - started, 1)}
            record["reached"] = stage
            return value, True
        except KeyboardInterrupt:
            # The one BaseException that must NOT be recorded as a result:
            # a ^C is the operator stopping a rented box, not the DFC
            # answering the question. The report on disk is already current.
            raise
        except BaseException as exc:      # SystemExit and MemoryError too:
            # the DFC raises both, and an OOM is a RESULT here, not a crash.
            text = traceback.format_exc()
            (log_dir / f"{tag}.{stage}.log").write_text(text)
            record["stages"][stage] = {
                "ok": False,
                "seconds": round(time.time() - started, 1),
                "error_type": type(exc).__name__,
                "error": str(exc)[:4000],
                "log": f"logs/{tag}.{stage}.log",
                # The first line of a Hailo parser error names the op it
                # stopped on, which is the whole substance of a failure.
                "headline": str(exc).strip().splitlines()[0][:400]
                if str(exc).strip() else type(exc).__name__,
            }
            return None, False

    print(f"\n=== {tag} ===", flush=True)

    _, ok = run("translate", lambda: runner.translate_onnx_model(
        str(onnx_path), f"owlv2_{tag}",
        start_node_names=[meta["input_name"]],
        end_node_names=list(OUT_NAMES),
        net_input_shapes={meta["input_name"]: [1, 3, size, size]},
    ))
    if not ok:
        return record

    har = out_dir / f"owlv2_{tag}.har"
    try:
        runner.save_har(str(har))
        record["har"] = har.name
    except BaseException:
        pass

    script = _model_script(layout, meta, extra_script)
    if script:
        record["model_script"] = script.strip()
        try:
            runner.load_model_script(script)
        except BaseException as exc:
            record["stages"]["translate"]["model_script_error"] = str(exc)[:800]

    _, ok = run("optimize", lambda: runner.optimize(calib))
    if not ok:
        return record

    hef, ok = run("compile", runner.compile)
    if ok and hef is not None:
        path = out_dir / f"owlv2_{tag}.hef"
        path.write_bytes(hef)
        record["hef"] = path.name
        record["hef_bytes"] = path.stat().st_size
        print(f"[compile] HEF written: {path} "
              f"({path.stat().st_size / 1e6:.1f} MB)", flush=True)
    return record


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--build", type=Path, default=Path("build/owlv2"))
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--arch", default="hailo8l",
                    help="hailo8l is the part 1.10 chose; hailo8 for the 26 TOPS")
    ap.add_argument("--opset", type=int, action="append", default=None)
    ap.add_argument("--head", action="append", default=None, choices=HEADS)
    ap.add_argument("--layout", action="append", default=None,
                    choices=["normalized", "uint8"])
    ap.add_argument("--calib-limit", type=int, default=None,
                    help="fewer calibration frames, for a fast first pass")
    ap.add_argument("--extra-script", default="",
                    help="model-script lines, ';'-separated, e.g. "
                         "'performance_param(compiler_optimization_level=max)'")
    ap.add_argument("--report", type=Path, default=None)
    ap.add_argument("--stop-on-success", action="store_true",
                    help="stop at the first variant that produces a HEF")
    args = ap.parse_args(argv)

    build = args.build
    out_dir = args.out or build
    out_dir.mkdir(parents=True, exist_ok=True)
    meta = json.loads((build / "owlv2_export.json").read_text())
    size = meta["image_size"]

    opsets = args.opset or list(meta.get("opsets", OPSETS))
    heads = args.head or list(meta.get("heads", HEADS))
    layouts = args.layout or ["normalized", "uint8"]

    report = {
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "arch": args.arch,
        "model_id": meta["model_id"],
        "image_size": size,
        "num_patches": meta["num_patches"],
        "factor_patch": bool(meta.get("factor_patch", False)),
        "dfc_version": _dfc_version(),
        "host": os.uname().nodename if hasattr(os, "uname") else "?",
        "calibration": _calib_meta(build, args.calib_limit),
        "attempts": [],
    }

    calib_cache: dict[str, object] = {}
    for head, opset, layout in itertools.product(heads, opsets, layouts):
        if layout not in calib_cache:
            calib_cache[layout] = _calibration(build, layout,
                                               args.calib_limit)
        record = attempt(build, args.arch, opset, head, size, layout,
                         meta, calib_cache[layout], out_dir,
                         args.extra_script)
        report["attempts"].append(record)
        _write(report, args.report or out_dir / "compile_report.json")
        if record["hef"] and args.stop_on_success:
            break

    _summarize(report)
    _write(report, args.report or out_dir / "compile_report.json")
    return 0 if any(a["hef"] for a in report["attempts"]) else 1


def _dfc_version() -> str:
    try:
        import hailo_sdk_client
        return getattr(hailo_sdk_client, "__version__", "unknown")
    except Exception:
        return "not installed"


def _calib_meta(build: Path, limit: int | None) -> dict:
    path = build / "calib_manifest.json"
    if not path.exists():
        return {"n": None}
    meta = json.loads(path.read_text())
    return {"n": min(meta["n"], limit) if limit else meta["n"],
            "seed": meta.get("seed"), "walks": meta.get("walks")}


def _write(report: dict, path: Path) -> None:
    # Rewritten after every cell: a sweep that dies in hour three still
    # leaves the cells it finished, which is most of the answer.
    path.write_text(json.dumps(report, indent=1))


def _summarize(report: dict) -> None:
    print("\n" + "=" * 72)
    print(f"OWLv2 -> {report['arch']}   DFC {report['dfc_version']}   "
          f"{report['image_size']}px / {report['num_patches']} tokens")
    print("=" * 72)
    width = max((len(a["tag"]) for a in report["attempts"]), default=10)
    for a in report["attempts"]:
        marks = []
        for stage in STAGES:
            s = a["stages"].get(stage)
            marks.append("...." if s is None
                         else ("ok" if s["ok"] else "FAIL"))
        line = f"  {a['tag']:<{width}}  " + "  ".join(
            f"{stage}={mark}" for stage, mark in zip(STAGES, marks))
        print(line)
        for stage in STAGES:
            s = a["stages"].get(stage)
            if s and not s["ok"]:
                print(f"      {stage}: {s.get('headline', '')}")
                break
        if a["hef"]:
            print(f"      HEF: {a['hef']}  "
                  f"{a.get('hef_bytes', 0) / 1e6:.1f} MB")
    if any(a["hef"] for a in report["attempts"]):
        print("\nVERDICT: OWLv2 COMPILES. -> Pi 5 + Hailo-8L (P5).")
    else:
        reached = {a["reached"] for a in report["attempts"]}
        print(f"\nVERDICT: no HEF. Furthest stage reached: "
              f"{sorted(r for r in reached if r) or ['none']}.")
        print("Read the per-stage headlines above before concluding "
              "anything about the part: a parser gap and a resource wall "
              "argue for different hardware.")


if __name__ == "__main__":
    sys.exit(main())
