"""P15 row D -- which variable actually caused P14's `is_null_split`?

The parse matrix (rows A/B) settled that DFC 5.4.0 parses yolov11s,
yolov8s and yolo_world_v2s on `hailo10h`, and parses OUR opset-13 export
too. So P14's "the compiler cannot parse real models" is wrong. What it
does NOT say is *why* P14 failed, because rows A/B differ from P14 in two
ways at once:

  1. the ENVIRONMENT -- vendor container vs our own pip-assembled host;
  2. the ENTRY POINT -- `hailomz parse`, which hands the parser the full
     model plus an end-node spec, vs P14's raw `ClientRunner` call on a
     graph we had already cut ourselves with onnx surgery.

This runs P14's exact input -- the pre-cut body ONNX, through the raw
ClientRunner API -- inside the vendor container. That holds the
environment at "theirs" and changes only the entry point, so:

  D1 FAIL  -> the pre-cut graph is the cause; our onnx surgery produced
              something the parser cannot assign a format to, and P14's
              error was self-inflicted by the cut, not by the toolchain.
  D1 OK    -> the graph is fine and the cause was our host environment,
              which means P14 measured an installation, not a compiler.

D2 is the same file through `translate_onnx_model`'s own end-node
argument rather than a pre-cut graph -- the vendor-recommended way to do
what our surgery was doing by hand.
"""

import json
import sys
import traceback

from hailo_sdk_client import ClientRunner

SHARE = "/local/shared_with_docker"
OUT = f"{SHARE}/out/rawparse.json"

# The six Conv end nodes the zoo's own yolov11s.yaml names. Our pre-cut
# body ONNX was cut at exactly these, which is worth stating: the cut
# POINT was never in question, only what the surgery left behind.
END_NODES = [
    "/model.23/cv2.0/cv2.0.2/Conv",
    "/model.23/cv3.0/cv3.0.2/Conv",
    "/model.23/cv2.1/cv2.1.2/Conv",
    "/model.23/cv3.1/cv3.1.2/Conv",
    "/model.23/cv2.2/cv2.2.2/Conv",
    "/model.23/cv3.2/cv3.2.2/Conv",
]

CASES = [
    # row, onnx, kwargs -- net_input_shapes is what the DFC wants when it
    # cannot infer a static input, and both of ours are static, so it is
    # passed identically in every case rather than being a hidden variable.
    ("D1", f"{SHARE}/ours_yolo11s_body_precut.onnx", {}),
    ("D2", f"{SHARE}/ours_yolo11s_op13.onnx", {"end_node_names": END_NODES}),
]


def main() -> int:
    results = []
    for row, onnx_path, kwargs in CASES:
        entry = {"row": row, "onnx": onnx_path.split("/")[-1], "kwargs": sorted(kwargs)}
        try:
            runner = ClientRunner(hw_arch="hailo10h")
            runner.translate_onnx_model(
                onnx_path,
                "yolo11s",
                net_input_shapes={"images": [1, 3, 640, 640]},
                **kwargs,
            )
            entry["status"] = "OK"
            entry["detail"] = "translate_onnx_model returned"
        except Exception as exc:  # noqa: BLE001 -- the exception IS the result
            entry["status"] = "FAIL"
            entry["detail"] = f"{type(exc).__name__}: {exc}"[:400]
            entry["traceback_tail"] = traceback.format_exc().strip().splitlines()[-6:]
        print(f"{row}: {entry['status']} -- {entry['detail']}", flush=True)
        results.append(entry)

    with open(OUT, "w") as fh:
        json.dump(results, fh, indent=2)
    print(f"\nwrote {OUT}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
