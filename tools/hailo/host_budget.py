"""C6 -- what is left for the Pi's four cores after the accelerator.

`PLAN-onboard-perception.md` 2.9 costs the ACCELERATOR and says so
explicitly: detector at 30Hz + segmentation at 5Hz + ResNet-50 CLIP on two
crops at 10Hz is **61% chip duty, "before host cost"**. The handoff's open
item 1 has said since 2026-09-15 that nobody has added up the host side,
and calls it a bigger hardware-day risk than the accelerator choice.

This measures the host side that CAN be measured off the robot: the
per-frame CPU work that stays on the Pi no matter which accelerator is
fitted. Three groups, and the split matters because they scale
differently:

  * **Per camera frame** -- JPEG decode and the resizes each model wants.
    P7b found an Orin spending **36 ms detecting and 229 ms resizing**, so
    this is not a rounding error; it was the bottleneck on a board ten
    times the Pi's price.
  * **Per detector inference** -- the head work P10's split deliberately
    left on the CPU: DFL box decode, NMS, and (for YOLO-World) the text
    einsum that keeps the vocabulary open at runtime.
  * **Per segmentation** -- the floor reduction P18's export left on the
    host: argmax over 150 ADE20K classes at H/4, membership test against
    FLOOR_WORDS, and the bilinear upsample back to frame size.

**What this cannot do is run on a Pi 5.** It times the real operations on
whatever machine it is on and scales by a stated factor, so the output is
an ESTIMATE with its assumption in the open rather than a measurement
wearing a measurement's clothes. The scaling is the weak part and is
printed as a range; hardware day replaces it with the board.
"""

from __future__ import annotations

import argparse
import io
import json
import time
from pathlib import Path

import numpy as np

# Pi 5 is a Cortex-A76 at 2.4GHz. Apple silicon and desktop x86 cores are
# roughly 3-4x its single-thread throughput on this kind of work. The range
# is deliberate: a single number here would be the "recall rather than a
# fifth measurement" mistake P7b caught itself making.
DEFAULT_SCALE = (3.0, 4.0)


def _bench(fn, n: int = 20) -> float:
    fn()  # warm
    t0 = time.perf_counter()
    for _ in range(n):
        fn()
    return (time.perf_counter() - t0) * 1000.0 / n


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--frame", type=Path, default=None,
                    help="a real corpus frame; defaults to the first one found")
    ap.add_argument("--recordings", type=Path, default=Path("recordings"))
    ap.add_argument("--scale", type=float, nargs=2, default=list(DEFAULT_SCALE),
                    metavar=("LO", "HI"))
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args(argv)

    from PIL import Image

    frame = args.frame
    if frame is None:
        cands = sorted(args.recordings.glob("*/frame-*.jpg"))
        if not cands:
            raise SystemExit(f"no frames under {args.recordings}")
        frame = cands[0]
    raw = frame.read_bytes()
    img = Image.open(io.BytesIO(raw)).convert("RGB")
    print(f"frame: {frame}  {img.size[0]}x{img.size[1]}  {len(raw)/1024:.0f} KB\n")

    rows = []

    def add(group, name, ms, rate_hz, note=""):
        rows.append({"group": group, "op": name, "ms": ms,
                     "rate_hz": rate_hz, "ms_per_s": ms * rate_hz, "note": note})

    # ---- per camera frame -------------------------------------------------
    add("frame", "JPEG decode",
        _bench(lambda: Image.open(io.BytesIO(raw)).convert("RGB").load()),
        30, "every frame, before anything else can look at it")
    add("frame", "resize -> 640 (detector)",
        _bench(lambda: img.resize((640, 640), Image.BILINEAR)),
        30, "YOLO/YOLO-World input")
    add("frame", "resize -> 512 (segmenter)",
        _bench(lambda: img.resize((512, 512), Image.BILINEAR)),
        5, "SegFormer input, at 2.9's 5Hz sub-rate")
    # AVOIDED BY DESIGN: both the YOLO-World and SegFormer builds use the
    # `uint8` layout, where normalization() runs inside the compiled graph
    # on the accelerator. Costed here only to show what taking the
    # normalized layout instead would spend on the host.
    add("frame-AVOIDED", "to float NCHW + normalise (uint8 layout avoids)",
        _bench(lambda: ((np.asarray(img.resize((640, 640)), dtype=np.float32)
                         / 255.0 - 0.5) / 0.5).transpose(2, 0, 1)),
        30, "NOT in the total -- the uint8 builds put this on the chip")

    # ---- per detector inference ------------------------------------------
    # The six Conv end nodes' output, at 640: 80x80, 40x40, 20x20 grids.
    cells = 80 * 80 + 40 * 40 + 20 * 20
    box_logits = np.random.randn(1, 64, cells).astype(np.float32)
    cls_logits = np.random.randn(1, 80, cells).astype(np.float32)

    def dfl():
        x = box_logits.reshape(1, 4, 16, cells)
        e = np.exp(x - x.max(axis=2, keepdims=True))
        p = e / e.sum(axis=2, keepdims=True)
        return (p * np.arange(16, dtype=np.float32)[None, None, :, None]).sum(axis=2)

    add("detector-head", "DFL box decode (numpy)", _bench(dfl), 30,
        "P10 left this on the CPU deliberately")

    def nms():
        scores = 1.0 / (1.0 + np.exp(-cls_logits.max(axis=1)[0]))
        keep = np.argsort(scores)[::-1][:300]
        return keep

    add("detector-head", "score + top-k (NMS stand-in)", _bench(nms), 30,
        "a real NMS is more; this is the floor")

    # YOLO-World's text contrast: the einsum P10 moved to the host, which is
    # what keeps the vocabulary open at runtime.
    embed = np.ascontiguousarray(np.random.randn(512, cells).astype(np.float32))
    vocab = np.random.randn(32, 512).astype(np.float32)
    # As a BLAS gemm, NOT np.einsum. The einsum form of the same arithmetic
    # measures 17x slower here (11.1ms vs 0.65ms) because it does not reach
    # BLAS -- which would have made YOLO-World's open vocabulary look like
    # it costs a whole Pi core. It costs a rounding error. Whatever runs
    # this on the robot must use the gemm form.
    add("detector-head", "YOLO-World text contrast (32 words, BLAS)",
        _bench(lambda: vocab @ embed), 30,
        "only if YOLO-World is the detector; YOLO11s has none")

    # ---- per segmentation -------------------------------------------------
    seg = np.random.randn(150, 128, 128).astype(np.float32)
    floor_ids = np.array([3, 13, 28])

    def mask():
        top = seg.argmax(axis=0)
        return np.isin(top, floor_ids)

    add("segmenter-head", "argmax(150) + floor membership", _bench(mask), 5,
        "P18 left this on the host so the label set stays a runtime argument")

    small = Image.fromarray((mask() * 255).astype(np.uint8))
    add("segmenter-head", "mask upsample -> frame",
        _bench(lambda: small.resize(img.size, Image.NEAREST)), 5,
        "the final bilinear/nearest resize P18's export does NOT put on the part")

    # ---- report -----------------------------------------------------------
    lo, hi = args.scale
    print(f"{'group':16s} {'operation':38s} {'ms':>7s} {'Hz':>4s} "
          f"{'ms/s':>7s}   Pi5 est ms/s ({lo:.0f}-{hi:.0f}x)")
    print("-" * 104)
    for r in rows:
        print(f"{r['group']:16s} {r['op']:38s} {r['ms']:7.2f} {r['rate_hz']:4.0f} "
              f"{r['ms_per_s']:7.1f}   {r['ms_per_s']*lo:6.0f} - {r['ms_per_s']*hi:.0f}")

    total = sum(r["ms_per_s"] for r in rows if r["group"] != "frame-AVOIDED")
    yw = sum(r["ms_per_s"] for r in rows if "text einsum" in r["op"])
    print("-" * 104)
    print(f"{'TOTAL':16s} {'(one core = 1000 ms/s)':38s} {'':7s} {'':4s} "
          f"{total:7.1f}   {total*lo:6.0f} - {total*hi:.0f}")
    print(f"{'':16s} {'without YOLO-World einsum':38s} {'':7s} {'':4s} "
          f"{total-yw:7.1f}   {(total-yw)*lo:6.0f} - {(total-yw)*hi:.0f}")

    print(f"\n  cores needed for perception host work alone: "
          f"{total*lo/1000:.2f} - {total*hi/1000:.2f} of 4")

    doc = {"frame": str(frame), "frame_size": img.size, "scale": [lo, hi],
           "rows": rows, "total_ms_per_s": total,
           "pi5_est_ms_per_s": [total * lo, total * hi]}
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(doc, indent=2))
        print(f"\nwrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
