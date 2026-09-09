"""Build a Hailo calibration set out of the recorded rig walks.

Post-training quantization needs a few hundred frames that look like what
the part will actually see. This project has exactly that and almost
nothing else: 968 frames from eleven walks, camera at floor height on a
wheeled rig, in one house. `PLAN-onboard-perception.md` P5 says plainly that
this corpus is the only eval that means anything about this viewpoint --
which makes it the only honest calibration source too.

**How frames are chosen, and why not at random.** Two rules:

* **Stratified across walks**, so no single room or target dominates. Four
  targets and three capture resolutions are in there; a random 128 would
  over-weight the 209-frame search walk by construction.
* **Balanced on `labels.json`** where a walk has one. Target-visible frames
  are a small minority of the corpus -- 74 of 299 in the four adjudicated
  walks, and far fewer in the searches -- and they are precisely the frames
  whose activations decide recall. A calibration set drawn without looking
  at the labels would under-represent them the same way the corpus does.

Both are deterministic given `--seed`, because a calibration set that
changes between runs makes two compiles incomparable.

**Two arrays are written**, and which one the DFC wants depends on the model
script:

* `calib_normalized.npy` -- float32 NHWC, already mean/std normalised.
  Feeds a parsed graph whose input is the ONNX `pixel_values` directly.
* `calib_uint8.npy` -- uint8 NHWC, resized and padded but not normalised.
  Feeds a graph with a `normalization()` layer added in the model script,
  which is the more common Hailo pattern and the one that lets the Pi hand
  the part raw camera bytes.

They come from the same frames, so the choice is a toolchain detail rather
than a different experiment.

Usage::

    python -m tools.hailo.calibration_set --out build/owlv2 --n 128
"""

from __future__ import annotations

import argparse
import json
import random
import sys
from pathlib import Path

DEFAULT_RECORDINGS = Path("recordings")


def _walk_frames(walk: Path) -> list[tuple[Path, bool | None]]:
    """Every frame in a walk, paired with its adjudicated visibility.

    `None` where the walk has no `labels.json`. Deliberately NOT falling
    back to `walk.jsonl` -- that is the same rule `control/perception_eval.py`
    enforces, for the same reason: the model's own claim is not a label.
    """
    labels: dict[str, bool] = {}
    labels_path = walk / "labels.json"
    if labels_path.exists():
        labels = json.loads(labels_path.read_text()).get(
            "target_visible_labels", {})
    out = []
    for frame in sorted(walk.glob("frame-*.jpg")):
        out.append((frame, labels.get(frame.name)))
    return out


def choose(recordings: Path, n: int, seed: int = 0,
           visible_share: float = 0.4) -> list[Path]:
    """Pick `n` frames, stratified by walk and balanced on visibility."""
    walks = sorted(p for p in recordings.iterdir()
                   if p.is_dir() and any(p.glob("frame-*.jpg")))
    if not walks:
        raise SystemExit(f"no walks with frames under {recordings}")

    rng = random.Random(seed)
    per_walk = {w: _walk_frames(w) for w in walks}

    want_visible = int(round(n * visible_share))
    visible, other = [], []
    for frames in per_walk.values():
        for path, is_visible in frames:
            (visible if is_visible else other).append(path)

    # Round-robin over walks inside each pool, so the stratification
    # survives the visibility balance rather than being overridden by it.
    def draw(pool: list[Path], count: int) -> list[Path]:
        by_walk: dict[Path, list[Path]] = {}
        for p in pool:
            by_walk.setdefault(p.parent, []).append(p)
        for group in by_walk.values():
            rng.shuffle(group)
        picked, order = [], sorted(by_walk)
        while len(picked) < count and any(by_walk[w] for w in order):
            for w in order:
                if by_walk[w] and len(picked) < count:
                    picked.append(by_walk[w].pop())
        return picked

    picked = draw(visible, min(want_visible, len(visible)))
    picked += draw(other, n - len(picked))
    if len(picked) < n:
        print(f"[calib] only {len(picked)} frames available, wanted {n}")
    return sorted(picked)


def build(out_dir: Path, recordings: Path = DEFAULT_RECORDINGS,
          n: int = 128, seed: int = 0, model_id: str | None = None,
          image_size: int | None = None):
    try:
        import numpy as np
    except ImportError:  # pragma: no cover - dependency shape
        raise SystemExit("this needs numpy")
    from PIL import Image
    from transformers import Owlv2Processor

    from tools.hailo.export_owlv2_onnx import DEFAULT_MODEL

    proc = Owlv2Processor.from_pretrained(model_id or DEFAULT_MODEL)
    ip = proc.image_processor
    size = image_size or ip.size["height"]
    mean = np.array(ip.image_mean, dtype=np.float32)
    std = np.array(ip.image_std, dtype=np.float32)

    frames = choose(recordings, n, seed)
    out_dir.mkdir(parents=True, exist_ok=True)

    # Written straight to disk-backed arrays: the float32 one is ~1.4 GB at
    # 128 frames and there is no reason to hold it and a copy in RAM.
    normalized = np.lib.format.open_memmap(
        out_dir / "calib_normalized.npy", mode="w+",
        dtype=np.float32, shape=(len(frames), size, size, 3))
    raw = np.lib.format.open_memmap(
        out_dir / "calib_uint8.npy", mode="w+",
        dtype=np.uint8, shape=(len(frames), size, size, 3))

    for i, path in enumerate(frames):
        img = Image.open(path).convert("RGB")
        # The processor, not a reimplementation of it. OWLv2 rescales, pads
        # to a square with grey 0.5, then resizes through a Gaussian
        # anti-aliasing pre-filter whose sigma it derives from the scale
        # factor -- hand-rolling that was tried here and came out 2.0 off in
        # normalised units, which is most of the input range.
        # size= is not optional on the non-native path: without it the
        # processor happily returns 960px tensors into an array shaped
        # for `size`, and the mismatch only surfaces as a shape error
        # much later, on the rented box.
        chw = proc(images=img, return_tensors="np",
                   size={"height": size, "width": size})["pixel_values"][0]
        normalized[i] = chw.transpose(1, 2, 0)
        # The uint8 array is the same frames with the normalisation undone,
        # so a model script carrying a normalization() layer reproduces the
        # float array to within 1/255 -- well under INT8 quantisation noise,
        # and exactly consistent by construction rather than by a second
        # preprocessing path that could drift.
        raw[i] = np.clip(
            (normalized[i] * std + mean) * 255.0, 0, 255).round().astype(
                np.uint8)

    normalized.flush()
    raw.flush()

    manifest = {
        "n": len(frames),
        "seed": seed,
        "image_size": size,
        "image_mean": ip.image_mean,
        "image_std": ip.image_std,
        "frames": [str(p.relative_to(recordings)) for p in frames],
        "walks": sorted({p.parent.name for p in frames}),
    }
    (out_dir / "calib_manifest.json").write_text(json.dumps(manifest, indent=1))

    by_walk: dict[str, int] = {}
    for p in frames:
        by_walk[p.parent.name] = by_walk.get(p.parent.name, 0) + 1
    print(f"[calib] {len(frames)} frames at {size}px from "
          f"{len(by_walk)} walks -> {out_dir}")
    for walk, count in sorted(by_walk.items()):
        print(f"[calib]   {count:>4}  {walk}")
    print(f"[calib] calib_normalized.npy  "
          f"{normalized.nbytes / 1e6:.0f} MB  float32 NHWC")
    print(f"[calib] calib_uint8.npy       "
          f"{raw.nbytes / 1e6:.0f} MB  uint8 NHWC")
    return out_dir


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, default=Path("build/owlv2"))
    ap.add_argument("--recordings", type=Path, default=DEFAULT_RECORDINGS)
    ap.add_argument("--n", type=int, default=128)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--image-size", type=int, default=None)
    args = ap.parse_args(argv)
    build(args.out, args.recordings, args.n, args.seed,
          image_size=args.image_size)
    return 0


if __name__ == "__main__":
    sys.exit(main())
