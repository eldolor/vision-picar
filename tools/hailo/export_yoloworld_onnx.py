"""YOLO-World -> ONNX for the Hailo compile loop, and what the export
settles before a single cent is spent.

Why this model. The owner ruled out the Jetson on cost (2026-09-13), so
the part is Pi + Hailo-8L and OWLv2 -- the best model measured, 82% at
3 FP -- cannot run on it: P6 proved it dies at ALLOCATION on 73 layernorm
and 38 softmax layers. Within what a Hailo CAN take, P9 measured
YOLO-World + CLIP at 72% recall / 99% precision against the shipped
YOLO11s + CLIP's 45% / 94%. So the reactive tier is worth 45% or 72% on
one question: does YOLO-World compile?

**Two things the ONNX export answers for free, before EC2.**

1. The op census is the opposite of OWLv2's. Counted on the real export:

       LayerNormalization    0   (OWLv2: 73)
       Softmax               1   (OWLv2: 38)
       Conv                 68   (OWLv2:  1)
       total nodes         430   (OWLv2: 575)

   The single Softmax is YOLOv8's DFL box decode, which is in 4.9's
   "compiles well" family. This is a convolutional graph. That is a
   reason to run the test, NOT a substitute for it -- OWLv2's row in
   Hailo's own table said it should compile too.

2. **The vocabulary is BAKED IN, and this is the finding that changes
   the architecture.** The exported graph's only input is
   `images[1,3,640,640]`; there is no text input. The text embeddings
   enter at seven Einsums -- four in the vision-language path
   aggregation neck, three at the contrastive head -- and every one of
   them consumes a folded constant.

   So a YOLO-World HEF is **not an open-vocabulary detector on the
   robot**. It is a *chosen-vocabulary* one: the class list is fixed at
   compile time and cannot be changed on a Pi. P9's 72% was measured
   with the target string set per walk, i.e. genuinely open -- on a
   baked HEF you get that only for targets inside the compiled list and
   nothing at all outside it.

   The deployable form is therefore an N-class vocabulary chosen up
   front, and it is free: a 32-class export is the same 430 nodes as a
   1-class export, with the output going from [1,5,8400] to
   [1,36,8400]. Still a large win over COCO's fixed 80 -- you choose the
   names, in free text, embedded by YOLO-World's own text encoder -- but
   a weaker claim than "open vocabulary", and it has to be written down
   as the weaker one.

Usage:
    python -m tools.hailo.export_yoloworld_onnx --build build/yoloworld
"""
from __future__ import annotations

import argparse
import collections
import json
from pathlib import Path

# A household vocabulary, not a demo list: every target the corpus
# actually contains, plus the distractors P9 identified (the cable-knit
# bolster and the exercise ball both fooled a model), plus the obstacles
# a floor robot has to name. Chosen here rather than at the call site
# because it is a DESIGN decision once it is baked into a HEF.
DEFAULT_VOCAB = [
    # the corpus's own targets
    "woven laundry basket", "blue bottle", "running shoes", "backpack",
    # P9's measured distractors, so the detector can name them rather
    # than silently score them as the target
    "knitted cushion", "exercise ball", "vacuum cleaner", "storage bin",
    # household objects a search mission plausibly asks for
    "cardboard box", "potted plant", "office chair", "laptop", "mug",
    "towel", "pillow", "blanket", "book", "remote control", "lamp",
    "umbrella", "suitcase", "toolbox", "pet bowl", "broom", "waste bin",
    # obstacles and structure, for the reactive tier's own job
    "door", "staircase", "dumbbell", "yoga mat", "sofa", "dining table",
    "television",
]

WEIGHTS = "yolov8s-worldv2.pt"


def export(build: Path, vocab: list, weights: str = WEIGHTS,
           imgsz: int = 640, opset: int = 13) -> Path:
    from ultralytics import YOLOWorld

    build.mkdir(parents=True, exist_ok=True)
    model = YOLOWorld(weights)
    model.set_classes(vocab)
    produced = Path(model.export(format="onnx", imgsz=imgsz, opset=opset,
                                 simplify=False, dynamic=False))
    target = build / f"yoloworld-vocab{len(vocab)}.onnx"
    produced.replace(target)
    return target


def census(path: Path) -> dict:
    """The op counts, and whether anything OWLv2 died on is present."""
    import onnx

    graph = onnx.load(str(path)).graph
    ops = collections.Counter(n.op_type for n in graph.node)
    return {
        "nodes": len(graph.node),
        "ops": dict(ops.most_common()),
        "layernorm": ops.get("LayerNormalization", 0) + ops.get("ReduceMean", 0),
        "softmax": ops.get("Softmax", 0),
        "einsum": ops.get("Einsum", 0),
        "conv": ops.get("Conv", 0),
        "inputs": [(i.name, [d.dim_value for d in i.type.tensor_type.shape.dim])
                   for i in graph.input],
        "outputs": [(o.name, [d.dim_value for d in o.type.tensor_type.shape.dim])
                    for o in graph.output],
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--build", default="build/yoloworld", type=Path)
    ap.add_argument("--weights", default=WEIGHTS)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--opset", type=int, default=13)
    ap.add_argument("--vocab-size", type=int, default=None,
                    help="truncate DEFAULT_VOCAB, for a size sweep")
    args = ap.parse_args(argv)

    vocab = DEFAULT_VOCAB[:args.vocab_size] if args.vocab_size else DEFAULT_VOCAB
    path = export(args.build, vocab, args.weights, args.imgsz, args.opset)
    meta = census(path)
    meta.update({"vocab": vocab, "weights": args.weights,
                 "imgsz": args.imgsz, "opset": args.opset,
                 "onnx": path.name})
    (args.build / "export_meta.json").write_text(json.dumps(meta, indent=1))

    print(f"{path}  ({len(vocab)} classes, {meta['nodes']} nodes)")
    print(f"  layernorm/reducemean {meta['layernorm']}   "
          f"softmax {meta['softmax']}   einsum {meta['einsum']}   "
          f"conv {meta['conv']}")
    print(f"  inputs  {meta['inputs']}")
    print(f"  outputs {meta['outputs']}")
    if meta["layernorm"] == 0 and meta["softmax"] <= 2:
        print("  -> convolutional graph; none of what stopped OWLv2 is here")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
