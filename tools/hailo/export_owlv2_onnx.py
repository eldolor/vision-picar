"""Export OWLv2's image tower + detection heads to ONNX, for the Hailo DFC.

`PLAN-onboard-perception.md` P5 ends on one unrun test: *can OWLv2 compile
to a Hailo HEF?* This is step one of answering it. Nothing here is about
accuracy -- OWLv2's accuracy is settled (68/68 visible frames, three
targets, zero false positives). This is about whether the graph survives a
toolchain built for CNNs.

**The split, and why it is this one.** OWLv2 is a ViT-B/16 image tower, a
CLIP-style text tower, and three small MLP heads. Only the image side goes
to the accelerator; the text tower stays on the Pi's CPU, exactly as CLIP's
already does (4.2). That is not a convenience -- the text tower runs once
per target string, not once per frame, so putting it on the Hailo would
spend the scarce part on the cheap half.

The seam is inside `Owlv2ClassPredictionHead`. Its forward is::

    image_class_embeds = dense0(image_feats)          # image only
    pred_logits = image_class_embeds @ query_embeds.T # the ONLY text mixing
    pred_logits = (pred_logits + logit_shift) * logit_scale

Everything up to the einsum is image-only and compiles; the einsum and what
follows is a [3600, 512] x [512, n_queries] matmul the Pi does in
microseconds. So this emits `image_class_embeds`, `logit_shift` and
`logit_scale` separately and leaves the join to `owlv2_host_head.py`.
`verify()` asserts the two halves reassemble into what the unmodified
`Owlv2ForObjectDetection` produces -- because a HEF of a subtly wrong graph
compiles perfectly and is worthless, and that would not surface until the
part was bought.

**Three axes, because a single "it did not compile" is not an answer.**
The DFC either supports an op or it does not, and which op it stopped on
decides whether the answer is "OWLv2 cannot go on a Hailo" or "move two
lines to the CPU". So this writes a matrix and `compile_owlv2.py` walks it:

* **opset**: at >= 17 torch emits `LayerNormalization` as one op; below it
  the same maths decomposes into ReduceMean/Sub/Pow/Div. A parser that
  rejects the fused op may accept the decomposition.
* **head**: `full` keeps the L2 normalise (`ReduceL2`), the `Elu` on
  logit_scale and the box `Sigmoid` on the accelerator. `minimal` moves all
  three to the host, where they cost microseconds on [3600, 512] and
  [3600, 1] tensors. Those are the three ops in this graph least likely to
  be in a CNN toolchain, and losing them costs nothing real.
* **image size**: OWLv2-base-patch16 is native at 960, which is 60x60 =
  **3600 tokens** -- against ~196 for the ImageNet ViTs in Hailo's zoo.
  Attention is quadratic in that, so it is the most likely thing to exhaust
  the part. `--image-size` interpolates the position encoding and is the
  one lever that shrinks it. **Any non-native size changes accuracy and
  must be re-scored** with `control/perception_eval.py` before it counts.

Usage::

    python -m tools.hailo.export_owlv2_onnx --out build/owlv2
    python -m tools.hailo.export_owlv2_onnx --out build/owlv2 --verify-only
    python -m tools.hailo.export_owlv2_onnx --out build/owlv2 --image-size 640
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Mapping, Sequence

DEFAULT_MODEL = "google/owlv2-base-patch16-ensemble"

# Tried in this order by the compile loop: the smallest graph first, then
# the decomposed one, because a parser that rejects a fused op is the
# cheapest failure to route around.
OPSETS = (17, 14)

# `full` first: if it compiles, nothing has to move to the CPU.
HEADS = ("full", "minimal")

OUT_NAMES = ("image_class_embeds", "logit_shift", "logit_scale",
             "pred_boxes", "objectness_logits")


def _torch():
    try:
        import torch
        return torch
    except ImportError as exc:  # pragma: no cover - dependency shape
        raise SystemExit(
            "This needs torch + transformers: "
            "pip install -r requirements-perception.txt") from exc


def build_wrapper(model, head: str = "full", factor_patch: bool = False):
    """Wrap `Owlv2ForObjectDetection` as an image-only, text-free graph."""
    if head not in HEADS:
        raise ValueError(f"head must be one of {HEADS}, got {head!r}")
    torch = _torch()
    nn = torch.nn

    class _ImageTower(nn.Module):
        def __init__(self, owl):
            super().__init__()
            self.head = head
            self.vision_model = owl.owlv2.vision_model
            if factor_patch:
                # Swapped in place: the ViT's own forward calls
                # embeddings.patch_embedding, so replacing the module is
                # enough and nothing else in the graph changes.
                self.vision_model.embeddings.patch_embedding = \
                    factor_patch_embedding(owl, owl.config.vision_config.patch_size)
            self.layer_norm = owl.layer_norm
            self.class_head = owl.class_head
            self.box_head = owl.box_head
            self.objectness_head = owl.objectness_head
            # A constant [num_patches, 4] bias. In `full` it is added on the
            # accelerator; in `minimal` the host adds it, so it ships in the
            # metadata instead.
            self.register_buffer("box_bias", owl.box_bias.clone(),
                                 persistent=False)

        def forward(self, pixel_values):
            # Mirrors Owlv2ForObjectDetection.image_embedder, minus the
            # reshape to an [h, w] grid -- that reshape exists only for
            # interpolate_pos_encoding, and a 4-D reshape is one more thing
            # for a parser to dislike.
            hidden = self.vision_model(pixel_values=pixel_values)[0]
            embeds = self.vision_model.post_layernorm(hidden)

            class_token = embeds[:, :1, :].expand(-1, embeds.shape[1] - 1, -1)
            feats = self.layer_norm(embeds[:, 1:, :] * class_token)

            # class_head, stopped one op short of the text einsum.
            image_class_embeds = self.class_head.dense0(feats)
            logit_shift = self.class_head.logit_shift(feats)
            logit_scale = self.class_head.logit_scale(feats)
            pred_boxes = self.box_head(feats)
            objectness_logits = self.objectness_head(feats)[..., 0]

            if self.head == "full":
                image_class_embeds = image_class_embeds / (
                    torch.linalg.norm(image_class_embeds, dim=-1,
                                      keepdim=True) + 1e-6)
                logit_scale = self.class_head.elu(logit_scale) + 1
                pred_boxes = torch.sigmoid(pred_boxes + self.box_bias)

            return (image_class_embeds, logit_shift, logit_scale,
                    pred_boxes, objectness_logits)

    return _ImageTower(model).eval()



def factor_patch_embedding(model, patch: int, split: int = 4):
    """Rewrite the 16x16-stride-16 patch conv as two small convs.

    **Why this exists.** DFC 3.34 translates and quantizes OWLv2 happily and
    then fails allocation with::

        Reshape is needed for layers: conv1, but adding a reshape has failed

    `conv1` is the patch embedding: the single Conv in a 575-node graph, and
    an unusual one -- 16x16 kernel at stride 16, 3 channels in, 768 out. The
    allocator wants to rewrite it as a space-to-depth plus a 1x1 (the SDK
    carries a `SPACE_TO_DEPTH` conversion type) and cannot place the reshape.
    Measured: neither `--image-size 640` nor
    `allocator_param(automatic_reshapes=enabled)` changes it.

    So do the factorisation in the export, where it is exact arithmetic
    rather than an allocator heuristic, and in a form that keeps everything
    inside a CNN toolchain's comfort zone -- **an image-shaped input and two
    small-kernel convs**, rather than a 768-channel input tensor:

        conv_a: 3 -> 48,  k=4, s=4   one-hot weights; a space-to-depth by 4
        conv_b: 48 -> 768, k=4, s=4  the original weights, re-indexed

    A 16x16 patch is a 4x4 grid of 4x4 blocks, so composing the two covers
    exactly the same receptive field with exactly the same weights. This is
    an identity, not an approximation, and `verify()` checks it the same way
    it checks the text-tower split.
    """
    torch = _torch()
    nn = torch.nn
    conv = model.owlv2.vision_model.embeddings.patch_embedding
    weight = conv.weight.data                      # [768, 3, 16, 16]
    out_ch, in_ch, kh, kw = weight.shape
    if (kh, kw) != (patch, patch) or patch % split:
        raise SystemExit(f"cannot factor a {kh}x{kw} patch by {split}")
    step = patch // split                          # 4

    mid = in_ch * step * step                      # 48
    conv_a = nn.Conv2d(in_ch, mid, kernel_size=step, stride=step, bias=False)
    wa = torch.zeros(mid, in_ch, step, step)
    for c in range(in_ch):
        for i in range(step):
            for j in range(step):
                wa[c * step * step + i * step + j, c, i, j] = 1.0
    conv_a.weight.data = wa

    conv_b = nn.Conv2d(mid, out_ch, kernel_size=split, stride=split,
                       bias=conv.bias is not None)
    wb = torch.zeros(out_ch, mid, split, split)
    for c in range(in_ch):
        for i in range(step):
            for j in range(step):
                p_ = c * step * step + i * step + j
                for bi in range(split):
                    for bj in range(split):
                        wb[:, p_, bi, bj] = weight[:, c, bi * step + i,
                                                   bj * step + j]
    conv_b.weight.data = wb
    if conv.bias is not None:
        conv_b.bias.data = conv.bias.data.clone()

    class _FactoredPatchEmbedding(nn.Module):
        # Not a bare nn.Sequential: Owlv2VisionTransformer.forward reads
        # `self.embeddings.patch_embedding.weight.dtype` to decide the input
        # dtype, so the replacement has to keep a `.weight` that means the
        # same thing. conv_b carries the real weights; conv_a is a constant
        # one-hot rearrangement.
        def __init__(self):
            super().__init__()
            self.space_to_depth = conv_a
            self.projection = conv_b

        @property
        def weight(self):
            return self.projection.weight

        @property
        def bias(self):
            return self.projection.bias

        def forward(self, x):
            return self.projection(self.space_to_depth(x))

    return _FactoredPatchEmbedding().eval()


def load(model_id: str = DEFAULT_MODEL):
    torch = _torch()
    from transformers import Owlv2ForObjectDetection, Owlv2Processor
    proc = Owlv2Processor.from_pretrained(model_id)
    model = Owlv2ForObjectDetection.from_pretrained(model_id).eval()
    return proc, model


def onnx_name(opset: int, head: str, size: int,
              factor_patch: bool = False) -> str:
    tail = "_factored" if factor_patch else ""
    return f"owlv2_image_tower_{size}_op{opset}_{head}{tail}.onnx"


def export(out_dir: Path, model_id: str = DEFAULT_MODEL,
           opsets: Sequence[int] = OPSETS,
           heads: Sequence[str] = HEADS,
           image_size: int | None = None,
           factor_patch: bool = False) -> list[Path]:
    torch = _torch()
    out_dir.mkdir(parents=True, exist_ok=True)
    proc, model = load(model_id)
    native = model.config.vision_config.image_size
    size = image_size or native
    patch = model.config.vision_config.patch_size
    if size % patch:
        raise SystemExit(f"--image-size must be a multiple of the patch "
                         f"size ({patch}); got {size}")
    interpolate = size != native
    if interpolate:
        print(f"[export] NON-NATIVE input {size} (native {native}): "
              f"{(size // patch) ** 2} tokens vs {(native // patch) ** 2}. "
              f"Accuracy changes -- re-score before quoting anything.")

    dummy = torch.zeros(1, 3, size, size)
    written = []
    for head in heads:
        tower = build_wrapper(model, head, factor_patch)
        if interpolate:
            _patch_for_size(tower, model, size)
        for opset in opsets:
            path = out_dir / onnx_name(opset, head, size, factor_patch)
            print(f"[export] {head} head, opset {opset} -> {path}",
                  flush=True)
            torch.onnx.export(
                tower, (dummy,), str(path),
                input_names=["pixel_values"],
                output_names=list(OUT_NAMES),
                opset_version=opset,
                do_constant_folding=True,
                dynamo=False,   # the legacy tracer: static shapes, no
                                # ExportedProgram indirection, and what
                                # every Hailo tutorial assumes
            )
            written.append(path)

    num_patches = (size // patch) ** 2
    meta = {
        "model_id": model_id,
        "input_name": "pixel_values",
        "input_shape": [1, 3, size, size],
        "image_size": size,
        "native_image_size": native,
        "output_names": list(OUT_NAMES),
        "num_patches": num_patches,
        "patch_size": patch,
        "hidden_size": model.config.vision_config.hidden_size,
        "text_embed_dim": model.config.text_config.hidden_size,
        "image_mean": list(proc.image_processor.image_mean),
        "image_std": list(proc.image_processor.image_std),
        "opsets": list(opsets),
        "heads": list(heads),
        "factor_patch": factor_patch,
        # `minimal` needs this on the host; `full` has it baked in.
        "box_bias": _box_bias_for(model, size).tolist(),
        "files": [p.name for p in written],
    }
    (out_dir / "owlv2_export.json").write_text(json.dumps(meta, indent=1))
    print(f"[export] metadata -> {out_dir / 'owlv2_export.json'}")
    return written


def _box_bias_for(model, size: int):
    n = size // model.config.vision_config.patch_size
    return model.compute_box_bias(n, n).detach().numpy()


def _patch_for_size(tower, model, size: int):
    """Re-fit the position encoding and box bias to a non-native input."""
    torch = _torch()
    n = size // model.config.vision_config.patch_size
    tower.box_bias = torch.from_numpy(_box_bias_for(model, size))
    # transformers interpolates when asked; bind the flag so the traced
    # graph carries the resized encoding rather than the 960 one.
    inner = tower.vision_model.forward

    def forward(pixel_values, **kw):
        kw["interpolate_pos_encoding"] = True
        return inner(pixel_values=pixel_values, **kw)

    tower.vision_model.forward = forward
    return n


def verify(out_dir: Path, image: Path | None, text: str,
           model_id: str = DEFAULT_MODEL, opset: int = OPSETS[0],
           head: str = "full", image_size: int | None = None,
           factor_patch: bool = False) -> bool:
    """Reassemble the split graph and compare it to the real model."""
    torch = _torch()
    try:
        import onnxruntime as ort
    except ImportError:
        raise SystemExit("verification needs onnxruntime: "
                         "pip install onnxruntime")
    import numpy as np
    from PIL import Image

    from tools.hailo.owlv2_host_head import join

    proc, model = load(model_id)
    size = image_size or model.config.vision_config.image_size
    onnx_path = out_dir / onnx_name(opset, head, size, factor_patch)
    if not onnx_path.exists():
        raise SystemExit(f"no export at {onnx_path} -- "
                         f"run without --verify-only first")

    img = Image.open(image).convert("RGB") if image else Image.new(
        "RGB", (1280, 720), (110, 96, 80))
    inputs = proc(text=[[text]], images=img, return_tensors="pt")

    with torch.no_grad():
        reference = model(**inputs)
        # .pooler_output, not the return value: transformers 5.x returns the
        # whole BaseModelOutputWithPooling and stashes the *projected*
        # embedding on pooler_output in place.
        query = model.owlv2.get_text_features(
            input_ids=inputs["input_ids"],
            attention_mask=inputs["attention_mask"]).pooler_output.numpy()

    sess = ort.InferenceSession(str(onnx_path),
                                providers=["CPUExecutionProvider"])
    parts = dict(zip(OUT_NAMES, sess.run(
        None, {"pixel_values": inputs["pixel_values"].numpy()})))

    box_bias = _box_bias_for(model, size) if head == "minimal" else None
    logits, boxes, objectness = join(parts, query, head=head,
                                     box_bias=box_bias)

    ref_logits = reference.logits.numpy()
    ref_obj = reference.objectness_logits.numpy()

    def sig(x):
        return 1.0 / (1.0 + np.exp(-x))

    # Compare SCORES, not logits. OWLv2's logits span roughly -39..0 here,
    # so an absolute tolerance on them is a tolerance on a number nothing
    # downstream reads: a 4e-4 relative wobble from fp32 reduction order in
    # a 12-layer ViT would read as a failure while changing no decision.
    # What the pipeline consumes is sigmoid(logit) and the box, so those get
    # the tolerance -- plus the top-scoring patch and the top-50 set, which
    # are the bits a structural break in the split could not survive.
    d_score = float(np.abs(sig(logits) - sig(ref_logits)).max())
    d_boxes = float(np.abs(boxes - reference.pred_boxes.numpy()).max())
    d_obj = float(np.abs(sig(objectness) - sig(ref_obj)).max())
    d_logit_rel = float((np.abs(logits - ref_logits)
                         / (np.abs(ref_logits) + 1e-6)).max())
    same_top = bool(logits.argmax() == ref_logits.argmax())
    top_n = 50
    overlap = len(set(np.argsort(-ref_logits.ravel())[:top_n].tolist())
                  & set(np.argsort(-logits.ravel())[:top_n].tolist()))

    ok = (d_score < 1e-3 and d_boxes < 1e-3 and d_obj < 1e-3
          and same_top and overlap == top_n)
    print(f"[verify] {head} head, opset {opset}, {size}px "
          f"on {image or '<flat test image>'!s}")
    print(f"[verify]   max |d score|      = {d_score:.3e}   (sigmoid)")
    print(f"[verify]   max |d pred_boxes| = {d_boxes:.3e}")
    print(f"[verify]   max |d objectness| = {d_obj:.3e}   (sigmoid)")
    print(f"[verify]   max rel |d logit|  = {d_logit_rel:.3e}")
    print(f"[verify]   top-1 patch agrees = {same_top};  "
          f"top-{top_n} overlap = {overlap}/{top_n}")
    print(f"[verify]   top score torch="
          f"{float(reference.logits.sigmoid().max()):.4f} "
          f"split={float(sig(logits).max()):.4f}")
    print("[verify]", "PASS" if ok else "FAIL -- the split is not equivalent")
    return ok


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, default=Path("build/owlv2"))
    ap.add_argument("--model", default=DEFAULT_MODEL)
    ap.add_argument("--opset", type=int, action="append", default=None)
    ap.add_argument("--head", action="append", default=None, choices=HEADS)
    ap.add_argument("--factor-patch", action="store_true",
                    help="split the 16x16 patch conv into two 4x4 convs "
                         "(exact); routes around the conv1 allocator failure")
    ap.add_argument("--image-size", type=int, default=None,
                    help="non-native input; changes accuracy, re-score it")
    ap.add_argument("--verify-only", action="store_true")
    ap.add_argument("--no-verify", action="store_true")
    ap.add_argument("--image", type=Path, default=None,
                    help="a real frame to verify on; defaults to a flat image")
    ap.add_argument("--text", default="a woven laundry basket")
    args = ap.parse_args(argv)

    opsets = tuple(args.opset) if args.opset else OPSETS
    heads = tuple(args.head) if args.head else HEADS
    if not args.verify_only:
        export(args.out, args.model, opsets, heads, args.image_size,
               args.factor_patch)
    if args.no_verify:
        return 0
    # Verification runs against the real processor, which always feeds the
    # native size, so a resized export cannot be checked this way.
    if args.image_size:
        print("[verify] skipped: a non-native export cannot be compared to "
              "the stock processor's own preprocessing")
        return 0
    ok = all(verify(args.out, args.image, args.text, args.model, o, h,
                    None, args.factor_patch)
             for h in heads for o in opsets)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
