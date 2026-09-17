"""SegFormer-B0 (ADE20K) -> ONNX, for the Hailo compile loop.

P16 found that **the floor mask is what the tier rests on**: it rescued a
detector degraded from 34% to 5% with no tier cost (49% against fp32's
50%), and with the mask off the same three configs read 20% / 13% / 3%.
So the model that decides whether the on-board tier works at all is this
one, and it has never been quantized.

Two things this has to keep straight, and the reason it is not just
`hailomz compile segformer_b0_bn`:

  * **ADE20K, not Cityscapes.** The zoo ships `segformer_b0_bn` trained on
    Cityscapes at 512x1024, and declares it for `hailo10h`. That proves
    the ARCHITECTURE places on the part; it does not give us a floor mask.
    ADE20K is one of the few segmentation sets carrying `floor`, `rug` and
    `earth` as classes at all (`brain/perceive.py` FLOOR_WORDS), which is
    the whole reason this checkpoint was chosen.
  * **The argmax stays on the host.** The pipeline does not want a class
    map, it wants "which pixels are floor" -- a reduction over
    FLOOR_WORDS' indices. Exporting logits and reducing on the Pi keeps
    the accelerator doing the convolutional work and leaves the label
    vocabulary a runtime argument, the same split the YOLO-World text
    einsum and CLIP's text encoder already use.

Writes the ONNX plus an `export_meta.json` recording the checkpoint, the
input size, the normalisation and the floor class indices -- so a later
run cannot silently score a different set of labels than the one the fp32
baseline used.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="nvidia/segformer-b0-finetuned-ade-512-512")
    ap.add_argument("--out", type=Path, default=Path("build/segformer"))
    ap.add_argument("--size", type=int, default=512)
    ap.add_argument("--opset", type=int, default=17)
    args = ap.parse_args(argv)

    import numpy as np
    import torch
    from transformers import SegformerForSemanticSegmentation

    args.out.mkdir(parents=True, exist_ok=True)
    model = SegformerForSemanticSegmentation.from_pretrained(args.model).eval()

    id2label = model.config.id2label
    floor_words = ("floor", "rug", "carpet", "earth", "ground")
    floor_ids = sorted(int(i) for i, lab in id2label.items()
                       if any(w in str(lab).lower() for w in floor_words))
    if not floor_ids:
        raise SystemExit(
            f"{args.model} has no label matching {floor_words} -- this is the "
            "wrong checkpoint for a floor mask, and compiling it would "
            "measure a model that cannot do the job.")

    class ConvDecodeHead(torch.nn.Module):
        """SegFormer's decode head with its Linear projections rewritten as
        1x1 convolutions -- an EXACT identity, not an approximation.

        `SegformerMLP.forward` is `flatten(2).transpose(1,2)` -> `Linear`,
        and the decode head then transposes and reshapes back. Those are
        the two ops the Hailo parser refuses:

            UnsupportedShuffleLayerError in op node_Reshape_581
            UnsupportedShuffleLayerError in op node_Reshape_654

        A Linear applied independently at every spatial position IS a 1x1
        Conv over the feature map -- same weights, same arithmetic, in the
        layout the accelerator already wants. Rewriting it deletes the
        flatten, both transposes and the reshape, so the unsupported nodes
        stop existing rather than being cut around.

        This is the same move as `export_owlv2_onnx.py --factor-patch`
        (rewriting one conv as two, verified at max |d| 1.4e-05): change
        the graph's SHAPE, never its numbers, and prove it afterwards.
        """

        def __init__(self, head):
            super().__init__()
            self.head = head
            convs = []
            for mlp in head.linear_projections:
                lin = mlp.proj
                conv = torch.nn.Conv2d(lin.in_features, lin.out_features, 1)
                with torch.no_grad():
                    conv.weight.copy_(lin.weight.view(lin.out_features,
                                                      lin.in_features, 1, 1))
                    conv.bias.copy_(lin.bias)
                convs.append(conv)
            self.convs = torch.nn.ModuleList(convs)

        def forward(self, encoder_hidden_states):
            h = self.head
            target = encoder_hidden_states[0].shape[2:]
            feats = []
            for state, conv in zip(encoder_hidden_states, self.convs):
                x = conv(state)
                x = torch.nn.functional.interpolate(
                    x, size=target, mode="bilinear", align_corners=False)
                feats.append(x)
            x = h.linear_fuse(torch.cat(feats[::-1], dim=1))
            x = h.activation(h.batch_norm(x))
            return h.classifier(x)

    class Logits(torch.nn.Module):
        """Logits only. The floor reduction is a host-side argmax over
        `floor_ids`, so the vocabulary stays a runtime argument. Output is
        at H/4 -- SegFormer's native logit resolution -- and the final
        upsample to frame size stays on the host, where it is one cheap
        bilinear resize."""

        def __init__(self, m):
            super().__init__()
            self.encoder = m.segformer
            self.head = ConvDecodeHead(m.decode_head)

        def forward(self, pixel_values):
            states = self.encoder(pixel_values, output_hidden_states=True,
                                  return_dict=True).hidden_states
            return self.head(states)

    wrapped = Logits(model)

    # Prove the rewrite before exporting it: a graph that compiles and
    # computes something else is the worst outcome available here.
    with torch.no_grad():
        ref = model(pixel_values=torch.randn(1, 3, args.size, args.size)).logits
    torch.manual_seed(0)
    probe = torch.randn(1, 3, args.size, args.size)
    with torch.no_grad():
        a = model(pixel_values=probe).logits
        b = wrapped(probe)
    rewrite_diff = float((a - b).abs().max())
    print(f"[segformer] conv-rewrite vs original: max |diff| {rewrite_diff:.3e}")
    if rewrite_diff > 1e-4:
        raise SystemExit(
            f"the 1x1-conv rewrite is NOT an identity (max |diff| "
            f"{rewrite_diff}) -- do not export it")
    dummy = torch.randn(1, 3, args.size, args.size)
    onnx_path = args.out / f"segformer_b0_ade_{args.size}_op{args.opset}.onnx"
    torch.onnx.export(
        wrapped, dummy, str(onnx_path),
        input_names=["pixel_values"], output_names=["logits"],
        opset_version=args.opset, do_constant_folding=True)

    # Verify the export reproduces the real model, because a HEF of a wrong
    # graph compiles fine and is worthless (the lesson export_owlv2_onnx.py
    # carries).
    import onnxruntime as ort
    sess = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    with torch.no_grad():
        ref = wrapped(dummy).numpy()
    got = sess.run(None, {"pixel_values": dummy.numpy()})[0]
    diff = float(np.abs(ref - got).max())

    meta = {
        "checkpoint": args.model,
        "image_size": args.size,
        "opset": args.opset,
        "floor_ids": floor_ids,
        "floor_labels": [str(id2label[i]) for i in floor_ids],
        "num_labels": int(model.config.num_labels),
        # ADE20K SegFormer uses ImageNet statistics.
        "image_mean": [0.485, 0.456, 0.406],
        "image_std": [0.229, 0.224, 0.225],
        "onnx": onnx_path.name,
        "max_abs_diff_vs_torch": diff,
        "conv_rewrite_max_abs_diff": rewrite_diff,
        "output": "logits at H/4; host does the final bilinear upsample",
    }
    (args.out / "export_meta.json").write_text(json.dumps(meta, indent=2))
    print(json.dumps(meta, indent=2))
    if diff > 1e-3:
        raise SystemExit(f"export does not reproduce the model: max |diff| {diff}")
    print(f"\nwrote {onnx_path} ({onnx_path.stat().st_size / 1e6:.1f} MB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
