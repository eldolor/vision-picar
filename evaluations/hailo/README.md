# OWLv2 -> Hailo-8L: the compile attempt, 2026-09-09

Raw output of `tools/hailo/compile_owlv2.py` against **Dataflow Compiler
3.34.0**, `hw_arch=hailo8l`, on an `r6i.4xlarge`. The instance is gone; these
are the records. See `PLAN-onboard-perception.md` P6 for what they decided.

> **Hailo path closed 2026-09-19.** The board is a Jetson Orin Nano Super
> (`PLAN-onboard-perception.md`, "The Hailo path is CLOSED"); these records
> are history and decide nothing now. **`zoo-probe/`** holds the later runs
> in Hailo's own AI Software Suite container (P15-P18): the DFC 5.4.0 parse
> matrix, YOLO-World and OWLv2 on the Hailo-10H, SegFormer allocation on
> both parts (the two `segformer_b0_ade-*.hef` files and the INT8 mask-IoU
> records), and the 365-frame tier rows scored from quantized detections --
> see P15-P18 for what each file measured.

**Verdict: OWLv2 does not compile to a Hailo-8L HEF.** Not for the reason
Hailo's own compatibility table gives.

| file | variant | translate | optimize | compile |
|---|---|---|---|---|
| `report-960-default.json` | 960px, stock graph | ok | ok | FAIL `conv1` |
| `report-640-default.json` | 640px (1600 tokens) | ok | ok | FAIL `conv1` |
| `report-640-autoreshapes.json` | 640px + `allocator_param(automatic_reshapes=enabled)` | ok | ok | FAIL `conv1` |
| `report-960-factored.json` | 960px, patch conv factored into two 4x4 convs | ok | ok | **FAIL, whole transformer body** |

The `.log` files are the DFC's own output, with the per-frame calibration
progress bars stripped. `logs/` holds the tracebacks the harness saved.

## What the four rows mean together

**Translation and quantization are not the problem, and that is the
surprise.** OWLv2's ViT-B/16 image tower parses in 44-107s and quantizes
without OOM at both 3600 and 1600 tokens. DFC 3.34 carries a LayerNorm
Decomposition pass, Matmul Equalization and MatmulDecompose, and uses all
three. `PLAN-onboard-perception.md` 1.10 said the 8-series has *"no efficient
attention path and no memory for the weights"*; the second half is measured
false and the first is more specific than that.

**The first three rows all die on `conv1`** -- the single Conv in a 575-node
graph, the 16x16-stride-16 patch embedding:

    Reshape is needed for layers: conv1, but adding a reshape has failed

Resolution does not change it (row 2) and neither does the allocator's own
reshape policy (row 3), so it is not capacity and not a missing flag.

**Row 4 is the one that answers the question.** Factoring the patch
embedding into `3->48 k4s4` (one-hot, a space-to-depth by 4) followed by
`48->768 k4s4` (the original weights, re-indexed) is an exact identity --
`export_owlv2_onnx.py --factor-patch`, verified against the unmodified model
at max |d score| 1.4e-05 with the top-50 patch set unchanged. It removes
`conv1` from the error. What appears in its place is the **entire transformer
body**:

| layers named in the failure | count |
|---|---|
| layer normalization (reduce_mean / sub / square / mult) | 73 |
| softmax (reduce_max / sub / sum / mult) | 38 |
| precision change | 36 |
| matmul | 9 |
| conv | 8 |

`conv1` was masking the real wall. The Hailo allocator cannot place the
reshapes that attention's and layernorm's per-token reductions require --
every one of them, not a subset. That is an architectural limit of the
dataflow design, not a size limit, which is why no smaller input and no flag
moved it.

## What is NOT established here

**Nothing about accuracy.** No HEF was produced, so nothing was measured
on-chip. The accuracy results OWLv2 won on remain the ones in
`../final-owlv2-*.json` and `../basket-owlv2-*.json`, measured in PyTorch.

**Nothing about other models.** YOLO11 n/s/m compile for the 8L off the
shelf (4.3.1) and this run says nothing to the contrary -- it was never
pointed at them. The reactive tier's part choice is untouched.

**Nothing about a newer DFC.** 3.34.0 was the newest available on
2026-09-09. If a later release adds allocator support for token-dimension
reductions, this is one `ec2.sh up` away from being re-run -- which is the
reason the loop was built rather than the answer being looked up.
