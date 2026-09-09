# The Hailo compile loop

`PLAN-onboard-perception.md` 1.10 item 1 asks for this before hardware day,
in almost these words:

> Every model goes PyTorch → ONNX → Hailo's Dataflow Compiler → HEF, on an
> x86-64 Ubuntu host with a calibration set of a few hundred frames. **Build
> that loop before the hardware arrives.** If the loop exists on day one the
> Hailo is a sandbox; if it never gets built, the Hailo is a fixed-function
> part and the IMX500 was the cheaper way to get one.

It was never started. It is started now, and pointed at the model that
actually decides the purchase.

## The question

P5 recommends **Raspberry Pi 5 + Hailo-8L with OWLv2** on measured accuracy:
68 of 68 visible frames across three targets at zero false positives, better
than Qwen3-VL-4B, better than Opus 4.5 on the walk that broke 1.11, at
~150M parameters and 2.1s/frame. The whole recommendation rests on one thing
nobody has tried:

> **Can OWLv2 compile to a Hailo HEF?**

Hailo's own compatibility table (1.10) lists OWLv2 under *"does not fit —
anything attention-heavy"*. So **failure is the expected outcome**, and it is
a perfectly good answer: it sends the decision to a Jetson, but for a 150M
model rather than the 4B one the VLM thread was going to justify. What is
*not* a good answer is a bare "it failed", because three failures point at
three different decisions — see "Reading the report" below.

## What is here

| file | runs where | does |
|---|---|---|
| `export_owlv2_onnx.py` | laptop | OWLv2 → ONNX, image side only, and **verifies the split reassembles into the real model's own numbers** |
| `owlv2_host_head.py` | laptop, and the Pi later | the arithmetic that stays on the CPU: the text einsum, and (for the `minimal` export) three ops moved off the accelerator |
| `calibration_set.py` | laptop | 128 frames out of `recordings/`, stratified by walk and balanced on `labels.json` |
| `compile_owlv2.py` | EC2 | walks the export matrix through translate → optimize → compile and writes a report |
| `ec2.sh` | laptop | launch, drive, **tear down** the compile host |
| `setup_host.sh` | EC2 | installs the DFC and its dependencies |

`tests/test_hailo_compile_loop.py` covers the head arithmetic and the
sweep's bookkeeping against fakes. Nothing in the test suite imports torch,
transformers or `hailo_sdk_client` — the same rule `tests/test_perceive.py`
follows.

## The split, and why it is that one

OWLv2 is a ViT-B/16 image tower, a CLIP-style text tower, and three small MLP
heads. **Only the image side goes to the accelerator.** The text tower stays
on the Pi's CPU, exactly as CLIP's already does (4.2) — not for convenience
but because it runs once per *target string*, not once per frame.

The seam is inside `Owlv2ClassPredictionHead`, one op before the text mixes
in:

```
image_class_embeds = dense0(image_feats)            # image only  -> Hailo
pred_logits = image_class_embeds @ query_embeds.T   # the only text op -> CPU
pred_logits = (pred_logits + logit_shift) * logit_scale
```

So the accelerator returns five per-patch tensors — `image_class_embeds`,
`logit_shift`, `logit_scale`, `pred_boxes`, `objectness_logits` — and the Pi
does a `[3600, 512] × [512, Q]` matmul in microseconds.

**This is verified, not asserted.** `export_owlv2_onnx.py --verify-only`
reassembles the split through `owlv2_host_head.join()` and compares it to an
unmodified `Owlv2ForObjectDetection` on a real recorded frame. A HEF of a
subtly wrong graph would compile perfectly and be worthless, and that would
not surface until the part was bought.

Measured on `woven-laundry-basket-20260908-215252/frame-0005.jpg`, all four
export variants:

```
max |d score|      = 1.552e-05   (sigmoid)
max |d pred_boxes| = 2.146e-04
max |d objectness| = 2.728e-05   (sigmoid)
top-1 patch agrees = True;  top-50 overlap = 50/50
```

The tolerance is on **scores, not logits**, deliberately. OWLv2's logits span
about −39…0 here, so an absolute tolerance on them is a tolerance on a number
nothing downstream reads — a 4e-4 relative wobble from fp32 reduction order in
a 12-layer ViT read as a failure while changing no decision. What the pipeline
consumes is `sigmoid(logit)` and the box.

## Three axes, because one "it failed" is not an answer

`export_owlv2_onnx.py` writes a matrix and `compile_owlv2.py` walks it.

- **opset 17 vs 14.** At ≥17 torch emits `LayerNormalization` as one op; below
  it the same maths decomposes into ReduceMean/Sub/Pow/Div. A parser that
  rejects the fused op may take the decomposition.
- **`full` vs `minimal` head.** `full` keeps the L2 normalise (`ReduceL2`),
  the `Elu` on `logit_scale` and the box `Sigmoid` on the accelerator.
  `minimal` moves all three to the CPU, where they cost microseconds on
  `[3600, 512]` and `[3600, 1]` tensors. Those are the three ops in this
  graph least likely to exist in a CNN toolchain, and the test suite pins
  that the two heads produce the same numbers — so the fallback is the same
  model, not a similar one.
- **`normalized` vs `uint8` calibration.** `uint8` adds a `normalization()`
  layer in the model script so the part takes raw camera bytes and does the
  mean/std itself, which is the arrangement worth having on the robot.
- **`--image-size`.** OWLv2-base-patch16 is native at 960 = 60×60 = **3600
  tokens**, against ~196 for the ImageNet ViTs in Hailo's zoo. Attention is
  quadratic in that, so it is the most likely thing to exhaust the part.
  640 gives 1600 tokens. **Any non-native size changes accuracy and must be
  re-scored with `control/perception_eval.py` before it counts** — P5's own
  tiling sweep found the peak at native resolution and worse either side.

For reference, what the exporter actually emits at 960 (opset 17): 575 nodes,
23 distinct ops — 105 `MatMul`, 27 `LayerNormalization`, 12 `Softmax`, 4
`Erf`, and exactly one `Conv` (the patch embedding). The `minimal` head drops
the `ReduceL2` and the `Elu`.

## Running it

```bash
# 1. laptop -- export and verify (needs requirements-perception.txt + onnx)
python -m tools.hailo.export_owlv2_onnx --out build/owlv2 \
    --image recordings/woven-laundry-basket-20260908-215252/frame-0005.jpg \
    --text "a woven laundry basket"

# 2. laptop -- calibration set out of the corpus
python -m tools.hailo.calibration_set --out build/owlv2 --n 128

# 3. put the Dataflow Compiler wheel where the host can reach it.
#    It is a Developer Zone download, gated behind a login, and NOT on PyPI.
aws s3 cp hailo_dataflow_compiler-*.whl \
    s3://vision-picar-deploy-303351622021-us-east-2/hailo/dfc/

# 4. the rented box
tools/hailo/ec2.sh up          # r6i.4xlarge, us-east-2, ~$1.01/hr
tools/hailo/ec2.sh push        # exports + calibration + this directory
tools/hailo/ec2.sh setup       # installs the DFC
tools/hailo/ec2.sh compile     # the sweep, under nohup
tools/hailo/ec2.sh run 'tail -40 /opt/hailo/compile.log'
tools/hailo/ec2.sh pull        # report, HEFs, per-stage logs
tools/hailo/ec2.sh down        # TERMINATE. Do this.
```

`up` is idempotent and `status` prints the running cost. `down` terminates
the instance, deletes the security group and the IAM role, and leaves only
the S3 prefix — which holds the wheel and the report, and is named in the
output so it can be removed too.

**Access is SSM Session Manager, so there is no key pair and the security
group authorises no inbound rules at all.** The instance profile can read one
S3 prefix, not the account.

## Reading the report

`build/owlv2/result/compile_report.json`, one record per variant, each with
three stages that fail independently:

| stage | what a failure means |
|---|---|
| `translate` | **op coverage.** The headline names the op it stopped on. If it is `ReduceL2` or `Elu`, try the `minimal` head — that is a fix, not a wall. If it is `Softmax`, `MatMul` or the attention block, it is a wall. |
| `optimize` | **numerics or host memory.** A `MemoryError` at 3600 tokens is the predicted outcome and is recorded as a result, not a crash. Try `--image-size 640`, and re-score the accuracy. |
| `compile` | **resource allocation on the part.** The graph is understood and does not fit in 13 TOPS / the 8L's memory. This is the cleanest possible "buy a Jetson". |

And then, per P5:

- **Compiles** → Pi 5 + Hailo-8L, decisively, and the next job is wiring
  OWLv2 in as the search-proposal tier beside YOLO. `owlv2_host_head.py` is
  already the Pi-side half of that.
- **Does not** → a Jetson, sized for a 150M ViT rather than a 4B VLM, which
  is a different board and a different power budget than 4.8 costed.

A HEF is **not** an accuracy result. Post-training quantization to INT8
changes scores, and P5's method rules apply unchanged: score on grounding,
read recall only at a matched false-positive budget, and check `separable`.
Re-scoring a compiled OWLv2 needs real silicon, so it is a hardware-day item.

## What is deliberately not here

**No `hailort` and no HEF execution.** Running a HEF needs the part; this
answers whether one can be built. `setup_host.sh` installs the compiler only.

**No accuracy re-run.** Nine configurations over seven walks are already
scored and committed under `evaluations/`. Nothing here re-measures them.
