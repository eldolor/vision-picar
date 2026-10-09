---
kind: engineering
domain: perception
status: current
verified: 2026-10-08
parent: docs/perception/ARCHITECTURE.md
---

# Perception -- engineering

How the on-board perception tier is built and measured today. Why it is a
detector plus a verifier with a three-state answer, why the sim never runs a
detector, and why the board is a Jetson are in the
[architecture spec](../../perception/ARCHITECTURE.md). This file is true only
until the code changes and is updated in the same commit.
`PLAN-onboard-perception.md` (4.10, 4.11, P3-P26) is the dated record behind
every number here.

## Implementation

| File | What it does |
|---|---|
| `brain/perceive.py` | The shipped tier. Tri-state constants; `Box`, `Detection`, `Candidate` (`margin`, `probability`), `Perception` (`as_dict()`); the `Detector`, `CropScorer`, `RegionProposer`, `TextScorer` protocols; `PerceptionPipeline`; `FrameReportedPipeline` (the sim); the concrete backends `YoloE`, `YoloDetector`, `ClipScorer`, `SegformerFloorProposer`, `OpenVocabCropSource`; `pipeline_for()`, `detector_for()`, `coco_class_for()`, `label_affinity()`, `Vocabulary`. |
| `brain/perceive_lab.py` | Candidates behind the same protocols: `GroundingDino`, `Owlv2`, `OmDetTurbo`, `LlmDet`, `YoloWorld`, `VlmDetector`, `SamProposer`, `OpenVocabPipeline`, a TensorRT OWLv2 loaded lazily from `tools/trt/trt_owlv2.py`; `pipeline_for_spec()` builds any of them from strings. Never imported by `brain/perceive.py`. |
| `brain/tiered.py` | Calls `pipeline.perceive(frame)` once per frame; computes 1.11a's corroboration verdict (`corroboration_for()`) and publishes the `Vocabulary` verdict. Policy-side details are in [policy](../policy/ENGINEERING.md). |
| `control/brain_server.py` | Builds the pipeline at mission start. The sequence is described once, in [mission](../mission/ENGINEERING.md) ("How a tiered mission is built"). `_perception_available()` answers health with `find_spec` for `ultralytics`, `open_clip`, `torch`. |
| `brain/inventory.py` | 3.46's object inventory: `frame_detections()` (a sim frame's per-cell detections grouped into one per object), `scan_returns()` / `range_at()` (the scan moved to the body centre; nearest return within 2 deg), `Landmark`, `Inventory` (`observe()`, `report()`, `counts()`, `summary()`). A merge inside one frame counts that frame's look once, in either merge order (`_count_look()`: each landmark taking part is due its score plus one look if this is a new view for it and it was not already counted; the merged score is the largest due, and a frame leaves one hit and one vote however its landmarks merge within that frame; landmarks counted in one frame and joined in a LATER one still sum their hits and votes, 3.46's rule, open in 3.48) -- never twice, never lost, and not again on the next frame from the same spot -- and a landmark joins the list only with its first point (3.47). Imports nothing but `robot.safety.LIDAR_X_M` |
| `control/inventory_store.py` | `InventoryStore.save(mission_id, report)`: a local JSON file (written to a temporary name, fsynced, renamed, then the directory fsynced, so a cut-off save leaves the last good file; a crash's leftover `.tmp` is swept by the next save once an hour old, never this process's saves in progress; 3.47), plus `put_object` to S3 (SSE AES256) when a bucket is set; failures returned and logged, never raised. `inventory_store_from_config()` |
| `control/mission_runner.py` (inventory) | `_HaltGate` calls `MissionRunner._observe(frame)` inside every `get_camera_frame()`; `_observe` reads `world.get_pose()` and `robot.get_scan(max_range_m=4.5)` only when `detections_fn(frame)` is not None, then updates the inventory under `MissionRunner._lock` (the lock every reader takes) and drops a frame that lands after the mission ended, before its reads (`_ended()`, the rule `_finish` uses; 3.47). `_finish()` logs `summary()`, which may not raise on the way to the stop, and calls `inventory_sink(report)` |
| `control/perception_eval.py` | P3's corpus scorer: `score` and `compare` subcommands. Reads each walk's `labels.json`; refuses a walk without one. |
| `control/target_probe.py` | Pre-flight for a target string: firing rate and peak probability over a fixed random sample of existing frames. |
| `tools/contact_sheet.py`, `tools/label_prepass.py` | Labelling aids for a walk's `labels.json`: contact sheets to adjudicate by eye, and a two-opinion pre-pass (OWLv2 against the shipped tier) that bounds what needs eyeballing. One-off evaluation helpers; the recordings domain owns `labels.json` itself |
| `tools/steer_check.py`, `tools/yoloworld_crops.py` | Historical evaluation scripts: does the tier act on what it sees (Phase G, against recorded frames), and YOLO-World as a crop source for CLIP. Kept as the record of the measurements they made; not on any shipped path |
| `tools/jetson/bench_perception.py` | Per-frame time split into detector pre/infer/post, CLIP GPU and CLIP CPU, over `tools/jetson/bench_frames.json` (60 pinned frames + 3 warm-up). |
| `tools/gpu/` | **Historical.** The rented-GPU runs of P7 (whole corpus on an A10G, 2026-09-11/12). They are not on any current path. Read `PLAN-onboard-perception.md` P7 before re-using them. |
| `tools/hailo/` | **Historical, closed.** The Hailo compile loop (P6, P14-P16), which showed OWLv2 does not allocate on a Hailo-8L. The Hailo path is closed (architecture spec), so do not fund a re-run. `tools/hailo/README.md` explains how it works. |
| `requirements-perception.txt` | The optional install: `ultralytics`, `open_clip_torch`, `torch`, `torchvision`, `transformers`, `scipy`. |

**`PerceptionPipeline.perceive(frame)`, in order:**

1. Read `pan_deg`, `tilt_deg` from the frame; base64-decode `image_base64`
   (failure -> `unavailable`).
2. `detector.detect(image, confidence)`; plus `proposer.propose(image)` boxes
   if a proposer is set. Any exception -> `unavailable`.
3. Crop rule (`_crops`): `label_gate` keeps only boxes labelled with the
   target's COCO word; `soft_gate` keeps labels in the CLIP-text accept set;
   `low_confidence` keeps all. Sort by area, keep `max_crops`. None left ->
   `absent`.
4. Score each crop with `scorer.score(image, box, (target,) + distractors)`;
   a wrong-length answer or an exception -> `unavailable`.
5. Gate: best candidate by `probability` (softmax of scores x 100); `>=
   match_probability` -> `detected`, else `absent` with candidates kept.
   If `match_margin` is set, gate on `margin` instead.
6. Bearing: `(box.centre_x / width - 0.5) * hfov_deg + pan`, width from
   `frame["image_width"]` or read from the JPEG header; no tilt correction.

With the shipped YOLOE, `detector_for()` wraps `YoloE` in
`OpenVocabCropSource`, which sets the target as YOLOE's text prompt and
returns its boxes labelled with the target string; YOLOE's own threshold
(0.05) applies and the pipeline's `confidence` is not applied again.

## Interfaces

**`Perception.as_dict()`** (published as the scene's `_perception` and on
`GET /mission/status` as `perception`): `status`, `crop_source`, `reason`,
`synthesised`, `pan_deg`, `tilt_deg`, `bearing_deg` (degrees, negative left,
null when unmeasurable), `match_margin`, `match_probability`, `similarity`,
`label`, `candidates`.

**Protocols:**
`Detector.detect(image: bytes, confidence: float) -> Sequence[Detection]`;
`CropScorer.score(image: bytes, box: Box, texts: Sequence[str]) -> Sequence[float]`;
`RegionProposer.propose(image: bytes) -> Sequence[Box]`;
`TextScorer.text_similarity(anchor: str, texts) -> Sequence[float]` (optional).

**Factories:**
`pipeline_for(target, *, weights="yoloe-11s-seg.pt", clip_model="RN50", device=None, floor_mask=False, segmenter=DEFAULT_SEGMENTER, **kwargs)`
raises `PerceptionUnavailable` naming the pip command when a library is
missing. `PerceptionPipeline(detector, scorer, target, *, hfov_deg=66.0, match_margin=None, match_probability=0.8, distractors=DEFAULT_DISTRACTORS, max_crops=None, crop_path="low_confidence", proposer=None, affinity_k=8, confidence=None)`.

**Sim frames.** `FrameReportedPipeline` reads `frame["detections"]`, a list
of `{label, bearing_deg, distance_m}` written by `sim/grid_world.py`
(bearing of the visible part via `renderer.visible_bearing`). No key ->
`unavailable`; no label containing the target -> `absent`; else `detected`,
nearest first, `synthesised: true`, models named `sim ground truth`.
`distance_m` is used only to order matches; arrival never reads it. Sim
frames carry `pan_deg` since 3.46 (the inventory composes it), but the
`Perception` this builds leaves `pan_deg` at 0, while the bearings are camera-relative (cast along
`GridWorld.view_angle()`). Arrival's panned-camera refusal therefore never
fires in the sim (policy ENG, Known gaps).

**Inventory** (3.46). `GET /mission/inventory` on the brain returns
`{"inventory": null}` before any mission, else
`{"inventory": {"reported": [...], "candidates": [...], "counts": {...},
"params": {...}, "map_id", "mission", "outcome"}}`. A landmark is
`{id, label, x_m, y_m, belief, reported, disputed, hits, misses, votes,
room, first_step, last_step, points}` in the map frame of
`WorldInterface.get_pose()` (x east, y south). `GET /mission/status` carries
only `inventory: {frames, observations, unplaced, no_pose, landmarks,
reported}`. `MissionRunner(..., inventory=True, detections_fn=None)`:
`detections_fn(frame) -> [{label, bearing_deg}] | None`, bearings off the
camera axis; None means the detector did not run on that frame, so no
miss is inferred from it.

**Health fields** (`GET /health` on the brain): `perception_available`,
`perception_detector`, `perception_clip_model`,
`perception_match_probability`, `perception_match_margin`,
`perception_crop_path`, `perception_floor_mask`, `tier_corroboration_bar`.

**Corroboration** (`_tier.corroboration`): `verdict` in `corroborated`,
`unclear`, `no_claim`, `unavailable`; `claimed`, `bar`, `local_probability`
(max over all candidates, not the gated best), `enforced: false`. Counted in
`_tier.stats` as `claims`, `corroborated`, `verdicts`.

## Parameters and configuration

**The detector choice.** The text-prompted detector is YOLOE (Ultralytics),
promoted in P22. P23 pinned the small `yoloe-11s-seg.pt` on all 1234 frames
of the then-labelled corpus. On the 365-frame subsample P22 had used, `26l`
led (91% vs 90% at 3 FP). On all 1234 frames the order inverted: `11s` 82%,
`v8l` 74%, `26l` 72%, `26x` 69%. `11s` also had the fewest false positives
at the 0.8 gate (2) and the lowest latency (139 ms vs 296 ms for `26l`).
The verifier is OpenAI CLIP RN50 via `open_clip`.

**Config keys** (`brain:` block of `config/robot.yaml`; empty or 0 means the
module default):

| Key | Shipped | Effective | Why |
|---|---|---|---|
| `perception_detector` | `""` | `yoloe-11s-seg.pt` | P22 promoted YOLOE; P23 on all 1234 labelled frames: 82% recall at 3 false positives vs 72% for `26l`, 139 ms vs 296 ms on laptop CPU |
| `perception_clip_model` | `""` | `RN50` (OpenAI weights) | 4.9: 7-10 ms a crop vs 25-50 for ViT-B/32, ~25M vs ~88M parameters |
| `perception_match_probability` | 0.0 | 0.8 | for the shipped detector, P23's calibration on all 1234 frames: `yoloe-11s-seg` reads 80% at 0.8 against its own best of 82%, with 2 false positives (`PLAN-onboard-perception.md` P23, "the gate problem dissolves"). The value was first set on 2026-09-07's two rig walks with an earlier pipeline (18/18 bottle, 25/31 backpack); those numbers do not describe today's detector |
| `perception_match_margin` | 0.0 | unused (probability gate) | override for sweeping the raw margin |
| `perception_crop_path` | `""` | `low_confidence` | bottle walk: label gate 7/18, open vocabulary 18/18, both with 0 false positives; backpack 9/31 vs 25/31 |
| `perception_floor_mask` | false | off | P24: with YOLOE the mask lost on 9 of 11 checkpoints, cost 15 points at a 3-FP budget, and took 836 ms against 139 |
| `tier_corroboration_bar` | 0.5 | 0.5 | **Canonical home of this key** (policy ENG points here). Passed to `TieredVision`; reported only, changes no decision. 1.11a, 209-frame search walk: at 0.8 kept 2 of 10 true sightings, at 0.5 kept 8 and rejected 34 of 34 storage-bin claims |

There is no device key: `ClipScorer` picks `cuda`, then `mps`, then `cpu`;
YOLOE takes Ultralytics' default unless a `device` is passed (the bench's
`--device`). On a Mac Ultralytics does not pick the GPU by itself.

**Constants in `brain/perceive.py`:**

| Constant | Value | Why |
|---|---|---|
| `DEFAULT_MAX_CROPS` | 16 | P23: 258 vs 251 true positives against 4, same 2 false positives, 139 vs 127 ms |
| `DEFAULT_MAX_CROPS_WITH_PROPOSER` | 32 | provisional, 2x the single-source budget; a second source must not crowd out the first |
| `LOW_CONFIDENCE` | 0.05 | open-vocabulary path and YOLOE's own threshold |
| `DEFAULT_CONFIDENCE` | 0.25 | label-gate path only |
| `DEFAULT_MATCH_MARGIN` | 0.05 | legacy margin gate; detected none of the bottle walk |
| `CLIP_LOGIT_SCALE` | 100.0 | CLIP's trained constant |
| `DEFAULT_DISTRACTORS` | 6 phrases ("a wall" ... "a household object") | bland, absorb "some object, not that one" |
| `DEFAULT_AFFINITY_K` | 8 | soft gate's accept-set size; an interior point of the 1..80 axis |
| `hfov_deg` | 66.0 | Camera Module 3's field of view (see Known gaps) |
| `DEFAULT_SEGMENTER` | `nvidia/segformer-b0-finetuned-ade-512-512` | floor proposer, off |
| `DEFAULT_YOLO_DETECTOR` | `yolo11s.pt` | the comparison baseline; label/soft gates only mean something for it |

**Instrument constants:** `control/perception_eval.py` `DEFAULT_GATES`
(0.90 ... 0.30), `--gate` 0.8, `MIN_USABLE_MARGIN` 1e-4;
`control/target_probe.py` `DEFAULT_SAMPLE` 200, `SEED` 7, gate 0.8, reject
above a 10% firing rate or a peak below 0.01 (labelled "inert", but see
Known gaps for what that peak actually measures);
`tools/jetson/bench_perception.py` `BUDGET_MS` 250, `DEFAULT_N` 60,
`WARMUP` 3.

**Inventory constants** (`brain/inventory.py`, all `[PLACEHOLDER]`s left
at their first values because 3.46's tuning half met every bar with
them):

| Name | Value | Meaning |
|---|---|---|
| `L_HIT`, `L_MISS` | +0.85, -0.4 | log-odds per independent hit / miss; one hit reads 0.70. An isolated miss is held pending and forgiven unless the next independent look also misses (3.46 amendment 2) |
| `REPORT_BELIEF` | 0.8 | reported vs candidate: two hits and no miss |
| `NEW_VIEW_M`, `NEW_VIEW_DEG` | 0.5 m, 30 deg | what makes a look independent; time alone never does |
| `MERGE_M`, `RELABEL_M` | 0.5 m, 0.3 m | single linkage within a label; a different label this close is a vote |
| `RANGE_GATE_DEG`, `MAX_RANGE_M` | 2 deg, 4.0 m | lidar return taken for a bearing; beyond it, unplaced |
| `MISS_RANGE_M`, `OCCLUSION_M` | 3.0 m, 0.3 m | a miss needs the place in view, near, and not behind something |
| `GROUP_DEG` | 8 deg | same-label sim cells this close in bearing are one detection |

Config keys (`brain:` block): `inventory_dir` (`recordings/inventory`),
`inventory_bucket` (`""`; env `INVENTORY_BUCKET`, which
`service/tunnel/run.sh` reads from the recordings stack's export unless
it is set, and leaves it unset if the lookup fails;
`INVENTORY_BUCKET=` (empty) means local only and overrides a bucket in
the yaml, 3.47),
`inventory_prefix` (`inventory`). Objects land at
`s3://<bucket>/<prefix>/<hostname>/<UTC time to the microsecond>-<target>.json`
(to the second before 3.47, when two quick missions shared a key).

## Procedures

**Install and warm the models** (the canonical home of the warm-up command;
other specs link here. Once per machine; the first run downloads
~600 MB: `yoloe-11s-seg.pt` 28 MB, YOLOE's text encoder
`mobileclip_blt.ts` 572 MB, and CLIP's weights):

```bash
pip install -r requirements-perception.txt
python -c 'from brain.perceive import pipeline_for; pipeline_for("x")'
```

Without the warm-up, the first tiered `POST /mission/start` downloads inside
the request and sits on "Starting..." for tens of seconds. Without the
install, health reports `perception_available: false` and the start is a
400. The detector is built first, so with the shipped YOLOE and no
`ultralytics` the detail is:

```text
The tiered policy cannot start: YOLOE needs ultralytics. `pip install -r requirements-perception.txt`.
```

With `ultralytics` present but no `open_clip`/`torch` it reads
`... open_clip_torch / torch are not installed. ...`. The older text
`ultralytics is not installed` appears only with a closed-vocabulary
`perception_detector` such as `yolo11s.pt`. Search the logs for
`The tiered policy cannot start`. A simulated robot never reaches this,
because it loads no model.

**Score the corpus** (free, local, minutes on CPU):

```bash
python -m control.perception_eval score --save shipped.json
python -m control.perception_eval score --walk <walk-name> --quiet
python -m control.perception_eval compare shipped.json other.json --fp-budget 0,3,16
```

`score` opens with these lines (the counts depend on the corpus on disk):

```text
corpus: <N> walk(s), <F> labelled frames, <V> visible   (recordings)
config: <detector label>, crop path low_confidence, metric <metric>
reference: labels.json (adjudicated) -- never walk.jsonl

  <walk-name>  target '<target>'  <frames> frames, <visible> visible
```

After them come the vocabulary verdict per walk, a per-walk table, a gate
table (TP, FP and both rates per gate), the `unavailable` count, and
milliseconds per frame on this machine. A walk without `labels.json` is
skipped by the corpus loader and refused when named.

**Pin the frame set before comparing.** The corpus grows, and rows over
different frames are not comparable. The supported way is to name the walks
with a repeated `--walk`, and quote the `corpus:` line's frame count beside
the result. P23/P24's 1234 frames are the 11 walks listed in that run's
records (`PLAN-onboard-perception.md` P23 note). `--frames-from FILE` is
**the replay detector's filter**. It is used with `--detector
replay:<same file>`, so that frames a recorded run never saw are not scored
as misses. It keeps the frames whose `<walk>/<frame>` key is among the JSON
file's keys or list items. A plain JSON list of keys therefore also works as
a pin, but that is not its purpose. Never compare a GPU row with a CPU row
(P24). Try a lab model with `--detector owlv2` or
`--detector crops:<backend>`.

**Pre-flight a target string** before a rig walk:

```bash
python -m control.target_probe "a red toolbox" "a white phone charger cable"
```

Read two numbers: a firing rate over 10% rejects the string. The peak is
read from `r.best`, which the pipeline sets only on a `detected` frame
(P >= 0.8), so a peak of 0.000 means "never passed 0.8 on any sampled frame",
not "the detector never grounds the prompt", and `--gate` below 0.8 changes
nothing (see Known gaps). A verdict of `REJECT -- gated to COCO ...` means a
colour word or noun put the target back into COCO's vocabulary.

**Look at one walk without labels:** `python -m tests.manual_perceive_walk
<dir> "red backpack"`.

**Time the tier** (the 3.33 risk-2 measurement; procedure in
`tools/jetson/README.md`):

```bash
python -m tools.jetson.bench_perception --recordings recordings --out bench-15w.json
python -m tools.jetson.bench_perception --recordings recordings --device mps   # a Mac's GPU
```

It prints devices, how many frames had crops, median and p90 of each stage,
and `budget 250 ms: WITHIN` or `OVER`. Recorded on the M1 MacBook Air,
2026-10-02, 60 pinned frames: detector on CPU and CLIP on MPS, median 119 ms
(p90 156), detector inference 108 ms, handling 6 ms, CLIP 31 ms a crop, 16 of
60 frames with a crop; both on MPS, median 36 ms (p90 66), detector 22 ms.
The Jetson numbers at 15 W and 25 W are not yet taken.

**Measuring the inventory** (3.46): `python -m tests.inventory_sweep
--seed 3462` (sweep 3, the current rule's judged set) runs 20 frontier missions in `complex_house` with injected
detector errors and prints recall, placement, duplicates and precision;
`--half 1|2`, `--clean`, `--steps`.

## Verification

| Test file (count 2026-10-02) | What it pins |
|---|---|
| `tests/test_perceive.py` (57) | tri-state (an empty good frame is `absent`; missing, undecodable, wedged detector or scorer is `unavailable`); open vocabulary is the default and `auto` still means 4.2's rule; probability over every score; margin override; crop cap and its growth with a second source; bearing from centre, pan added, width read from the image; soft gate and vocabulary verdict; real backends say what to install |
| `tests/test_perceive_lab.py` (22) | lab backends and `pipeline_for_spec()` on fakes |
| `tests/test_perception_eval.py` (47) | the four cells, inclusive gate, `unavailable` never scored, recall at an FP budget admits exactly the budget, a walk without labels cannot be scored |
| `tests/test_target_probe.py` (2) | handoff 4h, fake pipeline: the probe's peak and its hits at `--gate` are taken over every candidate, so a prompt grounding below 0.8 is not called inert and a gate below 0.8 counts hits |
| `tests/test_tiered.py` (92) | corroboration verdicts, a wedged camera is not a failure to corroborate, landed async verdicts shown once |
| `tests/test_brain_server.py` | missing extras or bad weights refuse at start, health reports availability and the effective gate, a simulated robot loads no model, a real camera is never taken for the sim (the start-time probe frame) |
| `tests/test_inventory.py` | placement and the compass convention, the lidar offset, one pose is one look (red on a per-frame count), misses need view + range + no occlusion, a skipped frame says nothing, a wrong label is a vote, single linkage and its bridge (red without it), a merge inside one frame counts its look once (3.47, red without it), another map's pose refused |
| `tests/test_inventory_mission.py` | criterion 1 (identical actions with it on and off, frontier and tiered, three houses), no agent holds it, criteria 3-5 on four pinned sweep missions, the capture-time pose, no reads for a frame without detections, no inventory failure fails a mission, the sink, the store (local + S3 + failure), the route; 3.47, each red without its fix: inventory writes under the runner lock, no frame after the end, a failing summary still stops the robot, a cut-off save keeps the last good file, `INVENTORY_BUCKET=` is local only, two missions in one second get two files |
| `tests/test_bearing_turns.py`, `tests/test_arrival.py` | the consumers, driven by `FrameReportedPipeline` (1.12) |

No automated test loads a model; the concrete backends' model-touching
methods are excluded from coverage. **P3's only validation** was reproducing
4.11's published row exactly on first run (62/74 at P >= 0.8, 3 false
positives, on the four walks then in `recordings/`).

**Checklist for a model change:** score the whole labelled corpus with the
frame set pinned; report recall at matched false-positive budgets (0, 3,
16), never alone; re-check the gate for the new detector (scores move with
the model); time it with `tools/jetson/bench_perception.py` on the same
pinned frames; change `DEFAULT_DETECTOR` and the config comment in the same
commit.

## Known gaps

- **The inventory fills only in the simulator.** 3.46 C (a second,
  prompt-free YOLOE pass on keyframes) is not built, so a real frame has
  no detections and `frame_detections()` returns None. Its criteria 2
  (Jetson budget) and 6 (labelled rig keyframes) are unmeasured.
- **Inventory precision costs of the forgiven miss.** 3.46 sweep 3
  (fresh missions) met every bar (recall 96.6%, precision 97.2%); on
  sweep 1's missions the same rule reads precision 91.7%: a 1:1 vote
  reported under the wrong label, and one object placed on another's
  surface.
- **Loop closures are unmeasured for the inventory.** In process the pose
  is the truth; an observation is placed in the pose at capture and is
  not re-placed when SLAM corrects.

- **Jetson numbers exist only for the shipped pipeline** (3.33, 2026-10-04,
  60 pinned frames): at 15 W median 60.6 ms, p90 109.9 ms a frame (GPU 44.4,
  CPU handling 18.6); at 25 W 64.2 / 120.4 ms. Torch-on-CUDA parity with
  the laptop: status 63/63, CLIP probability within 0.0002. P26 (decode
  once, batch crops, resize on GPU) is specified and not built, and not
  needed for the budget; `ClipScorer.score()` still decodes the whole JPEG
  again for every crop. **TensorRT was measured and not adopted**
  (`PLAN-ros-alignment.md` 3.41, `tools/jetson/bench_trt.py`):
  * fp16 engines move CLIP probabilities by up to 0.052;
  * a GPU resize flips one verdict;
  * p90 falls only 20-25%.
- **`hfov_deg` is 66.0, the Camera Module 3's field of view.** The Rover
  carries an OAK-D Lite; its field of view is not measured or configured, so
  every real-pixel bearing on the car will be scaled wrong until it is. Tilt
  correction is absent (no intrinsics).
- **INT8 is untested on the Jetson.**
- **1.11a is unenforced** and `oov_cold_search_after` has no config key.
  Enforcing it at arrival was rejected on 2026-10-02 in favour of a cloud
  identity check (policy).
- **A local false positive still steers.** For steering, the only bound is
  the per-frame `match_probability` (0.8). It can no longer end a mission
  `found`: since 2026-10-02 arrival also needs the cloud's yes on the
  arrival frame (`TieredVision.confirm_arrival()`; mechanism in the
  [policy engineering spec](../policy/ENGINEERING.md), "Arrival").
- **Probe peaks recorded before 2026-10-03 are peaks over DETECTED frames
  only.** Until handoff 4h `control/target_probe.py` read the peak from
  `r.best`, set only at P >= 0.8, so an old "0.000 -- inert" verdict means
  "never passed 0.8", not "never grounded". It now takes the peak, and
  counts hits at `--gate`, over every candidate's probability
  (`tests/test_target_probe.py`). Re-probe a string before trusting an old
  inert verdict.
- **`YoloDetector`'s default weights are the YOLOE checkpoint.** Its
  signature is `weights: str = DEFAULT_DETECTOR`, which is now
  `yoloe-11s-seg.pt`, loaded through Ultralytics' `YOLO` class.
  `detector_for()` always passes weights explicitly, so nothing hits this
  today. A bare `YoloDetector()` would load the wrong model, and its intended
  default is `DEFAULT_YOLO_DETECTOR` (`yolo11s.pt`). This is latent.
- **Stale prose:** `config/robot.yaml` still says a 0 margin means "the
  module default 0.05" (0 means the probability gate); `requirements-perception.txt` and
  `brain/perceive_lab.py`'s docstring still describe YOLO11s and the floor
  mask as the shipped tier. `YoloE`'s class docstring in
  `brain/perceive.py` (around lines 1316-1333) gives P22's numbers for the
  `26l` checkpoint (91% vs 90%, a 3-FP gate at 0.391, "74% at the old
  P>=0.8 ... 91% at a gate near 0.4") as if they described the shipped
  `11s`, which P23 found calibrated at 0.8.
