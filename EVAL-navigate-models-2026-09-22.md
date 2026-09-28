# Which vision model should drive the robot?

**A controlled comparison of four cloud vision-language models on 452 real
robot-camera frames.**

| | |
|---|---|
| **Date** | 2026-09-22 |
| **Models** | Claude Opus 4.5, Claude Opus 5, Claude Fable 5.1, OpenAI GPT-6 Astra |
| **Model ids** | `us.anthropic.claude-opus-4-5-20251101-v1:0`, `us.anthropic.claude-opus-5`, `us.anthropic.claude-fable-5-1`, `us.openai.gpt-6-astra` (Bedrock, `service/vision_analyze/vision_core.py`'s `/navigate` allow-list) |
| **Data** | 452 frames across 6 recorded walks, floor-level camera, one hotel room |
| **Method** | Frame-by-frame replay through the deployed service — pixels fixed, model varied |
| **Calls** | 1,808 (452 × 4), zero errors |
| **Cost** | $31.93 |

---

## Summary

- **Fable 5.1 was perfect on recall** — it found the target in all 212 frames
  where the target was present. It is also the most expensive, at 2.3× the
  current default.
- **Opus 5 never produced a false positive** — the only model of the four
  that never claimed to see a target that wasn't there. Same latency as the
  current default, ~22% more expensive.
- **GPT-6 Astra trades precision for recall** — 99% recall, but it accounts
  for 5 of the 7 false positives in the entire run, and it is the slowest at
  the 90th percentile.
- **The current default, Opus 4.5, is last on both accuracy axes**, though it
  is the cheapest and the fastest.
- **No default change is recommended on this evidence alone.** One room is
  not enough, and the single biggest effect we found was not the model at all
  — it was how the target was described.

---

## 1. Background

### The system

`vision-picar` is an indoor autonomous robot project. A camera frame goes to
a cloud vision-language model, which answers a fixed set of questions about
the scene — *is the target visible? which way? is the path clear? what should
the robot do next?* — and returns a single action (`FORWARD`, `LEFT`,
`RIGHT`, `STOP`). That endpoint is called `/navigate`, and the model behind it
is configurable.

A **walk** is a recording: a person pushes a phone on a low wheeled rig
through a room looking for a named object, and every frame plus the model's
answer is saved. Walks are the project's test data.

### Why this evaluation

Three new models became available on Amazon Bedrock for this account (Claude
Fable 5.1, Claude Opus 5, OpenAI GPT-6 Astra). They were added to the model
picker. The question was whether any of them should replace **Claude Opus
4.5**, which has been the `/navigate` default since 2026-08-29.

---

## 2. What was evaluated

Each model answered the same question about the same 452 images:

> Is the target object visible in this frame?

The four walks with the target present supply **recall** (how many visible
targets were found). Two walks where the target is genuinely absent supply
**precision** (how often a model claims to see something that isn't there).

| Walk | Frames | Target present |
|---|---:|---:|
| a-white-phone-charger-…203250 | 84 | 41 |
| a-white-phone-charger-…203355 | 92 | 77 |
| grey-backpack-…203748 | 89 | 71 |
| grey-backpack-opus-4-5-…210743 | 30 | 23 |
| **grey-backpack-…203627** | 92 | **0** |
| **red-backpack-…203152** | 65 | **0** |
| **Total** | **452** | **212** |

---

## 3. Methodology

### Replay, not fresh walks

Comparing two live walks by two models is not a controlled experiment. A walk
is a physical path, and the path is itself steered by whichever model is
answering — so the difference between two walks mixes model quality with
where the operator happened to point the camera.

**Replay holds the pixels fixed and varies exactly one thing.** Every model
saw byte-identical images, the same prompt wording (`default`), and the same
target string. Models were run sequentially, six frames at a time, against
the deployed production service rather than the API directly — so the
measurement includes the real prompt, the real JSON parsing, and the real
model resolution.

### Ground truth, and its limits

Each walk carries a `labels.json` recording whether the target is visible in
each frame. **These labels are mostly machine-proposed** (by OWLv2, an
open-vocabulary object detector) with only a handful of frames reviewed by a
person. They are a strong prior, not truth.

They have one known, systematic flaw: **at very close range the detector
loses the target.** In `grey-backpack-…203748`, frames 0056 and 0077 show
the backpack filling the entire frame and are labelled *not visible*. A model
that correctly says "visible" there would be scored as wrong.

### So the scoring keeps two populations apart

**Verified-negative walks** (157 frames). Both zero-target walks were checked
by eye before use: the red-backpack walk searches a hotel room and never
finds it; the grey-backpack walk wanders into a bathroom and ends facing a
door. Neither contains the target. A `target_visible` here is a **real false
positive**, and this is the clean precision measurement.

**Target-present walks** (295 frames, 212 positive). Recall is measured here.
But frames where the model says "visible" and the label says "not" are
**reported separately and never counted as errors** — some of them are the
model being right and the label being wrong.

That separation is the difference between a number you can act on and one
you would have to retract later.

---

## 4. The numbers

Recall is measured on the 212 target-present frames. False positives are
measured on the 157 frames verified to contain no target.

| Model | Recall | Misses | False positives | FP rate | Median latency | Cost / 452 frames |
|---|---:|---:|---:|---:|---:|---:|
| **Opus 4.5** *(current default)* | 88% | 26 | 2 | 1.3% | **2.9s** | **$5.31** |
| **Opus 5** | 92% | 17 | **0** | **0.0%** | 3.0s | $6.46 |
| **Fable 5.1** | **100%** | **0** | 1 | 0.6% | 4.2s | $12.40 |
| **GPT-6 Astra** | 99% | 2 | 5 | 3.2% | 4.3s | $7.77 |

### Per-walk detail

Recall on target-present walks; false positives on verified-empty walks.

| Walk | Opus 4.5 | Opus 5 | Fable 5.1 | GPT-6 Astra |
|---|---|---|---|---|
| charger …203250 | 100% (41/41) | 90% (37/41) | 100% (41/41) | 100% (41/41) |
| charger …203355 | 100% (77/77) | 95% (73/77) | 100% (77/77) | 99% (76/77) |
| grey-backpack …203748 | 77% (55/71) | 92% (65/71) | 100% (71/71) | 100% (71/71) |
| grey-backpack …210743 | 57% (13/23) | 87% (20/23) | 100% (23/23) | 96% (22/23) |
| **grey-backpack …203627** *(empty)* | 0 FP / 92 | 0 FP / 92 | 0 FP / 92 | 0 FP / 92 |
| **red-backpack …203152** *(empty)* | 2 FP / 65 | 0 FP / 65 | 1 FP / 65 | **5 FP / 65** |

Note that Opus 5's aggregate recall win over Opus 4.5 is **a trade, not a
sweep**: it loses 8 frames on the charger walks and gains 17 on the backpack
walks.

### Behaviour

Share of answered frames. These describe how each model drives, not whether
it is right — the walks were all originally recorded under Opus 4.5, so the
path is fixed and no model actually navigated anywhere.

| Model | FORWARD | LEFT | RIGHT | STOP | claimed "reached" | claimed obstacle |
|---|---:|---:|---:|---:|---:|---:|
| Opus 4.5 | 35% | 15% | 25% | 23% | 22% | 44% |
| Opus 5 | 30% | 8% | 24% | 34% | 26% | 41% |
| Fable 5.1 | 36% | 14% | 16% | 28% | 27% | 46% |
| GPT-6 Astra | 31% | 10% | 19% | 30% | 30% | 46% |

### Close-range robustness

Every disagreement with the labels clustered in the last 40% of each walk —
exactly the close-range frames where the labelling detector is known to fail.
Read this as *how many close-range frames each model recovered*, not as error.

| Model | Label-disagreements recovered |
|---|---:|
| GPT-6 Astra | 43 |
| Fable 5.1 | 35 |
| Opus 4.5 | 31 |
| Opus 5 | 21 |

---

## 5. What I'd actually conclude

**Fable 5.1 got every single visible frame — 212 of 212.** That is the
standout result and it was not expected to come back clean. It costs 2.3×
the current default.

**Opus 5 is the interesting one, and not for the reason the aggregate
suggests.** It is the only model that never produced a false positive. But
as the per-walk table shows, its recall advantage is a trade rather than
dominance.

**GPT-6 Astra buys recall with precision.** 99% recall, but 5 of the 7 clean
false positives in the whole run are its, and it is the slowest at the 90th
percentile (7.0s) — which matters for a robot loop with a timeout.

**Opus 5 costs 22% more than Opus 4.5 despite an identical rate card.** It
emits 150 output tokens per frame against 88. Same price per token, more
words. (Section 7 explains the rest of that effect.)

### Recommendation

**Do not promote anything to default on this evidence alone.** One room, one
session, and — as the next section shows — the largest effect measured was
not the model.

If a default change is wanted anyway, **Opus 5 is the defensible one**: zero
false positives, better net recall, same latency, about $1.15 more per 452
frames. **Fable 5.1 is right only if a missed frame costs more than 2.3× the
money.**

The cheapest next step is to record one walk **in a different room with an
unambiguously-described target**. That separates "Opus 4.5 is strict" from
"Opus 4.5 is worse", which this dataset cannot.

---

## 6. The finding that matters most

### Opus 4.5's bad walks are a colour judgement, not a detection failure

On `grey-backpack-opus-4-5-…210743` Opus 4.5 scored 57%. Every single miss
reads like this:

> *"I see a black backpack on the floor in the center-right area, but I'm
> searching for a grey backpack which is not visible"*

> *"There is a dark bag visible on the right side of the image but it appears
> to be black, not grey"*

It finds the bag in every frame and then **disqualifies it on colour**. The
bag is charcoal — defensibly either. The other three models accept it.

So that 57% measures strictness about the word *"grey"*, not blindness. The
target **string** is a bigger lever here than the model choice, which matches
a finding already recorded elsewhere in this project: changing a target from
a bare noun to a colour-plus-noun moved detection rates by roughly 5×.

### All seven false positives are the same object

Every clean false positive across all four models is one thing: **black
luggage on a dresser with a small red baggage tag** clipping the top edge of
the frame. Three models call that "the red backpack":

> *"A red object resembling the backpack is visible on top of the dresser in
> the left third"* — Fable 5.1

> *"The red backpack is partially visible on the furniture to the left"* —
> GPT-6 Astra

Zooming the frames confirms it is a luggage tag, not a backpack. The labels
are correct; the models over-claim a target from a red fragment at the edge
of view.

This is notable because it is the exact trap Opus 4.5 was previously credited
with resisting — it was once *"the only model that refused to call a red
blanket a red backpack."* Here it falls for it twice. **Only Opus 5 never
does.**

### Practical implication

Two of the four models' worst behaviours are driven by how the target is
*named*, not by how well they see. Before spending on model selection, spend
on target descriptions: unambiguous colour terms, or no colour term at all.

---

## 7. Why is GPT-6 Astra cheaper than Fable 5.1?

They are billed at **exactly the same rate** — $11.00 per million input
tokens and $55.00 per million output tokens. Yet Astra cost $7.77 for the run
and Fable 5.1 cost $12.40.

**The difference is entirely tokenization: the same picture is not the same
number of tokens.**

This was isolated by sending the identical prompt twice to each model — once
with the image and once without — and subtracting.

| Model | Text tokens | Image tokens | Total per frame |
|---|---:|---:|---:|
| Opus 4.5 | 491 | 1,200 | 1,691 |
| Opus 5 | 646 | 1,199 | 1,845 |
| **Fable 5.1** | 648 | **1,199** | 1,847 |
| **GPT-6 Astra** | 452 | **610** | **1,062** |

Two effects stack:

1. **Astra charges 610 tokens for the same 1280×720 frame where every Claude
   model charges ~1,200** — roughly half.
2. **Its text tokenizer is ~30% tighter** on the byte-identical prompt (452
   vs 648 tokens).

Together that is 43% fewer input tokens per frame, which is the whole of the
price difference.

### The same mechanism explains an oddity in Opus 5's cost

Opus 5 and Opus 4.5 have an identical rate card, yet Opus 5 cost 22% more.
They share an image tokenizer (1,199 vs 1,200 tokens — the same picture), but
Opus 5's newer **text** tokenizer counts the same prompt as 646 tokens rather
than 491, and it writes 150 output tokens per frame against 88. Not a price
change; a token-count change.

### There is a lever here

Image tokens are about 65% of Fable 5.1's input. The source frames are
1280×720, and downscaling reduces the bill proportionally:

| Image size | Fable 5.1 $/frame | vs full | GPT-6 Astra $/frame |
|---|---:|---:|---:|
| 1280×720 | $0.0272 | — | $0.0161 |
| 896×504 | $0.0204 | −25% | $0.0159 |
| 640×360 | $0.0174 | **−36%** | $0.0136 |
| 448×252 | $0.0157 | −42% | $0.0122 |

At 640×360, Fable 5.1 costs $0.0174 — close to Astra's full-resolution
$0.0161. **Downscaling roughly erases the price gap.**

**Caveat:** every one of those eight calls returned the correct answer, but
that was a single close-up frame where the target fills the view — the
easiest possible case. The frames that would break are distant ones, where a
target is a handful of pixels. Re-running the recall comparison at 640×360
(about $12 on this corpus) would show whether the 36% saving is free or paid
for in misses.

---

## 8. Limitations

Read every number above against these.

- **One room, one session.** Six walks recorded in a single hotel room on one
  evening. Nothing here generalises to other environments yet.
- **Ground truth is machine-proposed.** Labels come from OWLv2 with only a
  few frames human-reviewed, and they are known to fail at close range.
  Recall is therefore *understated* for all four models.
- **This measures perception, not navigation.** All six walks were recorded
  under Opus 4.5, so the path is fixed. No model navigated anywhere, and
  nothing here says whether a model would *reach* a target.
- **Two walks carry most of the signal.** The two grey-backpack walks account
  for almost the entire spread between models — and they are the walks with
  the ambiguous colour term.
- **Latency was measured through a deployed service**, so it includes network
  and service overhead, not just model time.
  **Fable 5.1's latency may also include a cross-region hop:** it is the one
  model `vision_core.MODEL_REGIONS` pins to `us-east-1` (it is refused from
  the service's own region), so its calls leave the region the service runs
  in. Its median should not be compared to the others' as pure model time.

---

## 9. Reproducing this

The replay calls the deployed `/navigate` endpoint once per frame with
`model_id` and `prompt_variant: "default"` pinned, six frames concurrently,
with backoff that catches transport errors as well as HTTP status codes.

Scoring keeps the verified-negative walks and the target-present walks in
separate populations, as described in Section 3.

Rates used for all cost figures are the **actual billed unit costs** for this
AWS account, read from Cost Explorer grouped by service, not from a published
price list. (The AWS Pricing API carries only Claude 2.x/3 for this region,
and the public Bedrock pricing page does not list any of these four models.)

| Model | Input $/1M | Output $/1M |
|---|---:|---:|
| Claude Opus 4.5 | 5.50 | 27.50 |
| Claude Opus 5 | 5.50 | 27.50 |
| Claude Fable 5.1 | 11.00 | 55.00 |
| OpenAI GPT-6 Astra | 11.00 | 55.00 |

For reference, Bedrock charges **exactly 10% over Anthropic's first-party
list price**, confirmed independently on three models.

---

*Raw per-frame results: 24 JSON files (6 walks × 4 models), one record per
frame carrying the action, visibility claim, reasoning, token usage and
latency.*

*Where they are (checked 2026-09-28): **not in the repo.** No file under
`recordings/` or `evaluations/` carries these model ids, and the six walks'
directories hold only frames plus their label, meta and `walk.jsonl` files.
This document does not name the replay tool; `control/walk_replay.py` is the
repo's instrument for re-asking a walk under another model, but whether it
produced these results is not recorded. Until the 24 files are found, the
tables above cannot be recomputed.*
