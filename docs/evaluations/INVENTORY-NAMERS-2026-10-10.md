# Inventory namers: Qwen3.5-4B vs Gemma 4 E2B / E4B (2026-10-10)

The record behind `PLAN-ros-alignment.md` 3.46 phase C and its amendments
6-8: which small vision model could name the objects a search walks past,
on the Jetson, for the object inventory. The shareable version, with the
eight comparison images, is the artifact "Qwen vs Gemma on the Jetson"
(https://claude.ai/artifact/5tAVENrDAtu6EYtigUUQLt, private to its owner).

## Why this experiment

vision-picar is an indoor robot that searches a house for a named object.
Small models on its Jetson look at every frame for free and steer; a
cloud model is asked only at key moments. A search drives past most of the
house and then forgets everything but the target. The object inventory
(3.46) remembers every object seen, and where, so a later search can start
where a thing was last seen. It works in the simulator, but on the real
car it gets nothing: the on-board detector looks only for the target. It
needs a model that names every object, runs on the Jetson, and fits in the
memory left beside the robot stack. Cheaper namers failed first (table
below); small vision language models were the next step.

## The job, and the bar

The inventory needs, for every object in a frame, a **name** and a **box**.
The lidar supplies the distance at the box's bearing, so the box's
horizontal centre is what places the object on the map; the name is what
the inventory remembers.

* **Memory:** the Jetson Orin Nano Super has 8 GB; with the whole robot
  stack running, 4,253 MB is available (3.33 / 3.37), so a model may use
  **3.25 GB** (3.48 criterion 6).
* **Names:** a cloud judge (Claude Opus 5.5 on Bedrock) marks each named
  box right or wrong. **75%** right on 300 tuning frames earns a check by
  the user on 150 frames, where the bar is **70%** (criterion 6).

## What has been measured (cloud judge, 300 tuning frames)

| Namer | Names right | Right objects a frame | Fits 3.25 GB | Verdict |
|---|---|---|---|---|
| YOLOE-11s prompt-free, every object name | 29.7% | 2.9 | yes | failed |
| YOLOE, 11 reliable names only | 90.5% | 0.6 | yes | waits on the user's check |
| CLIP RN50 confirming YOLOE | 54.6% (when it agrees) | -- | yes | failed |
| CLIP RN50 renaming (50-frame pilot) | 12.0% | -- | yes | failed |
| Qwen2.5-VL-3B, full precision | 58.9% | 1.56 | no (7.5 GB) | failed; primed by the prompt's examples |
| Qwen3.5-4B, 3-bit (IQ3_XXS) | **64.1%** (909 boxes; 72.4% even if every unjudged box were right) | 2.52 | 2.91 GB of files; not measured on the Jetson | failed |
| Gemma 4 E2B, LiteRT-LM GPU (amendment 9) | **41.0%** (625 boxes; 48.0% even if every unjudged box were right) | 0.85 | **yes: 2.67 GB measured under the robot stack** | failed; "office chair" 4 of 73 right |
| Gemma 4 E2B, prompt with no object words (amendment 10, 40-frame pilot) | **60.0%** (110 boxes; 95% upper bound 68.7%) | 1.65 | yes (as above) | failed the continue rule |
| Gemma 4 E4B, prompt with no object words (amendment 10, 40-frame pilot) | **63.5%** (148 boxes; 95% upper bound 70.8%) | 2.35 | **no: 4.07 GB under the stack** (amendment 9) | failed on memory and the continue rule |
| InternVL3.5-2B, 4-bit | not judged | -- | 1.92 GB | cancelled by the user before judging |

**Qwen3.5-4B's judging** reached 361 of 450 frames (231 of the 300 tuning
frames) before Bedrock's overload errors stopped it, for about $4 in all
(an estimate). It was not resumed: the 273 unjudged boxes could lift
precision to 72.4% at most, still under the 75% bar.

## The three small models side by side (8 frames, by eye, no cloud calls)

Same prompt, same frames (seed 3467, one per walk, from the frames the
judge had reached for Qwen3.5-4B). Compared by eye by Claude: an
impression, not a measurement.

| | Qwen3.5-4B (3-bit) | Gemma 4 E2B (4-bit) | Gemma 4 E4B (4-bit) |
|---|---|---|---|
| Boxes | tight | consistently **low**; left/right edges about right | same as E2B |
| Box format | 0-1000, [x1, y1, x2, y2] | 0-1000, **[y1, x1, y2, x2]** | same as E2B |
| Names | good, but primed ("office chair" for an armchair, "box" for a wicker basket) | as good or better, less primed | best: more objects, precise names; also said "office chair" once |
| Speed on the Mac | 18-22 s a frame | 8-21 s | 7-39 s |
| Resident under llama.cpp (Mac) | 3.4 GB | 4.7 GB | 6.7 GB |
| Fits the Jetson under llama.cpp | borderline; to measure | no | no |

Cloud judge on Qwen3.5-4B's boxes in these 8 frames: 26 of 33 right.

## Why Gemma is bigger than "E2B / E4B" suggests

"E" means *effective* parameters. Gemma 4 E2B is 5.1B parameters with its
per-layer embeddings (PLE) and E4B is 8B (model card). Google's figures of
~2 GB and ~3 GB assume a runtime that keeps the embedding tables out of
memory and fetches rows as needed; that is how they run on phones.
llama.cpp evidently loads them: 4.7 / 6.7 GB resident. A 3-bit build
does not rescue it (E2B 3.10-3.13 GB, E4B 3.72 GB of files, plus a 0.56 GB
vision encoder), because the tables dominate the size.

## Google's runtime (LiteRT-LM) on the Jetson, as of October 2026

* **CPU:** runs on Orin and Thor; slow (~3 tokens/s on an Orin NX through
  Python).
* **GPU:** through WebGPU/Vulkan, not CUDA. One user ran Gemma 4 E2B text
  on an Orin NX GPU (1.9 s a command; LiteRT-LM #2001, closed April 2026);
  another found the GPU backend failing on Orin and Thor before the first
  token (#2570, closed July 2026), whose cause -- missing ARM libraries in
  the pip package -- got only a manual workaround in September.
* **Images:** in #2570 the multimodal E2B crashed (SIGSEGV) on the Jetson
  GPU; no report says the workaround fixes it.
* **Memory on a Jetson:** not published anywhere found.
* A second GPU runtime (Vulkan) beside CUDA's YOLOE and CLIP is an
  unmeasured risk on a small board.

## Decision (the user's, 2026-10-09), and where it stands (2026-10-10)

If Qwen3.5-4B is acceptable, it is the inventory's namer; other models
wait. **It is not acceptable: 64.1% against the 75% bar.** No VLM namer
has passed, so the inventory on the real car still has no namer;
configuration F (YOLOE limited to 11 reliable names, 90.5% but 0.6 right
objects a frame) still waits on the user's 150-frame check. Gemma is the backup: its names are the best of the three, but it
needs its box convention understood and a runtime that fits the Jetson
before it can be measured fairly. A free hands-on test of LiteRT-LM on the
Jetson (5 frames with images, memory measured) was to be the next step
if Qwen failed; it has, so that test is now open (free, no cloud calls).
A prompt without example names, on fresh frames, is the other next step.

**Update, 2026-10-10 (amendment 9).** The LiteRT-LM test ran on all 450
frames, on the Jetson's GPU, with the robot stack up. E2B fits (2.67 GB,
no effect on the wheel loop, nav2 or perception; 8 s a frame) but the
judge found only 41.0% of its names right, under Qwen3.5-4B's 64.1%. The
by-eye comparison above, on 8 frames, did not hold up at scale. E4B does
not fit (the board fell to 474 MB free). Both copy the prompt's example
names heavily, so a prompt without them is now the next step for every
VLM namer.

## Sources

* Gemma 4 model card: https://ai.google.dev/gemma/docs/core/model_card_4
* Gemma 3n announcement (memory figures): https://developers.googleblog.com/introducing-gemma-3n/
* LiteRT-LM #2001: https://github.com/google-ai-edge/LiteRT-LM/issues/2001
* LiteRT-LM #2570: https://github.com/google-ai-edge/LiteRT-LM/issues/2570
* Gemma 4 on a Jetson Orin Nano (llama.cpp): https://zilligm.github.io/blog/2026/04/12/jetson-nano-llm
* Images: the eight full-resolution comparisons are in the private
  recordings bucket under evaluations/inventory-namers-2026-10-10/
  (full-res/ and web/).
* Builds: bartowski/google_gemma-4-E2B-it-GGUF, bartowski/google_gemma-4-E4B-it-GGUF,
  bartowski/Qwen_Qwen3.5-4B-GGUF (Hugging Face)

**Update, 2026-10-10 (amendment 10).** With every object word taken out
of the prompt (no example names, no category list), labels taken from the
prompt fell from 47% to 2%. On the 29 frames judged under both prompts,
E2B rose from 42.7% to 54.4% right. On the 40-frame pilot, E2B scored
60.0% and E4B 63.5%. Neither 95% upper bound reaches 75%, so neither went
on to the full 300 (pilot grading ~$0.75, an estimate). **Where it
stands:** every small VLM tried lands at 60-64% at best, and the one that
fits beside the running robot (E2B) is the weakest.

* **No runtime makes Qwen fit like Gemma.** LiteRT-LM can keep Gemma's
  per-layer embedding tables on disk, because each token reads only a few
  rows. Qwen is dense: every weight is read for every token, so mapping it
  from disk (which llama.cpp already does) saves nothing. TensorRT-LLM
  and MLC are faster, not smaller.
* **The one idea left that changes the budget is naming after a mission**,
  parked, with nav2, SLAM and the perception loop stopped (~2 GB more
  free). That could hold E4B, Qwen3.5-4B at 4-bit or more, or a larger
  Qwen. It is an architecture change and waits for the user's decision.

## A different job: searching for one named object (amendment 11, 2026-10-10)

The search tier's job, not the inventory's: find one described object.
Measured free against the walks' human labels, on six walks (one per
target, 685 frames, 313 with the target), on the Jetson GPU.

| | target frames found | false alarms | ms a frame |
|---|---|---|---|
| YOLOE + CLIP (shipped, gate 0.8) | 80.8% | 0 | 69 |
| Gemma 4 E2B | 98.4% | 72 | ~5,000 |

Gemma finds almost every sighting but says yes too readily: 45 of its
false alarms are a teal bin it calls the "blue bottle". That is the bin
that fooled the cloud model too. YOLOE + CLIP never fires on it. The
pre-set rule (Gemma +10 points at matched false alarms) is not met, and
the shipped tier stays. **For naming everything, the vision models beat
YOLOE + CLIP; for finding one named thing, YOLOE + CLIP is the better
tool.**

## Stepping back: remember what things look like, not their names (amendments 12-13)

The inventory's job is "where did I last see X?", which needs no names.
So every object the prompt-free YOLOE boxes is stored with its CLIP image
embedding (4 KB), and a request is matched against them with the shipped
CLIP rule. Free; no cloud calls.

| | six tuning walks | 15 held-out walks |
|---|---|---|
| Lookup: top match shows the target | 6 of 6 | 5 of 5 |
| Lookup: top-5 matches that show it | 25 of 30 | 23 of 25 |
| Lookup time (laptop CPU) | 7-10 ms | 13-15 ms (12.6 at least) |
| Frames with the target found, memory vs live search | 84.3% vs 80.8% (13 vs 0 false alarms) | 64.0% vs 77.5% (11 vs 5 false alarms) |

**Looking up works:** the right object comes first for all 11 targets,
and it found shoes and the laundry basket on walks recorded for other
targets. **Recall per frame fails:** a memory is made before anyone asks,
so it holds only what the prompt-free model boxed, and that model misses
the bottle, basket and cable on many frames the target-prompted live
search catches. Both amendments' pre-set rules required both halves, so
nothing is built and phase C (a namer for the inventory) stops here.
