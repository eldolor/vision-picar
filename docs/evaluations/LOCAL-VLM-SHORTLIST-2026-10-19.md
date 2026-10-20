# Local VLMs to try on the Rover (2026-10-19)

A shortlist, for Rover day, of the small vision-language models measured on
the Jetson Orin Nano Super (15 W, beside the idle stack) in 3.48, 3.78 and
3.80.I-R. Every number below comes from those plan sections; this page only
ranks them. They were measured on phone frames shot at floor height, so they
need to be re-measured on the Rover's own camera.

Each model has two jobs:
* **identity:** "is this the target?" at arrival, scored on 3.48's 299
  frames. The bars are precision >= 95% and recall >= 85%. "Teal" counts
  YES answers on the 34 frames where the cloud confabulated.
* **steering:** "where is it in the frame?", scored as S1 right of 60
  frames. The bar is 34.

Memory is the drop in `MemAvailable`; the bar is 3,000 MB. Those drops
come from each section's own bench, so compare rows within a section only.
3.80.S measured VE and Florence the same way, each beside YOLOE + CLIP, and
VE's drop came out at 5,063 MB rather than 2,978 MB (see "Florence behind a
gate" below).

## Top 3 (the identity check)

| # | model, runtime | identity precision / recall, teal | steering S1 | memory drop | decision p90 | why it is here |
|---|---|---|---|---|---|---|
| 1 | **VE: Qwen2.5-VL-3B, TensorRT Edge-LLM** (3.78) | 97.2% / 94.6%, 1/34 | 42/60 (box) | 2,978 MB; 606 MB free beside the stack | 1.08 s (1.32 s under full load) | Meets every bar. It is 3.80's local model (`service/local_vlm`, off by default). Start here. |
| 2 | **Bonsai 2 2B (ternary), PrismML's llama.cpp fork `118091848`** (3.80.R) | 94.5% / 93.2%, 1/34 | 48/60 (box) | **1,238 MB** | 1.38 s | One false positive short of the 95% bar. It has the smallest memory footprint and is fast without TensorRT. Run with `LLAMA_ARG_REASONING=off`. A higher gate, or the Rover's own frames, may push it over. |
| 3 | **MiniCPM-V 4.0, llama.cpp Q4_K_M** (3.80.M) | 97.3% / 97.3%, 1/34 | 8/60 | 2,836 MB | 3.21 s | Matches VE on identity, but is too slow under llama.cpp and steers badly. Worth a faster build (TensorRT) before trying it. |

## Top 5 (adds the two best steerers)

| # | model, runtime | identity | steering S1 | memory drop | p90 | why it is here |
|---|---|---|---|---|---|---|
| 4 | **Florence-2-base (0.23B), torch fp16, `<CAPTION_TO_PHRASE_GROUNDING>`** (3.80.R, 3.80.S) | fails: it always draws a box | **56/60** (-large: 57/60, 1.17 s, 2,252 MB); **53/60 behind a YOLOE + CLIP gate**, with 2/40 out of view | 1,355 MB alone; 3,118-3,372 MB with YOLOE + CLIP (VE with YOLOE + CLIP: 5,063) | **0.58 s** (0.75 s beside YOLOE + CLIP) | The best and fastest local steerer. It always draws a box, even with the target out of view (40/40), so it can only steer behind something that says the target is there. 3.80.S built that gate from YOLOE + CLIP. Uses the `florence-community` checkpoint, MIT licence. |
| 5 | **InternVL3.5-4B, llama.cpp Q4_K_M or Edge-LLM** (3.80.M, 3.80.N) | fails: 82.8% / 97.3%, 7/34 | 50/60 (llama.cpp), 52-53/60 (Edge-LLM) | 3,677 MB | 5.91 s (llama.cpp) | The best chat-VLM steerer, and it never drew a box with the target out of view. It says yes too easily at arrival, and it is over the memory bar. |

## Florence behind a gate (3.80.S, 2026-10-20)

**The gate:** YOLOE + CLIP runs on every frame. If it detects the target
(P >= 0.8), its own box steers. If it detected the target in any of the 3
frames before this one but not in this one, Florence's box steers. If
neither, YOLOE + CLIP's unsure candidate (P 0.5-0.8) steers if there is
one; otherwise there is no steer. Measured on the board at 15 W, beside
the desktop and the idle twin stack, on 3.80.I's frames: 60 with the
target in view and 40 without, each also mirrored.

| On the phone's frames (`orig`) | YOLOE + CLIP alone | **The gate** | Florence alone | YOLOE + CLIP, then VE's box (3.80.I) |
|---|---|---|---|---|
| right, of 60 with the target in view | 44 | **53** | 56 | 51 |
| answers, of 40 without the target | 1 | **2** | 40 | -- |
| same answer when mirrored | 93.3% | 94.6% | 96.7% | -- |
| time per frame where Florence is asked, p90 | -- | **0.81 s** (n 21) | 0.74 s | -- |

**Memory, measured the same way for both (amendment 1):** `MemAvailable`
sampled every second from before the second model loads to the end of
the run.

| beside YOLOE + CLIP | before | median | lowest | drop to lowest |
|---|---|---|---|---|
| Florence, run 1 | 5,397 MB | 2,761 | 2,025 | 3,372 MB |
| Florence, run 2 | 5,449 MB | 2,718 | 2,331 | 3,118 MB |
| **VE** | 5,409 MB | 683 | 346 | **5,063 MB** |

* Florence plus perception uses about 2 GB less than VE plus perception.
  The two Florence runs differ by 254 MB at the lowest point, so the gap
  is far larger than run-to-run noise.
* The lowest points are short dips when the target changes (a new
  target's text encoder loads). The median is the steady level.
* Same frames, VE's box in Florence's place: VE alone steers 42/60 with
  0/40 out of view; a gate with VE 50/60; VE's answer p90 2.37 s.

**At smaller sizes (the depth camera's width and below):**

| right of 60 | phone frames | 640 wide | 320 wide |
|---|---|---|---|
| YOLOE + CLIP alone | 44 | 42 | 21 |
| Florence alone | 56 | 56 | 51 |
| the gate | 53 | 48 | 27 |

Smaller frames hurt YOLOE + CLIP, not Florence. The gate loses frames at
640 because YOLOE + CLIP detects less often, so the gate opens on 6 frames
instead of 9. Frames that were only re-saved as JPEG (the 11 already
640x480) scored the same, so the loss comes from the resize. A downscaled
phone frame does not show the Rover's lens or field of view; the OAK-D
Lite's colour camera is 13 MP and its two depth cameras are greyscale.

**What it cannot do:** check identity at arrival, because Florence draws a
box whether or not the target is there. And **Florence and VE do not fit
together** beside YOLOE + CLIP (amendment 2):

| seconds | `MemAvailable` | loaded |
|---|---|---|
| 0 | 5,358 MB | nothing of the bench |
| 10 | 2,791 MB | VE |
| 36 | 1,059 MB | VE and Florence |
| 45 | **25 MB** | YOLOE + CLIP loading; the kernel then killed Florence on its first request |

So on this board it is one or the other:
* **VE:** checks identity, and its box steers 42/60 (50/60 behind the
  gate).
* **Florence behind the gate:** steers 53/60, and every arrival's
  identity check goes to the cloud.

Decide on the Rover's own frames.

## Nearly made the list

| model | result | what would change it |
|---|---|---|
| PaliGemma 2 3B mix-448 (`detect <name>`) | Best identity measured: 97.4% / 100%, teal 0/34; steers 54/60. Measured on the Mac (MPS); at 6.1 GB of bf16 it will not load on the board. | An int4 build that loads beside the stack, with a runtime that decides in under 2.0 s. Gemma licence terms apply. |
| RoboBrain2.5-4B, llama.cpp Q4_K_M | The only model whose left/right answer is real (direction 47/60); box 54/60. Fails identity (42.8%) and memory (3,918 MB). | Steering only, if memory allows. |

## Not worth trying

Measured and failed clearly:
* LocateAnything-3B: research licence only.
* FastVLM-0.5B / 1.5B.
* Bonsai 2 4B.
* Eagle2-2B and NVILA-Lite-2B: out of memory at load on the board.
* Youtu-VL-4B.
* InternVL3.5-2B.
* Qwen3-VL, Cosmos-Reason2 and LFM2 (3.48).

Would not load at all:
* Moondream 2 / 3.1 under Photon: needs CUDA 12.8; JetPack 6.2.1 ships 12.6.
* RoboRefer-2B: out of memory at load.
* Molmo2-4B: 9.1 GB at fp16, more than the board's memory; on the 16 GB
  Mac it hung the machine (3.80.R).

## Rover-day notes

* **Stop `picar-stack`, `picar-ngrok` and `picar-local-vlm` before any
  bench** (`tools/jetson/README.md`).
* **zram has been off on the board since 2026-10-17**
  (`nvzramconfig.service` failed). Check `free -m` before trusting a memory
  number.
* **Harnesses, ready to point at new frames:**
  * `evaluations/tiered-380/R/`: `drive.sh` for llama.cpp and torch rows,
    `native_server.py` for torch backends, `score_identity.sh`,
    `score_steer.py`.
  * `evaluations/tiered-380/M/` for llama.cpp.
  * 3.78's Edge-LLM build for VE.
* **What is still on the board** (everything else measured was deleted
  2026-10-19, and can be fetched again by the SHA256s in each section):
  * VE: `~/bench-378/ws91/q25VE` (the live engines), `ell91`, and the
    Qwen2.5-VL-3B checkpoint in `ws91` to rebuild from.
  * Bonsai 2 2B: `~/models-380R/Ternary-Bonsai-2-2B-gguf`, with the
    fork built in `~/bench-380R/prism`.
  * MiniCPM-V 4.0 (`ggml-model-Q4_K_M.gguf`, `mmproj-model-f16.gguf`) and
    InternVL3.5-4B Q4_K_M: `~/models-380M`.
  * InternVL3.5-4B under Edge-LLM, VE's recipe: `~/bench-380N/NVE`.
  * Florence-2 base and large: `~/models-380R/fc-Florence-2-*`, run
    from `~/venv-380R`. The gate's runner is
    `evaluations/tiered-380/S/board/run.sh <orig|640|320> <florence|ve|both>`
    from the board checkout `~/vp-380R`; it serves Florence on :8006 and
    VE on :8007, so the production unit on :8005 is never touched.
  * RoboBrain2.5-4B, Q4_K_M and mmproj only: `~/models-380R/RoboBrain2.5-4B`.
  * Gemma 4 E2B/E4B (LiteRT, the inventory namers): `~/litert-346`.
* **Never merge Mac rows with board rows (P24).** The Rover's camera needs
  its own pinned frame set before any comparison.
