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

Memory is the drop in `MemAvailable`; the bar is 3,000 MB.

## Top 3 (the identity check)

| # | model, runtime | identity precision / recall, teal | steering S1 | memory drop | decision p90 | why it is here |
|---|---|---|---|---|---|---|
| 1 | **VE: Qwen2.5-VL-3B, TensorRT Edge-LLM** (3.78) | 97.2% / 94.6%, 1/34 | 42/60 (box) | 2,978 MB; 606 MB free beside the stack | 1.08 s (1.32 s under full load) | Meets every bar. It is 3.80's local model (`service/local_vlm`, off by default). Start here. |
| 2 | **Bonsai 2 2B (ternary), PrismML's llama.cpp fork `118091848`** (3.80.R) | 94.5% / 93.2%, 1/34 | 48/60 (box) | **1,238 MB** | 1.38 s | One false positive short of the 95% bar. It has the smallest memory footprint and is fast without TensorRT. Run with `LLAMA_ARG_REASONING=off`. A higher gate, or the Rover's own frames, may push it over. |
| 3 | **MiniCPM-V 4.0, llama.cpp Q4_K_M** (3.80.M) | 97.3% / 97.3%, 1/34 | 8/60 | 2,836 MB | 3.21 s | Matches VE on identity, but is too slow under llama.cpp and steers badly. Worth a faster build (TensorRT) before trying it. |

## Top 5 (adds the two best steerers)

| # | model, runtime | identity | steering S1 | memory drop | p90 | why it is here |
|---|---|---|---|---|---|---|
| 4 | **Florence-2-base (0.23B), torch fp16, `<CAPTION_TO_PHRASE_GROUNDING>`** (3.80.R) | fails: it always draws a box | **56/60** (-large: 57/60, 1.17 s, 2,252 MB) | 1,355 MB | **0.58 s** | The best and fastest local steerer. It always draws a box, even with the target out of view, so it can only steer behind an identity check such as VE. Uses the `florence-community` checkpoint, MIT licence. |
| 5 | **InternVL3.5-4B, llama.cpp Q4_K_M or Edge-LLM** (3.80.M, 3.80.N) | fails: 82.8% / 97.3%, 7/34 | 50/60 (llama.cpp), 52-53/60 (Edge-LLM) | 3,677 MB | 5.91 s (llama.cpp) | The best chat-VLM steerer, and it never drew a box with the target out of view. It says yes too easily at arrival, and it is over the memory bar. |

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
* **Never merge Mac rows with board rows (P24).** The Rover's camera needs
  its own pinned frame set before any comparison.
