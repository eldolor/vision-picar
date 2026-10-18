frames: 299 in 4 walks, 74 visible (today's labels.json)
VE: trt-edgellm:Qwen2.5-VL-3B-Instruct-VE int4_awq + visual fp16 on jetson-orin-nano-super-15w None -- 299 frames, 0 labels flipped since scoring, 0 labelled frames not covered
ivl2b-A: llama.cpp:OpenGVLab_InternVL3_5-2B-Q4_K_M.gguf on jetson-orin-nano-super-15w None -- 299 frames, 0 labels flipped since scoring, 0 labelled frames not covered
ivl2b-B: llama.cpp:OpenGVLab_InternVL3_5-2B-Q4_K_M.gguf on jetson-orin-nano-super-15w None -- 299 frames, 0 labels flipped since scoring, 0 labelled frames not covered
ivl4b-A: llama.cpp:OpenGVLab_InternVL3_5-4B-Q4_K_M.gguf on jetson-orin-nano-super-15w None -- 299 frames, 0 labels flipped since scoring, 0 labelled frames not covered
ivl4b-B: llama.cpp:OpenGVLab_InternVL3_5-4B-Q4_K_M.gguf on jetson-orin-nano-super-15w None -- 299 frames, 0 labels flipped since scoring, 0 labelled frames not covered
minicpm-A: llama.cpp:ggml-model-Q4_K_M.gguf on jetson-orin-nano-super-15w None -- 299 frames, 0 labels flipped since scoring, 0 labelled frames not covered
minicpm-B: llama.cpp:ggml-model-Q4_K_M.gguf on jetson-orin-nano-super-15w None -- 299 frames, 0 labels flipped since scoring, 0 labelled frames not covered
youtu-A: llama.cpp:Youtu-VL-4B-Instruct-Q4_K_M.gguf on jetson-orin-nano-super-15w None -- 299 frames, 0 labels flipped since scoring, 0 labelled frames not covered
youtu-B: llama.cpp:Youtu-VL-4B-Instruct-Q4_K_M.gguf on jetson-orin-nano-super-15w None -- 299 frames, 0 labels flipped since scoring, 0 labelled frames not covered

| model | gate | precision (TP/YES) | recall (TP/visible) | YES on cloud confabulations | @ 0 FP (fitted) |
|---|---|---|---|---|---|
| cloud (us.anthropic.claude-opus-4-5-20251101-v1:0) | 0.5 | 67.9% (72/106) | 97.3% (72/74) | 34/34 | 0.0% |
| VE | 0.5 | 97.2% (70/72) | 94.6% (70/74) | 1/34 | 40.5% (not separable) |
| ivl2b-A | 0.5 | 100.0% (15/15) | 20.3% (15/74) | 0/34 | 20.3% |
| ivl2b-B | 0.5 | 100.0% (1/1) | 1.4% (1/74) | 0/34 | 2.7% |
| ivl4b-A | 0.5 | 82.8% (72/87) | 97.3% (72/74) | 7/34 | 73.0% (not separable) |
| ivl4b-B | 0.5 | 67.0% (73/109) | 98.6% (73/74) | 25/34 | 87.8% |
| minicpm-A | 0.5 | 97.3% (72/74) | 97.3% (72/74) | 1/34 | 1.4% |
| minicpm-B | 0.5 | 65.5% (72/110) | 97.3% (72/74) | 6/34 | 1.4% (not separable) |
| youtu-A | 0.5 | 34.1% (74/217) | 100.0% (74/74) | 32/34 | 0.0% |
| youtu-B | 0.5 | 26.2% (74/282) | 100.0% (74/74) | 34/34 | 0.0% |
