frames: 299 in 4 walks, 74 visible (today's labels.json)
VE: trt-edgellm:Qwen2.5-VL-3B-Instruct-VE int4_awq + visual fp16 on jetson-orin-nano-super-15w None -- 299 frames, 0 labels flipped since scoring, 0 labelled frames not covered
ivl4b-llama-A: llama.cpp:OpenGVLab_InternVL3_5-4B-Q4_K_M.gguf on jetson-orin-nano-super-15w None -- 299 frames, 0 labels flipped since scoring, 0 labelled frames not covered
NT-V512-A: trt-edgellm:InternVL3_5-4B-NT-V512-A int4_awq + visual fp16 on jetson-orin-nano-super-15w None -- 299 frames, 0 labels flipped since scoring, 0 labelled frames not covered
NVE-V1024-A: trt-edgellm:InternVL3_5-4B-NVE-V1024-A int4_awq + visual fp16 on jetson-orin-nano-super-15w None -- 299 frames, 0 labels flipped since scoring, 0 labelled frames not covered
NVE-V1024-B: trt-edgellm:InternVL3_5-4B-NVE-V1024-B int4_awq + visual fp16 on jetson-orin-nano-super-15w None -- 299 frames, 0 labels flipped since scoring, 0 labelled frames not covered
NVE-V512-A: trt-edgellm:InternVL3_5-4B-NVE-V512-A int4_awq + visual fp16 on jetson-orin-nano-super-15w None -- 299 frames, 0 labels flipped since scoring, 0 labelled frames not covered
NVE-V512-B: trt-edgellm:InternVL3_5-4B-NVE-V512-B int4_awq + visual fp16 on jetson-orin-nano-super-15w None -- 299 frames, 0 labels flipped since scoring, 0 labelled frames not covered

| model | gate | precision (TP/YES) | recall (TP/visible) | YES on cloud confabulations | @ 0 FP (fitted) |
|---|---|---|---|---|---|
| cloud (us.anthropic.claude-opus-4-5-20251101-v1:0) | 0.5 | 67.9% (72/106) | 97.3% (72/74) | 34/34 | 0.0% |
| VE | 0.5 | 97.2% (70/72) | 94.6% (70/74) | 1/34 | 40.5% (not separable) |
| ivl4b-llama-A | 0.5 | 82.8% (72/87) | 97.3% (72/74) | 7/34 | 73.0% (not separable) |
| NT-V512-A | 0.5 | 59.2% (74/125) | 100.0% (74/74) | 27/34 | 51.4% (not separable) |
| NVE-V1024-A | 0.5 | 58.3% (74/127) | 100.0% (74/74) | 27/34 | 52.7% (not separable) |
| NVE-V1024-B | 0.5 | 67.8% (40/59) | 54.1% (40/74) | 10/34 | 9.5% |
| NVE-V512-A | 0.5 | 59.2% (74/125) | 100.0% (74/74) | 27/34 | 54.1% (not separable) |
| NVE-V512-B | 0.5 | 64.5% (40/62) | 54.1% (40/74) | 12/34 | 8.1% |
