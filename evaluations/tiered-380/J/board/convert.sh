#!/bin/bash
# 3.80.J: RoboBrain2.0-3B -> GGUF the way the board's Qwen2.5-VL-3B files are: Q4_K_M LLM, Q8_0 projector.
pkill -x llama-server; sleep 2
cd ~/bench-348/llama.cpp || exit 1
M=~/models-380J; P=~/vp-380/.venv/bin/python; export PYTHONPATH=$PWD/gguf-py:$HOME/out-380J/pylib
{ time $P convert_hf_to_gguf.py $M/RoboBrain2.0-3B --outtype f16 --outfile $M/RoboBrain2.0-3B-F16.gguf; } > ~/out-380J/convert.log 2>&1
{ time build/bin/llama-quantize $M/RoboBrain2.0-3B-F16.gguf $M/RoboBrain2.0-3B-Q4_K_M.gguf Q4_K_M; } > ~/out-380J/quantize.log 2>&1
echo convert-done > ~/out-380J/convert.done
