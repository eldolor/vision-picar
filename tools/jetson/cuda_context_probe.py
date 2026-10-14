"""3.81.A: what a GPU process costs before it loads any weights, on the board.

  sudo .venv/bin/python tools/jetson/cuda_context_probe.py <stage>

Stages, each in a fresh process, each adding to the one before:
  python  -- the interpreter alone
  cudart  -- libcudart through ctypes, then cudaFree(0): a bare CUDA context
  torch   -- import torch, one CUDA tensor (torch's CUDA context and libraries)
  cudnn   -- plus one conv and one matmul (cuDNN and cuBLAS loaded and initialised)

Prints one JSON line: the process's own smaps_rollup (PSS split) and its nvmap
total (the GPU allocations, read from debugfs, hence root). The difference
between stages is what each layer costs; perception and the VLM driver each
pay the context and its libraries once.
"""
import ctypes
import json
import os
import re
import sys
import time
from pathlib import Path


def own_memory() -> dict:
    out = {}
    for line in Path("/proc/self/smaps_rollup").read_text().splitlines()[1:]:
        k, _, v = line.partition(":")
        if k in ("Rss", "Pss", "Pss_Anon", "Pss_File", "Pss_Shmem"):
            out[k + "_mb"] = round(int(v.split()[0]) / 1024, 1)
    gpu = 0
    for line in Path("/sys/kernel/debug/nvmap/iovmm/clients").read_text().splitlines():
        m = re.search(r"\s(\d+)\s+(\d+)K\s*$", line)
        if m and m.group(1) == str(os.getpid()):
            gpu += int(m.group(2))
    out["gpu_mb"] = round(gpu / 1024, 1)
    return out


def main(stage: str) -> int:
    if stage == "cudart":
        lib = None
        for name in ("libcudart.so", "libcudart.so.12"):
            for d in ("/usr/local/cuda/lib64", "/usr/local/cuda/targets/aarch64-linux/lib", ""):
                try:
                    lib = ctypes.CDLL(os.path.join(d, name) if d else name)
                    break
                except OSError:
                    continue
            if lib:
                break
        if lib is None:
            raise SystemExit("libcudart not found")
        err = lib.cudaFree(ctypes.c_void_p(0))  # creates the context; never inside an assert
        if err != 0:
            raise SystemExit(f"cudaFree(0) failed: {err}")
    elif stage in ("torch", "cudnn"):
        import torch
        x = torch.zeros(1, device="cuda")
        if stage == "cudnn":
            conv = torch.nn.Conv2d(3, 16, 3).cuda()
            y = conv(torch.zeros(1, 3, 64, 64, device="cuda"))
            z = torch.randn(256, 256, device="cuda") @ torch.randn(256, 256, device="cuda")
            torch.cuda.synchronize()
            del y, z
        del x
    elif stage != "python":
        raise SystemExit(f"unknown stage {stage}")
    time.sleep(1)
    print(json.dumps({"stage": stage, **own_memory()}))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
