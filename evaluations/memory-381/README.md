# 3.81 memory accounting records

`docs/plans/ros-alignment/3.81-rover-memory-budget.md` is the reading of
these; this file says what each one is.

* `A1/` -- step A, 2026-10-14: 3.78.F's loaded window under VE with
  `ACCOUNT=1` (`tools/jetson/headroom_identity.sh`).
  * `account.jsonl`: one sample every 5 s from
    `tools/jetson/memory_account.py sample` (meminfo, nvmap per process and
    its page pool, zram, every process's smaps_rollup), tagged by phase.
    The `idle` phase also covers the stack's start and mapping (the phase
    file changes to `stack` only once mapped); its first samples are the
    true idle board, and the `stack` phase's last samples cover perception loading.
  * `tables.md`: `memory_account.py table` at the window's and the
    perception phase's lowest `MemAvailable`.
  * `torch.jsonl`: perception's torch allocator, every 20th frame.
  * `cuda_probe.jsonl`: `tools/jetson/cuda_context_probe.py`, the cost of
    a bare CUDA context, torch, and torch + cuDNN, on the quiet board.
  * `perception_probe.jsonl`: `tools/jetson/perception_memory_probe.py`.
  * The rest is the harness's usual output (3.78.F's): `summary.json`,
    `loaded.json`, `mem.log`, `tegrastats.log`, `perception.log`,
    `nav.log`, `wheel.log`, `run.log`.
* The void first attempt (weights downloaded inside the window) stays on
  the board as `~/out-381/A1-invalid-download`, not copied.
* `C1/`: 3.81.C, A1's window with T1-T3 (NOTE.md says which files ran); `tables.md` at the window's minimum.
