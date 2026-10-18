"""
evaluations/tiered-380/R/identity.py

3.80.R identity: `tools.jetson.bench_identity_llama` on the pinned 299 frames
with the prompt and the reader swapped, as `M/llama_identity.py` does:
* `--arm A`: 3.48's JSON prompt (the service's), read by `readers.p_box`;
* `--arm N --native <backend>:<skill>`: the model's native call from
  `prompts.json`, read by `readers.p_native`.
`--server` is llama-server (a GGUF) or `nsrv.sh` (`native_server.py`). On a
machine without /proc (the Mac rows) memory reads 0: those rows are never
judged on R3.

    python evaluations/tiered-380/R/identity.py --arm N --native locateanything:box <bench args...>
"""

import json
import sys
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.resolve().parents[2]))
import readers                                              # noqa: E402
import tools.jetson.bench_identity_llama as bench           # noqa: E402

JSON_PROMPT = bench.PROMPT


def _pop(argv, flag):
    if flag not in argv:
        return argv, None
    i = argv.index(flag)
    return argv[:i] + argv[i + 2:], argv[i + 1]


def main(argv):
    argv, arm = _pop(list(argv), "--arm")
    argv, native = _pop(argv, "--native")
    if arm == "N":
        backend, skill = native.split(":")
        bench.PROMPT = json.loads((HERE / "prompts.json").read_text())["native"][backend][skill]
        bench.p_localised = readers.p_native
        bench.decision_reached = readers.decided_native
    else:
        bench.PROMPT = JSON_PROMPT
        bench.p_localised = readers.p_box
        bench.decision_reached = readers.decided
    if not Path("/proc/meminfo").exists():
        bench.mem_available_mb = lambda: 0.0
    return bench.main(argv)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
