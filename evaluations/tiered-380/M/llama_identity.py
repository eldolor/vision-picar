"""
evaluations/tiered-380/M/llama_identity.py

3.80.M identity: `tools.jetson.bench_identity_llama` on the pinned 299
frames with the prompt (`--arm A`, 3.48's JSON prompt; `--arm B`, 3.80.L's
list prompt) and the reader (`readers.p_box`, `readers.decided`) swapped, as
`L/rb_identity.py` swaps them for Edge-LLM. Run with `--stop-at-decision` and
`--server` pointing at `srvwrap.sh` (the image-token bounds, oom adj 1000).

    python evaluations/tiered-380/M/llama_identity.py --arm A <bench args...>
"""

import sys
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.resolve().parents[2]))
import readers                                              # noqa: E402
import rb_identity as rb                                    # noqa: E402  (L/, on readers' path)
import tools.jetson.bench_identity_llama as bench           # noqa: E402

JSON_PROMPT = bench.PROMPT


def main(argv):
    arm = argv[argv.index("--arm") + 1]
    rest = argv[:argv.index("--arm")] + argv[argv.index("--arm") + 2:]
    bench.PROMPT = rb.LIST_PROMPT if arm == "B" else JSON_PROMPT
    bench.p_localised = readers.p_box
    bench.decision_reached = readers.decided
    return bench.main(rest)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
