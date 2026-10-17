"""
evaluations/tiered-380/N/trt_identity.py

3.80.N identity: `tools.jetson.bench_identity_trt` (the Edge-LLM driver) on the
pinned 299 frames with the prompt (`--arm A`, 3.48's JSON prompt; `--arm B`,
3.80.L's list prompt) and the reader (3.80.M's `readers.p_box`, `decided`)
swapped, as `L/rb_identity.py` swaps them.

    python evaluations/tiered-380/N/trt_identity.py --arm A <bench_identity_trt args...>
"""

import sys
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent / "M"))
sys.path.insert(0, str(HERE.resolve().parents[2]))
import readers                                              # noqa: E402
import rb_identity as rb                                    # noqa: E402  (L/, on readers' path)
import tools.jetson.bench_identity_trt as trt               # noqa: E402

JSON_PROMPT = trt.PROMPT


def main(argv):
    arm = argv[argv.index("--arm") + 1]
    rest = argv[:argv.index("--arm")] + argv[argv.index("--arm") + 2:]
    trt.PROMPT = rb.LIST_PROMPT if arm == "B" else JSON_PROMPT
    trt.p_localised = readers.p_box
    trt.decision_reached = readers.decided
    return trt.main(rest)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
