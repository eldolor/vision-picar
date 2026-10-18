"""
evaluations/tiered-380/R/steer.py

3.80.R steering: `M/run_steer.py` (3.80.I's 142 frames, each also mirrored:
the identity-box question and the direction question) with the box question
swapped for the model's native call when `--native <backend>:<skill>` is
given, and both readers swapped for R's. Writes `R/<name>.jsonl`.

    .venv/bin/python evaluations/tiered-380/R/steer.py <name> [server-url] [--native b:s]
"""

import json
import sys
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent / "M"))
sys.path.insert(0, str(HERE))
import readers                                              # noqa: E402  (R's: first on the path)
import run_steer as ms                                      # noqa: E402
assert hasattr(readers, "p_native"), "M's readers shadowed R's"

rl = ms.rl
rl.HERE = HERE
rl.parse_box = readers.parse_any_box
rl.p_localised = readers.p_box

if __name__ == "__main__":
    args = sys.argv[1:]
    if "--native" in args:
        i = args.index("--native")
        backend, skill = args[i + 1].split(":")
        args = args[:i] + args[i + 2:]
        rl.PROMPT = json.loads((HERE / "prompts.json").read_text())["native"][backend][skill]
        rl.p_localised = readers.p_native
    rl.main(*args)
