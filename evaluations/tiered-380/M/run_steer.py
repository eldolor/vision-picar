"""
evaluations/tiered-380/M/run_steer.py

3.80.M steering: `J/run_llama.py` as is (the service's identity-box and
direction prompts, 3.80.I's 142 frames each also mirrored), with the
identity score read by `readers.p_box` and the box by `readers.parse_any_box`,
and the frame's size kept beside the raw box so `decide_scale.py` can read
the box in the model's own scale. Writes `M/<name>.jsonl`.

    .venv/bin/python evaluations/tiered-380/M/run_steer.py <name> [server-url]
"""

import io
import sys
from pathlib import Path

from PIL import Image

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "J"))
import readers                                              # noqa: E402
import run_llama as rl                                      # noqa: E402

rl.HERE = HERE
rl.p_localised = readers.p_box
rl.parse_box = readers.parse_any_box
_answer = rl.answer


def answer(c, url, jpeg, target):
    row = _answer(c, url, jpeg, target)
    row["img"] = list(Image.open(io.BytesIO(jpeg)).size)
    return row


rl.answer = answer

if __name__ == "__main__":
    rl.main(*sys.argv[1:])
