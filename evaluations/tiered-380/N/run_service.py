"""
evaluations/tiered-380/N/run_service.py

3.80.N steering: `I/run_qwen.py` through the board's local VLM service
(serving an InternVL engine set), with each frame's size kept beside the raw
box so `M/decide_scale.py` reads the box in InternVL's scale. Writes
`N/<name>.jsonl`.

    .venv/bin/python evaluations/tiered-380/N/run_service.py <name> [service-url]
"""

import io
import sys
from pathlib import Path

from PIL import Image

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent / "I"))
import run_qwen as rq                                       # noqa: E402

_answer = rq.answer


def answer(c, url, jpeg, target):
    row = _answer(c, url, jpeg, target)
    row["img"] = list(Image.open(io.BytesIO(jpeg)).size)
    return row


def main(name, url="http://127.0.0.1:18005"):
    rq.answer = answer
    rq.HERE = HERE
    out = HERE / f"{name}.jsonl"
    # run_qwen names its file qwen-<engine>.jsonl in its HERE.
    rq.main(name, url)
    src = HERE / f"qwen-{name}.jsonl"
    if src.exists():
        src.rename(out)


if __name__ == "__main__":
    main(*sys.argv[1:])
