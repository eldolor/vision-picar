"""
evaluations/tier-tree-394/live.py

3.94 criterion 6: one live tiered mission in `home_first_floor` through the
brain's HTTP API, on the tree. It is 3.80.O's `live.py` (a fresh ROS stack,
`tier_search: explore`, the local tier on, the counting proxy, contacts on
ground truth) with its free stand-ins for the local VLM and the cloud, both
answering from the image they are sent. Perception is NOT blinded here: the
mission runs every tree (what to do, when to ask, who to ask) as the car
would. Free.

    eval "$(python tools/plan_section.py env)"
    .venv/bin/python evaluations/tier-tree-394/live.py --out evaluations/tier-tree-394/live.jsonl
"""

import importlib.util
import os
import sys
import threading
from http.server import ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
# O/live.py imports G/live.py as `live`, so it is loaded under its own name.
_spec = importlib.util.spec_from_file_location("o_live", ROOT / "evaluations/tiered-380/O/live.py")
olive = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(olive)


def main():
    olive.glive.Proxy = olive.TruthProxy
    port = olive.free_port()
    httpd = ThreadingHTTPServer(("127.0.0.1", port), olive.StandinLocal)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    os.environ.update(LOCAL_VLM_URL=f"http://127.0.0.1:{port}")
    os.environ.pop("SIM_PERCEPTION_BLIND", None)
    sys.argv += ["--cloud", "standin", "--only", "home_first_floor"]
    try:
        olive.glive.main()
    finally:
        httpd.shutdown()


if __name__ == "__main__":
    main()
