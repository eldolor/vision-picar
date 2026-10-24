"""3.80.E: pilot.py's stand-in cloud as its own server, $0 -- every /navigate
answered "not visible, RIGHT" -- logging each /navigate request as one JSON line, so a
board run (tools/jetson/headroom_mission.sh CLOUD=standin) can count the
escalations that reached the cloud and when.

    python evaluations/tiered-380/E/standin_cloud.py <port> <calls.jsonl>
"""

import json
import sys
import time
from http.server import ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from pilot import Proxy  # noqa: E402


def main():
    port, log = int(sys.argv[1]), sys.argv[2]
    out = open(log, "a", buffering=1)

    class Logged(Proxy):
        mode = "standin"

        def do_POST(self):
            if self.path == "/navigate":
                out.write(json.dumps({"t": round(time.time(), 3), "path": self.path}) + "\n")
            super().do_POST()

    ThreadingHTTPServer(("127.0.0.1", port), Logged).serve_forever()


if __name__ == "__main__":
    main()
