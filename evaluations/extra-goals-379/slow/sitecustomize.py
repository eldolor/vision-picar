"""3.79 arm S: slow explore's goal choice back to its pre-3.77 laptop time.

Loaded by the brain process when this directory is on PYTHONPATH (see
run.py). Adds PICAR_SLOW_CHOICE_MS of sleep inside every `find_frontiers`
call -- after the pose and map are fetched, before the decision -- and
changes nothing else. Only in the brain server; never product code."""
import os
import sys

_MS = float(os.environ.get("PICAR_SLOW_CHOICE_MS") or 0)
if _MS > 0 and any("control.brain_server" in a for a in sys.argv):
    import time
    sys.path.insert(0, os.getcwd())
    import brain.frontier as _f

    _orig = _f.find_frontiers

    def find_frontiers(*a, **kw):
        out = _orig(*a, **kw)
        time.sleep(_MS / 1000)
        return out

    _f.find_frontiers = find_frontiers
    sys.stderr.write(f"3.79 arm S: find_frontiers slowed by {_MS:.0f} ms\n")
