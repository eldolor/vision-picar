"""
Makes app.py/vision_core.py/rooms_core.py importable as the flat modules
they are (`from vision_core import ...`, matching the Dockerfile's
COPY app.py vision_core.py rooms_core.py . into one flat /app WORKDIR --
see app.py's own docstring). Run from anywhere with:

    pytest service/vision_analyze/tests/ -q

Deliberately not under tests/ at the repo root: this service was built
with dependency-light copies of vision_core.py/rooms_core.py specifically
so its Docker build context stays self-contained (CLAUDE.md section 6),
and its tests follow the same boundary rather than reaching into the
top-level tests/ package.
"""

import sys
from pathlib import Path

_SERVICE_DIR = Path(__file__).resolve().parent.parent
if str(_SERVICE_DIR) not in sys.path:
    sys.path.insert(0, str(_SERVICE_DIR))
