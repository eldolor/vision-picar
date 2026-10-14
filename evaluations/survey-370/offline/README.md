# 3.70 S2 amendment 1: offline label checks (2026-10-13)

Scratch scripts, kept to re-run, not part of the suite. Run from the repo root
with `PYTHONPATH=.`.

* `trace.py houses|procthor A B out.pkl` -- records every tick's pose and
  true room, and the final map, for sim surveys (no labeller).
* `evalp.py runs.pkl spacing,c0,p ...` -- ask by (weighted) walking distance
  on the final map; `spacing` 0 = ask at every pose (the ceiling).
* `eval2.py` -- extra asks where two answers disagree; `eval3.py` -- lines
  continuing wall ends; `eval4.py` -- a penalty per crossing of the
  watershed cut's rooms (the rule built).
* `diag.py` -- why each room misses (unvisited, unseeded, boundary).
* `variant.py '{"spacing_m": 1.5}' [procthor]` -- the real sweep with one
  labeller setting changed.
