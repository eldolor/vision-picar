import sys, json, statistics as st
import brain.room_labels as rl
import control.mission_runner as mr
kw = json.loads(sys.argv[1])
for k in ("RESEGMENT_S",):
    if k in kw: setattr(rl, k, kw.pop(k))
class V(rl.RoomLabeller):
    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        for n, v in kw.items(): setattr(self, n, v)
mr.RoomLabeller = V
from tests import survey_sweep as ss
target = sys.argv[2] if len(sys.argv) > 2 else "houses"
if target == "houses":
    rs = [ss.run(h, s0, mode="spacing") for h in ss.HOUSES for s0 in ss.starts(h)]
else:
    from sim.maps.procthor import usable
    rs = [ss.run(f"procthor:val:{i}", mode="spacing") for i in usable()[200:300]]
per = [v for r in rs for v in r["per_room"].values()]; c = [r["label_calls"] for r in rs]
print(sys.argv[1], target, "exh", sum(r["outcome"] == "exhausted" for r in rs), "calls med", st.median(c), "p90", sorted(c)[int(.9*len(c))], "max", max(c),
      "good", sum(v >= .95 for v in per), "/", len(per), "room med", st.median(per), "wd med", st.median(r["wrong_deep"] for r in rs), flush=True)
