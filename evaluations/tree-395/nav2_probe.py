"""
evaluations/tree-395/nav2_probe.py

3.95, for the user's question "are there open-source trees we could load on
py_trees?": loads every behavior tree nav2 ships (BehaviorTree.CPP XML, read
from our ROS image) with py_trees 2.6's XML parser. Each nav2 tag is given a
stand-in class: a leaf that succeeds, or a py_trees Sequence, Selector or
Decorator for nav2's control nodes, with each `{key}` attribute declared as
a port. It tests the parse only: nothing here runs nav2's behaviour. Free.

    mkdir -p /tmp/nav2bt && docker run --rm --entrypoint bash vision-picar-ros:latest \\
        -c 'tar c -C /opt/ros/humble/share/nav2_bt_navigator behavior_trees' | tar x -C /tmp/nav2bt
    cd /tmp/nav2bt && .venv/bin/python <repo>/evaluations/tree-395/nav2_probe.py
"""
import glob, re, sys
from py_trees.parsers.behaviour_tree_xml import parse_behaviour_tree_xml
from py_trees import composites, common, ports, decorators, display

class P:
    INPUT_PORTS = {}
    OUTPUT_PORTS = {}
class Leaf(P, ports.BehaviourWithPorts):
    def __init__(self, name="leaf", **kw):
        super().__init__(name=name); self.kw = kw
    def update(self): return common.Status.SUCCESS
class Seq(P, ports.PortsMixin, composites.Sequence):
    def __init__(self, name="seq", children=None, **kw):
        composites.Sequence.__init__(self, name=name, memory=False, children=children)
class Sel(P, ports.PortsMixin, composites.Selector):
    def __init__(self, name="sel", children=None, **kw):
        composites.Selector.__init__(self, name=name, memory=False, children=children)
class Dec(P, ports.PortsMixin, decorators.Decorator):
    def __init__(self, name="dec", child=None, **kw):
        decorators.Decorator.__init__(self, name=name, child=child)
    def update(self): return self.decorated.status

files = sorted(glob.glob("behavior_trees/*.xml"))
tags=set()
for f in files:
    tags |= set(re.findall(r"<([A-Z][A-Za-z]+)", open(f).read()))
tags -= {"BehaviorTree"}
import collections
curly = collections.defaultdict(set)
for f in files:
    for tag, attrs in re.findall(r"<([A-Z][A-Za-z]+)([^>]*)>", open(f).read()):
        for k, v in re.findall(r'(\w+)="([^"]*)"', attrs):
            if v.startswith("{"):
                curly[tag].add(k)
def leaf_for(tag):
    return type(tag, (Leaf,), {"INPUT_PORTS": {k: ports.PortInformation(data_type=object, required=False) for k in curly[tag]},
                               "OUTPUT_PORTS": {}})
reg = {t: leaf_for(t) for t in tags}
for t in ("Sequence","PipelineSequence","ReactiveSequence","SequenceStar"): reg[t]=Seq
for t in ("ReactiveFallback","Fallback","RoundRobin","RecoveryNode"): reg[t]=Sel
for t in ("RateController","DistanceController","SpeedController","GoalUpdater","Inverter","KeepRunningUntilFailure","Repeat","RetryUntilSuccessful","PathLongerOnApproach","GoalUpdatedController"):
    reg[t] = type(t, (Dec,), {"INPUT_PORTS": {k: ports.PortInformation(data_type=object, required=False) for k in curly[t]}, "OUTPUT_PORTS": {}})
ok=0
for f in files:
    try:
        root = parse_behaviour_tree_xml(f, node_registry=reg)
        n = sum(1 for _ in root.iterate()); ok+=1
        print("OK  ", f.split('/')[-1], n, "nodes")
        if "to_pose_w_replanning_and_recovery" in f: shown = root
    except Exception as e:
        print("FAIL", f.split('/')[-1], type(e).__name__, str(e)[:220])
print(ok, "of", len(files))

