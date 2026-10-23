"""The behavior tree's node kinds, on py_trees (3.94, 3.95).

The tiered policy's decisions are drawn as trees ticked once per frame,
synchronously, by the policy that owns them. Since 3.95 every node is a
py_trees class and py_trees does the ticking; this module only adapts it
to how the policy writes its nodes:

* `Sequence` -- py_trees' `Sequence` without memory: its children in
  order from the first, every tick, stopping at the first that does not
  succeed (an AND).
* `Fallback` -- py_trees' `Selector` without memory: the first child that
  does not fail wins (an OR; "the first rung that has an answer wins").
* `Condition` -- a test of the blackboard: SUCCESS or FAILURE, no effect.
* `Action` -- does something; SUCCESS, FAILURE or RUNNING.
* `Decorator` -- py_trees' `Decorator`: py_trees ticks the child, then
  `after(bb, child_status)` decides the decorator's own status.

The statuses are py_trees' own (`py_trees.common.Status`). A node returning
anything else raises: py_trees would quietly turn it into INVALID.

Nodes carry `settings`, the numbers they act on, so `describe()` can print
the tree with them; where the numbers come from (config, a constant) is the
owning policy's business. The blackboard is any object the policy passes
to `tick(bb)`; every node of the tree is handed it for that tick, reads
and writes it, and keeps no state of its own across ticks unless it says
so.
"""

from typing import Any, Callable, Iterable, Optional

from py_trees import behaviour, composites, decorators
from py_trees.common import Status

SUCCESS, FAILURE, RUNNING = Status.SUCCESS, Status.FAILURE, Status.RUNNING
STATUSES = (SUCCESS, FAILURE, RUNNING)


def _checked(node, status) -> Status:
    if status is True:
        return SUCCESS
    if status is False or status is None:
        return FAILURE
    if status not in STATUSES:
        raise ValueError(f"{node.kind} {node.name!r} returned {status!r}, not a status")
    return status


class _Node:
    """What every node adds to its py_trees class: settings, the
    blackboard for this tick, `tick(bb)`, `describe`, `walk`, `find`."""
    kind = "node"

    def _setup_node(self, settings) -> None:
        # A dict, or a function returning one (read when printed, so a
        # setting changed after construction is shown as it is now).
        self._settings = settings
        self._bb: Any = None

    @property
    def settings(self) -> dict:
        s = self._settings() if callable(self._settings) else self._settings
        return dict(s or {})

    def tick(self, bb: Any = ..., /):
        """With a blackboard: one tick of the tree under this node, `bb`
        handed to every node of it; returns the node's status. Without one:
        py_trees' own generator, which its composites call on their
        children (and `tick_once()` on any node)."""
        if bb is ...:
            return super().tick()
        for n in self.iterate():
            n._bb = bb
        for _ in super().tick():
            pass
        return self.status

    def describe(self, indent: int = 0) -> list:
        """The tree as text, one node a line, with each node's settings."""
        pad = "  " * indent
        s = (" " + ", ".join(f"{k}={v}" for k, v in self.settings.items())) if self.settings else ""
        lines = [f"{pad}{self.kind} {self.name}{s}"]
        for c in self.children:
            lines.extend(c.describe(indent + 1))
        return lines

    def walk(self) -> Iterable["_Node"]:
        return self.iterate()

    def find(self, name: str) -> Optional["_Node"]:
        return next((n for n in self.iterate() if n.name == name), None)


class Sequence(_Node, composites.Sequence):
    kind = "sequence"

    def __init__(self, name: str, children: Iterable, settings=None):
        children = list(children)
        if not children:
            raise ValueError(f"{self.kind} {name!r} has no children")
        composites.Sequence.__init__(self, name=name, memory=False, children=children)
        self._setup_node(settings)



class Fallback(_Node, composites.Selector):
    kind = "fallback"

    def __init__(self, name: str, children: Iterable, settings=None):
        children = list(children)
        if not children:
            raise ValueError(f"{self.kind} {name!r} has no children")
        composites.Selector.__init__(self, name=name, memory=False, children=children)
        self._setup_node(settings)



class Condition(_Node, behaviour.Behaviour):
    kind = "condition"

    def __init__(self, name: str, test: Callable[[Any], bool], settings=None):
        behaviour.Behaviour.__init__(self, name=name)
        self._setup_node(settings)
        self.test = test

    def update(self) -> Status:
        return SUCCESS if self.test(self._bb) else FAILURE



class Action(_Node, behaviour.Behaviour):
    """`act(bb)` returns a status, or a bool (True SUCCESS, False FAILURE)."""
    kind = "action"

    def __init__(self, name: str, act: Callable[[Any], Any], settings=None):
        behaviour.Behaviour.__init__(self, name=name)
        self._setup_node(settings)
        self.act = act

    def update(self) -> Status:
        return _checked(self, self.act(self._bb))



class Decorator(_Node, decorators.Decorator):
    """py_trees ticks `child`; then `after(bb, child_status)` returns this
    node's status (or a bool)."""
    kind = "decorator"

    def __init__(self, name: str, child, after: Callable[[Any, Status], Any], settings=None):
        decorators.Decorator.__init__(self, name=name, child=child)
        self._setup_node(settings)
        self.child = child
        self.after = after

    def update(self) -> Status:
        return _checked(self, self.after(self._bb, self.decorated.status))



Node = _Node
