"""A small behavior tree (3.94).

The tiered policy's decisions are drawn as a tree that is ticked once per
frame, synchronously, by the policy that owns it. Every node returns one of
three statuses: SUCCESS, FAILURE or RUNNING (still at work; the tiered
tree never uses it today, but it is part of the vocabulary so a later node
that spans frames needs no new kind).

* `Sequence` -- its children in order, stopping at the first that does not
  succeed (an AND).
* `Fallback` -- its children in order, stopping at the first that does not
  fail (an OR; "the first rung that has an answer wins").
* `Condition` -- a test of the blackboard: SUCCESS or FAILURE, no effect.
* `Action` -- does something; SUCCESS, FAILURE or RUNNING.
* `Decorator` -- wraps one child and changes how it runs or what it returns.

The semantics are `py_trees`' (sequence, selector, condition, behaviour,
decorator), without its dependency: moving to it later is mechanical.

Nodes carry `settings`, the numbers they act on, so `describe()` can print
the tree with them; where the numbers come from (config, a constant) is the
owning policy's business. The blackboard is any object the policy passes
in; nodes read and write it, and keep no state of their own across ticks
unless they say so.
"""

from typing import Any, Callable, Iterable, Optional

SUCCESS, FAILURE, RUNNING = "success", "failure", "running"
STATUSES = (SUCCESS, FAILURE, RUNNING)


class Node:
    kind = "node"

    def __init__(self, name: str, settings=None):
        self.name = name
        # A dict, or a function returning one (read when printed, so a
        # setting changed after construction is shown as it is now).
        self._settings = settings

    @property
    def settings(self) -> dict:
        s = self._settings() if callable(self._settings) else self._settings
        return dict(s or {})

    def tick(self, bb: Any) -> str:
        raise NotImplementedError

    def children(self) -> list:
        return []

    def describe(self, indent: int = 0) -> list:
        """The tree as text, one node a line, with each node's settings."""
        pad = "  " * indent
        s = (" " + ", ".join(f"{k}={v}" for k, v in self.settings.items())) if self.settings else ""
        lines = [f"{pad}{self.kind} {self.name}{s}"]
        for c in self.children():
            lines.extend(c.describe(indent + 1))
        return lines

    def walk(self) -> Iterable["Node"]:
        yield self
        for c in self.children():
            yield from c.walk()

    def find(self, name: str) -> Optional["Node"]:
        return next((n for n in self.walk() if n.name == name), None)


def _checked(node: Node, status) -> str:
    if status not in STATUSES:
        raise ValueError(f"{node.kind} {node.name!r} returned {status!r}, not a status")
    return status


class Composite(Node):
    def __init__(self, name: str, children: Iterable[Node], settings=None):
        super().__init__(name, settings)
        self._children = list(children)
        if not self._children:
            raise ValueError(f"{self.kind} {name!r} has no children")

    def children(self) -> list:
        return list(self._children)


class Sequence(Composite):
    kind = "sequence"

    def tick(self, bb: Any) -> str:
        for c in self._children:
            status = _checked(c, c.tick(bb))
            if status != SUCCESS:
                return status
        return SUCCESS


class Fallback(Composite):
    kind = "fallback"

    def tick(self, bb: Any) -> str:
        for c in self._children:
            status = _checked(c, c.tick(bb))
            if status != FAILURE:
                return status
        return FAILURE


class Condition(Node):
    kind = "condition"

    def __init__(self, name: str, test: Callable[[Any], bool], settings=None):
        super().__init__(name, settings)
        self.test = test

    def tick(self, bb: Any) -> str:
        return SUCCESS if self.test(bb) else FAILURE


class Action(Node):
    """`act(bb)` returns a status, or a bool (True SUCCESS, False FAILURE)."""
    kind = "action"

    def __init__(self, name: str, act: Callable[[Any], Any], settings=None):
        super().__init__(name, settings)
        self.act = act

    def tick(self, bb: Any) -> str:
        r = self.act(bb)
        if r is True:
            return SUCCESS
        if r is False or r is None:
            return FAILURE
        return _checked(self, r)


class Decorator(Node):
    """`wrap(bb, child)` ticks the child (or not) and returns a status."""
    kind = "decorator"

    def __init__(self, name: str, child: Node, wrap: Callable[[Any, Node], str],
                 settings=None):
        super().__init__(name, settings)
        self.child = child
        self.wrap = wrap

    def tick(self, bb: Any) -> str:
        return _checked(self, self.wrap(bb, self.child))

    def children(self) -> list:
        return [self.child]
