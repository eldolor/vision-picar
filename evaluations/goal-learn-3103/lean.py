"""
evaluations/goal-learn-3103/lean.py -- 3.103.B: the networks of four shapes
on F-lean's 18 facts (3.96's 17 plus 3.101's `lidar_gain`), as drop-ins for
3.96's `model.torch_net` / `model.score_np`.

* `lidar_gain`: unknown cells visible from the goal within RAY_M by RAYS
  rays, an occupied cell ending a ray -- 3.101's `rich.py` computation, the
  one fact of its 25 that clearly mattered, here alone (no other rich fact
  is computed).
* Shapes: `w . f + MLP(f)`, the MLP's hidden widths from SHAPES (tanh), the
  same form as 3.96's F network, which is S32x32.
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "goal-learn-3101"))
sys.path.insert(0, os.path.join(HERE, "..", "goal-learn-396"))
from features import coarse                                   # noqa: E402
from rich import RAY_M, RICH_FEATURES, _DIRS                  # noqa: E402
from world.interface import CELL_OCCUPIED, CELL_UNKNOWN       # noqa: E402

SHAPES = {"S32x32": (32, 32), "S64x32": (64, 32), "S32": (32,), "S64": (64,)}
N_FEAT = 18
# 3.101's oracle shards hold 42 columns: 3.96's 17, then RICH_FEATURES.
LEAN_COLS = list(range(17)) + [17 + RICH_FEATURES.index("lidar_gain")]


def lean_features(agent, m: dict, pose: dict, pool: list, feats: list, trail: list,
                  now: float) -> list:
    """3.96's EXTRA_FEATURES hook: one column, `lidar_gain`, cast on the map
    at the training grid's 0.30 m (3.96's `coarse`, as its room facts are):
    the simulator's maps already are, so there it is 3.101's computation
    exactly; a 0.05 m SLAM map is resampled first, which cuts the rays' steps
    36-fold and reads the map as training did."""
    m = coarse(m)
    w, h, res = m["width"], m["height"], m["resolution_m"]
    cells = np.asarray(m["cells"]).reshape(h, w)
    unknown = cells == CELL_UNKNOWN
    occ = cells == CELL_OCCUPIED
    goals = np.asarray([c.goal for _, c, _ in pool], dtype=float)
    steps = np.arange(1, int(RAY_M / res) + 1) * res
    PX = goals[:, None, None, 0] + _DIRS[None, :, None, 0] * steps[None, None, :]
    PY = goals[:, None, None, 1] + _DIRS[None, :, None, 1] * steps[None, None, :]
    I = np.floor((PX - m["origin_x_m"]) / res).astype(int)
    J = np.floor((PY - m["origin_y_m"]) / res).astype(int)
    on = (I >= 0) & (I < w) & (J >= 0) & (J < h)
    Ic, Jc = np.clip(I, 0, w - 1), np.clip(J, 0, h - 1)
    blocked = occ[Jc, Ic] & on
    visible = (np.cumsum(blocked, axis=2) - blocked == 0) & on
    total = visible.shape[1] * visible.shape[2]
    gain = (visible & unknown[Jc, Ic]).sum(axis=(1, 2)) / total
    return [[float(g)] for g in gain]


def score_np(P: dict, kind: str, feats, crop=None) -> np.ndarray:
    """Any shape: one hidden layer (W1) or two (W1, W2), then W3."""
    f = np.asarray(feats, dtype=np.float32)
    out = f @ np.asarray(P["w"], dtype=np.float32)
    h = np.tanh(f @ np.asarray(P["W1"]).T + P["b1"])
    if "W2" in P:
        h = np.tanh(h @ np.asarray(P["W2"]).T + P["b2"])
    return out + (h @ np.asarray(P["W3"]).T + P["b3"]).ravel()


def make_torch_net(shape: str):
    widths = SHAPES[shape]

    def torch_net(kind: str = "F", w_init=None):
        import torch
        nn = torch.nn

        class Net(nn.Module):
            def __init__(self):
                super().__init__()
                self.lin = nn.Linear(N_FEAT, 1, bias=False)
                dims = (N_FEAT,) + widths
                self.hidden = nn.ModuleList(nn.Linear(a, b) for a, b in zip(dims, dims[1:]))
                self.out = nn.Linear(widths[-1], 1)

            def forward(self, f, crop=None):
                h = f
                for layer in self.hidden:
                    h = torch.tanh(layer(h))
                return (self.lin(f) + self.out(h)).squeeze(-1)

            def export(self):
                g = lambda t: t.detach().double().numpy()  # noqa: E731
                P = {"w": g(self.lin.weight)[0], "W3": g(self.out.weight), "b3": g(self.out.bias)}
                for k, layer in enumerate(self.hidden, 1):
                    P[f"W{k}"], P[f"b{k}"] = g(layer.weight), g(layer.bias)
                return P

        net = Net()
        if w_init is not None:
            with torch.no_grad():
                net.lin.weight.copy_(torch.tensor([w_init], dtype=torch.float32))
        return net

    return torch_net


def size(shape: str) -> int:
    dims = (N_FEAT,) + SHAPES[shape] + (1,)
    return N_FEAT + sum(a * b + b for a, b in zip(dims, dims[1:]))


def check(seed: int = 0) -> float:
    """Max |torch - numpy| over every shape, random inputs."""
    import torch
    torch.manual_seed(seed)
    f = np.random.default_rng(seed).random((9, N_FEAT)).astype(np.float32)
    worst = 0.0
    for s in SHAPES:
        net = make_torch_net(s)()
        t = net(torch.tensor(f)).detach().numpy()
        P = {k: np.asarray(v, dtype=np.float32) for k, v in net.export().items()}
        worst = max(worst, float(np.abs(t - score_np(P, "F", f)).max()))
    return worst


if __name__ == "__main__":
    print({s: size(s) for s in SHAPES}, "max |torch - numpy|:", check())
