"""
evaluations/goal-learn-396/model.py -- 3.96 amendment 2: the map crop and
the two scoring networks, each in numpy (what runs in the workers, and on
the Jetson) and in torch (what trains on the instance). `check()` proves
the two agree.

* **The crop:** a 32 x 32 patch of the brain's map at 0.2 m (6.4 m square)
  centred on the candidate, turned so the robot-to-candidate line points
  up (row 0 is the far side). Six channels, uint8: free, occupied, unknown
  (off the map counts as unknown), seen by the camera, the robot's trail,
  and the labelled room's prior (x 255).
* **F:** score = w . f + MLP(f), f the 17 features (MLP 17 -> 32 -> 32 -> 1,
  tanh), 3.65's network on more inputs.
* **MAP:** score = w . f + head([cnn(crop), f]); cnn = conv 6->8 (3x3,
  stride 2, pad 1), ReLU, conv 8->16 (same), ReLU, flatten 1024 -> 32,
  ReLU; head 49 -> 32 -> 1, tanh.
"""
import math

import numpy as np

from brain.frontier import _cell_of
from world.interface import CELL_FREE, CELL_OCCUPIED, CELL_UNKNOWN

CROP, CROP_RES_M, CH = 32, 0.2, 6
N_FEAT = 17
HIDDEN = 32


# ---------- the crop ----------

def layers(m: dict, seen_pts, trail_pts, prior_cells: dict) -> np.ndarray:
    """The map as (CH-1 ... ) float layers, (CH, h, w): free, occupied,
    unknown, seen, trail, prior. `prior_cells`: map cell -> prior."""
    w, h = m["width"], m["height"]
    cells = np.asarray(m["cells"]).reshape(h, w)
    L = np.zeros((CH, h, w), dtype=np.float32)
    L[0] = cells == CELL_FREE
    L[1] = cells == CELL_OCCUPIED
    L[2] = cells == CELL_UNKNOWN
    for pts, k in ((seen_pts, 3), (trail_pts, 4)):
        for x, y in pts:
            i, j = _cell_of(m, x, y)
            if 0 <= i < w and 0 <= j < h:
                L[k, j, i] = 1.0
    for (i, j), p in prior_cells.items():
        if 0 <= i < w and 0 <= j < h:
            L[5, j, i] = p
    return L


_r = (np.arange(CROP) - (CROP - 1) / 2) * CROP_RES_M
_FWD = -_r[:, None] * np.ones((1, CROP))      # row 0 is the far side
_SIDE = np.ones((CROP, 1)) * _r[None, :]


def crops(m: dict, L: np.ndarray, pose: dict, goals: list) -> np.ndarray:
    """(n, CH, CROP, CROP) uint8, one crop per goal (nearest-cell sampling)."""
    h, w = L.shape[1:]
    res, ox, oy = m["resolution_m"], m["origin_x_m"], m["origin_y_m"]
    out = np.zeros((len(goals), CH, CROP, CROP), dtype=np.uint8)
    for n, (gx, gy) in enumerate(goals):
        dx, dy = gx - pose["x_m"], gy - pose["y_m"]
        d = math.hypot(dx, dy)
        ux, uy = (dx / d, dy / d) if d > 1e-6 else (0.0, -1.0)
        px, py = -uy, ux
        X = gx + _FWD * ux + _SIDE * px
        Y = gy + _FWD * uy + _SIDE * py
        i = np.floor((X - ox) / res).astype(int)
        j = np.floor((Y - oy) / res).astype(int)
        ok = (i >= 0) & (i < w) & (j >= 0) & (j < h)
        ic, jc = np.clip(i, 0, w - 1), np.clip(j, 0, h - 1)
        patch = L[:, jc, ic] * ok
        patch[2][~ok] = 1.0                    # off the map: unknown
        out[n] = np.round(patch * 255).astype(np.uint8)
    return out


# ---------- numpy forward ----------

def _conv(x, W, b):
    """x (n, c, H, W), W (o, c, 3, 3), stride 2, pad 1 -> (n, o, H/2, W/2)."""
    n, c, H, Wd = x.shape
    xp = np.pad(x, ((0, 0), (0, 0), (1, 1), (1, 1)))
    Ho, Wo = H // 2, Wd // 2
    cols = np.empty((n, c, 3, 3, Ho, Wo), dtype=x.dtype)
    for a in range(3):
        for bb in range(3):
            cols[:, :, a, bb] = xp[:, :, a:a + 2 * Ho:2, bb:bb + 2 * Wo:2]
    out = np.einsum("ncabhw,ocab->nohw", cols, W, optimize=True)
    return out + b[None, :, None, None]


def score_np(P: dict, kind: str, feats, crop=None) -> np.ndarray:
    f = np.asarray(feats, dtype=np.float32)
    lin = f @ np.asarray(P["w"], dtype=np.float32)
    if kind == "F":
        h = np.tanh(f @ P["W1"].T + P["b1"])
        h = np.tanh(h @ P["W2"].T + P["b2"])
        return lin + (h @ P["W3"].T + P["b3"]).ravel()
    x = crop.astype(np.float32) / 255.0
    x = np.maximum(_conv(x, P["C1"], P["c1"]), 0)
    x = np.maximum(_conv(x, P["C2"], P["c2"]), 0)
    e = np.maximum(x.reshape(len(x), -1) @ P["D"].T + P["d"], 0)
    z = np.concatenate([e, f], axis=1)
    h = np.tanh(z @ P["H1"].T + P["h1"])
    return lin + (h @ P["H2"].T + P["h2"]).ravel()


# ---------- torch ----------

def torch_net(kind: str, w_init=None):
    import torch
    nn = torch.nn

    class FNet(nn.Module):
        def __init__(self):
            super().__init__()
            self.lin = nn.Linear(N_FEAT, 1, bias=False)
            self.l1, self.l2, self.l3 = (nn.Linear(N_FEAT, HIDDEN), nn.Linear(HIDDEN, HIDDEN),
                                         nn.Linear(HIDDEN, 1))

        def forward(self, f, crop=None):
            h = torch.tanh(self.l2(torch.tanh(self.l1(f))))
            return (self.lin(f) + self.l3(h)).squeeze(-1)

        def export(self):
            g = lambda t: t.detach().double().numpy()  # noqa: E731
            return {"w": g(self.lin.weight)[0], "W1": g(self.l1.weight), "b1": g(self.l1.bias),
                    "W2": g(self.l2.weight), "b2": g(self.l2.bias),
                    "W3": g(self.l3.weight), "b3": g(self.l3.bias)}

    class MapNet(nn.Module):
        def __init__(self):
            super().__init__()
            self.lin = nn.Linear(N_FEAT, 1, bias=False)
            self.c1 = nn.Conv2d(CH, 8, 3, stride=2, padding=1)
            self.c2 = nn.Conv2d(8, 16, 3, stride=2, padding=1)
            self.d = nn.Linear(16 * (CROP // 4) ** 2, HIDDEN)
            self.h1, self.h2 = nn.Linear(HIDDEN + N_FEAT, HIDDEN), nn.Linear(HIDDEN, 1)

        def forward(self, f, crop):
            x = crop.float() / 255.0
            x = torch.relu(self.c2(torch.relu(self.c1(x))))
            e = torch.relu(self.d(x.flatten(1)))
            h = torch.tanh(self.h1(torch.cat([e, f], dim=1)))
            return (self.lin(f) + self.h2(h)).squeeze(-1)

        def export(self):
            g = lambda t: t.detach().double().numpy()  # noqa: E731
            return {"w": g(self.lin.weight)[0], "C1": g(self.c1.weight), "c1": g(self.c1.bias),
                    "C2": g(self.c2.weight), "c2": g(self.c2.bias), "D": g(self.d.weight),
                    "d": g(self.d.bias), "H1": g(self.h1.weight), "h1": g(self.h1.bias),
                    "H2": g(self.h2.weight), "h2": g(self.h2.bias)}

    net = FNet() if kind == "F" else MapNet()
    if w_init is not None:
        with torch.no_grad():
            net.lin.weight.copy_(torch.tensor([w_init], dtype=torch.float32))
    return net


def to_np(P: dict) -> dict:
    return {k: np.asarray(v, dtype=np.float32) for k, v in P.items()}


def check(seed: int = 0) -> float:
    """Max |torch - numpy| score difference on random inputs, both kinds."""
    import torch
    torch.manual_seed(seed)
    rng = np.random.default_rng(seed)
    f = rng.random((9, N_FEAT)).astype(np.float32)
    c = rng.integers(0, 256, (9, CH, CROP, CROP)).astype(np.uint8)
    worst = 0.0
    for kind in ("F", "MAP"):
        net = torch_net(kind)
        t = net(torch.tensor(f), torch.tensor(c)).detach().numpy()
        n = score_np(to_np(net.export()), kind, f, c)
        worst = max(worst, float(np.abs(t - n).max()))
    return worst


if __name__ == "__main__":
    print("max |torch - numpy|:", check())
