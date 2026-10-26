"""
evaluations/goal-learn-3103/pool.py -- 3.103.C: F-pool, a chooser that scores
the whole pool together, as a drop-in for 3.96's `model.torch_net` /
`model.score_np` (amendment 1 of the 3.103 file fixes the design):

    e = tanh(W1 f + b1)                    B's S64 per-candidate layer, 18 -> 64
    a = MHA(e) over the pool               4 heads of 16, no mask but padding
    h = e + Wo a + bo                      residual; Wo starts at zero
    s = w . f + W3 h + b3                  one score per candidate

Every candidate attends to every candidate of its own pool (itself
included); nothing crosses pools. With Wo = 0 the network is exactly an S64
F-lean, so the clone starts from B's form and learns how much to compare.

Training batches hold many pools concatenated, so the torch net takes the
pools' lengths (`seg`); `score_np` scores one pool, as the mission does.
"""
import math
import os

import numpy as np

# Amendment 2: the width (embedding and attention alike) is swept, 32 / 64 / 128,
# always 4 heads; POOL_WIDTH picks it for training (score_np reads it off P).
N_FEAT, HEADS = 18, 4
WIDTH = int(os.environ.get("POOL_WIDTH", "64"))
HD = WIDTH // HEADS
PAD_CELLS = 1 << 20          # B x L x L per padded group in training (x4 heads x 4 bytes: 16 MB)


def _softmax(z: np.ndarray, axis: int = -1) -> np.ndarray:
    z = z - z.max(axis=axis, keepdims=True)
    e = np.exp(z)
    return e / e.sum(axis=axis, keepdims=True)


def score_np(P: dict, kind: str, feats, crop=None) -> np.ndarray:
    """One pool's scores, numpy only (the board's forward pass)."""
    f = np.asarray(feats, dtype=np.float32)
    n = len(f)
    g = lambda k: np.asarray(P[k], dtype=np.float32)  # noqa: E731
    e = np.tanh(f @ g("W1").T + g("b1"))
    width = e.shape[1]
    hd = width // HEADS
    q, k, v = ((e @ g(f"W{x}").T + g(f"b{x}")).reshape(n, HEADS, hd).transpose(1, 0, 2)
               for x in ("q", "k", "v"))
    att = _softmax(q @ k.transpose(0, 2, 1) / math.sqrt(hd))
    a = (att @ v).transpose(1, 0, 2).reshape(n, width)
    h = e + a @ g("Wo").T + g("bo")
    return f @ g("w") + (h @ g("W3").T + g("b3")).ravel()


def torch_net(kind: str = "F", w_init=None):
    import torch
    nn = torch.nn

    class Net(nn.Module):
        def __init__(self):
            super().__init__()
            self.lin = nn.Linear(N_FEAT, 1, bias=False)
            self.embed = nn.Linear(N_FEAT, WIDTH)
            self.q, self.k, self.v = (nn.Linear(WIDTH, WIDTH) for _ in range(3))
            self.o = nn.Linear(WIDTH, WIDTH)
            nn.init.zeros_(self.o.weight)
            nn.init.zeros_(self.o.bias)
            self.out = nn.Linear(WIDTH, 1)

        def _attend(self, e, seg):
            """Attention for pools of lengths `seg` whose rows are e, in order,
            padded to the longest of them."""
            B, L = len(seg), max(seg)
            valid = torch.arange(L)[None, :] < torch.tensor(seg)[:, None]  # B x L
            E = e.new_zeros(B, L, WIDTH)
            E[valid] = e
            split = lambda x: x.view(B, L, HEADS, HD).transpose(1, 2)  # noqa: E731
            Q, K, V = split(self.q(E)), split(self.k(E)), split(self.v(E))
            z = Q @ K.transpose(-1, -2) / math.sqrt(HD)                 # B x H x L x L
            z = z.masked_fill(~valid[:, None, None, :], float("-inf"))
            return (torch.softmax(z, dim=-1) @ V).transpose(1, 2).reshape(B, L, WIDTH)[valid]

        def forward(self, f, crop=None, seg=None):
            """f: candidates of one pool, or of several concatenated with
            their lengths in `seg`; returns one score per row of f. Pools
            are padded in groups of like length, at most PAD_CELLS of B x L
            x L each, so a few long pools never pad thousands of short ones."""
            seg = [len(f)] if seg is None else list(seg)
            e = torch.tanh(self.embed(f))
            if len(seg) == 1:
                a = self._attend(e, seg)
            else:
                start = np.concatenate([[0], np.cumsum(seg)[:-1]])
                order = sorted(range(len(seg)), key=lambda q: seg[q])
                a = e.new_zeros(len(e), WIDTH)
                g = 0
                while g < len(order):
                    h = g + 1
                    while h < len(order) and (h + 1 - g) * seg[order[h]] ** 2 <= PAD_CELLS:
                        h += 1
                    group = order[g:h]
                    rows = torch.from_numpy(np.concatenate(
                        [np.arange(start[q], start[q] + seg[q]) for q in group]))
                    a = a.index_copy(0, rows, self._attend(e[rows], [seg[q] for q in group]))
                    g = h
            h = e + self.o(a)
            return (self.lin(f) + self.out(h)).squeeze(-1)

        def export(self):
            g = lambda t: t.detach().double().numpy()  # noqa: E731
            P = {"w": g(self.lin.weight)[0], "W1": g(self.embed.weight), "b1": g(self.embed.bias),
                 "W3": g(self.out.weight), "b3": g(self.out.bias)}
            for x in ("q", "k", "v", "o"):
                lay = getattr(self, x)
                P[f"W{x}"], P[f"b{x}"] = g(lay.weight), g(lay.bias)
            return P

    net = Net()
    if w_init is not None:
        with torch.no_grad():
            net.lin.weight.copy_(torch.tensor([w_init], dtype=torch.float32))
    return net


def size(width: int = WIDTH) -> int:
    return N_FEAT + (N_FEAT * width + width) + 4 * (width * width + width) + width + 1


def check(seed: int = 0) -> dict:
    """Max |torch - numpy| on random pools with a non-zero Wo, and that a
    batch of pools scores each pool exactly as it scores alone, and that a
    pool's scores follow its candidates when they are reordered."""
    import torch
    torch.manual_seed(seed)
    rng = np.random.default_rng(seed)
    net = torch_net()
    with torch.no_grad():
        net.o.weight.normal_(0, 0.3)
        net.o.bias.normal_(0, 0.1)
    P = {k: np.asarray(v, dtype=np.float32) for k, v in net.export().items()}
    pools = [rng.random((n, N_FEAT)).astype(np.float32) for n in [1, 7, 85, 3] + [2] * 300]
    with torch.no_grad():
        batch = net(torch.tensor(np.concatenate(pools)), seg=[len(p) for p in pools]).numpy()
        alone = np.concatenate([net(torch.tensor(p)).numpy() for p in pools])
    npy = np.concatenate([score_np(P, "F", p) for p in pools])
    perm = rng.permutation(len(pools[2]))
    perm_err = np.abs(score_np(P, "F", pools[2][perm]) - score_np(P, "F", pools[2])[perm]).max()
    return {"numbers": size(), "torch_vs_numpy": float(np.abs(batch - npy).max()),
            "batch_vs_alone": float(np.abs(batch - alone).max()), "permutation": float(perm_err)}


if __name__ == "__main__":
    print(check())
