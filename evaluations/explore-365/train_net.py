"""
evaluations/explore-365/train_net.py -- 3.65 phase 2, amendment 2: train the
fallback network (`score = w_lin . f + MLP(f)`, MLP 10 -> 32 -> 32 -> 1) by
PPO on the TRAINING houses (usable 300-569), choosing between iterations on
the SELECTION houses (usable 100-199). The test houses are never touched here.

Each iteration: HOUSES training houses, ROLLOUTS missions each, the goal
drawn from softmax(scores) over explore's pool at every choice. A mission's
advantage is its reward minus its house's mean that iteration, over the
house's std (group-relative, no critic), shared by every choice it made.
Then EPOCHS passes of PPO's clipped objective over every choice. Every
SELECT_EVERY iterations the greedy policy (highest score) is run on the
selection houses; the best is written in `brain/explore_goal_net.json`'s
format.

    python evaluations/explore-365/train_net.py OUT.json
"""
import json
import math
import os
import random
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, ".")
from harness import run_house                                    # noqa: E402
from brain.goal_policy import FEATURES, LinearGoalPolicy, NetGoalPolicy  # noqa: E402
from sim.maps.procthor import usable                             # noqa: E402

SEED = 3653
HOUSES, ROLLOUTS, ITERS, EPOCHS = 16, 8, 80, 4
LR, CLIP, ENTROPY = 1e-3, 0.2, 0.01
HIDDEN = 32
SELECT_EVERY = 10
WORKERS = max(1, (os.cpu_count() or 4) - 2)


class Net(torch.nn.Module):
    def __init__(self, w_lin):
        super().__init__()
        n = len(FEATURES)
        self.lin = torch.nn.Linear(n, 1, bias=False)
        self.mlp = torch.nn.Sequential(torch.nn.Linear(n, HIDDEN), torch.nn.Tanh(),
                                       torch.nn.Linear(HIDDEN, HIDDEN), torch.nn.Tanh(),
                                       torch.nn.Linear(HIDDEN, 1))
        with torch.no_grad():
            self.lin.weight.copy_(torch.tensor([w_lin], dtype=torch.float32))
            self.mlp[-1].weight.zero_()
            self.mlp[-1].bias.zero_()

    def forward(self, x):
        return (self.lin(x) + self.mlp(x)).squeeze(-1)

    def export(self) -> dict:
        """`brain/explore_goal_net.json`'s format."""
        layers = [m for m in self.mlp if isinstance(m, torch.nn.Linear)]
        return {"features": list(FEATURES),
                "weights": [round(float(x), 5) for x in self.lin.weight.detach()[0]],
                "layers": [{"W": m.weight.detach().double().numpy().round(5).tolist(),
                            "b": m.bias.detach().double().numpy().round(5).tolist()}
                           for m in layers]}


class Sampler(NetGoalPolicy):
    """Draws the goal from softmax(scores) and records each choice."""

    def __init__(self, params, seed):
        super().__init__(params["weights"], [(l["W"], l["b"]) for l in params["layers"]])
        self.rng = random.Random(seed)
        self.choices = []

    def best(self, candidates):
        if not candidates:
            return None
        s = np.array([self.score(f) for f in candidates])
        p = np.exp(s - s.max())
        p /= p.sum()
        i = int(min(np.searchsorted(np.cumsum(p), self.rng.random()), len(p) - 1))
        self.choices.append((candidates, i, float(math.log(max(p[i], 1e-12)))))
        return i


def _rollout(args):
    house, params, seed = args
    pol = Sampler(params, seed)
    row = run_house(house, policy=pol)
    return {"house": house, "reward": row["reward"], "outcome": row["outcome"],
            "choices": pol.choices}


def _greedy(args):
    house, params = args
    return run_house(house, policy=NetGoalPolicy(params["weights"],
                                                 [(l["W"], l["b"]) for l in params["layers"]]))


def ppo_update(net, opt, missions):
    """Clipped PPO over every choice, each carrying its mission's advantage."""
    feats, seg, chosen, old, adv = [], [], [], [], []
    for m in missions:
        for cands, i, lp in m["choices"]:
            seg.append((len(feats), len(cands)))
            feats.extend(cands)
            chosen.append(i)
            old.append(lp)
            adv.append(m["adv"])
    if not seg:
        return {}
    X = torch.tensor(feats, dtype=torch.float32)
    old_t, adv_t = torch.tensor(old), torch.tensor(adv, dtype=torch.float32)
    for _ in range(EPOCHS):
        scores = net(X)
        new, ent = [], []
        for (start, n), i in zip(seg, chosen):
            lp = torch.log_softmax(scores[start:start + n], dim=0)
            new.append(lp[i])
            ent.append(-(lp.exp() * lp).sum())
        new, ent = torch.stack(new), torch.stack(ent)
        ratio = torch.exp(new - old_t)
        loss = -torch.min(ratio * adv_t, ratio.clamp(1 - CLIP, 1 + CLIP) * adv_t).mean() \
            - ENTROPY * ent.mean()
        opt.zero_grad()
        loss.backward()
        opt.step()
    return {"choices": len(seg), "loss": round(float(loss), 4),
            "entropy": round(float(ent.mean()), 3),
            "clipped": round(float(((ratio - 1).abs() > CLIP).float().mean()), 3)}


def main(out: str) -> None:
    torch.manual_seed(SEED)
    rng = random.Random(SEED)
    u = usable()
    train, select = u[300:570], u[100:200]
    net = Net(LinearGoalPolicy.load().weights)
    opt = torch.optim.Adam(net.parameters(), lr=LR)
    log = {"features": list(FEATURES), "seed": SEED, "houses": HOUSES, "rollouts": ROLLOUTS,
           "epochs": EPOCHS, "lr": LR, "clip": CLIP, "entropy": ENTROPY,
           "iters": [], "selection": []}
    best = None
    t0 = time.time()
    with ProcessPoolExecutor(max_workers=WORKERS) as ex:
        def select_now(it):
            nonlocal best
            params = net.export()
            rows = list(ex.map(_greedy, [(h, params) for h in select]))
            r = sum(x["reward"] for x in rows) / len(rows)
            found = sum(x["outcome"] == "found" for x in rows)
            entry = {"it": it, "reward": round(r, 4), "found": found, "net": params}
            log["selection"].append({k: v for k, v in entry.items() if k != "net"})
            print(f"   selection: reward {r:.3f}  found {found}/100  ({time.time() - t0:.0f} s)",
                  flush=True)
            if best is None or r > best["reward"]:
                best = entry

        select_now(-1)   # the start: the linear policy exactly
        for it in range(ITERS):
            houses = rng.sample(train, HOUSES)
            params = net.export()
            jobs = [(h, params, rng.randrange(2 ** 31)) for h in houses for _ in range(ROLLOUTS)]
            missions = list(ex.map(_rollout, jobs))
            for h in houses:
                group = [m for m in missions if m["house"] == h]
                rs = [m["reward"] for m in group]
                mu, sd = sum(rs) / len(rs), float(np.std(rs))
                for m in group:
                    m["adv"] = (m["reward"] - mu) / (sd + 1e-6) if sd > 1e-6 else 0.0
            stats = ppo_update(net, opt, missions)
            mean_r = sum(m["reward"] for m in missions) / len(missions)
            found = sum(m["outcome"] == "found" for m in missions)
            log["iters"].append({"it": it, "mean_reward": round(mean_r, 4),
                                 "found": found, **stats})
            print(f"it {it:2d}  reward {mean_r:.3f}  found {found}/{len(missions)}  {stats}"
                  f"  ({time.time() - t0:.0f} s)", flush=True)
            if (it + 1) % SELECT_EVERY == 0:
                select_now(it)
            json.dump({**log, "best": best}, open(out, "w"), indent=1)
    print("best", {k: v for k, v in best.items() if k != "net"})


if __name__ == "__main__":
    main(sys.argv[1])
