"""
evaluations/goal-learn-3103/pool_run.py -- 3.103.C: F-pool (`pool.py`) on
F-lean's 18 facts, by 3.96's own stages (`goal-learn-396/train.py`) with the
network, the fact and the pool-aware losses patched in; 3.96's files and
B's `run.py` stages are unchanged.

    POOL_WIDTH=W python evaluations/goal-learn-3103/pool_run.py clone ROOT  # oracle clone -> ROOT/F-pool-WW/clone.pt
    POOL_WIDTH=W python evaluations/goal-learn-3103/pool_run.py ppo ROOT    # PPO from the clone, 3.101's budget -> best.json
    python evaluations/goal-learn-3103/pool_run.py reselect ROOT  # W64's best re-run on this host's selection houses
    python evaluations/goal-learn-3103/pool_run.py choose ROOT  # amendment 2's rule over the widths -> ROOT/chosen-pool.json
    python evaluations/goal-learn-3103/pool_run.py test ROOT    # the chosen width on the 588 test houses, R and U
    python evaluations/goal-learn-3103/pool_run.py score ROOT   # C's criteria 1-4 against B's F-lean rows; the oracle gap
    python evaluations/goal-learn-3103/pool_run.py time ROOT    # criterion 5: one choice, F-lean vs the chosen F-pool

ROOT is B's (`run/`: `oracle-large` and `oracle-select` from 3.101, B's
`F-lean-S64/best.json` and `test/F-lean-*.json`). Every stage resumes as
3.96's do.

3.96's clone loss and PPO update score many pools concatenated in one call;
a per-candidate network does not care where one pool ends, attention does.
`_batch_loss` and `ppo_update` below are 3.96's, line for line, with the
pools' lengths handed to the network (`seg=`).
"""
import glob
import json
import multiprocessing
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT_REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT_REPO, "evaluations", "goal-learn-3102"))
sys.path.insert(0, HERE)
import pool as POOL                                           # noqa: E402
import run as B                                               # noqa: E402  (3.96's path set up, WORKERS defaulted)
from cem import _parts                                        # noqa: E402  (3.102 amendment 1's split)

C396, T = B.C396, B.T
# Amendment 2: one arm per width, F-pool-W32 / -W64 / -W128 (POOL_WIDTH, default 64).
WIDTHS = (32, 64, 128)
MATCH = 0.02                 # within this of the best mode-R selection reward "matches" it
ARM = f"F-pool-W{POOL.WIDTH}"
LEAN = "F-lean-S64"          # B's chosen arm: C's comparison and cost base
ORACLE_ROWS = os.path.join(ROOT_REPO, "evaluations", "goal-learn-3102", "run", "test")
# B's oracle sets (amendment 1): the regenerated ones must give the same counts.
B_CHOICES = {"n_train": 37064, "n_held": 609}


def _batch_loss(net, batch, kind, torch, poni=False):
    if poni:
        raise ValueError("F-pool has no PONI loss")
    X = torch.tensor(np.concatenate([b[0] for b in batch]))
    s = net(X, seg=[len(b[0]) for b in batch])
    loss, k = 0.0, 0
    for x, _, y, _ in batch:
        n = len(x)
        loss = loss - torch.log_softmax(s[k:k + n], dim=0)[y]
        k += n
    return loss / len(batch)


def ppo_update(net, opt, missions, kind, torch):
    seg, X, chosen, old, adv = [], [], [], [], []
    at = 0
    for m in missions:
        for c in m["choices"]:
            seg.append((at, len(c["X"])))
            at += len(c["X"])
            X.append(c["X"])
            chosen.append(c["i"])
            old.append(c["lp"])
            adv.append(m["adv"])
    if not seg:
        return {}
    Xt = torch.tensor(np.concatenate(X))
    old_t, adv_t = torch.tensor(old, dtype=torch.float32), torch.tensor(adv, dtype=torch.float32)
    CH = 8192        # candidates per chunk; gradients summed, so one step per epoch
    for _ in range(T.EPOCHS):
        opt.zero_grad()
        tot_loss, ents, ratios = 0.0, [], []
        k = 0
        while k < len(seg):
            j, n = k, 0
            while j < len(seg) and (n == 0 or n + seg[j][1] <= CH):
                n += seg[j][1]
                j += 1
            a0 = seg[k][0]
            s = net(Xt[a0:a0 + n], seg=[seg[q][1] for q in range(k, j)])
            new, ent = [], []
            for q in range(k, j):
                st, ln = seg[q][0] - a0, seg[q][1]
                lp = torch.log_softmax(s[st:st + ln], dim=0)
                new.append(lp[chosen[q]])
                ent.append(-(lp.exp() * lp).sum())
            new, ent = torch.stack(new), torch.stack(ent)
            ratio = torch.exp(new - old_t[k:j])
            a = adv_t[k:j]
            loss = (-torch.min(ratio * a, ratio.clamp(1 - T.CLIP, 1 + T.CLIP) * a).sum()
                    - T.ENTROPY * ent.sum()) / len(seg)
            loss.backward()
            tot_loss += float(loss.detach())
            ents.append(ent.detach())
            ratios.append(ratio.detach())
            k = j
        opt.step()
    ent, ratio = torch.cat(ents), torch.cat(ratios)
    return {"choices": len(seg), "loss": round(tot_loss, 4), "entropy": round(float(ent.mean()), 3),
            "clipped": round(float(((ratio - 1).abs() > T.CLIP).float().mean()), 3)}


def use_pool() -> None:
    """Patch 3.96's trainer before the pool forks (start method pinned to fork)."""
    B.use_shape("S64")                    # F-lean's fact and column slice
    T.torch_net = POOL.torch_net
    T.score_np = POOL.score_np
    T._batch_loss = _batch_loss
    T.ppo_update = ppo_update


def _torch():
    import torch
    torch.set_num_threads(int(os.environ.get("TORCH_THREADS", "4")))
    return torch


def check_oracle(root: str) -> None:
    """Refuse to start unless 3.101's 42-column oracle sets are in ROOT:
    under the patched hook a missing set would be rebuilt with 18 columns,
    which the column slice cannot read, and cached there."""
    import pickle
    for d in ("oracle-large", "oracle-select"):
        ps = sorted(glob.glob(os.path.join(root, d, "*.pkl")))
        if not ps:
            raise SystemExit(f"{root}/{d}: no oracle shards; make them with 3.101's harness first")
        x = next((c["X"] for r in pickle.load(open(ps[0], "rb")) for c in r["choices"]), None)
        if x is None or x.shape[1] != 42:
            raise SystemExit(f"{ps[0]}: not 3.101's 42-column oracle shard")


def cmd_clone(root: str) -> None:
    check_oracle(root)
    use_pool()
    out = os.path.join(root, ARM)
    os.makedirs(out, exist_ok=True)
    _torch()
    with ProcessPoolExecutor(T.WORKERS) as ex:
        shards = T.stage_oracle("large", root, ex)        # all exist: nothing is run
        st = T.stage_clone(ARM, out, shards, ex)
    print(f"{ARM}: {POOL.size()} numbers, held-out agreement {st['held_out_agreement']}", flush=True)
    got = {k: st[k] for k in B_CHOICES}
    print(f"oracle choices {got}: " + ("the same as B's" if got == B_CHOICES
                                       else f"MISMATCH, B's were {B_CHOICES}"), flush=True)
    if got != B_CHOICES and not T.SMOKE:
        raise SystemExit("not training on oracle sets that differ from B's (amendment 1)")


def cmd_ppo(root: str) -> None:
    use_pool()
    out = os.path.join(root, ARM)
    if os.path.exists(os.path.join(out, "best.json")):
        print(f"{ARM}: already done")
        return
    torch = _torch()
    clone = torch.load(os.path.join(out, "clone.pt"), weights_only=False)
    with ProcessPoolExecutor(T.WORKERS) as ex:
        T.stage_ppo(ARM, out, clone, ex)


def cmd_reselect(root: str) -> None:
    """Width 64 was trained in the cloud container, 32 and 128 on the
    instance: re-run W64's best on the selection houses here, so `choose`
    can show the two hosts give the same numbers (the missions are
    deterministic for fixed weights) before comparing across them."""
    out = os.path.join(root, "F-pool-W64", "reselect.json")
    if os.path.exists(out):
        return
    use_pool()
    params = json.load(open(os.path.join(root, "F-pool-W64", "best.json")))["params"]
    with ProcessPoolExecutor(T.WORKERS) as ex:
        sel = T.run_selection("F", params, ex)
    T.save_atomic(out, lambda p: json.dump(sel, open(p, "w"), indent=1))
    print("F-pool-W64 re-selected here:", {m: sel[m]["mean_reward"] for m in sel}, flush=True)


def cmd_choose(root: str) -> None:
    """Amendment 2: the smallest width whose best mode-R selection reward is
    within MATCH of the best width's."""
    sel = {w: json.load(open(os.path.join(root, f"F-pool-W{w}", "best.json")))["best_selection"]
           for w in WIDTHS}
    r = {w: sel[w]["R"]["mean_reward"] for w in WIDTHS}
    top = max(r.values())
    chosen = min(w for w in WIDTHS if r[w] >= top - MATCH)
    rp = os.path.join(root, "F-pool-W64", "reselect.json")
    same = None
    if os.path.exists(rp):
        again = json.load(open(rp))
        same = all(abs(again[m]["mean_reward"] - sel[64][m]["mean_reward"]) < 1e-3 for m in ("R", "U"))
        print("F-pool-W64 on this host vs the container:",
              {m: (again[m]["mean_reward"], sel[64][m]["mean_reward"]) for m in ("R", "U")},
              "the same" if same else "DIFFERENT -- the widths' selection rewards are not like for like",
              flush=True)
        if not same:
            raise SystemExit("not choosing (amendment 2): W64 does not reproduce on this host")
    T.save_atomic(os.path.join(root, "chosen-pool.json"),
                  lambda p: json.dump({"selection": {f"F-pool-W{w}": sel[w] for w in WIDTHS},
                                       "best_R": top, "match": MATCH, "w64_same_on_host": same,
                                       "chosen": f"F-pool-W{chosen}"}, open(p, "w"), indent=1))
    print("selection R", r, "-> test", f"F-pool-W{chosen}", flush=True)


def chosen(root: str) -> str:
    return json.load(open(os.path.join(root, "chosen-pool.json")))["chosen"]


def _job(args):
    s, h, mode, params = args
    use_pool()
    try:
        r = T.mission(s, h, mode, ("greedy", "F", params), 0, 0.0)
    except Exception as e:  # noqa: BLE001 -- a house that cannot start is a row, not a lost run
        r = {"split": s, "index": h, "mode": mode, "outcome": "exception",
             "error": f"{type(e).__name__}: {e}", "goals": 0, "metres": 0.0,
             "reward": -T.CRASH_COST}
    r.pop("choices", None)
    return r


def cmd_test(root: str) -> None:
    ARM = chosen(root)
    params = json.load(open(os.path.join(root, ARM, "best.json")))["params"]
    hs = C396.test_houses()
    with ProcessPoolExecutor(T.WORKERS) as ex:
        for mode in ("R", "U"):
            path = os.path.join(root, "test", f"{ARM}-{mode}.json")
            if os.path.exists(path):
                continue
            os.makedirs(os.path.dirname(path), exist_ok=True)
            rows = list(ex.map(_job, [(s, h, mode, params) for s, h in hs], chunksize=2))
            T.save_atomic(path, lambda p: json.dump(rows, open(p, "w")))
            print(f"{ARM} {mode}: {T.summary(rows)}", flush=True)


def cmd_score(root: str) -> dict:
    ARM = chosen(root)
    rng = np.random.default_rng(B.SEED)
    want = sorted(h for _, h in C396.test_houses())
    best = json.load(open(os.path.join(root, ARM, "best.json")))
    rep = {"arm": ARM, "widths": json.load(open(os.path.join(root, "chosen-pool.json"))),
           "best_it": best["best_it"], "selection": best["best_selection"],
           "clone_agreement": best["log"]["clone_agreement"]}
    for m in ("R", "U"):
        arms = {ARM: B._rows(os.path.join(root, "test", f"{ARM}-{m}.json")),
                "F-lean": B._rows(os.path.join(root, "test", f"F-lean-{m}.json")),
                "F-large": B._rows(os.path.join(B.F_ROWS, f"F-large-{m}.json")),
                "F-rich": B._rows(os.path.join(B.F_ROWS, f"F-rich-{m}.json")),
                "oracle": B._rows(os.path.join(ORACLE_ROWS, f"oracle-{m}.json"))}
        for name, rows in arms.items():
            if sorted(rows) != want:
                raise SystemExit(f"{name}-{m}: not the {len(want)} test houses -- not scoring")
        n, L = arms[ARM], arms["F-lean"]
        sn, sl = T.summary(list(n.values())), T.summary(list(L.values()))
        both = [k for k in n if n[k]["outcome"] == "found" and L[k]["outcome"] == "found"]
        po = _parts(arms["oracle"])
        gap = {}
        for a in (ARM, "F-lean", "F-large", "F-rich"):
            pa = _parts(arms[a])
            g = {k: round(po[k] - pa[k], 4) for k in po}
            g["total"] = round(sum(g.values()), 4)
            gap[a] = g
        rep[m] = {**{a: T.summary(list(r.values())) for a, r in arms.items()},
                  "vs_F_lean": B._boot(n, L, rng), "vs_F_large": B._boot(n, arms["F-large"], rng),
                  "vs_F_rich": B._boot(n, arms["F-rich"], rng),
                  "fewer_goals_than_F_lean": sum(n[k]["goals"] < L[k]["goals"] for k in both),
                  "more_goals_than_F_lean": sum(n[k]["goals"] > L[k]["goals"] for k in both),
                  "finds_ok": sn["found"] >= 0.99 * sl["found"],
                  "trips_ok": sn["median_goals_found"] <= sl["median_goals_found"],
                  "unclean": sum(1 for r in n.values() if r["outcome"] != "no_target"
                                 and (r["outcome"] == "exception" or r.get("capped"))),
                  "oracle_parts": po, "gap_to_oracle": gap}
    rep["criteria"] = {"1_better_R": rep["R"]["vs_F_lean"]["ci95"][0] > 0,
                       "2_finds": rep["R"]["finds_ok"] and rep["U"]["finds_ok"],
                       "3_trips": rep["R"]["trips_ok"] and rep["U"]["trips_ok"],
                       "4_clean": rep["R"]["unclean"] + rep["U"]["unclean"] == 0}
    T.save_atomic(os.path.join(root, "test", f"score-{ARM}.json"),
                  lambda p: json.dump(rep, open(p, "w"), indent=1))
    print(json.dumps({k: rep[k] for k in ("criteria", "R", "U")}, indent=1))
    return rep


def cmd_time(root: str) -> None:
    """Criterion 5: F-lean's whole choice and F-pool's at the same poses, in
    one run (the base is measured here, beside it, not taken from B)."""
    load = lambda a: {k: np.asarray(v, dtype=np.float32)  # noqa: E731
                      for k, v in json.load(open(os.path.join(root, a, "best.json")))["params"].items()}
    ARM = chosen(root)
    P_lean, P_pool = load(LEAN), load(ARM)
    out = B.bench({"F-lean": lambda m, x, y, h, t: B.lean_choice(m, x, y, h, t, P_lean),
                   ARM: lambda m, x, y, h, t: B.lean_choice(m, x, y, h, t, P_pool, POOL.score_np)})
    out["5_cost"] = {"bar_ms": round(1.1 * out["F-lean"]["p90_ms"], 1),
                     "met": out[ARM]["p90_ms"] <= 1.1 * out["F-lean"]["p90_ms"]}
    print(out["5_cost"])
    out["arm"] = ARM
    T.save_atomic(os.path.join(root, f"time-{ARM}.json"), lambda p: json.dump(out, open(p, "w"), indent=1))


if __name__ == "__main__":
    multiprocessing.set_start_method("fork", force=True)
    cmd, root = sys.argv[1:]
    {"clone": cmd_clone, "ppo": cmd_ppo, "reselect": cmd_reselect, "choose": cmd_choose, "test": cmd_test, "score": cmd_score,
     "time": cmd_time}[cmd](root)
