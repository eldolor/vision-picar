"""3.81.A (docs/plans/ros-alignment/3.81-rover-memory-budget.md): where every
MB of the board's memory is, with the whole stack and the VLM resident.

Two halves:

  sudo python3 -m tools.jetson.memory_account sample <out.jsonl> [--interval 5]
      [--phase-file F] [--torch-log F]
  python3 -m tools.jetson.memory_account table <out.jsonl> [--phase window]
      [--engines DIR] [--torch-log F]

`sample` runs on the board as root (nvmap's debugfs and the container's
processes need it) and writes one JSON line per interval: /proc/meminfo, the
nvmap allocations per process (the GPU's memory: the board has one pool, and
CUDA allocates from it through nvmap), nvmap's page pool, every zram
device's mm_stat, and every process's smaps_rollup with its cgroup. It is
stdlib only, so root's python3 runs it without the venv.

`table` reads those lines, takes the sample at the window's lowest
MemAvailable, and prints two tables: the whole of MemTotal by kernel
category (criterion 1: the explained share), and the processes grouped by
component, each split into anonymous, file-backed, shmem, swapped and GPU.
"""
import argparse
import json
import os
import re
import sys
import time

# A secret on a command line (`curl -H 'x-app-secret: ...'` in the headroom
# harnesses' waits) must not reach `account.jsonl`, which is committed
# (3.76 P1: none was caught in the five committed files; latent).
_SECRET_ARG = re.compile(r"""(x-app-secret:\s*|APP_SHARED_SECRET=|LOCAL_SECRET=)[^\s'"]+""", re.I)


def redact(cmd: str) -> str:
    return _SECRET_ARG.sub(r"\1***", cmd)
from pathlib import Path

NVMAP = Path("/sys/kernel/debug/nvmap")
PAGE = 4096
SMAPS_KEYS = ("Rss", "Pss", "Pss_Anon", "Pss_File", "Pss_Shmem", "Swap", "SwapPss")


def read(path, default=""):
    try:
        return Path(path).read_text()
    except OSError:
        return default


def meminfo() -> dict:
    out = {}
    for line in read("/proc/meminfo").splitlines():
        k, v = line.split(":", 1)
        out[k] = int(v.split()[0])  # kB
    return out


def nvmap_clients() -> dict:
    """{pid: kB} from iovmm/clients ("user <comm> <pid> <size>K" lines)."""
    out = {}
    for line in read(NVMAP / "iovmm/clients").splitlines():
        # Anchored at the end: a process name may contain spaces.
        m = re.search(r"\s(\d+)\s+(\d+)K\s*$", line)
        if m:
            out[m.group(1)] = out.get(m.group(1), 0) + int(m.group(2))
    return out


def nvmap_pool() -> dict:
    d = NVMAP / "pagepool"
    pages = read(d / "page_pool_available_pages", "0").strip() or "0"
    total = read(NVMAP / "stats/total_memory", "0").strip() or "0"
    return {"pool_kb": int(pages) * PAGE // 1024, "total_memory_kb": int(total) // 1024}


def zram() -> list:
    out = []
    for dev in sorted(Path("/sys/block").glob("zram*")):
        f = read(dev / "mm_stat").split()
        if len(f) >= 3:
            # orig_data_size, compr_data_size, mem_used_total (bytes)
            out.append({"dev": dev.name, "orig_kb": int(f[0]) // 1024,
                        "compr_kb": int(f[1]) // 1024, "used_kb": int(f[2]) // 1024})
    return out


def processes() -> list:
    out = []
    for p in Path("/proc").iterdir():
        if not p.name.isdigit():
            continue
        roll = read(p / "smaps_rollup")
        if not roll:
            continue  # a kernel thread, or gone
        vals = {}
        for line in roll.splitlines()[1:]:
            k, _, v = line.partition(":")
            if k in SMAPS_KEYS:
                vals[k] = int(v.split()[0])
        if vals.get("Rss", 0) < 512 and vals.get("Swap", 0) < 512:
            continue
        cmd = redact(read(p / "cmdline").replace("\0", " ").strip())[:240]
        cg = read(p / "cgroup").strip().split("::")[-1]
        out.append({"pid": p.name, "comm": read(p / "comm").strip(), "cmd": cmd,
                    "cgroup": cg, **vals})
    return out


def sample(args) -> int:
    end = time.time() + args.seconds
    with open(args.out, "a") as f:
        while time.time() < end:
            t0 = time.time()
            phase = read(args.phase_file).strip() if args.phase_file else ""
            rec = {"t": round(t0, 1), "phase": phase, "meminfo": meminfo(),
                   "nvmap": nvmap_clients(), **nvmap_pool(), "zram": zram(),
                   "procs": processes()}
            rec["cost_s"] = round(time.time() - t0, 3)
            f.write(json.dumps(rec) + "\n")
            f.flush()
            if args.phase_file and phase == "end":
                break
            time.sleep(max(0.0, args.interval - (time.time() - t0)))
    return 0


# --- the table -------------------------------------------------------------

COMPONENTS = [  # first match wins; matched against "comm cmd"
    ("robot server", r"robot\.server:app"),
    ("brain", r"control\.brain_server:app"),
    ("simulator body", r"sim\.body_server"),
    ("simulator sensors", r"sim\.sensor_server"),
    ("tunnel proxy", r"service\.tunnel\.proxy"),
    ("perception (YOLOE + CLIP)", r"perception_loop\.py"),
    # The client before the driver: its command line names the driver's path.
    ("VLM bench client", r"bench_identity_trt|bench_identity_llama"),
    ("VLM driver (VE, Edge-LLM)", r"edgellm_identity"),
    ("VLM server (llama.cpp)", r"llama-server"),
    ("local VLM service", r"local_vlm|vlm_service"),
    ("nav load / harness python", r"demo_nav_goals|python - "),
    ("measurement (sampler, tegrastats)", r"memory_account|tegrastats"),
    ("docker daemons", r"^(dockerd|containerd|containerd-shim)"),
    ("desktop (gdm, gnome)", r"gnome|gdm|Xorg|gsd-|pulseaudio|gjs|ibus|evolution|tracker"),
]


def component(p: dict) -> str:
    if "docker" in p["cgroup"] and "docker" not in p["comm"]:
        exe = p["cmd"].split()[0].rsplit("/", 1)[-1] if p["cmd"] else p["comm"]
        if exe.startswith("python"):
            m = re.search(r"(ros2 launch|uvicorn \S+|[\w./-]+\.py|-m \S+)", p["cmd"])
            exe = m.group(1) if m else exe
        return f"ROS container: {exe}"
    hay = f"{p['comm']} {p['cmd']}"
    for name, pat in COMPONENTS:
        if re.search(pat, hay):
            return name
    return "system (other)"


def mb(kb) -> int:
    return round(kb / 1024)


def pick(recs: list, phase: str) -> dict:
    cand = [r for r in recs if r["phase"] == phase]
    if not cand:  # never a table of some other phase under this one's name
        raise SystemExit(f"no sample has phase {phase!r}; phases: "
                         f"{sorted({r['phase'] for r in recs})}")
    return min(cand, key=lambda r: r["meminfo"]["MemAvailable"])


def system_rows(r: dict) -> list:
    m = r["meminfo"]
    zram_used = sum(z["used_kb"] for z in r["zram"])
    gpu = sum(r["nvmap"].values())
    file_cache = m["Buffers"] + m["Cached"] - m["Shmem"]
    rows = [
        ("free (MemFree)", m["MemFree"]),
        ("page cache, file-backed (Buffers + Cached - Shmem)", file_cache),
        ("shmem / tmpfs", m["Shmem"]),
        ("anonymous, process heaps (AnonPages)", m["AnonPages"]),
        ("GPU allocations (nvmap, all processes)", gpu),
        ("nvmap page pool (freed GPU pages kept for reuse)", r["pool_kb"]),
        ("zram: compressed swap held in RAM", zram_used),
        ("kernel slab (reclaimable)", m["SReclaimable"]),
        ("kernel slab (unreclaimable)", m["SUnreclaim"]),
        ("kernel stacks + page tables + per-cpu", m["KernelStack"] + m["PageTables"] + m.get("Percpu", 0)),
        ("vmalloc", m.get("VmallocUsed", 0)),
    ]
    explained = sum(v for _, v in rows)
    rows.append(("unexplained (MemTotal - the rows above)", m["MemTotal"] - explained))
    return rows


def component_rows(r: dict) -> dict:
    groups = {}
    for p in r["procs"]:
        g = groups.setdefault(component(p), {"n": 0, "anon": 0, "file": 0, "shmem": 0,
                                              "swap": 0, "gpu": 0, "pids": []})
        g["n"] += 1
        g["anon"] += p.get("Pss_Anon", 0)
        g["file"] += p.get("Pss_File", 0)
        g["shmem"] += p.get("Pss_Shmem", 0)
        g["swap"] += p.get("SwapPss", p.get("Swap", 0))
        g["gpu"] += r["nvmap"].get(p["pid"], 0)
        g["pids"].append(p["pid"])
    seen = {p["pid"] for p in r["procs"]}
    orphan = sum(v for pid, v in r["nvmap"].items() if pid not in seen)
    if orphan:
        groups["GPU, process not listed"] = {"n": 0, "anon": 0, "file": 0, "shmem": 0,
                                             "swap": 0, "gpu": orphan, "pids": []}
    return groups


def engine_sizes(d: str) -> dict:
    out = {}
    for f in sorted(Path(d).rglob("*")):
        if f.is_file() and f.stat().st_size > (1 << 20):
            out[str(f.relative_to(d))] = f.stat().st_size // 1024
    return out


def table(args) -> int:
    recs = [json.loads(l) for l in open(args.jsonl) if l.strip()]
    r = pick(recs, args.phase)
    m = r["meminfo"]
    total = m["MemTotal"]
    win = [x["meminfo"]["MemAvailable"] for x in recs if x["phase"] == args.phase]
    print(f"sample t={r['t']} phase={r['phase']!r}: MemAvailable {mb(m['MemAvailable'])} MB "
          f"(window min/median over {len(win)} samples: "
          f"{mb(min(win)) if win else '-'} / {mb(sorted(win)[len(win)//2]) if win else '-'} MB); "
          f"sampler cost {r['cost_s']} s\n")
    print(f"### MemTotal {mb(total)} MB, by kernel category\n")
    print("| category | MB | share |\n|---|---:|---:|")
    rows = system_rows(r)
    for name, kb in rows:
        print(f"| {name} | {mb(kb)} | {100 * kb / total:.1f}% |")
    unexpl = rows[-1][1]
    print(f"\nexplained: {100 * (total - abs(unexpl)) / total:.1f}% of MemTotal "
          f"(criterion 1: >= 95%)\n")
    print("### Processes by component (PSS; GPU from nvmap)\n")
    print("| component | procs | anon | file | shmem | swapped | GPU | total in RAM |")
    print("|---|---:|---:|---:|---:|---:|---:|---:|")
    groups = component_rows(r)
    order = sorted(groups.items(), key=lambda kv: -(kv[1]["anon"] + kv[1]["file"]
                                                     + kv[1]["shmem"] + kv[1]["gpu"]))
    tot = dict(anon=0, file=0, shmem=0, swap=0, gpu=0)
    for name, g in order:
        ram = g["anon"] + g["file"] + g["shmem"] + g["gpu"]
        if ram < 5 * 1024 and g["swap"] < 5 * 1024:
            continue
        for k in tot:
            tot[k] += g[k]
        print(f"| {name} | {g['n']} | {mb(g['anon'])} | {mb(g['file'])} | {mb(g['shmem'])} "
              f"| {mb(g['swap'])} | {mb(g['gpu'])} | {mb(ram)} |")
    small = [g for g in groups.values()
             if g["anon"] + g["file"] + g["shmem"] + g["gpu"] < 5 * 1024 and g["swap"] < 5 * 1024]
    sm = {k: sum(g[k] for g in small) for k in tot}
    print(f"| (components under 5 MB, {len(small)}) | {sum(g['n'] for g in small)} | {mb(sm['anon'])} "
          f"| {mb(sm['file'])} | {mb(sm['shmem'])} | {mb(sm['swap'])} | {mb(sm['gpu'])} "
          f"| {mb(sm['anon'] + sm['file'] + sm['shmem'] + sm['gpu'])} |")
    if args.engines:
        print("\n### VLM engine files (weights on disk)\n")
        for k, v in engine_sizes(args.engines).items():
            print(f"* {k}: {mb(v)} MB")
    if args.torch_log and Path(args.torch_log).exists():
        tl = [json.loads(l) for l in open(args.torch_log) if l.strip()]
        near = min(tl, key=lambda x: abs(x["t"] - r["t"])) if tl else None
        if near:
            print(f"\n### Perception's torch allocator (nearest log, t={near['t']})\n")
            for k, v in near.items():
                if k != "t":
                    print(f"* {k}: {v}")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("sample")
    s.add_argument("out")
    s.add_argument("--interval", type=float, default=5.0)
    s.add_argument("--seconds", type=float, default=7200)
    s.add_argument("--phase-file")
    t = sub.add_parser("table")
    t.add_argument("jsonl")
    t.add_argument("--phase", default="window")
    t.add_argument("--engines")
    t.add_argument("--torch-log")
    args = ap.parse_args(argv)
    return sample(args) if args.cmd == "sample" else table(args)


if __name__ == "__main__":
    sys.exit(main())
