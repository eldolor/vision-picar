"""3.80.E criterion 6's OOM drill: take memory until the kernel's OOM killer
acts, then let it all go -- so dmesg says whom the kernel picks under real
pressure (the local VLM, at oom_score_adj 1000, and never the robot server).

Grows 32 MB every 0.25 s, each chunk written so it is resident, and stops at
the first OOM kill -- `oom_kill` in /proc/vmstat rising (monotonic; dmesg's
ring buffer can rotate an old kill line out as a new one comes in, review) --
or when <watch_pid> (the VLM) is gone, or after <limit> seconds. Checked
every chunk. If the kernel picks this process instead, dmesg says so. Prints
one line per second: elapsed, MB held, MemAvailable.

    python tools/jetson/memory_hog.py <limit_s> [watch_pid]
"""

import os
import sys
import time

CHUNK = 32 * 1024 * 1024


def kills() -> int:
    with open("/proc/vmstat") as f:
        for line in f:
            if line.startswith("oom_kill "):
                return int(line.split()[1])
    raise SystemExit("no oom_kill in /proc/vmstat: cannot see a kill, not starting")


def alive(pid) -> bool:
    if pid is None:
        return True
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def mem_available_mb() -> int:
    with open("/proc/meminfo") as f:
        for line in f:
            if line.startswith("MemAvailable:"):
                return int(line.split()[1]) // 1024
    return -1


def main():
    limit = float(sys.argv[1])
    pid = int(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[2] else None
    k0, held, t0, last = kills(), [], time.time(), 0.0
    while time.time() - t0 < limit:
        if kills() > k0 or not alive(pid):
            print(f"kill seen after {time.time() - t0:.1f} s (oom_kill +{kills() - k0}, "
                  f"watched pid alive: {alive(pid)}), holding {len(held) * 32} MB", flush=True)
            break
        held.append(bytearray(b"\x01") * CHUNK)
        now = time.time()
        if now - last >= 1.0:
            last = now
            print(f"{now - t0:.1f} {len(held) * 32} {mem_available_mb()}", flush=True)
        time.sleep(0.25)
    else:
        print(f"no kill in {limit:g} s, holding {len(held) * 32} MB", flush=True)
    held.clear()


if __name__ == "__main__":
    main()
