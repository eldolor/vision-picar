"""3.80.E criterion 6's OOM drill: take memory until the kernel's OOM killer
acts, then let it all go -- so dmesg says whom the kernel picks under real
pressure (the local VLM, at oom_score_adj 1000, and never the robot server).

Grows 32 MB every 0.25 s, each chunk written so it is resident, and stops at
the first new "Killed process" line in dmesg, or after <limit> seconds. If
the kernel picks this process instead, that is in dmesg too. Prints one line
per second: elapsed, MB held, MemAvailable.

    python tools/jetson/memory_hog.py <limit_s>
"""

import subprocess
import sys
import time

CHUNK = 32 * 1024 * 1024


def kills() -> int:
    out = subprocess.run(["sudo", "dmesg"], capture_output=True, text=True).stdout
    return sum("killed process" in line.lower() for line in out.splitlines())


def mem_available_mb() -> int:
    with open("/proc/meminfo") as f:
        for line in f:
            if line.startswith("MemAvailable:"):
                return int(line.split()[1]) // 1024
    return -1


def main():
    limit = float(sys.argv[1])
    k0, held, t0, last = kills(), [], time.time(), 0.0
    while time.time() - t0 < limit:
        held.append(bytearray(b"\x01") * CHUNK)
        now = time.time()
        if now - last >= 1.0:
            last = now
            print(f"{now - t0:.1f} {len(held) * 32} {mem_available_mb()}", flush=True)
            if kills() > k0:
                print(f"kill seen after {now - t0:.1f} s, holding {len(held) * 32} MB",
                      flush=True)
                break
        time.sleep(0.25)
    else:
        print(f"no kill in {limit:g} s, holding {len(held) * 32} MB", flush=True)
    held.clear()


if __name__ == "__main__":
    main()
