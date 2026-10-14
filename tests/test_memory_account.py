"""3.81.A: tools/jetson/memory_account.py's table half (the sampler reads
/proc and debugfs, which only the board has)."""
import json

from tools.jetson import memory_account as ma


def _rec(t, phase, avail, procs=(), nvmap=None):
    m = {"MemTotal": 8000, "MemFree": 1000, "MemAvailable": avail, "Buffers": 100,
         "Cached": 2100, "Shmem": 200, "AnonPages": 1500, "SReclaimable": 100,
         "SUnreclaim": 100, "KernelStack": 20, "PageTables": 20, "Percpu": 10,
         "VmallocUsed": 50}
    return {"t": t, "phase": phase, "meminfo": m, "nvmap": nvmap or {"11": 2000},
            "pool_kb": 500, "total_memory_kb": 2000,
            "zram": [{"dev": "zram0", "orig_kb": 900, "compr_kb": 280, "used_kb": 300}],
            "procs": list(procs), "cost_s": 0.1}


def _proc(pid, cmd, anon, file=0, cgroup="/user.slice", swap=0, comm=None):
    return {"pid": pid, "comm": comm or cmd.split()[0].rsplit("/", 1)[-1], "cmd": cmd,
            "cgroup": cgroup, "Pss_Anon": anon, "Pss_File": file, "Pss_Shmem": 0,
            "SwapPss": swap}


def test_pick_takes_the_windows_lowest_memavailable():
    recs = [_rec(1, "stack", 100), _rec(2, "window", 900), _rec(3, "window", 700),
            _rec(4, "end", 50)]
    assert ma.pick(recs, "window")["t"] == 3


def test_system_rows_add_up_to_memtotal_with_the_remainder_unexplained():
    rows = dict(ma.system_rows(_rec(1, "window", 700)))
    # page cache excludes shmem, which has its own row
    assert rows["page cache, file-backed (Buffers + Cached - Shmem)"] == 2000
    assert rows["GPU allocations (nvmap, all processes)"] == 2000
    assert rows["zram: compressed swap held in RAM"] == 300
    assert sum(rows.values()) == 8000
    assert rows["unexplained (MemTotal - the rows above)"] == 8000 - (
        1000 + 2000 + 200 + 1500 + 2000 + 500 + 300 + 100 + 100 + 50 + 50)


def test_components_group_by_command_and_container_and_carry_gpu():
    procs = [
        _proc("11", "python tools/jetson/perception_loop.py 3600", 600, 300),
        _proc("12", "python -m uvicorn robot.server:app --port 8000", 40),
        _proc("13", "/opt/ros/humble/lib/nav2_controller/controller_server --ros-args", 30,
              cgroup="/system.slice/docker-abc123.scope"),
        _proc("14", "/usr/bin/python3 /opt/ros/humble/bin/ros2 launch picar_bringup x", 20,
              cgroup="/system.slice/docker-abc123.scope"),
        _proc("15", "/usr/bin/dockerd -H fd://", 25, cgroup="/system.slice/docker.service"),
    ]
    g = ma.component_rows(_rec(1, "window", 700, procs, nvmap={"11": 1200, "99": 30}))
    assert g["perception (YOLOE + CLIP)"]["gpu"] == 1200
    assert g["perception (YOLOE + CLIP)"]["file"] == 300
    assert g["robot server"]["anon"] == 40
    assert g["ROS container: controller_server"]["anon"] == 30
    assert g["ROS container: ros2 launch"]["anon"] == 20
    assert g["docker daemons"]["anon"] == 25
    # GPU memory whose process the sample did not list is still counted
    assert g["GPU, process not listed"]["gpu"] == 30


def test_table_prints_the_explained_share(tmp_path, capsys):
    p = tmp_path / "a.jsonl"
    p.write_text("\n".join(json.dumps(r) for r in
                           [_rec(1, "window", 700, [_proc("11", "python perception_loop.py", 600 * 1024)])]))
    assert ma.main(["table", str(p)]) == 0
    out = capsys.readouterr().out
    assert "explained: 97.5% of MemTotal" in out  # 200 of 8000 kB unexplained
    assert "| perception (YOLOE + CLIP) | 1 | 600 |" in out
