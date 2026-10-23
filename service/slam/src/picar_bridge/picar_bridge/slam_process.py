"""slam_process -- slam_toolbox, started and stopped by the bridge (3.50.D3).

The launch file used to start slam_toolbox once, as the async (mapping)
node. A reload needs it in localisation mode on a saved graph, and a bad
match (or D4's re-map) needs it swapped for the async node at run time,
which a launch file cannot do. So the bridge owns the process: one
slam_toolbox at a time, named `slam_toolbox` and given slam.yaml as the
launch file gave it, plus the map to load. Plain Python (no rclpy), so it
is tested on a laptop.
"""

import os
import signal
import subprocess
import threading
from typing import Callable, Optional, Sequence

# 3.50.D2-b: the async node maps (map_and_localization bent the furnished
# home's map 1.6-3x more); a reload localises with the localisation node.
EXECUTABLES = {
    "mapping": "async_slam_toolbox_node",
    "localisation": "localization_slam_toolbox_node",
}
NODE_NAME = "slam_toolbox"
STOP_TIMEOUT_S = 5.0


def command(exe_dir: str, params_file: str, mode: str, map_prefix: Optional[str] = None,
            start_pose: Optional[Sequence[float]] = None) -> list:
    """The argv for one slam_toolbox. `map_prefix` (a store version's
    `graph`, without extension) and `start_pose` load a saved graph: the
    localisation node turns them into LOCALIZE_AT_POSE (3.50.D1)."""
    if mode not in EXECUTABLES:
        raise ValueError(f"unknown slam mode {mode!r}")
    if (map_prefix is None) != (start_pose is None):
        raise ValueError("a saved map needs both map_prefix and start_pose")
    argv = [os.path.join(exe_dir, EXECUTABLES[mode]), "--ros-args",
            "-r", f"__node:={NODE_NAME}", "--params-file", params_file,
            # slam.yaml says `mode: mapping`; the node in use decides, and the
            # parameter is made to agree with it.
            "-p", f"mode:={'localization' if mode == 'localisation' else 'mapping'}"]
    if map_prefix is not None:
        x, y, yaw = (float(v) for v in start_pose)
        argv += ["-p", f"map_file_name:={map_prefix}",
                 "-p", f"map_start_pose:=[{x!r}, {y!r}, {yaw!r}]"]
    return argv


def _die_with_parent() -> None:
    """In the child, before exec: SIGTERM when the bridge dies, so a crashed
    bridge leaves no slam_toolbox publishing map -> odom behind (Linux only;
    a laptop test never execs one)."""
    try:
        import ctypes
        ctypes.CDLL("libc.so.6", use_errno=True).prctl(1, signal.SIGTERM)   # PR_SET_PDEATHSIG
    except Exception:  # noqa: BLE001 -- not Linux: nothing to set
        pass


class SlamProcess:
    """One slam_toolbox at a time. `start` stops any running one first."""

    def __init__(self, exe_dir: str, params_file: str,
                 spawn: Callable[..., subprocess.Popen] = subprocess.Popen):
        self.exe_dir = exe_dir
        self.params_file = params_file
        self._spawn = spawn
        self._lock = threading.Lock()
        self.proc = None
        self.mode = None
        self.starts = 0

    def start(self, mode: str, map_prefix: Optional[str] = None,
              start_pose: Optional[Sequence[float]] = None) -> None:
        argv = command(self.exe_dir, self.params_file, mode, map_prefix, start_pose)
        with self._lock:
            self._stop_locked()
            self.proc = self._spawn(argv, preexec_fn=_die_with_parent)
            self.mode = mode
            self.starts += 1

    def stop(self) -> None:
        with self._lock:
            self._stop_locked()

    def _stop_locked(self) -> None:
        proc, self.proc, self.mode = self.proc, None, None
        if proc is None or proc.poll() is not None:
            return
        # SIGINT is what launch sends: rclcpp shuts down cleanly on it.
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=STOP_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=STOP_TIMEOUT_S)

    def alive(self) -> bool:
        with self._lock:
            return self.proc is not None and self.proc.poll() is None
