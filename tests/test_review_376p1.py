"""
tests/test_review_376p1.py

`PLAN-ros-alignment.md` 3.76, step P1: the bugs the review of the Jetson
tools (`tools/jetson/`, whole files) confirmed. Each test was written before
its fix and seen to fail on `dev` for the reason it names. The board itself
is out of reach here, so the shell scripts are pinned by what they say.
"""

import re
import socket
from pathlib import Path

from tests.conftest import REPO_ROOT

JETSON = Path(REPO_ROOT) / "tools" / "jetson"


def _text(name):
    return (JETSON / name).read_text()


def _code(name):
    """The script without its comment lines."""
    return "\n".join(l for l in _text(name).splitlines() if not l.lstrip().startswith("#"))


# ---------- P1.1: the local VLM bench broke the production unit ----------


def test_the_local_vlm_bench_leaves_the_production_unit_alone():
    """It installed, then DELETED, /etc/systemd/system/picar-local-vlm.service
    and overwrote ~/.vision-picar-local-vlm.env -- since 3.80.B the
    installed service's own files; a re-run left it "not found", or crashing
    at every start on the bench's driver-log path (both reviewers)."""
    code = _code("bench_local_vlm.sh")
    assert "SVC=picar-local-vlm-bench" in code
    assert not re.search(r"/etc/systemd/system/picar-local-vlm\.service", code)
    assert not re.search(r"\.vision-picar-local-vlm\.env", code)
    assert 'EnvironmentFile=$ENVF' in code                  # the bench unit reads its own
    assert 'OUT="$(cd "$OUT" && pwd)"' in code              # systemd needs absolute paths


def test_the_local_vlm_bench_stops_and_restores_the_production_service():
    code = _code("bench_local_vlm.sh")
    assert re.search(r"systemctl stop [^\n]*picar-local-vlm\b(?!-)", code)
    # 3.108: held off for the run (it is enabled at boot), put back as found.
    teardown = code[code.index("teardown() {"):code.index("trap teardown EXIT")]
    assert teardown.index("isolate graphical.target") < teardown.index("vlm_restore")
    assert (code.index("trap teardown EXIT") < code.index('vlm_hold "$OUT')
            < code.index("systemctl isolate multi-user.target"))
    assert '"sport = :8005"' in code                         # refuses another VLM


def test_the_driver_death_wait_is_bounded():
    """`until ! health` spun for good on a service that kept answering."""
    code = _code("bench_local_vlm.sh")
    loop = re.search(r"until ! health[^\n]*", code).group(0)
    assert "break" in loop, loop
    # Round 1 (/code-review): a wait that timed out logged "health gone".
    assert "health STILL answering" in code


# ---------- P1.2: a stale G4 container measured in place of the run's ----------


def test_the_headroom_runs_refuse_a_busy_bridge_port():
    """README's G4 left `picar-ros` with `--restart unless-stopped`; its
    bridge answered "bridge up" for the run's own (/code-review)."""
    for name in ("headroom_identity.sh", "headroom_mission.sh"):
        code = _code(name)
        # Round 1 (/code-review): `ss -ltn | grep -q` could pass a busy port
        # under pipefail (grep exits, ss dies on SIGPIPE); a filter instead.
        check = code.index('ss -ltnH "sport = :8090" | grep -q .')
        assert check < code.index('docker run -d --name "$NAME"'), name
    readme = _text("README.md")
    g4 = readme[readme.index("tests/test_ros_chain_live.py"):]
    assert "docker rm -f picar-ros" in g4.split("```")[0]


def test_the_identity_headroom_run_stops_the_production_vlm():
    assert re.search(r"systemctl stop [^\n]*picar-local-vlm", _code("headroom_identity.sh"))


# ---------- P1.3: counts and windows ----------


def test_an_oom_kill_is_counted_once():
    """Matching "out of memory|oom-kill|killed process" counted every kill's
    three lines (Thermos: one kill read 3)."""
    for name in ("headroom_identity.sh", "headroom_mission.sh"):
        line = next(l for l in _code(name).splitlines() if l.startswith("oom()"))
        assert '"killed process"' in line and "oom-kill" not in line, (name, line)


def test_identity_headroom_perception_is_read_inside_the_window():
    """Every frame since the loop began counted, the cold first one too."""
    assert "t0 <= _at(m.group(1)) <= t1" in _text("headroom_identity.sh")


def test_a_list_reply_is_not_a_bare_object():
    """After a bare `[`, the deciding token is the `{`: every ordinary list
    reply counted as a bare object (records: 68-70 of 299, all lists)."""
    from tools.jetson.bench_identity_llama import bare_object
    tok = lambda t: {"token": t}  # noqa: E731
    assert not bare_object([tok("["), tok("\n\t"), tok("{")], 2)
    assert bare_object([tok("{")], 0)
    assert bare_object([tok(" "), tok('{"')], 1)
    assert not bare_object([tok("["), tok('{"')], None)
    # Round 1 (/code-review): a code fence before it, as `p_localised` reads it.
    assert bare_object([tok("```json"), tok("\n"), tok("{")], 2)


def test_one_refused_call_is_one_failed_frame():
    """`post()` caught only HTTPError: a refused connection ended the
    299-frame run with nothing written."""
    from tools.jetson.bench_local_vlm import post
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()                                             # nothing listens there
    got, ms = post(f"http://127.0.0.1:{port}", b"\xff\xd8", "cup", 0.5, "x")
    assert got["score"] is None and got["status"].startswith("error"), got


def test_a_secret_on_a_command_line_is_not_recorded():
    """`account.jsonl` (committed) kept every short command line, the
    harnesses' `curl -H 'x-app-secret: ...'` waits among them (latent)."""
    from tools.jetson import memory_account
    cmd = "sh -c curl -s -H 'x-app-secret: s3cret-value-1234' localhost:8000/wheels"
    assert "s3cret" not in memory_account.redact(cmd)
    assert "localhost:8000/wheels" in memory_account.redact(cmd)
    assert "redact(" in (JETSON / "memory_account.py").read_text().split("def processes")[1]


# ---------- the fixes' round 1 ----------


import threading  # noqa: E402


def test_the_headroom_runs_put_a_running_vlm_back():
    """Round 1 (Thermos): the identity run now stopped `picar-local-vlm`
    and nothing started it again (installed, not enabled), so identity fell
    to the cloud after every bench."""
    for name in ("headroom_identity.sh", "headroom_mission.sh"):
        code = _code(name)
        # 3.108: the VLM is enabled at boot, so it is held off (vlm_hold.sh)
        # after the trap and before the isolate, which would start it, and
        # put back as found after the teardown's isolate. vlm_hold.sh sets
        # VLM_WAS when sourced, so an early exit's teardown under `set -u`
        # finds it bound (round 2's reason for reading it first).
        assert 'source "$(dirname "${BASH_SOURCE[0]}")/vlm_hold.sh"' in code, name
        assert (code.index("trap teardown EXIT") < code.index('vlm_hold "$OUT')
                < code.index("systemctl isolate multi-user.target")), name
        teardown = code[code.index("teardown() {"):code.index("trap teardown EXIT")]
        assert teardown.index("isolate graphical.target") < teardown.index("vlm_restore"), name


def test_a_missing_ss_refuses_rather_than_passes():
    """Round 1 (Thermos): with no `ss`, every busy-port check passed."""
    for name in ("bench_local_vlm.sh", "headroom_identity.sh", "headroom_mission.sh"):
        for line in (l for l in _code(name).splitlines() if '"sport = :' in l):
            assert "! command -v ss >/dev/null ||" in line, (name, line)


def _serve_once(reply: bytes):
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    s.listen(1)

    def run():
        c, _ = s.accept()
        c.recv(65536)
        c.sendall(reply)
        c.close()
        s.close()
    threading.Thread(target=run, daemon=True).start()
    return f"http://127.0.0.1:{s.getsockname()[1]}"


def test_a_garbled_reply_is_one_failed_frame():
    """Round 1 (Thermos): `http.client`'s own errors (a garbage status line,
    a short body) are not `OSError` and still ended the run."""
    from tools.jetson.bench_local_vlm import post
    for reply in (b"garbage\r\n\r\n",
                  b"HTTP/1.1 200 OK\r\nContent-Length: 100\r\n\r\n{\"status\": "):
        got, _ = post(_serve_once(reply), b"\xff\xd8", "cup", 0.5, "x")
        assert got["status"].startswith("error") and got["score"] is None, (reply, got)


def test_a_reply_of_the_wrong_shape_is_one_failed_frame():
    """Round 1 (Thermos): `{"status": [..]}` or a string score passed
    through and crashed the summary."""
    from tools.jetson.bench_local_vlm import post
    for body in (b'{"status": ["a"], "score": 0.5}', b'{"status": "answered", "score": "0.9"}',
                 b'{"status": "answered", "score": true}'):
        reply = b"HTTP/1.1 200 OK\r\nContent-Length: %d\r\n\r\n%s" % (len(body), body)
        got, _ = post(_serve_once(reply), b"\xff\xd8", "cup", 0.5, "x")
        assert got == {"status": "error malformed reply", "score": None}, (body, got)


def test_the_mission_run_waits_for_its_own_vlm_before_restoring_the_production_one():
    """Round 2 (/code-review): teardown sent SIGTERM to its hand-started
    VLM and started the production unit at once, over a :8005 still held."""
    code = _code("headroom_mission.sh")
    teardown = code[code.index("teardown() {"):code.index("trap teardown EXIT")]
    assert teardown.index('kill -0 "$VLMP"') < teardown.index("vlm_restore")


# ---------- 3.108: the VLM enabled at boot, held off by benches ----------
import subprocess  # noqa: E402

_STUB = """
systemctl() { case "$1" in
    is-active) cat "$D/state";; is-enabled) echo enabled;; cat) cat "$D/unit";;
    show) echo no;;
    stop) echo inactive > "$D/state";;
    start) echo active > "$D/state"; echo start >> "$D/calls";;
    reset-failed) echo reset >> "$D/calls";; esac; }
sudo() { case "$1" in tee) [ -e "$D/notouch" ] && return 1; cat > "$D/flag";;
                      rm) rm -f "$D/flag";; *) "$@";; esac; }
source "$HELPER"; VLM_HOLD_FLAG="$D/flag"
"""


def _hold(tmp_path, state, body, unit=None, flag=None, notouch=False):
    (tmp_path / "state").write_text(state + "\n")
    (tmp_path / "unit").write_text(unit if unit is not None else _text("systemd/picar-local-vlm.service"))
    if flag is not None:
        (tmp_path / "flag").write_text(flag)
    if notouch:
        (tmp_path / "notouch").write_text("")
    r = subprocess.run(["bash", "-c", _STUB + body],
                       env={"D": str(tmp_path), "HELPER": str(JETSON / "vlm_hold.sh"),
                            "PATH": "/usr/bin:/bin"},
                       capture_output=True, text=True)
    calls = (tmp_path / "calls").read_text().split() if (tmp_path / "calls").exists() else []
    return r, (tmp_path / "state").read_text().strip(), (tmp_path / "flag").exists(), calls


def test_the_unit_skips_its_start_while_a_bench_holds_it():
    unit = _text("systemd/picar-local-vlm.service")
    assert "\nConditionPathExists=!/run/picar-vlm-hold\n" in unit
    assert "VLM_HOLD_FLAG=/run/picar-vlm-hold" in _text("vlm_hold.sh")


def test_the_bench_units_copy_drops_the_hold_condition():
    """Round 4: bench_local_vlm.sh's own unit is a sed copy of the production
    one; with the condition kept, the hold it sets first skipped its start."""
    code = _code("bench_local_vlm.sh")
    expr = re.search(r"-e '(/\^ConditionPathExists=[^']*/d)'", code).group(1)
    r = subprocess.run(["sed", "-e", expr], input=_text("systemd/picar-local-vlm.service"),
                       capture_output=True, text=True)
    assert r.returncode == 0 and "ConditionPathExists" not in r.stdout and "ExecStart=" in r.stdout


def test_a_running_vlm_is_held_then_started_again(tmp_path):
    r, state, flag, calls = _hold(tmp_path, "active", "vlm_hold; [ -e $D/flag ] && echo HELD; vlm_restore")
    assert r.returncode == 0, r.stdout + r.stderr
    assert "HELD" in r.stdout and not flag and state == "active"
    assert calls == ["reset", "start"]          # the start limit cleared before its one start


def test_a_stopped_vlm_stays_stopped(tmp_path):
    r, state, flag, calls = _hold(tmp_path, "inactive", "vlm_hold; vlm_restore")
    assert r.returncode == 0, r.stdout + r.stderr
    assert not flag and state == "inactive" and calls == []


def test_restore_without_a_hold_does_nothing(tmp_path):
    r, state, flag, calls = _hold(tmp_path, "inactive", "vlm_restore", flag="1")
    assert r.returncode == 0 and flag and calls == []      # someone else's flag is left


def test_a_nested_hold_leaves_the_outer_benchs_flag(tmp_path):
    """Round 4: top5.sh holds, then calls S/board/run.sh, which held again and
    cleared the shared flag at its exit."""
    live = subprocess.Popen(["sleep", "30"])
    try:
        r, state, flag, calls = _hold(tmp_path, "inactive", "vlm_hold; vlm_restore", flag=str(live.pid))
    finally:
        live.kill()
    assert r.returncode == 0 and flag and calls == [] and "the bench that called" in r.stdout


def test_a_stale_hold_is_taken_over_and_cleared(tmp_path):
    """Round 5: a flag left by a killed bench read as a caller's, so the
    next bench never cleared it and the VLM stayed off until a reboot."""
    r, state, flag, calls = _hold(tmp_path, "active", "vlm_hold; vlm_restore", flag="999999999")
    assert r.returncode == 0 and "stale" in r.stdout
    assert not flag and state == "active" and calls == ["reset", "start"]


def test_a_hold_that_cannot_set_its_flag_stops_nothing(tmp_path):
    r, state, flag, calls = _hold(tmp_path, "active", "vlm_hold || exit $?; echo GO", notouch=True)
    assert r.returncode == 2 and "GO" not in r.stdout and state == "active"


def test_a_hold_that_does_not_stop_it_refuses(tmp_path):
    r, *_ = _hold(tmp_path, "active",
                  'systemctl() { case $1 in is-active) echo active;; is-enabled) echo enabled;;'
                  ' cat) cat "$D/unit";; show) echo no;; esac; }; vlm_hold || exit $?; echo GO')
    assert r.returncode == 2 and "GO" not in r.stdout and "could not stop" in r.stdout


def test_a_board_whose_unit_has_no_condition_refuses(tmp_path):
    old = _text("systemd/picar-local-vlm.service").replace("ConditionPathExists=!/run/picar-vlm-hold\n", "")
    r, state, flag, calls = _hold(tmp_path, "active", "vlm_hold || exit $?; echo GO", unit=old)
    assert r.returncode == 2 and "GO" not in r.stdout and "no hold condition" in r.stdout
    assert state == "active" and not flag


def test_the_other_board_scripts_hold_it_too():
    root = JETSON.parent.parent / "evaluations"
    for rel in ("tiered-380/S/board/run.sh", "tiered-380/U/board/top5.sh", "inventory-346/mem346.sh"):
        code = (root / rel).read_text()
        assert 'vlm_hold.sh" || { echo "STOP' in code or 'vlm_hold.sh || { echo "STOP' in code, rel
        assert code.index("trap vlm_restore EXIT") < code.index("vlm_hold \""), rel   # the trap first
        assert "picar-local-vlm is running" not in code, rel
    top5 = (root / "tiered-380/U/board/top5.sh").read_text()
    assert top5.index("a bench server is running") < top5.index("vlm_hold \"")
    s = (root / "tiered-380/S/board/run.sh").read_text()
    assert s.index("vlm_hold \"") < s.index('if [ -n "$WANT_VE" ]')   # every arm, not VE only


def test_the_installer_checks_the_lock_and_starts_the_vlm_after_the_stack():
    code = _code("install_services.sh")
    assert 'source "$HERE/board_lock_check.sh"' in code
    assert code.index("brain/health >/dev/null && break") < code.index("restart picar-local-vlm")
    assert "reset-failed" not in code
    assert "rm -f /run/picar-vlm-hold" in code


def test_a_refused_hold_exits_without_the_teardown():
    """Round 5: a refusal after the trap ran the teardown, which kills the
    live twin stack's processes before the bench had touched anything."""
    for name in ("headroom_identity.sh", "headroom_mission.sh", "bench_local_vlm.sh"):
        assert 'vlm_hold "$OUT/vlm_hold.log" || { vlm_restore; trap - EXIT; exit 2; }' in _code(name), name
