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
    assert 'case "$PROD_WAS" in active|activating) sudo systemctl start picar-local-vlm' in code
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
        read = code.index("VLM_WAS=$(systemctl is-active picar-local-vlm")
        # Before the trap too (round 2): read after it, an early exit's
        # teardown would stop on an unbound VLM_WAS under `set -u`.
        assert read < code.index("trap teardown EXIT") < code.index(
            "systemctl isolate multi-user.target"), name
        assert 'case "$VLM_WAS" in active|activating) sudo systemctl start picar-local-vlm' in code


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
    assert teardown.index('kill -0 "$VLMP"') < teardown.index("systemctl start picar-local-vlm")
