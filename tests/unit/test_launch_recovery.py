"""Recovery contract exercised with real, non-ROS, non-motion processes."""

import json
import os
import shlex
import signal
import subprocess
import sys
import time
from contextlib import contextmanager
from pathlib import Path

import pytest


@contextmanager
def recovered_process(tmp_path, code):
    from senior_control.runtime.launch_recovery import restart_prefix

    environment = dict(os.environ)
    environment["PYTHONPATH"] = str(Path(__file__).parents[2] / "src")
    environment.pop("SENIOR_LAUNCH_RECOVERED", None)
    environment["SENIOR_STARTUP_LEARNING_CLAIM"] = "unchanged-run"
    with (tmp_path / "recovery.log").open("w+") as log:
        process = subprocess.Popen(
            [*shlex.split(restart_prefix()), sys.executable, "-c", code, str(tmp_path)],
            env=environment,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            yield process
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGTERM)
                process.wait(timeout=8)


def wait_for(path, process):
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline:
        if path.exists():
            return path.read_text()
        assert process.poll() is None, "wrapper exited before child was ready"
        time.sleep(0.02)
    pytest.fail(f"child did not write {path.name}")


def test_sigsegv_recovers_once_then_normal_exit_stays_finished(tmp_path):
    code = """
import json, os, resource, signal, sys, time
from pathlib import Path
resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
root = Path(sys.argv[1])
recovered = os.environ.get('SENIOR_LAUNCH_RECOVERED')
record = [os.getpid(), time.monotonic(), recovered,
          os.environ['SENIOR_STARTUP_LEARNING_CLAIM']]
with (root / 'attempts').open('a') as attempts:
    attempts.write(json.dumps(record) + '\\n')
(root / ('second' if recovered else 'first')).write_text(json.dumps(record))
if not recovered:
    print('original crash diagnostic', flush=True)
    os.kill(os.getpid(), signal.SIGSEGV)
"""
    with recovered_process(tmp_path, code) as process:
        first = json.loads(wait_for(tmp_path / "first", process))
        assert process.wait(timeout=8) == 0
        second = json.loads((tmp_path / "second").read_text())
    assert first[0] != second[0]
    assert second[1] - first[1] >= 2
    assert first[2:] == [None, "unchanged-run"]
    assert second[2:] == ["1", "unchanged-run"]
    log = (tmp_path / "recovery.log").read_text()
    assert "original crash diagnostic" in log
    assert f"pid={first[0]} returncode=-{signal.SIGSEGV}" in log
    assert "attempt=2" in log
    # Waiting beyond the retry delay catches a detached retry owner, too.
    time.sleep(2.1)
    assert len((tmp_path / "attempts").read_text().splitlines()) == 2


def test_signal_during_backoff_cancels_replacement(tmp_path):
    code = """
import os, sys
from pathlib import Path
root = Path(sys.argv[1])
path = root / ('second' if os.environ.get('SENIOR_LAUNCH_RECOVERED') else 'first')
path.write_text(str(os.getpid()))
raise SystemExit(9)
"""
    with recovered_process(tmp_path, code) as process:
        wait_for(tmp_path / "first", process)
        deadline = time.monotonic() + 2
        while "returncode=9" not in (tmp_path / "recovery.log").read_text():
            assert time.monotonic() < deadline
            time.sleep(0.01)
        start = time.monotonic()
        process.send_signal(signal.SIGTERM)
        assert process.wait(timeout=3) == 0
        assert time.monotonic() - start < 1
    time.sleep(2.1)
    assert not (tmp_path / "second").exists()


def test_old_descendant_exits_before_replacement_and_stop_reaps_new_group(tmp_path):
    code = """
import os, subprocess, sys, time
from pathlib import Path
root = Path(sys.argv[1])
if not os.environ.get('SENIOR_LAUNCH_RECOVERED'):
    child = subprocess.Popen([sys.executable, '-c',
        "import signal,sys,time; from pathlib import Path; "
        "signal.signal(signal.SIGINT, lambda *_: "
        "(Path(sys.argv[1]).write_text('exited'), sys.exit(0))); "
        "Path(sys.argv[2]).write_text('ready'); time.sleep(60)",
        str(root / 'old-exit'), str(root / 'old-ready')])
    (root / 'old-pid').write_text(str(child.pid))
    while not (root / 'old-ready').exists(): time.sleep(.01)
    raise SystemExit(7)
assert (root / 'old-exit').exists()
(root / 'second').write_text(str(os.getpid()))
time.sleep(60)
"""
    with recovered_process(tmp_path, code) as process:
        old = int(wait_for(tmp_path / "old-pid", process))
        new = int(wait_for(tmp_path / "second", process))
        with pytest.raises(ProcessLookupError):
            os.kill(old, 0)
        process.send_signal(signal.SIGINT)
        assert process.wait(timeout=8) == 0
        with pytest.raises(ProcessLookupError):
            os.kill(new, 0)
