"""Reproduce a second SIGINT inside real multiprocessing atexit cleanup."""

import os
import queue
import signal
import subprocess
import sys
import threading
import time

import pytest

_CHILD = r"""
import multiprocessing
import os
import signal
import sys
import time
from senior_robot import safety_node

child = multiprocessing.get_context("spawn").Process(target=time.sleep, args=(30,), daemon=True)
child.start()
original_waitpid = os.waitpid
injected = False

def audited_waitpid(pid, flags):
    global injected
    frame = sys._getframe()
    while frame is not None:
        if (frame.f_code.co_name == "_exit_function" and
                frame.f_globals.get("__name__") == "multiprocessing.util"):
            if not injected:
                injected = True
                print("IN_MP_ATEXIT", flush=True)
                os.kill(os.getpid(), signal.SIGINT)
            break
        frame = frame.f_back
    return original_waitpid(pid, flags)

os.waitpid = audited_waitpid
original_factory = safety_node.SafetyGate

def make_node():
    node = original_factory()
    print("READY", child.pid, flush=True)
    return node

safety_node.SafetyGate = make_node
safety_node.main()
print("MAIN_RETURNED", flush=True)
"""


def test_safety_cli_handles_late_sigint_during_multiprocessing_atexit(monkeypatch):
    pytest.importorskip("rclpy")
    domain = os.environ.get("SENIOR_MOTION_DOMAIN", "")
    assert domain.isdecimal() and 0 <= int(domain) <= 101
    monkeypatch.setenv("ROS_DOMAIN_ID", domain)
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    process = subprocess.Popen(
        [sys.executable, "-u", "-c", _CHILD],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        start_new_session=True,
    )
    lines = []
    pending = queue.Queue()

    def read():
        for line in process.stdout:
            lines.append(line)
            pending.put(line)

    reader = threading.Thread(target=read, daemon=True)
    reader.start()
    try:
        deadline = time.monotonic() + 15
        ready = None
        while time.monotonic() < deadline:
            try:
                line = pending.get(timeout=0.1)
            except queue.Empty:
                if process.poll() is not None:
                    break
                continue
            if line.startswith("READY "):
                ready = line
                break
        assert ready, "child failed to start: " + "".join(lines)
        process.send_signal(signal.SIGINT)
        process.wait(timeout=10)
        reader.join(timeout=2)
        output = "".join(lines)
        assert "MAIN_RETURNED" in output and "IN_MP_ATEXIT" in output, output
        assert process.returncode == 0, output
        assert "Traceback (most recent call last)" not in output, output
        assert "Exception ignored in atexit callback" not in output, output
        assert "KeyboardInterrupt" not in output, output
    finally:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=5)
        reader.join(timeout=2)
        if process.stdout is not None:
            process.stdout.close()
