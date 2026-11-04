"""Real signal shutdown leaves neither traceback nor an owned IPC socket."""

import os
import signal
import subprocess
import sys

import pytest


def adapter_command(script, tmp_path):
    # Missing latch state starts fail-closed, which is sufficient for IPC lifecycle tests.
    state = tmp_path / "estop.json"
    return [sys.executable, "-c", script, "--ros-args", "-p", f"safety_state_path:={state}"]


@pytest.mark.parametrize("signal_name", ["SIGINT", "SIGTERM"])
def test_adapter_signal_closes_ipc_and_exits_cleanly(tmp_path, signal_name):
    pytest.importorskip("rclpy")
    socket_path = tmp_path / "adapter.sock"
    script = '''
from senior_control.runtime.ipc import IpcServer
from senior_robot.agent_adapter import main

start = IpcServer.start
async def ready(self):
    await start(self)
    print("SOCKET_READY", flush=True)
IpcServer.start = ready
main()
print("STOPPED", flush=True)
'''
    env = dict(os.environ, SENIOR_AGENT_SOCKET=str(socket_path))
    proc = subprocess.Popen(adapter_command(script, tmp_path), stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, text=True, env=env)
    try:
        import selectors
        selector = selectors.DefaultSelector()
        selector.register(proc.stdout, selectors.EVENT_READ)
        assert selector.select(timeout=10), "adapter did not start"
        assert proc.stdout.readline().strip() == "SOCKET_READY"
        assert socket_path.exists()
        os.kill(proc.pid, getattr(signal, signal_name))
        stdout, stderr = proc.communicate(timeout=10)
        assert proc.returncode == 0, stdout + stderr
        assert "STOPPED" in stdout
        assert "Traceback" not in stderr and "Exception ignored" not in stderr
        assert not socket_path.exists()
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.communicate(timeout=5)


@pytest.mark.parametrize("signal_name", ["SIGINT", "SIGTERM"])
@pytest.mark.parametrize("stage", ["startup", "serving"])
def test_adapter_repeated_signals_close_owned_resources(tmp_path, signal_name, stage):
    pytest.importorskip("rclpy")
    socket_path = tmp_path / "adapter.sock"
    script = '''
import os
import signal
import rclpy
from senior_control.runtime.ipc import IpcServer
from senior_robot.agent_adapter import main

signum = getattr(signal, os.environ["TEST_SIGNAL"])
def interrupt():
    print("SIGNALS_SENT", flush=True)
    os.kill(os.getpid(), signum)
    os.kill(os.getpid(), signum)
if os.environ["TEST_STAGE"] == "startup":
    init = rclpy.init
    def interrupted_init(*args, **kwargs):
        interrupt()
        return init(*args, **kwargs)
    rclpy.init = interrupted_init
else:
    start = IpcServer.start
    async def interrupted_start(self):
        await start(self)
        interrupt()
    IpcServer.start = interrupted_start
main()
print("STOPPED", flush=True)
'''
    env = dict(os.environ, SENIOR_AGENT_SOCKET=str(socket_path), TEST_SIGNAL=signal_name,
               TEST_STAGE=stage)
    proc = subprocess.run(adapter_command(script, tmp_path), capture_output=True, text=True,
                          timeout=10, env=env, check=False)
    assert "SIGNALS_SENT" in proc.stdout, proc.stdout + proc.stderr
    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert "STOPPED" in proc.stdout
    assert "Traceback" not in proc.stderr and "Exception ignored" not in proc.stderr
    assert not socket_path.exists()


@pytest.mark.parametrize("failure", ["spin", "release"])
def test_adapter_live_failure_propagates_and_removes_socket(tmp_path, failure):
    pytest.importorskip("rclpy")
    socket_path = tmp_path / "adapter.sock"
    script = '''
import os
import rclpy
from rclpy.executors import SingleThreadedExecutor
from senior_control.runtime.ipc import IpcServer
from senior_robot.agent_adapter import AdapterCore, main

if os.environ["TEST_FAILURE"] == "spin":
    def fail_spin(self, timeout_sec=None):
        raise ValueError("live spin failure")
    SingleThreadedExecutor.spin_once = fail_spin
else:
    start = IpcServer.start
    async def stop_after_start(self):
        await start(self)
        rclpy.try_shutdown()
    IpcServer.start = stop_after_start
    def fail_release(self):
        raise ValueError("release failure")
    AdapterCore.release = fail_release
try:
    main()
except ValueError as exc:
    assert str(exc) == ("live spin failure" if os.environ["TEST_FAILURE"] == "spin"
                        else "release failure")
    print("FAILURE_PROPAGATED", flush=True)
else:
    raise AssertionError("failure was swallowed")
'''
    env = dict(os.environ, SENIOR_AGENT_SOCKET=str(socket_path), TEST_FAILURE=failure)
    proc = subprocess.run(adapter_command(script, tmp_path), capture_output=True, text=True,
                          timeout=10, env=env, check=False)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert "FAILURE_PROPAGATED" in proc.stdout
    assert "Traceback" not in proc.stderr and "Exception ignored" not in proc.stderr
    assert not socket_path.exists()


def test_adapter_failed_ipc_start_preserves_existing_file(tmp_path):
    pytest.importorskip("rclpy")
    socket_path = tmp_path / "occupied"
    socket_path.write_text("keep")
    script = '''
from senior_robot.agent_adapter import main
try:
    main()
except OSError:
    print("BIND_FAILED", flush=True)
else:
    raise AssertionError("binding over a regular file succeeded")
'''
    proc = subprocess.run(adapter_command(script, tmp_path), capture_output=True, text=True,
                          timeout=10, env=dict(os.environ, SENIOR_AGENT_SOCKET=str(socket_path)),
                          check=False)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert "BIND_FAILED" in proc.stdout
    assert socket_path.read_text() == "keep"
