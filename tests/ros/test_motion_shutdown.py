"""Bounded signal tests for the task-owned ROS nodes and geometry worker."""

import os
import queue
import re
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

_CONVERSION_ERROR = (
    "Unable to convert call argument to Python object (compile in debug mode for details)"
)

_CHILD = r"""
import importlib
import faulthandler
import signal
import sys
import time

from senior_control.models import Velocity
from senior_control.safety.async_guard import SafetyTicket

faulthandler.enable(all_threads=True)
faulthandler.register(signal.SIGUSR1, all_threads=True)
name = sys.argv[1]
sys.argv = [sys.argv[0], *sys.argv[2:]]
module = importlib.import_module("senior_robot." + name)
class_name = {"scan_adapter": "ScanAdapter", "safety_node": "SafetyGate",
              "motion_fusion": "MotionFusion", "safety_scan_ingress": "SafetyScanIngress"}[name]
original = getattr(module, class_name)

def construct():
    node = original()
    destroy = node.destroy_node
    def delayed_destroy():
        print("DESTROYING", flush=True)
        time.sleep(0.25)
        return destroy()
    node.destroy_node = delayed_destroy
    return node

setattr(module, class_name, construct)
from senior_robot import motion_lifecycle
real_owned_spin = motion_lifecycle._spin_cli

def owned_spin(node, executor, should_stop):
    shutdown = executor.shutdown
    def traced_shutdown(*args, **kwargs):
        print("EXECUTOR_SHUTDOWN_START", flush=True)
        try:
            return shutdown(*args, **kwargs)
        finally:
            print("EXECUTOR_SHUTDOWN_END", flush=True)
    executor.shutdown = traced_shutdown
    worker_pid = 0
    if name == "safety_node":
        ticket = SafetyTicket(1, 2, 3, "test", Velocity(0.0, 0.0), 1.0, 2.0, 1.0, 2.0)
        assert node._worker.submit(ticket, None)
        deadline = time.monotonic() + 10
        reply = None
        while reply is None and time.monotonic() < deadline:
            reply = node._worker.poll()
            time.sleep(0.01)
        assert reply is not None and reply.error
        assert node._worker._process is not None and node._worker._process.is_alive()
        worker_pid = node._worker._process.pid
    print("READY", worker_pid, flush=True)
    return real_owned_spin(node, executor, should_stop)

motion_lifecycle._spin_cli = owned_spin
module.main()
"""

_ACTIVE_CHILD = (
    _CHILD.rsplit("module.main()", 1)[0]
    + r"""
import json
from senior_robot import motion_lifecycle

real_spin = motion_lifecycle._spin_cli
def audited_spin(node, executor, should_stop):
    try:
        return real_spin(node, executor, should_stop)
    except RuntimeError as exc:
        frames = []
        tb = exc.__traceback__
        while tb is not None:
            frames.append([tb.tb_frame.f_globals.get("__name__"), tb.tb_frame.f_code.co_name])
            tb = tb.tb_next
        print("NATIVE_RUNTIME_DIAGNOSTIC", json.dumps({
            "context_ok": motion_lifecycle.rclpy.ok(), "message": str(exc), "frames": frames,
        }), flush=True)
        raise
motion_lifecycle._spin_cli = audited_spin
module.main()
"""
)


@pytest.mark.parametrize(
    "entry", ["scan_adapter", "safety_node", "motion_fusion", "safety_scan_ingress"]
)
@pytest.mark.parametrize("first_signal", [signal.SIGINT, signal.SIGTERM])
def test_owned_nodes_handle_signal_and_repeated_interrupt(entry, first_signal, monkeypatch):
    pytest.importorskip("rclpy")
    domain = os.environ.get("SENIOR_MOTION_DOMAIN", "")
    assert domain.isdecimal() and 0 <= int(domain) <= 232, "choose an isolated ROS domain"
    monkeypatch.setenv("ROS_DOMAIN_ID", domain)
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    params = Path(__file__).parents[2] / "ros2/src/senior_robot/config/simulation_motion.yaml"
    process = subprocess.Popen(
        [
            sys.executable,
            "-u",
            "-c",
            _CHILD,
            entry,
            "--ros-args",
            "--params-file",
            str(params),
            "-p",
            "use_sim_time:=false",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        start_new_session=True,
    )
    lines = []
    pending = queue.Queue()

    def read_lines():
        assert process.stdout is not None
        for line in process.stdout:
            lines.append(line)
            pending.put(line)

    reader = threading.Thread(target=read_lines, daemon=True)
    reader.start()
    worker_pid = 0

    def wait_for(marker):
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            try:
                line = pending.get(timeout=0.1)
            except queue.Empty:
                if process.poll() is not None:
                    break
                continue
            if line.startswith(marker):
                return line
        if process.poll() is None:
            process.send_signal(signal.SIGUSR1)
            time.sleep(0.2)
        pytest.fail(f"child did not report {marker}; exit={process.poll()}: {''.join(lines)}")

    try:
        ready = wait_for("READY ")
        worker_pid = int(ready.split()[1])
        if entry == "safety_node" and first_signal == signal.SIGINT:
            assert worker_pid > 0
            os.killpg(process.pid, first_signal)
        else:
            process.send_signal(first_signal)
        wait_for("DESTROYING")
        # Exercise a second signal while node and worker cleanup is still running.
        process.send_signal(signal.SIGINT)
        process.wait(timeout=8)
        reader.join(timeout=2)
        output = "".join(lines)
        assert process.returncode == 0, output
        assert "Traceback (most recent call last)" not in output, output
        assert "rcl_shutdown already called" not in output, output
        assert "leaked semaphore" not in output, output
        if worker_pid:

            def worker_active():
                try:
                    state = Path(f"/proc/{worker_pid}/stat").read_text().rsplit(")", 1)[1]
                    return state.strip().split()[0] != "Z"
                except FileNotFoundError:
                    return False

            deadline = time.monotonic() + 3
            while worker_active() and time.monotonic() < deadline:
                time.sleep(0.02)
            assert not worker_active(), output
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
        if worker_pid:
            try:
                if os.getpgid(worker_pid) == process.pid:
                    os.kill(worker_pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        reader.join(timeout=2)
        if process.stdout is not None:
            process.stdout.close()


@pytest.mark.parametrize("context_running", [True, False])
def test_context_error_is_suppressed_only_after_shutdown(monkeypatch, context_running):
    pytest.importorskip("rclpy")
    from rclpy._rclpy_pybind11 import RCLError  # //Compatible: Humble exposes it here.
    from senior_robot import motion_lifecycle

    calls = []

    class Node:
        def destroy_node(self):
            calls.append("destroy")

    def fail_spin(_node, *, executor):
        raise RCLError("wait set context invalid")

    monkeypatch.setattr(
        motion_lifecycle,
        "SingleThreadedExecutor",
        lambda: type("TestExecutor", (), {"shutdown": lambda self: None})(),
    )
    monkeypatch.setattr(motion_lifecycle.rclpy, "init", lambda: calls.append("init"))
    monkeypatch.setattr(motion_lifecycle.rclpy, "spin", fail_spin)
    monkeypatch.setattr(motion_lifecycle.rclpy, "ok", lambda: context_running)
    monkeypatch.setattr(motion_lifecycle.rclpy, "try_shutdown", lambda: calls.append("shutdown"))
    monkeypatch.setattr(motion_lifecycle.signal, "signal", lambda *_: None)
    if context_running:
        with pytest.raises(RCLError, match="wait set context invalid"):
            motion_lifecycle.run(Node)
    else:
        motion_lifecycle.run(Node)
    assert calls == ["init", "destroy", "shutdown"]


@pytest.mark.parametrize(
    ("context_running", "message", "subscription_frame", "suppressed"),
    [
        (False, _CONVERSION_ERROR, True, True),
        (True, _CONVERSION_ERROR, True, False),
        (False, "different subscription failure", True, False),
        (False, _CONVERSION_ERROR, False, False),
    ],
)
def test_subscription_conversion_is_only_suppressed_at_closed_context_boundary(
    monkeypatch, context_running, message, subscription_frame, suppressed
):
    from types import SimpleNamespace

    pytest.importorskip("rclpy")
    from rclpy.executors import Executor
    from senior_robot import motion_lifecycle

    calls = []

    class Node:
        def destroy_node(self):
            calls.append("destroy")

    class FailingHandle:
        def __enter__(self):
            return self

        def __exit__(self, *_):
            return False

        def take_message(self, *_):
            raise RuntimeError(message)

    def fail_spin(_node, *, executor):
        if subscription_frame:
            # Actual Humble executor method; only its test-owned handle is injected.
            sub = SimpleNamespace(handle=FailingHandle(), msg_type=object, raw=False)
            return Executor._take_subscription(None, sub)
        raise RuntimeError(message)

    monkeypatch.setattr(
        motion_lifecycle,
        "SingleThreadedExecutor",
        lambda: type("TestExecutor", (), {"shutdown": lambda self: None})(),
    )
    monkeypatch.setattr(motion_lifecycle.rclpy, "init", lambda: calls.append("init"))
    monkeypatch.setattr(motion_lifecycle.rclpy, "spin", fail_spin)
    monkeypatch.setattr(motion_lifecycle.rclpy, "ok", lambda: context_running)
    monkeypatch.setattr(motion_lifecycle.rclpy, "try_shutdown", lambda: calls.append("shutdown"))
    monkeypatch.setattr(motion_lifecycle.signal, "signal", lambda *_: None)
    if suppressed:
        motion_lifecycle.run(Node)
    else:
        with pytest.raises(RuntimeError, match=re.escape(message)):
            motion_lifecycle.run(Node)
    assert calls == ["init", "destroy", "shutdown"]


def test_safety_shutdown_with_active_command_publisher(monkeypatch):
    rclpy = pytest.importorskip("rclpy")
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Twist
    from senior_motion_msgs.msg import FinalCommand

    domain = os.environ.get("SENIOR_MOTION_DOMAIN", "")
    assert domain.isdecimal() and 0 <= int(domain) <= 232, "choose an isolated ROS domain"
    monkeypatch.setenv("ROS_DOMAIN_ID", domain)
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    params = Path(__file__).parents[2] / "ros2/src/senior_robot/config/simulation_motion.yaml"
    process = subprocess.Popen(
        [
            sys.executable,
            "-u",
            "-c",
            _ACTIVE_CHILD,
            "safety_node",
            "--ros-args",
            "--params-file",
            str(params),
            "-p",
            "use_sim_time:=false",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        start_new_session=True,
    )
    lines = []
    pending = queue.Queue()

    def read_lines():
        assert process.stdout is not None
        for line in process.stdout:
            lines.append(line)
            pending.put(line)

    reader = threading.Thread(target=read_lines, daemon=True)
    reader.start()
    node = None
    worker_pid = 0
    stop = threading.Event()
    publisher_thread = None
    try:
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            try:
                line = pending.get(timeout=0.1)
            except queue.Empty:
                if process.poll() is not None:
                    break
                continue
            if line.startswith("READY "):
                worker_pid = int(line.split()[1])
                break
        assert worker_pid > 0, "owned worker did not start"
        rclpy.init()
        node = rclpy.create_node("command_node")
        publisher = node.create_publisher(FinalCommand, "/safety/final_command", 10)
        statuses, outputs = [], []
        node.create_subscription(DiagnosticArray, "/safety/status", statuses.append, 10)
        node.create_subscription(Twist, "/cmd_vel", outputs.append, 10)
        barrier = FinalCommand(
            robot_id="robot",
            owner_boot_id="shutdown-test",
            generation=1,
            source_kind="idle",
            received_steady=time.monotonic(),
            close_reason="prepared",
        )
        sequence = 0

        def publish_command():
            nonlocal sequence
            sequence += 1
            received = time.monotonic()
            publisher.publish(
                FinalCommand(
                    robot_id="robot",
                    owner_boot_id="shutdown-test",
                    session_id="shutdown-fixture",
                    generation=1,
                    connection_epoch=1,
                    sequence=sequence,
                    steady_deadline=received + 0.3,
                    received_steady=received,
                    source_kind="manual",
                    source_id="shutdown-fixture",
                    source_epoch=1,
                    close_reason="intent",
                    gesture=1,
                    request_id=f"shutdown-{sequence}",
                    vx=0.05,
                )
            )

        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            publisher.publish(barrier)
            publish_command()
            rclpy.spin_once(node, timeout_sec=0.02)
            if outputs and any(
                status.status
                and any(
                    item.key == "request_vx" and float(item.value) == 0.05
                    for item in status.status[0].values
                )
                for status in statuses
            ):
                break
        else:
            pytest.fail("SafetyGate did not consume the active test-only command stream")
        assert all(item.linear.x == 0.0 and item.angular.z == 0.0 for item in outputs)

        def flood():
            while not stop.is_set():
                publish_command()
                time.sleep(0.002)

        publisher_thread = threading.Thread(target=flood, daemon=True)
        publisher_thread.start()
        time.sleep(0.2)
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=8)
        reader.join(timeout=2)
        output = "".join(lines)
        assert process.returncode == 0, output
        assert "Traceback (most recent call last)" not in output, output
        assert "leaked semaphore" not in output, output
    finally:
        stop.set()
        if publisher_thread is not None:
            publisher_thread.join(timeout=2)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
        if worker_pid:
            try:
                if os.getpgid(worker_pid) == process.pid:
                    os.kill(worker_pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        reader.join(timeout=2)
        if process.stdout is not None:
            process.stdout.close()
