"""The library lifecycle restores caller signal policy and propagates live errors."""

import signal
from types import SimpleNamespace

import pytest


@pytest.mark.parametrize("mode", ["shutdown", "live_failure"])
def test_library_run_restores_sigint_and_propagates_live_failures(monkeypatch, mode):
    pytest.importorskip("rclpy")
    from rclpy.executors import ExternalShutdownException
    from senior_robot import motion_lifecycle as lifecycle

    events = []
    node = SimpleNamespace(destroy_node=lambda: events.append("destroyed"))
    monkeypatch.setattr(lifecycle.rclpy, "init", lambda: events.append("init"))
    monkeypatch.setattr(lifecycle.rclpy, "try_shutdown", lambda: events.append("shutdown"))
    monkeypatch.setattr(lifecycle.rclpy, "ok", lambda: True)
    monkeypatch.setattr(lifecycle, "SingleThreadedExecutor",
                        lambda: SimpleNamespace(shutdown=lambda: None))
    monkeypatch.setattr(lifecycle.rclpy.signals, "uninstall_signal_handlers", lambda: None)

    def spin(_node, *, executor):
        if mode == "shutdown":
            raise ExternalShutdownException()
        raise RuntimeError("real live failure")

    monkeypatch.setattr(lifecycle.rclpy, "spin", spin)
    previous = signal.getsignal(signal.SIGINT)

    def custom_handler(_signum, _frame):
        pass

    signal.signal(signal.SIGINT, custom_handler)
    try:
        if mode == "shutdown":
            lifecycle.run(lambda: node)
        else:
            with pytest.raises(RuntimeError, match="real live failure"):
                lifecycle.run(lambda: node)
        assert signal.getsignal(signal.SIGINT) is custom_handler
        assert events == ["init", "destroyed", "shutdown"]
    finally:
        signal.signal(signal.SIGINT, previous)


def test_cli_mode_does_not_swallow_a_live_runtime_failure(monkeypatch):
    pytest.importorskip("rclpy")
    from senior_robot import motion_lifecycle as lifecycle

    events = []
    node = SimpleNamespace(destroy_node=lambda: events.append("destroyed"))
    monkeypatch.setattr(lifecycle.rclpy, "init", lambda **_: None)
    monkeypatch.setattr(lifecycle.rclpy, "try_shutdown", lambda: events.append("shutdown"))
    monkeypatch.setattr(lifecycle.rclpy, "ok", lambda: True)
    monkeypatch.setattr(lifecycle, "SingleThreadedExecutor",
                        lambda: SimpleNamespace(shutdown=lambda: None))
    monkeypatch.setattr(lifecycle.rclpy.signals, "uninstall_signal_handlers", lambda: None)
    monkeypatch.setattr(lifecycle.rclpy.signals, "install_signal_handlers", lambda _: None)

    def spin(_node, *, executor):
        raise RuntimeError("real live failure")

    monkeypatch.setattr(lifecycle.rclpy, "spin", spin)
    previous = signal.getsignal(signal.SIGINT)
    try:
        with pytest.raises(RuntimeError, match="real live failure"):
            lifecycle.run(lambda: node, cli=True)
        assert signal.getsignal(signal.SIGINT) == signal.SIG_IGN
        assert events == ["destroyed", "shutdown"]
    finally:
        signal.signal(signal.SIGINT, previous)


def test_context_shutdown_between_factory_and_spin_cleans_up_without_global_executor_race():
    """Exercise Humble's real already-closed context at the spin boundary."""
    import subprocess
    import sys

    pytest.importorskip("rclpy")
    script = '''
import rclpy
from rclpy.node import Node
from senior_robot.motion_lifecycle import run

def factory():
    node = Node("shutdown_before_spin")
    rclpy.try_shutdown()
    return node

run(factory)
print("SHUTDOWN_COMPLETE", flush=True)
'''
    result = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True,
                            timeout=10, check=False)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "SHUTDOWN_COMPLETE" in result.stdout
    assert "Traceback" not in result.stderr


def test_factory_cleans_partial_resources_before_reraising_interrupt(monkeypatch):
    pytest.importorskip("rclpy")
    from senior_robot import motion_lifecycle as lifecycle

    events = []
    monkeypatch.setattr(lifecycle.rclpy, "init", lambda: None)
    monkeypatch.setattr(lifecycle.rclpy, "try_shutdown", lambda: events.append("shutdown"))
    monkeypatch.setattr(lifecycle, "SingleThreadedExecutor",
                        lambda: SimpleNamespace(shutdown=lambda: events.append("executor_closed")))

    def factory():
        try:
            events.append("resource_created")
            raise KeyboardInterrupt()
        except BaseException:
            events.append("factory_closed_resource")
            raise

    lifecycle.run(factory)
    assert events == ["resource_created", "factory_closed_resource", "executor_closed", "shutdown"]


@pytest.mark.parametrize(("module", "class_name"), [
    ("scan_adapter", "ScanAdapter"), ("safety_node", "SafetyGate"),
    ("motion_fusion", "MotionFusion"), ("safety_scan_ingress", "SafetyScanIngress"),
    ("depth_ingress", "DepthIngress"), ("semantic_ingress", "SemanticIngress"),
])
def test_entrypoint_constructor_does_not_start_worker(module, class_name, monkeypatch):
    import importlib
    import multiprocessing.process

    rclpy = pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter

    values = {}
    if module == "motion_fusion":
        values = dict(sensor_max_age=0.8, alignment_max_gap=0.25, pending_wait=0.25,
                      max_variance=0.1, min_quality=0.3, max_yaw_error=0.1,
                      startup_window=0.6, wheel_motion_limit=0.004, stall_distance=0.04,
                      lateral_limit=0.04, yaw_limit=0.12)
    elif module == "depth_ingress":
        values = dict(min_range_m=0.1, max_range_m=8.0, range_error_m=0.01, max_age_s=0.5)
    monkeypatch.setattr(multiprocessing.process.BaseProcess, "start",
                        lambda _: pytest.fail("constructor started a process"))
    rclpy.init()
    node = None
    try:
        node = getattr(importlib.import_module("senior_robot." + module), class_name)(
            parameter_overrides=[Parameter(key, value=value) for key, value in values.items()]
        )
        if module == "safety_node":
            assert node._worker._process is None and node._worker._supervisor is None
        if module == "semantic_ingress":
            worker = node._coordinator._worker
            assert worker._process is None and worker._supervisor is None
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("first_mode", ["real_signal", "injected_exception"])
def test_second_sigint_during_first_interrupt_matching_finishes_cleanup(first_mode):
    """Exercise two actual signals and an interrupt already being handled."""
    import subprocess
    import sys

    pytest.importorskip("rclpy")
    script = '''
import inspect
import faulthandler
import signal
import sys
from rclpy.node import Node
from senior_robot import motion_lifecycle as lifecycle

faulthandler.enable(all_threads=True)
real_spin = lifecycle.rclpy.spin
source, start = inspect.getsourcelines(lifecycle.run)
match_line = next(start + i for i, line in enumerate(source)
                  if "except (KeyboardInterrupt, ExternalShutdownException)" in line)
def trace(frame, event, arg):
    if frame.f_code is lifecycle.run.__code__ and event == "line" and frame.f_lineno == match_line:
        sys.settrace(None)
        print("SIGINT_SECOND", flush=True)
        signal.raise_signal(signal.SIGINT)
    return trace

def spin(node, *, executor):
    FIRST_INTERRUPT

lifecycle.rclpy.spin = spin
sys.settrace(trace)
lifecycle.run(lambda: Node("double_signal_boundary"), cli=True)
sys.settrace(None)
print("SHUTDOWN_COMPLETE", flush=True)
'''
    first = ('''print("SIGINT_FIRST", flush=True)
    signal.raise_signal(signal.SIGINT)
    print("SIGINT_SECOND", flush=True)
    signal.raise_signal(signal.SIGINT)
    return real_spin(node, executor=executor)''' if first_mode == "real_signal"
             else "raise KeyboardInterrupt()")
    script = script.replace("FIRST_INTERRUPT", first)
    result = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True,
                            timeout=10, check=False)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "SIGINT_SECOND" in result.stdout, result.stdout + result.stderr
    if first_mode == "real_signal":
        assert "SIGINT_FIRST" in result.stdout
    assert "SHUTDOWN_COMPLETE" in result.stdout
    assert "Traceback" not in result.stderr


def test_cli_sigint_still_chains_handler_and_wakes_native_executor():
    import subprocess
    import sys

    pytest.importorskip("rclpy")
    script = '''
import os
import signal
import threading
import time
import rclpy
from rclpy.node import Node
from senior_robot.motion_lifecycle import run

seen = []
def prior(signum, frame):
    seen.append(signum)
signal.signal(signal.SIGINT, prior)
def factory():
    node = Node("native_signal_wakeup")
    def send():
        time.sleep(0.2)
        os.kill(os.getpid(), signal.SIGINT)
    threading.Thread(target=send, daemon=True).start()
    return node
run(factory, cli=True)
assert seen == [signal.SIGINT], seen
assert not rclpy.ok()
print("NATIVE_WAKE_COMPLETE", flush=True)
'''
    result = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True,
                            timeout=10, check=False)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "NATIVE_WAKE_COMPLETE" in result.stdout


def test_cli_initialization_failure_restores_original_signal_handler(monkeypatch):
    pytest.importorskip("rclpy")
    from senior_robot import motion_lifecycle as lifecycle

    previous = signal.getsignal(signal.SIGINT)
    previous_term = signal.getsignal(signal.SIGTERM)
    def custom(_signum, _frame):
        pass
    signal.signal(signal.SIGINT, custom)
    signal.signal(signal.SIGTERM, custom)
    monkeypatch.setattr(lifecycle.rclpy, "init",
                        lambda **_: (_ for _ in ()).throw(ValueError("init")))
    try:
        with pytest.raises(ValueError, match="init"):
            lifecycle.run(lambda: None, cli=True)
        assert signal.getsignal(signal.SIGINT) is custom
        assert signal.getsignal(signal.SIGTERM) is custom
    finally:
        signal.signal(signal.SIGINT, previous)
        signal.signal(signal.SIGTERM, previous_term)


def test_cli_sigint_during_non_shutdown_error_still_calls_prior_handler():
    import subprocess
    import sys

    pytest.importorskip("rclpy")
    script = '''
import inspect
import signal
import sys
from rclpy.node import Node
from senior_robot import motion_lifecycle as lifecycle

seen = []
def previous(signum, frame):
    seen.append(signum)
signal.signal(signal.SIGINT, previous)
source, start = inspect.getsourcelines(lifecycle.run)
cleanup_line = next(start + i for i, line in enumerate(source)
                    if "previous = signal.signal(signal.SIGINT, signal.SIG_IGN)" in line)
def trace(frame, event, arg):
    if (frame.f_code is lifecycle.run.__code__ and event == "line"
        and frame.f_lineno == cleanup_line):
        sys.settrace(None)
        signal.raise_signal(signal.SIGINT)
    return trace
def spin(node, *, executor):
    raise ValueError("live failure")
lifecycle.rclpy.spin = spin
sys.settrace(trace)
try:
    lifecycle.run(lambda: Node("failure_boundary"), cli=True)
except ValueError as exc:
    assert str(exc) == "live failure"
else:
    raise AssertionError("live error was swallowed")
assert seen == [signal.SIGINT], seen
print("LIVE_ERROR_PRESERVED", flush=True)
'''
    result = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True,
                            timeout=10, check=False)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "LIVE_ERROR_PRESERVED" in result.stdout


@pytest.mark.parametrize("cli", [False, True])
def test_cli_joins_native_signal_thread_before_executor_guard_destruction(monkeypatch, cli):
    pytest.importorskip("rclpy")
    from senior_robot import motion_lifecycle as lifecycle

    events = []
    monkeypatch.setattr(lifecycle.rclpy, "init", lambda **_: None)
    monkeypatch.setattr(lifecycle.rclpy, "ok", lambda: True)
    monkeypatch.setattr(lifecycle.rclpy, "spin", lambda node, *, executor: None)
    monkeypatch.setattr(lifecycle.rclpy, "try_shutdown", lambda: events.append("context"))
    monkeypatch.setattr(lifecycle.rclpy.signals, "uninstall_signal_handlers",
                        lambda: events.append("native_signal_join"))
    monkeypatch.setattr(lifecycle.rclpy.signals, "install_signal_handlers", lambda _: None)
    monkeypatch.setattr(lifecycle, "SingleThreadedExecutor",
                        lambda: SimpleNamespace(shutdown=lambda: events.append("executor")))
    monkeypatch.setattr(lifecycle.signal, "getsignal", lambda _: signal.SIG_IGN)
    monkeypatch.setattr(lifecycle.signal, "signal", lambda *_: signal.SIG_IGN)
    lifecycle.run(lambda: SimpleNamespace(destroy_node=lambda: events.append("node")), cli=cli)
    assert events == (["native_signal_join"] if cli else []) + ["executor", "node", "context"]


def test_real_sigint_cannot_interrupt_ros_task_before_future_initialized():
    """Deliver SIGINT at the actual Humble Future field-initialization boundary."""
    import subprocess
    import sys

    pytest.importorskip("rclpy")
    script = '''
import faulthandler
import gc
import inspect
import os
import signal
import sys
from rclpy.node import Node
from rclpy.task import Future, Task
from senior_robot import motion_lifecycle as lifecycle

faulthandler.enable(all_threads=True)
source, start = inspect.getsourcelines(Future.__init__)
target = next(start + i for i, line in enumerate(source) if "self._exception = None" in line)
real_spin = lifecycle.rclpy.spin

def trace(frame, event, arg):
    if frame.f_code is Future.__init__.__code__ and event == "line" and frame.f_lineno == target:
        sys.settrace(None)
        print("SIGINT_AT_FUTURE_BOUNDARY", flush=True)
        os.kill(os.getpid(), signal.SIGINT)
    return trace

def spin(node, *, executor):
    sys.settrace(trace)
    task = Task(lambda: None)
    del task
    gc.collect()
    return real_spin(node, executor=executor)

lifecycle.rclpy.spin = spin
lifecycle.run(lambda: Node("future_signal_boundary"), cli=True)
gc.collect()
print("SHUTDOWN_COMPLETE", flush=True)
'''
    result = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True,
                            timeout=10, check=False)
    assert "SIGINT_AT_FUTURE_BOUNDARY" in result.stdout, result.stdout + result.stderr
    assert result.returncode == 0, result.stdout + result.stderr
    assert "SHUTDOWN_COMPLETE" in result.stdout
    assert "Exception ignored" not in result.stderr, result.stderr
    assert "Traceback" not in result.stderr, result.stderr


def test_cli_sigint_before_native_init_is_remembered_and_prevents_factory():
    """An early real SIGINT must not be lost before ROS installs its own handler."""
    import subprocess
    import sys

    pytest.importorskip("rclpy")
    script = '''
import os
import signal
import rclpy
from senior_robot import motion_lifecycle as lifecycle

real_init = lifecycle.rclpy.init
def interrupted_init(**kwargs):
    print("EARLY_SIGINT_FIRST", flush=True)
    os.kill(os.getpid(), signal.SIGINT)
    print("EARLY_SIGINT_SECOND", flush=True)
    os.kill(os.getpid(), signal.SIGINT)
    return real_init(**kwargs)
def factory():
    print("FACTORY_STARTED", flush=True)
    raise ValueError("factory must not run after early stop")
lifecycle.rclpy.init = interrupted_init
lifecycle.run(factory, cli=True)
assert not rclpy.ok()
print("EARLY_STOP_COMPLETE", flush=True)
'''
    result = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True,
                            timeout=10, check=False)
    assert "EARLY_SIGINT_FIRST" in result.stdout and "EARLY_SIGINT_SECOND" in result.stdout
    assert result.returncode == 0, result.stdout + result.stderr
    assert "EARLY_STOP_COMPLETE" in result.stdout
    assert "FACTORY_STARTED" not in result.stdout
    assert "Traceback" not in result.stderr and "Exception ignored" not in result.stderr


def test_cli_context_closed_during_init_does_not_construct_executor_or_factory():
    import subprocess
    import sys

    pytest.importorskip("rclpy")
    script = '''
import rclpy
from senior_robot import motion_lifecycle as lifecycle

real_init = lifecycle.rclpy.init
def interrupted_init(**kwargs):
    real_init(**kwargs)
    rclpy.try_shutdown()
def factory():
    print("FACTORY_STARTED", flush=True)
    raise ValueError("closed context must not start a node")
lifecycle.rclpy.init = interrupted_init
lifecycle.run(factory, cli=True)
print("CLOSED_CONTEXT_STOP_COMPLETE", flush=True)
'''
    result = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True,
                            timeout=10, check=False)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "CLOSED_CONTEXT_STOP_COMPLETE" in result.stdout
    assert "FACTORY_STARTED" not in result.stdout
    assert "Traceback" not in result.stderr and "Exception ignored" not in result.stderr


@pytest.mark.parametrize("signum", ["SIGINT", "SIGTERM"])
def test_cli_signal_during_executor_construction_cannot_close_context_mid_init(signum):
    import subprocess
    import sys

    pytest.importorskip("rclpy")
    script = '''
import os
import signal
import sys
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from senior_robot import motion_lifecycle as lifecycle

sent = getattr(signal, "SIGNAL_NAME")
def trace(frame, event, arg):
    if frame.f_code is SingleThreadedExecutor.__init__.__code__ and event == "call":
        sys.settrace(None)
        print("CONSTRUCTOR_SIGNAL_FIRST", flush=True)
        os.kill(os.getpid(), sent)
        print("CONSTRUCTOR_SIGNAL_SECOND", flush=True)
        os.kill(os.getpid(), sent)
        deadline = time.monotonic() + 0.3
        while rclpy.ok() and time.monotonic() < deadline:
            pass
    return trace
def factory():
    print("FACTORY_STARTED", flush=True)
    raise ValueError("factory must not run after startup stop")
sys.settrace(trace)
lifecycle.run(factory, cli=True)
sys.settrace(None)
print("CONSTRUCTOR_STOP_COMPLETE", flush=True)
'''.replace("SIGNAL_NAME", signum)
    result = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True,
                            timeout=10, check=False)
    assert "CONSTRUCTOR_SIGNAL_FIRST" in result.stdout, result.stdout + result.stderr
    assert "CONSTRUCTOR_SIGNAL_SECOND" in result.stdout, result.stdout + result.stderr
    assert result.returncode == 0, result.stdout + result.stderr
    assert "CONSTRUCTOR_STOP_COMPLETE" in result.stdout
    assert "FACTORY_STARTED" not in result.stdout
    assert "Traceback" not in result.stderr and "Exception ignored" not in result.stderr


@pytest.mark.parametrize("signum", ["SIGINT", "SIGTERM"])
@pytest.mark.parametrize("policy", ["ignored", "custom"])
def test_cli_startup_preserves_ignored_and_custom_signal_policies(signum, policy):
    import subprocess
    import sys

    pytest.importorskip("rclpy")
    script = '''
import os
import signal
import rclpy
from rclpy.node import Node
from senior_robot import motion_lifecycle as lifecycle

signum = getattr(signal, "SIGNAL_NAME")
seen = []
def custom(number, frame):
    seen.append(number)
signal.signal(signum, signal.SIG_IGN if "POLICY" == "ignored" else custom)
real_init = lifecycle.rclpy.init
def interrupted_init(**kwargs):
    print("EARLY_FIRST", flush=True)
    os.kill(os.getpid(), signum)
    print("EARLY_SECOND", flush=True)
    os.kill(os.getpid(), signum)
    return real_init(**kwargs)
def factory():
    print("FACTORY_STARTED", flush=True)
    node = Node("ignored_startup_signal")
    node.create_timer(0.05, rclpy.try_shutdown)
    return node
lifecycle.rclpy.init = interrupted_init
lifecycle.run(factory, cli=True)
if "POLICY" == "custom":
    assert seen == [signum], seen
else:
    assert not seen
print("TEST_COMPLETE", flush=True)
'''.replace("SIGNAL_NAME", signum).replace("POLICY", policy)
    result = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True,
                            timeout=10, check=False)
    assert "EARLY_FIRST" in result.stdout and "EARLY_SECOND" in result.stdout
    assert result.returncode == 0, result.stdout + result.stderr
    assert "TEST_COMPLETE" in result.stdout
    assert ("FACTORY_STARTED" in result.stdout) == (policy == "ignored")
    assert "Traceback" not in result.stderr and "Exception ignored" not in result.stderr


def test_cli_sigterm_before_native_init_requests_stop():
    import subprocess
    import sys

    pytest.importorskip("rclpy")
    script = '''
import os
import signal
import rclpy
from senior_robot import motion_lifecycle as lifecycle
real_init = lifecycle.rclpy.init
def interrupted_init(**kwargs):
    print("SIGTERM_FIRST", flush=True)
    os.kill(os.getpid(), signal.SIGTERM)
    print("SIGTERM_SECOND", flush=True)
    os.kill(os.getpid(), signal.SIGTERM)
    return real_init(**kwargs)
def factory():
    raise ValueError("factory must not start")
lifecycle.rclpy.init = interrupted_init
lifecycle.run(factory, cli=True)
assert not rclpy.ok()
print("SIGTERM_STOP_COMPLETE", flush=True)
'''
    result = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True,
                            timeout=10, check=False)
    assert "SIGTERM_FIRST" in result.stdout and "SIGTERM_SECOND" in result.stdout
    assert result.returncode == 0, result.stdout + result.stderr
    assert "SIGTERM_STOP_COMPLETE" in result.stdout
    assert "Traceback" not in result.stderr and "Exception ignored" not in result.stderr


@pytest.mark.parametrize("signum", ["SIGINT", "SIGTERM"])
def test_cli_signal_after_native_install_cleans_owned_node_before_spin(signum):
    import subprocess
    import sys

    pytest.importorskip("rclpy")
    script = '''
import os
import signal
from rclpy.node import Node
from senior_robot import motion_lifecycle as lifecycle

actual_install = lifecycle.rclpy.signals.install_signal_handlers
signum = getattr(signal, "SIGNAL_NAME")
def install_then_interrupt(options):
    actual_install(options)
    print("NATIVE_SIGNAL_FIRST", flush=True)
    os.kill(os.getpid(), signum)
    print("NATIVE_SIGNAL_SECOND", flush=True)
    os.kill(os.getpid(), signum)
def factory():
    node = Node("owned_before_native_signal")
    destroy = node.destroy_node
    def traced_destroy():
        print("OWNED_NODE_DESTROYED", flush=True)
        return destroy()
    node.destroy_node = traced_destroy
    return node
lifecycle.rclpy.signals.install_signal_handlers = install_then_interrupt
lifecycle.run(factory, cli=True)
print("NATIVE_SIGNAL_STOP_COMPLETE", flush=True)
'''.replace("SIGNAL_NAME", signum)
    result = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True,
                            timeout=10, check=False)
    assert "NATIVE_SIGNAL_FIRST" in result.stdout and "NATIVE_SIGNAL_SECOND" in result.stdout
    assert result.returncode == 0, result.stdout + result.stderr
    assert "OWNED_NODE_DESTROYED" in result.stdout
    assert "NATIVE_SIGNAL_STOP_COMPLETE" in result.stdout
    assert "Traceback" not in result.stderr and "Exception ignored" not in result.stderr
