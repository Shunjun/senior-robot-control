"""The CLI must revoke motion before its ROS context is closed by a signal."""

import importlib.util
import signal
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest


@pytest.fixture
def lifecycle(monkeypatch):
    calls = []
    handlers = {signal.SIGINT: signal.default_int_handler, signal.SIGTERM: signal.SIG_DFL}
    live = [False]
    native = [False]

    def set_handler(signum, handler):
        previous = handlers[signum]
        handlers[signum] = handler
        return previous

    monkeypatch.setattr(signal, "getsignal", lambda signum: handlers[signum])
    monkeypatch.setattr(signal, "signal", set_handler)

    def interrupt(signum):
        handler = handlers[signum]
        if callable(handler):
            handler(signum, None)
        # Humble's installed native handler shuts the context independently.
        if native[0]:
            live[0] = False

    def init(**kwargs):
        live[0] = True
        calls.append("init")

    def install(options):
        native[0] = True

    def shutdown():
        calls.append("context_shutdown")
        live[0] = False

    class Executor:
        def add_node(self, node):
            pass

        def remove_node(self, node):
            pass

        def spin_once(self, timeout_sec=None):
            calls.append("spin")
            interrupt(signal.SIGINT)

        def wake(self):
            calls.append("wake")

        def shutdown(self):
            calls.append("executor_shutdown")

    class Node:
        def create_guard_condition(self, callback):
            return SimpleNamespace(trigger=lambda: calls.append("wake"))

        def destroy_node(self):
            assert live[0], "motion revocation attempted after context shutdown"
            calls.append("revoke_and_destroy")

    async def await_or_execute(callback, *args):
        return callback(*args)

    rclpy = ModuleType("rclpy")
    rclpy.init, rclpy.ok, rclpy.try_shutdown = init, lambda: live[0], shutdown
    rclpy.spin = lambda node, executor: executor.spin_once()
    rclpy.signals = SimpleNamespace(
        SignalHandlerOptions=SimpleNamespace(NO=0, ALL=1),
        install_signal_handlers=install,
        uninstall_signal_handlers=lambda: None,
    )
    executors = ModuleType("rclpy.executors")
    executors.SingleThreadedExecutor = Executor
    executors.ExternalShutdownException = type("ExternalShutdownException", (Exception,), {})
    executors.await_or_execute = await_or_execute
    binding = ModuleType("rclpy._rclpy_pybind11")
    binding.RCLError = type("RCLError", (RuntimeError,), {})
    node_module = ModuleType("rclpy.node")
    node_module.Node = Node
    for name, module in {
        "rclpy": rclpy,
        "rclpy.executors": executors,
        "rclpy._rclpy_pybind11": binding,
        "rclpy.node": node_module,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)
    path = Path("ros2/src/senior_robot/senior_robot/motion_lifecycle.py")
    spec = importlib.util.spec_from_file_location("cli_lifecycle_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return SimpleNamespace(module=module, calls=calls, Node=Node, interrupt=interrupt, live=live)


def test_cli_signal_keeps_context_live_until_motion_is_revoked(lifecycle):
    lifecycle.module.run(lifecycle.Node, cli=True)
    assert lifecycle.calls.index("spin") < lifecycle.calls.index("revoke_and_destroy")
    assert lifecycle.calls.index("revoke_and_destroy") < lifecycle.calls.index("context_shutdown")
    assert not lifecycle.live[0]


def test_custom_spin_receives_cooperative_stop_before_async_cleanup(lifecycle):
    def spin(node, executor, should_stop):
        assert not should_stop()
        lifecycle.interrupt(signal.SIGTERM)
        assert should_stop() and lifecycle.live[0]
        lifecycle.calls.append("ipc_close_and_source_revoke")

    lifecycle.module.run(lifecycle.Node, cli=True, spin=spin)
    assert lifecycle.calls.index("ipc_close_and_source_revoke") < lifecycle.calls.index(
        "revoke_and_destroy"
    )


def test_signal_during_construction_finishes_owned_cleanup(lifecycle):
    def factory():
        node = lifecycle.Node()
        lifecycle.interrupt(signal.SIGINT)
        lifecycle.interrupt(signal.SIGINT)
        return node

    lifecycle.module.run(factory, cli=True)
    assert "spin" not in lifecycle.calls
    assert "revoke_and_destroy" in lifecycle.calls


def test_live_cleanup_failure_still_propagates_and_closes_context(lifecycle):
    class FailingNode(lifecycle.Node):
        def destroy_node(self):
            assert lifecycle.live[0]
            raise ValueError("revocation failure")

    with pytest.raises(ValueError, match="revocation failure"):
        lifecycle.module.run(FailingNode, cli=True)
    assert "spin" in lifecycle.calls
    assert "context_shutdown" in lifecycle.calls and not lifecycle.live[0]
