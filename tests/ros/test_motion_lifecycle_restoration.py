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

    def spin(_node):
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
    monkeypatch.setattr(lifecycle.rclpy, "init", lambda: None)
    monkeypatch.setattr(lifecycle.rclpy, "try_shutdown", lambda: events.append("shutdown"))
    monkeypatch.setattr(lifecycle.rclpy, "ok", lambda: True)

    def spin(_node):
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
