"""Real Humble executor tests for producer failures and partial construction."""

import time

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from senior_perception.rgbd_candidate_producer import RgbdCandidateProducer  # noqa: E402


@pytest.mark.parametrize("sim_time", [False, True])
def test_background_failure_reaches_owned_executor(monkeypatch, sim_time):
    args = ["--ros-args", "-p", "use_sim_time:=true"] if sim_time else None
    rclpy.init(args=args)
    producer = RgbdCandidateProducer()
    executor = SingleThreadedExecutor()
    executor.add_node(producer)
    monkeypatch.setattr(
        producer, "_owner", lambda *_: (_ for _ in ()).throw(RuntimeError("clock failure"))
    )
    try:
        deadline = time.monotonic() + 2
        with pytest.raises(RuntimeError, match="clock failure"):
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
    finally:
        executor.shutdown()
        producer.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("operation", ["create_publisher", "create_subscription"])
def test_partial_construction_destroys_node_and_keeps_original_error(monkeypatch, operation):
    rclpy.init()
    destroyed = []
    original = Node.destroy_node

    def destroy(node):
        destroyed.append(True)
        return original(node)

    original_operation = getattr(Node, operation)

    def fail(node, *args, **kwargs):
        if operation == "create_publisher" and args[1] != "/rgbd_diagnostic/candidate":
            return original_operation(node, *args, **kwargs)
        raise RuntimeError("subscription failed")

    monkeypatch.setattr(Node, "destroy_node", destroy)
    monkeypatch.setattr(Node, operation, fail)
    try:
        with pytest.raises(RuntimeError, match="subscription failed"):
            RgbdCandidateProducer()
        assert destroyed
    finally:
        rclpy.shutdown()


def test_thread_start_failure_rolls_back_and_preserves_exception(monkeypatch):
    import threading

    rclpy.init()
    destroyed = []
    original_destroy = Node.destroy_node
    original_start = threading.Thread.start

    def destroy(node):
        destroyed.append(True)
        return original_destroy(node)

    def fail(thread):
        if thread.name == "rgbd-coordinator":
            raise RuntimeError("coordinator start failed")
        return original_start(thread)

    monkeypatch.setattr(Node, "destroy_node", destroy)
    monkeypatch.setattr(threading.Thread, "start", fail)
    try:
        with pytest.raises(RuntimeError, match="coordinator start failed"):
            RgbdCandidateProducer()
        assert destroyed
    finally:
        rclpy.shutdown()


def test_bounded_input_burst_is_drained_without_per_item_timer_delay():
    rclpy.init()
    producer = RgbdCandidateProducer()
    try:
        producer._enabled = True
        for _ in range(12):
            producer._enqueue((0, None))
        deadline = time.monotonic() + 0.15
        while time.monotonic() < deadline:
            with producer._inbox._lock:
                pending = len(producer._inbox._items)
            if not pending:
                break
            time.sleep(0.005)
        assert pending == 0
    finally:
        producer.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("failure", [RuntimeError, SystemExit])
def test_shutdown_surfaces_unreported_worker_fault(monkeypatch, failure):
    import threading

    entered = threading.Event()

    def work(producer):
        entered.set()
        assert producer._stop.wait(2)
        raise failure("worker shutdown failed")

    monkeypatch.setattr(RgbdCandidateProducer, "_work", work)
    rclpy.init()
    producer = RgbdCandidateProducer()
    try:
        assert entered.wait(2)
        with pytest.raises(RuntimeError) as error:
            producer.destroy_node()
        fault = error.value if failure is RuntimeError else error.value.__cause__
        assert isinstance(fault, failure)
        assert str(fault) == "worker shutdown failed"
        assert not producer._thread.is_alive()
    finally:
        rclpy.shutdown()
