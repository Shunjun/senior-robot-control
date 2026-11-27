"""Internal socket-to-callback flow; ROS deserialization alone is substituted."""

import socket
import sys
import threading
import time
from types import ModuleType, SimpleNamespace

import pytest
from senior_sensor_ingress import native_receipt as receipt


@pytest.fixture
def channel(monkeypatch):
    serialization = ModuleType("rclpy.serialization")
    serialization.deserialize_message = lambda payload, _: payload.decode("ascii")
    monkeypatch.setitem(sys.modules, "rclpy", ModuleType("rclpy"))
    monkeypatch.setitem(sys.modules, serialization.__name__, serialization)
    diagnostics = SimpleNamespace(pump=lambda _: None, duration=lambda *_: None)
    monkeypatch.setattr(receipt, "process_diagnostics", lambda: diagnostics)
    delivered = []
    group = receipt.NativeReceiptGroup.__new__(receipt.NativeReceiptGroup)
    logger = SimpleNamespace(error=lambda *_: None, warning=lambda *_: None)
    group.node = SimpleNamespace(get_logger=lambda: logger)
    group.inputs = [
        receipt._Input(
            str,
            "/unit/input",
            "configured_writer",
            "/",
            "sensor",
            lambda message, info: delivered.append((message, info)),
            lambda: None,
        )
    ]
    group._lock = threading.Lock()
    group._pending = {}
    group._queued_bytes = 0
    group._invalid = set()
    group._last_receipt = float("-inf")
    group._started_at = time.monotonic()
    group._boot_id = "unit-native-boot"
    group._failed = False
    group._failure_reason = ""
    group._closed = False
    group._late_warning_at = float("-inf")
    group._process = None
    receiver, writer = socket.socketpair()
    receiver.settimeout(0.1)  # Match the production channel so close can join its reader.
    group._socket = receiver
    group._thread = threading.Thread(target=group._receive)
    group._thread.start()

    def send(payload, generation, sequence):
        body = payload.encode("ascii")
        received = time.monotonic_ns() - 5_000_000_000
        header = receipt._HEADER.pack(
            b"SDR2",
            1,
            0,
            generation,
            sequence,
            received,
            0,
            0,
            bytes(24),
            0,
            0,
            0,
            0,
            0,
            0,
            len(body),
        )
        writer.sendall(header + body)

    def drain(expected):
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            with group._lock:
                ready = sum(map(len, group._pending.values())) >= expected or group._failed
            if ready:
                break
            time.sleep(0.001)
        assert not group._failed, group._failure_reason
        group._drain()

    try:
        yield SimpleNamespace(group=group, delivered=delivered, send=send, drain=drain)
    finally:
        group.close()
        writer.close()
        assert not group._thread.is_alive()


def test_transport_forwards_late_payloads_with_record_only_gid_and_sequence(channel):
    channel.send("first", 1, 10)
    channel.send("second", 1, 1)
    channel.drain(2)
    assert [message for message, _ in channel.delivered] == ["first", "second"]
    assert [info["receipt_sequence"] for _, info in channel.delivered] == [10, 1]
    assert all(info["publisher_gid"] == bytes(24) for _, info in channel.delivered)


def test_old_generation_is_skipped_without_failing_the_whole_channel(channel):
    channel.send("current", 2, 1)
    channel.send("old", 1, 100)
    channel.send("next", 2, 0)
    channel.drain(2)
    assert [message for message, _ in channel.delivered] == ["current", "next"]
    assert not channel.group._failed
