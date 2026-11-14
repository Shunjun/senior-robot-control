"""One bounded Humble native-MessageInfo channel per Python consumer node.

The child alone inherits a private socketpair endpoint; there is no listening
socket, ROS relay publisher, or discovery-as-sample identity fallback. Native
callback receipts and RMW timestamps survive serialization and Python scheduling.
"""

import os
import socket
import struct
import subprocess
import threading
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

_HEADER = struct.Struct("!4sBBQQQqq24sI")
_MAX_PAYLOAD = 20 * 1024 * 1024
_MAX_TRANSPORT_AGE = 0.25
_MAX_PENDING_SAMPLES = 64
_MAX_PENDING_BYTES = 32 * 1024 * 1024


def received_steady(info: Any) -> float:
    # //Compatible: ordinary rclpy callbacks retain their local receipt behavior.
    value = info.get("native_received_steady") if isinstance(info, dict) else None
    return float(value) if value is not None else time.monotonic()


@dataclass
class _Input:
    message_type: Any
    topic: str
    writer: str
    namespace: str
    qos: str
    callback: Callable[..., None]
    invalidate: Callable[[], None]
    expired: Callable[..., None] | None = None
    generation: int = 0
    sequence: int = 0
    barrier: int = 0


class NativeReceiptGroup:
    def __init__(self, node: Any):
        from rclpy.clock import Clock, ClockType

        self.node = node
        self.inputs: list[_Input] = []
        self._lock = threading.Lock()
        self._pending: dict[int, deque[tuple[bytes, dict[str, Any]]]] = {}
        self._queued_bytes = 0
        self._invalid: set[int] = set()
        self._last_receipt = float("-inf")
        self._started_at: float | None = None
        self._failed = False
        self._failure_reason = ""
        self._closed = False
        self._late_warning_at = float("-inf")
        self._process: subprocess.Popen[bytes] | None = None
        self._socket: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._timer = node.create_timer(
            0.01, self._drain, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )
        node.context.on_shutdown(self.close)

    def add(self, message_type, topic, writer, namespace, qos, callback, invalidate, expired=None):
        if self._started_at is not None or len(self.inputs) >= 16:
            raise ValueError("native receipt inputs must be registered before spinning (max 16)")
        entry = _Input(
            message_type,
            self.node.resolve_topic_name(topic),
            writer,
            namespace,
            qos,
            callback,
            invalidate,
            expired,
        )
        self.inputs.append(entry)
        return entry

    def _start(self):
        from ament_index_python.packages import get_package_prefix

        executable = (
            Path(get_package_prefix("senior_motion_msgs")) / "lib/senior_motion_msgs/native_receipt"
        )
        parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_STREAM)
        arguments = [str(executable), str(child.fileno()), str(os.getpid())]
        for entry in self.inputs:
            message = entry.message_type
            kind = f"{message.__module__.split('.')[0]}/msg/{message.__name__}"
            arguments.extend([kind, entry.topic, entry.writer, entry.namespace, entry.qos])
        self._started_at = time.monotonic()
        try:
            self._process = subprocess.Popen(arguments, pass_fds=(child.fileno(),), close_fds=True)
        except BaseException:
            parent.close()
            raise
        finally:
            child.close()
        self._socket = parent
        parent.settimeout(0.1)
        self._thread = threading.Thread(
            target=self._receive, daemon=True, name="native-dds-receipt"
        )
        self._thread.start()

    def _read(self, size):
        assert self._socket is not None
        result = bytearray(size)
        offset = 0
        while offset < size and not self._closed:
            try:
                count = self._socket.recv_into(memoryview(result)[offset:])
            except TimeoutError:
                if self._process is None or self._process.poll() is not None:
                    raise OSError("native receipt process exited") from None
                limit = 5.0 if self._last_receipt == float("-inf") else _MAX_TRANSPORT_AGE
                since = (
                    self._started_at if self._last_receipt == float("-inf") else self._last_receipt
                )
                if since is None or time.monotonic() - since > limit:
                    # Silence from a living child withdraws every input. It is
                    # not an EOF or corrupt frame: preserve framing and queued
                    # negative edges while waiting for fresh original receipts.
                    with self._lock:
                        self._invalid.update(range(len(self.inputs)))
                        for entry in self.inputs:
                            entry.barrier += 1
                continue
            if count == 0:
                raise OSError("native receipt channel closed")
            offset += count
        if offset != size:
            raise OSError("native receipt stopped")
        return bytes(result)

    def _receive(self):
        try:
            while not self._closed:
                (
                    magic,
                    kind,
                    key,
                    generation,
                    sequence,
                    stamp_ns,
                    source_ns,
                    received_ns,
                    gid,
                    size,
                ) = _HEADER.unpack(self._read(_HEADER.size))
                if (
                    magic != b"SDR1"
                    or kind not in (0, 1, 2)
                    or key >= len(self.inputs)
                    or size > _MAX_PAYLOAD
                    or (kind != 1 and size)
                    or generation < 1
                ):
                    raise ValueError("invalid native receipt frame")
                stamp = stamp_ns / 1e9
                age = time.monotonic() - stamp
                if age < 0:
                    raise ValueError(
                        f"native receipt age={age:.6f}s exceeds {_MAX_TRANSPORT_AGE}s "
                        f"on {self.inputs[key].topic} (kind={kind})"
                    )
                payload = self._read(size) if size else b""
                with self._lock:
                    entry = self.inputs[key]
                    if generation < entry.generation:
                        raise ValueError("native receipt generation rollback")
                    if kind == 0 or generation != entry.generation:
                        self._invalid.add(key)
                        entry.barrier += 1
                    entry.generation = generation
                    self._last_receipt = stamp
                    if kind != 1 and age > _MAX_TRANSPORT_AGE:
                        # Late revocation still revokes. A late positive heartbeat
                        # cannot qualify anything, but neither corrupts the stream.
                        # Only a later real sample can supply fresh evidence.
                        self._invalid.add(key)
                        entry.barrier += 1
                        continue
                    if kind == 1:
                        if not any(gid) or sequence <= entry.sequence or not payload:
                            raise ValueError("native receipt sample identity/order invalid")
                        entry.sequence = sequence
                        packets = self._pending.setdefault(key, deque())
                        # Every consumer receives ordered transitions, including
                        # negative states followed by positive states in one drain.
                        # Never infer delivery semantics from a message class name.
                        if (
                            len(packets) >= _MAX_PENDING_SAMPLES
                            or self._queued_bytes + len(payload) > _MAX_PENDING_BYTES
                        ):
                            raise ValueError("native ordered receipt queue overflow")
                        self._queued_bytes += len(payload)
                        packets.append(
                            (
                                payload,
                                {
                                    "publisher_gid": gid,
                                    "native_receipt_authenticated": True,
                                    "native_writer_name": entry.writer,
                                    "native_writer_namespace": entry.namespace,
                                    "native_received_steady": stamp,
                                    "source_timestamp": source_ns,
                                    "received_timestamp": received_ns,
                                    "receipt_generation": generation,
                                    "receipt_sequence": sequence,
                                    "receipt_barrier": entry.barrier,
                                },
                            )
                        )

        except (OSError, ValueError, struct.error) as exc:
            with self._lock:
                self._failed = True
                self._failure_reason = f"{type(exc).__name__}: {exc}"
                self._pending.clear()
                self._queued_bytes = 0
                self._invalid.update(range(len(self.inputs)))

    def _drain(self):
        if self._closed:
            return
        if self._started_at is None:
            try:
                self._start()
            except (OSError, ValueError, LookupError) as exc:
                self.node.get_logger().error(f"native DDS receipt unavailable: {exc}")
                self._failed = True
                self._failure_reason = f"startup: {exc}"
                self._invalid.update(range(len(self.inputs)))
        from rclpy.serialization import deserialize_message

        with self._lock:
            invalid, self._invalid = self._invalid, set()
            pending, self._pending = self._pending, {}
            self._queued_bytes = 0
            failed = self._failed
        for key in sorted(invalid):
            self.inputs[key].invalidate()
        if failed:
            returncode = self._process.poll() if self._process is not None else None
            self.close()
            self.node.get_logger().error(
                "native DDS receipt failed; all dependent evidence revoked; "
                f"reason={self._failure_reason}; child_exit={returncode}; "
                f"topics={[entry.topic for entry in self.inputs]}"
            )
            return
        for key, packets in pending.items():
            for payload, info in packets:
                with self._lock:
                    if self._failed:
                        return
                    revoked = (
                        key in self._invalid
                        or self.inputs[key].generation != info["receipt_generation"]
                        or self.inputs[key].barrier != info["receipt_barrier"]
                    )
                entry = self.inputs[key]
                expired = time.monotonic() - received_steady(info) > _MAX_TRANSPORT_AGE
                try:
                    message = deserialize_message(payload, entry.message_type)
                    if revoked or expired:
                        entry.invalidate()
                        # Latching consumers may preserve a negative state from
                        # an old receipt, but may never refresh its original TTL
                        # or consume its positive state as current evidence.
                        if entry.expired is not None:
                            entry.expired(message, info)
                        if time.monotonic() - self._late_warning_at >= 2:
                            self._late_warning_at = time.monotonic()
                            self.node.get_logger().warning(
                                f"native DDS receipt stale/revoked on {entry.topic}; "
                                "evidence withdrawn; waiting for fresh samples"
                            )
                    else:
                        entry.callback(message, info)
                except (ValueError, TypeError, RuntimeError) as exc:
                    # An undecodable state could contain a latch edge. Do not
                    # skip it and let a later positive frame restore authority.
                    entry.invalidate()
                    with self._lock:
                        self._failed = True
                        self._failure_reason = f"payload on {entry.topic}: {exc}"
                        self._pending.clear()
                        self._queued_bytes = 0
                        self._invalid.update(range(len(self.inputs)))
                    return

    def close(self):
        if self._closed:
            return
        self._closed = True
        if self._socket is not None:
            try:
                self._socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._socket.close()
        if self._process is not None and self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=1)
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=1)


def subscribe(
    node,
    message_type,
    topic,
    callback,
    *,
    writer,
    namespace="/",
    qos="sensor",
    invalidate,
    expired=None,
):
    """Subscribe with bounded ordered delivery; expired hooks preserve negative edges only."""
    group = getattr(node, "_native_receipt_group", None)
    if group is None:
        group = NativeReceiptGroup(node)
        node._native_receipt_group = group
    return group.add(message_type, topic, writer, namespace, qos, callback, invalidate, expired)
