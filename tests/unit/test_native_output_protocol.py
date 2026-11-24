"""Compile exact production C++ receive logic; ROS is a recording test boundary."""

import importlib.util
import shutil
import socket
import struct
import subprocess
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest

from vehicle_core.values import Velocity

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module")
def native_harness(tmp_path_factory):
    compiler = shutil.which("c++")
    if compiler is None:
        pytest.skip("C++ compiler unavailable; native protocol not verified")
    directory = ROOT / "tests/support/native_output"
    executable = tmp_path_factory.mktemp("native-output") / "protocol"
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(directory),
            "-I",
            str(ROOT / "ros2/src/senior_motion_control/native"),
            "-I",
            str(ROOT / "ros2/src/senior_sensor_ingress/include"),
            str(directory / "protocol_harness.cpp"),
            "-o",
            str(executable),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    return executable


@pytest.mark.skipif(
    sys.platform != "linux",
    reason="Unix SOCK_SEQPACKET requires Linux; native runtime unverified here",
)
@pytest.mark.parametrize(
    "scenario", ["deadline", "expired", "eof", "overflow", "duplicate", "malformed"]
)
def test_production_native_receiver_protocol(native_harness, scenario):
    subprocess.run([str(native_harness), scenario], check=True, capture_output=True, timeout=5)


@pytest.fixture
def output_module(monkeypatch):
    package = ModuleType("ament_index_python.packages")
    package.get_package_prefix = lambda _: "/nonexistent-batch-test-install"
    monkeypatch.setitem(sys.modules, package.__name__, package)
    spec = importlib.util.spec_from_file_location(
        "native_output_under_test",
        ROOT / "ros2/src/senior_motion_control/senior_motion_control/motion_output.py",
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_python_pack_preserves_values_original_deadline_and_sequence(output_module):
    parent, peer = socket.socketpair()
    output = output_module.MotionOutput.__new__(output_module.MotionOutput)
    output.process = SimpleNamespace(poll=lambda: None)
    output.channel, output.sequence = parent, 0
    try:
        output.publish(Velocity(0.2, -0.3), 10.25, computed_steady=10.0)
        assert struct.unpack("=ddddQ", peer.recv(64)) == (0.2, -0.3, 10.25, 10.0, 1)
        output.publish(Velocity(0, 0), 0, computed_steady=10.2)
        assert struct.unpack("=ddddQ", peer.recv(64)) == (0, 0, 0, 10.2, 2)
    finally:
        parent.close()
        peer.close()


def test_missing_native_executable_fails_explicitly(output_module, monkeypatch):
    # Isolate executable discovery on macOS, which cannot create SOCK_SEQPACKET.
    create_pair = socket.socketpair
    monkeypatch.setattr(output_module.socket, "socketpair", lambda *_: create_pair())
    with pytest.raises(FileNotFoundError):
        output_module.MotionOutput()


def test_closed_transport_fails_without_queued_retry(output_module):
    parent, peer = socket.socketpair()
    output = output_module.MotionOutput.__new__(output_module.MotionOutput)
    output.process = SimpleNamespace(poll=lambda: None)
    output.channel, output.sequence = parent, 0
    peer.close()
    with pytest.raises(ValueError, match="motion_output_transport_lost"):
        output.publish(Velocity(0.2, 0), 10, computed_steady=9.9)
    assert parent.fileno() == -1


def test_native_output_cpp_compiles_without_ros_with_recording_headers(native_harness):
    assert native_harness.is_file()


@pytest.fixture(scope="module")
def native_state_harness(tmp_path_factory):
    compiler = shutil.which("c++")
    if compiler is None:
        pytest.skip("C++ compiler unavailable; native state protocol not verified")
    directory = ROOT / "tests/support/native_output"
    executable = tmp_path_factory.mktemp("native-state") / "protocol"
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(directory),
            "-I",
            str(ROOT / "ros2/src/senior_motion_control/native"),
            "-I",
            str(ROOT / "ros2/src/senior_sensor_ingress/include"),
            str(directory / "protocol_state_harness.cpp"),
            "-o",
            str(executable),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    return executable


@pytest.mark.parametrize(
    "scenario",
    [
        "expired_queue",
        "original_order",
        "zero",
        "future",
        "nan",
        "replay",
        "eof",
        "topic",
        "publish_deadline",
    ],
)
def test_exact_native_receiver_state_with_deterministic_clock_and_io(
    native_state_harness, scenario
):
    subprocess.run(
        [str(native_state_harness), scenario], check=True, capture_output=True, timeout=5
    )


# Retired: ActuatorTarget admission/driver boot proof belongs to the removed
# external protocol. Standard Twist output is exercised by both receivers above.


def test_trace_pipe_backpressure_preserves_exact_packet_and_independent_flags(
    output_module, monkeypatch
):
    import fcntl
    import gc
    import os
    import time

    now = time.monotonic_ns()
    monkeypatch.setenv("SENIOR_MOTION_TRACE_ID", "protocol-test")
    monkeypatch.setenv("SENIOR_MOTION_TRACE_START_NS", str(now - 1_000_000))
    monkeypatch.setenv("SENIOR_MOTION_TRACE_END_NS", str(now + 1_000_000_000))
    reader, writer = os.pipe()
    source_flags, stderr_flags = fcntl.fcntl(writer, fcntl.F_GETFL), fcntl.fcntl(2, fcntl.F_GETFL)
    open_fd = os.open
    if sys.platform != "linux":
        # macOS has no /proc reopening; exercise the real FIFO, not Linux flag independence.
        os.set_blocking(writer, False)

    def open_trace(path, flags):
        if path == "/proc/self/fd/2":
            return (
                open_fd(f"/proc/self/fd/{writer}", flags)
                if sys.platform == "linux"
                else os.dup(writer)
            )
        return open_fd(path, flags)

    monkeypatch.setattr(os, "open", open_trace)
    ordinary_gc = gc.isenabled(), gc.get_threshold()
    packets = []
    try:
        for enabled in (False, True):
            parent, peer = socket.socketpair()
            output = output_module.MotionOutput.__new__(output_module.MotionOutput)
            output.channel, output.sequence = parent, 0
            output.process = SimpleNamespace(poll=lambda: None)
            with monkeypatch.context() as mode:
                if not enabled:
                    mode.delenv("SENIOR_MOTION_TRACE_ID")
                output._init_trace()
            try:
                if enabled:
                    assert output._trace_state == "armed" and output._trace_fd >= 0
                    if sys.platform == "linux":
                        assert fcntl.fcntl(writer, fcntl.F_GETFL) == source_flags
                    assert fcntl.fcntl(2, fcntl.F_GETFL) == stderr_flags
                    while True:
                        try:
                            os.write(output._trace_fd, b"x" * 4096)
                        except BlockingIOError:
                            break
                output.publish(Velocity(0.2, -0.3), 10.25, computed_steady=10.0)
                output.flush_send_trace()
                packets.append(peer.recv(64))
                if enabled:
                    assert output._trace_lost == 1
                    mode_now = output._trace_end
                    with monkeypatch.context() as ended:
                        ended.setattr(time, "monotonic_ns", lambda mode_now=mode_now: mode_now)
                        assert not output.trace_active()
                    assert output._trace_fd == -1 and output._trace_gc_event not in gc.callbacks
                    assert fcntl.fcntl(2, fcntl.F_GETFL) == stderr_flags
            finally:
                output.finish_trace("test_end")
                parent.close()
                peer.close()
        assert packets[0] == packets[1] == struct.pack("=ddddQ", 0.2, -0.3, 10.25, 10.0, 1)
        assert (gc.isenabled(), gc.get_threshold()) == ordinary_gc
    finally:
        os.close(reader)
        os.close(writer)


def test_native_trace_proves_watchdog_fence_without_changing_publications(native_state_harness):
    import json
    import os

    reader, writer = os.pipe()
    os.set_blocking(writer, False)
    os.set_blocking(reader, False)
    try:
        off = subprocess.run(
            [str(native_state_harness), "watchdog_inflight"],
            check=True,
            capture_output=True,
            timeout=5,
        )
        environment = {
            **os.environ,
            "SENIOR_MOTION_TRACE_ID": "native-test",
            "SENIOR_MOTION_TRACE_START_NS": "9000000000",
            "SENIOR_MOTION_TRACE_END_NS": "12000000000",
            "SENIOR_MOTION_TRACE_FD": str(writer),
        }
        on = subprocess.run(
            [str(native_state_harness), "watchdog_inflight"],
            env=environment,
            pass_fds=(writer,),
            check=True,
            capture_output=True,
            timeout=5,
        )
        assert off.stdout == on.stdout and off.stderr == on.stderr
        rows = [json.loads(line) for line in os.read(reader, 65536).splitlines()]
        stop = next(
            row for row in rows if row["event"] == "stop" and row["reason"] == "watchdog_timeout"
        )
        stale = next(row for row in rows if row["reason"] == "stale_produced")
        resumed = next(row for row in rows if row["event"] == "publish" and row["sequence"] == 3)
        assert stop["deadline_ns"] == stop["checked_ns"] == 10_500_000_000
        assert stop["last_produced_after"] == 10_500_000_000
        assert stop["remaining_ms"] == 0 and stop["since_computed_ms"] == 600
        assert stale["sequence"] == 2 and stale["remaining_ms"] == 500
        assert stale["computed_ns"] < stale["last_produced_before"] == stop["last_produced_after"]
        assert resumed["computed_ns"] == 10_510_000_000 and resumed["deadline_ns"] == 10_660_000_000
        assert all(row["lost"] == 0 for row in rows)
        assert rows[-1]["event"] == "trace_end"
        # The identical N1 motion input must also survive a vanished diagnostic reader.
        os.close(reader)
        reader = -1
        closed_log = subprocess.run(
            [str(native_state_harness), "watchdog_inflight"],
            env=environment,
            pass_fds=(writer,),
            check=True,
            capture_output=True,
            timeout=5,
            restore_signals=True,
        )
        assert closed_log.stdout == off.stdout and closed_log.stderr == off.stderr
    finally:
        if reader >= 0:
            os.close(reader)
        os.close(writer)
