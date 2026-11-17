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

from senior_control.models import Velocity

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
            str(ROOT / "ros2/src/senior_motion_msgs/src"),
            "-I",
            str(ROOT / "ros2/src/senior_motion_msgs/include"),
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
        "native_output_under_test", ROOT / "ros2/src/senior_robot/senior_robot/motion_output.py"
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
            str(ROOT / "ros2/src/senior_motion_msgs/src"),
            "-I",
            str(ROOT / "ros2/src/senior_motion_msgs/include"),
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
