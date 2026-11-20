"""Exercise the native receipt processes after ROS graph discovery."""

import os
import signal
import subprocess
import time
from pathlib import Path

import pytest


@pytest.mark.parametrize(
    ("executable", "node_name"),
    [
        ("raw_receipt_node", "raw_receipt"),
        ("scan_receipt_node", "scan_receipt"),
    ],
)
@pytest.mark.parametrize("signal_name", ["SIGINT", "SIGTERM"])
@pytest.mark.parametrize("repeat", [False, True])
def test_receipt_process_shutdown(executable, node_name, signal_name, repeat):
    rclpy = pytest.importorskip("rclpy")
    binary_dir = os.environ.get("SENIOR_RECEIPT_BINARY_DIR")
    if not binary_dir:
        pytest.skip("set SENIOR_RECEIPT_BINARY_DIR to an isolated receipt build")
    binary = Path(binary_dir) / executable
    assert binary.is_file(), binary
    proc = subprocess.Popen(
        [str(binary)], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )
    rclpy.init()
    observer = rclpy.create_node("receipt_shutdown_observer")
    try:
        deadline = time.monotonic() + 10
        while node_name not in observer.get_node_names() and time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=0.1)
        assert node_name in observer.get_node_names(), "receipt node did not start"
        os.kill(proc.pid, getattr(signal, signal_name))
        if repeat:
            os.kill(proc.pid, getattr(signal, signal_name))
        stdout, stderr = proc.communicate(timeout=10)
        assert proc.returncode == 0, stdout + stderr
        assert "terminate called" not in stderr and "what():" not in stderr
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.communicate(timeout=5)
        observer.destroy_node()
        rclpy.shutdown()
