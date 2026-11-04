"""Exercise one-shot receipt acquisition through the actual native scan relay."""

import os
import secrets
import subprocess
import sys
import time
from contextlib import contextmanager
from tempfile import TemporaryFile

import rclpy
from ament_index_python.packages import get_package_prefix
from builtin_interfaces.msg import Time
from probe_bounded_scan_ingress import BOUNDED_TOPIC, endpoint_gid, spin_for
from probe_candidate_load import candidate
from probe_read_graph import LASER, scan_in_room
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from senior_motion_msgs.msg import BoundedRawLaserScan
from sensor_msgs.msg import LaserScan

RAW = "/probe_acquire_scan"
ARMED = "Scan receipt armed "


@contextmanager
def collector(node, executable):
    with TemporaryFile(mode="w+b") as log:
        process = subprocess.Popen(
            [
                executable,
                "--ros-args",
                "-p",
                f"raw_topic:={RAW}",
                "-p",
                f"expected_frame:={LASER}",
            ],
            stdout=subprocess.PIPE,
            stderr=log,
        )
        try:
            deadline = time.monotonic() + 6.0
            while time.monotonic() < deadline:
                lines = os.pread(log.fileno(), 65536, 0).decode(errors="replace").splitlines()
                armed = [line for line in lines if line.startswith(ARMED)]
                if armed:
                    yield process, int(armed[-1].removeprefix(ARMED))
                    return
                assert process.poll() is None, lines
                spin_for(node, 0.03)
            raise AssertionError(f"Collector did not arm: {lines}")
        finally:
            if process.poll() is None:
                process.kill()
            process.communicate(timeout=2.0)


def stamp_from_ns(ns):
    seconds, nanoseconds = divmod(ns, 1_000_000_000)
    return Time(sec=seconds, nanosec=nanoseconds)


def probe(executable):
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["ROS_DOMAIN_ID"] = str(190 + secrets.randbelow(12))
    prefix = get_package_prefix("senior_sensor_ingress")
    relay = os.path.join(prefix, "lib", "senior_sensor_ingress", "scan_receipt_node")
    rclpy.init(args=[])
    node = rclpy.create_node("scan_receipt_acquisition_probe")
    try:
        raw = node.create_publisher(LaserScan, RAW, qos_profile_sensor_data)
        relayed = []
        subscription = node.create_subscription(
            BoundedRawLaserScan, BOUNDED_TOPIC, relayed.append, qos_profile_sensor_data
        )
        with candidate([relay, "--ros-args", "-p", f"scan_topic:={RAW}"]):
            bridge_gid = endpoint_gid(node, BOUNDED_TOPIC)
            raw_gid = endpoint_gid(node, RAW)
            with collector(node, executable) as (process, ready_ros):
                # A valid relay receipt cannot turn a pre-ready sensor stamp
                # into a post-ready sample. Send two while still freshly stamped.
                for offset in (2, 1):
                    raw.publish(scan_in_room(stamp_from_ns(ready_ros - offset), 0.0, 0.37))
                    spin_for(node, 0.06)
                assert process.poll() is None
                stale = [item for item in relayed if item.capture.valid]
                assert len(stale) == 2
                assert all(
                    item.capture.stamp == stamp_from_ns(ready_ros - offset)
                    for item, offset in zip(stale, (2, 1), strict=True)
                )

                # Send held-out values, including exact float32 intensity
                # and +infinity fields that must survive unchanged.
                published = []
                deadline = time.monotonic() + 4.0
                while process.poll() is None and time.monotonic() < deadline:
                    stamp = node.get_clock().now().to_msg()
                    scan = scan_in_room(stamp, 0.08, 0.41)
                    scan.ranges[0] = float("inf")
                    scan.intensities = [0.25] * len(scan.ranges)
                    published.append(scan)
                    raw.publish(scan)
                    spin_for(node, 0.06)
                output, _ = process.communicate(timeout=2.0)
                assert process.returncode == 0
                message = deserialize_message(output, BoundedRawLaserScan)
                header = message.capture
                assert header.valid and header.frame_id == LASER
                assert header.bridge_gid == bridge_gid and header.raw_gid == raw_gid
                assert header.bridge_boot_id and header.source_epoch
                assert header.capture_generation > 0 and header.sequence > 0
                assert header.received_steady > 0
                sample_ns = header.stamp.sec * 1_000_000_000 + header.stamp.nanosec
                receipt_ns = header.received_ros.sec * 1_000_000_000 + header.received_ros.nanosec
                assert ready_ros < sample_ns <= receipt_ns
                matching = [item for item in published if item.header.stamp == header.stamp]
                assert len(matching) == 1
                assert list(message.ranges) == list(matching[0].ranges)
                assert list(message.intensities) == list(matching[0].intensities)
                spin_for(node, 0.1)
                assert message in relayed

            with collector(node, executable) as (process, _):
                # Discovery of another raw publisher retires this armed cut
                # even before another message arrives.
                duplicate = node.create_publisher(LaserScan, RAW, qos_profile_sensor_data)
                try:
                    output, _ = process.communicate(timeout=4.0)
                    assert process.returncode == 1 and not output
                finally:
                    node.destroy_publisher(duplicate)
        node.destroy_subscription(subscription)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    probe(sys.argv[1])
