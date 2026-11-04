"""Match an exact live receipt against an isolated archived candidate."""

import json
import math
import os
import re
import secrets
import subprocess
import sys
import time
from pathlib import Path
from tempfile import TemporaryDirectory, TemporaryFile

import rclpy
from ament_index_python.packages import get_package_prefix
from probe_acquire_scan_receipt import ARMED
from probe_bounded_scan_ingress import BOUNDED_TOPIC, endpoint_gid, spin_for
from probe_candidate_load import SERIALIZE, candidate, capture, pair_digest
from probe_read_graph import BASE, LASER, MAP, ODOM, SCAN, SERVICE, scan_in_room, transform
from rclpy.qos import qos_profile_sensor_data
from senior_motion_msgs.msg import BoundedRawLaserScan
from senior_slam_bridge.srv import ReadGraph
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import SerializePoseGraph
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def run_live_match(node, raw, preflight, prefix, digest, domain, sample_frame):
    command = [
        sys.executable,
        preflight,
        str(prefix),
        digest,
        "--map-frame",
        MAP,
        "--odom-frame",
        ODOM,
        "--live-scan",
        "--live-domain",
        str(domain),
        "--live-clock",
        "wall",
        "--live-lidar-frame",
        LASER,
        "--live-seed",
        "0.05",
        "-0.02",
        "0.39",
        "--live-raw-topic",
        SCAN,
        "--live-upright",
    ]
    with TemporaryFile(mode="w+b") as log:
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=log)
        try:
            deadline = time.monotonic() + 22.0
            armed_ns = None
            while process.poll() is None and time.monotonic() < deadline:
                log_text = os.pread(log.fileno(), 65536, 0).decode(errors="replace")
                for line in log_text.splitlines():
                    if line.startswith(ARMED) and line.removeprefix(ARMED).isdigit():
                        armed_ns = int(line.removeprefix(ARMED))
                # The observer and candidate run in a different domain.
                assert not any(
                    name.startswith("slam_preflight_")
                    for name, _ in node.get_node_names_and_namespaces()
                )
                if armed_ns is not None:
                    scan = scan_in_room(node.get_clock().now().to_msg(), 0.08, 0.41)
                    scan.header.frame_id = sample_frame
                    raw.publish(scan)
                spin_for(node, 0.06)
            stdout, _ = process.communicate(timeout=2.0)
            stderr = os.pread(log.fileno(), 65536, 0).decode(errors="replace")
            assert armed_ns is not None, stderr
            return process.returncode, stdout, stderr, armed_ns
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.communicate(timeout=6.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.communicate(timeout=2.0)


def probe(executable, preflight, root):
    domain = 201 + secrets.randbelow(12)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    relay_prefix = Path(get_package_prefix("senior_sensor_ingress"))
    relay = relay_prefix / "lib" / "senior_sensor_ingress" / "scan_receipt_node"
    rclpy.init(args=[])
    node = rclpy.create_node("candidate_live_scan_probe")
    try:
        raw = node.create_publisher(LaserScan, SCAN, qos_profile_sensor_data)
        static_tf = StaticTransformBroadcaster(node)
        odom_tf = TransformBroadcaster(node)
        static_tf.sendTransform(transform(node.get_clock().now().to_msg(), BASE, LASER))
        graph = node.create_client(ReadGraph, SERVICE)
        serialize = node.create_client(SerializePoseGraph, SERIALIZE)
        prefix = root / "archive"
        snapshot = capture(executable, node, graph, serialize, raw, odom_tf, prefix)
        digest = pair_digest(prefix)

        relayed = []
        subscription = node.create_subscription(
            BoundedRawLaserScan, BOUNDED_TOPIC, relayed.append, qos_profile_sensor_data
        )
        with candidate([str(relay), "--ros-args", "-p", f"scan_topic:={SCAN}"]):
            bridge_gid = endpoint_gid(node, BOUNDED_TOPIC)
            raw_gid = endpoint_gid(node, SCAN)
            status, output, error, armed = run_live_match(
                node, raw, preflight, prefix, digest, domain, LASER
            )
            assert status == 0, error
            proof = json.loads(output)
            assert proof["boot_id"] != snapshot.boot_id and proof["pair_digest"] == digest
            assert (proof["map_frame"], proof["odom_frame"]) == (MAP, ODOM)
            assert proof["keyframes"] == [
                {"id": frame.id, "x": frame.x, "y": frame.y, "yaw": frame.yaw}
                for frame in snapshot.keyframes
            ]
            receipt = proof["live_scan_receipt"]
            assert receipt["raw_gid"] == raw_gid and receipt["bridge_gid"] == bridge_gid
            assert receipt["frame_id"] == LASER and receipt["stamp_ns"] > armed
            assert re.fullmatch(r"[0-9a-f]{64}", receipt["cdr_sha256"])
            spin_for(node, 0.1)
            (observed,) = [
                msg.capture
                for msg in relayed
                if msg.capture.valid and msg.capture.sequence == receipt["sequence"]
            ]
            for field in (
                "received_steady",
                "bridge_boot_id",
                "source_epoch",
                "capture_generation",
            ):
                assert getattr(observed, field) == receipt[field]
            assert (
                observed.stamp.sec * 1_000_000_000 + observed.stamp.nanosec == receipt["stamp_ns"]
            )
            diagnostic = proof["stationary_match_diagnostic"]
            assert diagnostic["response"] > 0
            assert diagnostic["reference_ids"] == [snapshot.keyframes[0].id]
            fitted = diagnostic["fitted_sensor_pose"]
            assert abs(fitted["x"] - 0.08) <= 0.06 and abs(fitted["y"]) <= 0.06
            assert abs(math.remainder(fitted["yaw"] - 0.41, math.tau)) <= 0.04

            relayed.clear()
            status, output, error, _ = run_live_match(
                node, raw, preflight, prefix, digest, domain, "wrong_frame"
            )
            assert status == 1 and not output, error
            assert "Live scan acquisition returned no usable receipt" in error, error
            assert any(
                msg.capture.valid and msg.capture.frame_id == "wrong_frame" for msg in relayed
            )
        node.destroy_subscription(subscription)
        assert pair_digest(prefix) == digest
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    with TemporaryDirectory(prefix="candidate-live-scan-") as directory:
        probe(sys.argv[1], sys.argv[2], Path(directory))
