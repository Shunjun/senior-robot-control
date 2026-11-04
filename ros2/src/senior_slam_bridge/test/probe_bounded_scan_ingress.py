"""Verify exact bounded input lineage without using the legacy TF message filter."""

import os
import secrets
import sys
import tempfile
import time
from pathlib import Path

import rclpy
from probe_candidate_load import (
    RECEIPT,
    candidate,
    candidate_arguments,
    observe,
    request,
    wait_for_disappearance,
)
from probe_native_capture import CAPTURE_SERVICE, OWNED_NODE, check_capture, owned_arguments
from probe_read_graph import (
    BASE,
    LASER,
    ODOM,
    SCAN,
    SERVICE,
    read_snapshot,
    scan_in_room,
    transform,
)
from rclpy.qos import qos_profile_sensor_data
from senior_motion_msgs.msg import BoundedRawLaserScan
from senior_slam_bridge.srv import CaptureGraph, ReadGraph, ReadLoadedGraph
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

BOUNDED_TOPIC = "/safety/raw/scan"


def spin_for(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


def wait_for_graph(node, client, predicate):
    end = time.monotonic() + 3.0
    while time.monotonic() < end:
        snapshot = read_snapshot(node, client)
        if predicate(snapshot):
            return snapshot
        spin_for(node, 0.03)
    raise AssertionError("Expected graph or lineage state was not observed")


def endpoint_gid(node, topic):
    end = time.monotonic() + 4
    while time.monotonic() < end:
        endpoints = node.get_publishers_info_by_topic(topic)
        if len(endpoints) == 1:
            return bytes(endpoints[0].endpoint_gid).hex()
        spin_for(node, 0.03)
    raise AssertionError(f"Unique publisher on {topic} was not discovered")


def bounded_scan(scan, received_ros, bridge_gid, raw_gid, sequence):
    result = BoundedRawLaserScan()
    for field in (
        "angle_min",
        "angle_max",
        "angle_increment",
        "time_increment",
        "scan_time",
        "range_min",
        "range_max",
        "ranges",
        "intensities",
    ):
        setattr(result, field, getattr(scan, field))
    result.capture.stamp = scan.header.stamp
    result.capture.received_ros = received_ros
    result.capture.received_steady = time.monotonic()
    result.capture.frame_id = scan.header.frame_id
    result.capture.raw_gid = raw_gid
    result.capture.bridge_gid = bridge_gid
    result.capture.bridge_boot_id = "boot-bounded-probe"
    result.capture.source_epoch = "epoch-bounded-probe"
    result.capture.capture_generation = 1
    result.capture.sequence = sequence
    result.capture.valid = True
    return result


def arguments(executable, capture_directory):
    result = owned_arguments(executable, capture_directory or "")
    if capture_directory is None:
        index = result.index("capture_directory:=")
        del result[index - 1 : index + 1]
    result.extend(["-p", f"bounded_scan_topic:={BOUNDED_TOPIC}"])
    return result


def probe(executable, root, combined):
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["ROS_DOMAIN_ID"] = str(secrets.randbelow(12) + (128 if combined else 115))
    staging = root / "staging" if combined else None
    if staging is not None:
        staging.mkdir(mode=0o700)
    rclpy.init(args=[])
    node = rclpy.create_node("slam_candidate_probe")
    bridge = rclpy.create_node("scan_receipt")
    try:
        graph = node.create_client(ReadGraph, SERVICE)
        capture = node.create_client(CaptureGraph, CAPTURE_SERVICE)
        saved = None
        raw = node.create_publisher(LaserScan, SCAN, qos_profile_sensor_data)
        publisher = bridge.create_publisher(
            BoundedRawLaserScan, BOUNDED_TOPIC, qos_profile_sensor_data
        )
        static_tf = StaticTransformBroadcaster(node)
        odom_tf = TransformBroadcaster(node)
        static_tf.sendTransform(transform(node.get_clock().now().to_msg(), BASE, LASER))
        with candidate(arguments(executable, staging)) as process:
            assert graph.wait_for_service(timeout_sec=8)
            if staging is None:
                assert not capture.service_is_ready()
            else:
                assert capture.wait_for_service(timeout_sec=4)
                empty_cut = request(node, capture, CaptureGraph.Request())
                assert empty_cut.status == CaptureGraph.Response.EMPTY
                assert not empty_cut.last_committed_scan_valid
                assert not list(staging.iterdir())
            raw_gid, bridge_gid = endpoint_gid(node, SCAN), endpoint_gid(node, BOUNDED_TOPIC)
            end = time.monotonic() + 4
            while not publisher.get_subscription_count() and time.monotonic() < end:
                spin_for(node, 0.03)
            assert publisher.get_subscription_count() == 1
            assert not raw.get_subscription_count(), "The legacy LaserScan filter still subscribes"

            # No odometry TF is available. Fill the bounded queue and let every entry expire.
            expired = []
            for sequence in range(1, 12):
                stamp = node.get_clock().now().to_msg()
                expired.append(stamp)
                scan = scan_in_room(stamp, 0.0, 0.37)
                publisher.publish(bounded_scan(scan, stamp, bridge_gid, raw_gid, sequence))
                spin_for(node, 0.015)
            spin_for(node, 0.55)
            for stamp in expired:
                odom_tf.sendTransform(transform(stamp, ODOM, BASE, 0.0, 0.37))
            spin_for(node, 0.1)
            empty = read_snapshot(node, graph)
            assert empty.graph_commit_watermark_valid and not empty.graph_commit_sequence
            assert not empty.last_committed_scan_valid

            stamp = node.get_clock().now().to_msg()
            odom_tf.sendTransform(transform(stamp, ODOM, BASE, 0.0, 0.37))
            scan = scan_in_room(stamp, 0.0, 0.37)
            # The same typed writer cannot substitute an unauthenticated bridge GID.
            spoof = bounded_scan(scan, stamp, raw_gid, raw_gid, 12)
            publisher.publish(spoof)
            raw.publish(scan)
            spin_for(node, 0.12)
            assert not read_snapshot(node, graph).graph_commit_sequence

            valid = bounded_scan(scan, node.get_clock().now().to_msg(), bridge_gid, raw_gid, 13)
            publisher.publish(valid)
            # Same header, different receipt/body. It must never replace the exact queued scan.
            duplicate = bounded_scan(scan, node.get_clock().now().to_msg(), bridge_gid, raw_gid, 14)
            duplicate.ranges[90] += 0.25
            publisher.publish(duplicate)
            committed = wait_for_graph(node, graph, lambda value: value.graph_commit_sequence == 1)
            assert committed.last_committed_scan_valid
            assert committed.last_committed_scan == valid.capture
            assert committed.last_committed_keyframe_id == committed.keyframes[0].id

            if staging is not None:
                # A capture is a historical graph cut, not a renewal of scan freshness.
                spin_for(node, 0.6)
                assert time.monotonic() - valid.capture.received_steady > 0.5
                cut = request(node, capture, CaptureGraph.Request())
                prefix = check_capture(cut, staging, committed.boot_id)
                assert cut.last_committed_scan_valid and cut.last_committed_scan == valid.capture
                assert not cut.static_lidar_tf_receipt_valid
                assert not cut.static_lidar_tf_receipt.edges
                assert cut.graph_commit_sequence == committed.graph_commit_sequence
                assert cut.last_committed_keyframe_id == committed.last_committed_keyframe_id
                assert cut.keyframes == committed.keyframes
                prefix.parent.rename(root / "saved")
                saved = root / "saved" / prefix.name, cut

            # A lost writer breaks exclusivity and revokes the lineage.
            bridge.destroy_publisher(publisher)
            retired = wait_for_graph(node, graph, lambda value: not value.last_committed_scan_valid)
            assert retired.graph_commit_sequence == 1 and retired.keyframes == committed.keyframes
            if staging is not None:
                refused = request(node, capture, CaptureGraph.Request())
                assert refused.status == CaptureGraph.Response.UNAVAILABLE
                assert not refused.last_committed_scan_valid and not refused.keyframes
                assert not refused.pair_digest and not refused.relative_prefix
                assert not list(staging.iterdir())
            assert process.poll() is None
        assert process.returncode == 0
        wait_for_disappearance(node, OWNED_NODE, graph, capture)
        if saved is not None:
            prefix, cut = saved
            receipt = node.create_client(ReadLoadedGraph, RECEIPT)
            attempt = "bounded-capture"
            args = candidate_arguments(executable, prefix, cut.pair_digest, attempt)
            with candidate(args) as reloaded:
                observe(node, receipt, graph, prefix, attempt, {cut.boot_id}, cut)
                assert not read_snapshot(node, graph).last_committed_scan_valid
            assert reloaded.returncode == 0
    finally:
        bridge.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    with tempfile.TemporaryDirectory() as scratch:
        probe(sys.argv[1], Path(scratch).resolve(), "--capture" in sys.argv[2:])
