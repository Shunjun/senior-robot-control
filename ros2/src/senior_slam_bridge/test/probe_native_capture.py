"""Humble probe for capturing native graph bytes and keyframes in one observation."""

import os
import secrets
import stat
import sys
import tempfile
from pathlib import Path

import rclpy
from probe_candidate_load import (
    RECEIPT,
    candidate,
    candidate_arguments,
    observe,
    pair_digest,
    request,
    wait_for_disappearance,
)
from probe_read_graph import (
    BASE,
    LASER,
    MAP,
    ODOM,
    SCAN,
    SERVICE,
    read_snapshot,
    scan_in_room,
    transform,
    wait_for_scan,
)
from rclpy.qos import qos_profile_sensor_data
from senior_slam_bridge.srv import CaptureGraph, ReadGraph, ReadLoadedGraph
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

CAPTURE_SERVICE = "/senior/slam/capture_graph"
OWNED_NODE = "slam_owned_capture"


def owned_arguments(executable, root):
    args = [executable, "--ros-args", "-r", f"__node:={OWNED_NODE}"]
    for name, value in {
        "capture_directory": root,
        "map_frame": MAP,
        "odom_frame": ODOM,
        "base_frame": BASE,
        "scan_topic": SCAN,
        "use_map_saver": "false",
        "transform_publish_period": "0.0",
        "map_update_interval": "0.1",
        "minimum_time_interval": "0.0",
        "minimum_travel_distance": "0.1",
        "minimum_travel_heading": "0.1",
        "do_loop_closing": "false",
    }.items():
        args.extend(["-p", f"{name}:={value}"])
    return args


def check_capture(result, root, boot):
    assert result.status == CaptureGraph.Response.OK
    assert result.boot_id == boot
    assert (result.map_frame, result.odom_frame) == (MAP, ODOM)
    assert result.sample_stamp.sec > 0
    prefix = root / result.relative_prefix
    assert prefix == root / "capture" / "graph"
    assert stat.S_IMODE(prefix.parent.stat().st_mode) == 0o700
    for suffix in (".posegraph", ".data"):
        assert stat.S_IMODE(prefix.with_suffix(suffix).stat().st_mode) == 0o600
    assert pair_digest(prefix) == result.pair_digest
    return prefix


def check_watermark(result, sequence, keyframe_id=None):
    assert result.graph_commit_watermark_valid
    assert result.graph_commit_sequence == sequence
    if keyframe_id is not None:
        assert result.last_committed_keyframe_id == keyframe_id


def probe(executable):
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["ROS_DOMAIN_ID"] = str(secrets.randbelow(12) + 102)
    rclpy.init(args=[])
    node = rclpy.create_node("slam_candidate_probe")
    try:
        graph = node.create_client(ReadGraph, SERVICE)
        capture = node.create_client(CaptureGraph, CAPTURE_SERVICE)
        receipt = node.create_client(ReadLoadedGraph, RECEIPT)
        publisher = node.create_publisher(LaserScan, SCAN, qos_profile_sensor_data)
        static_tf = StaticTransformBroadcaster(node)
        odom_tf = TransformBroadcaster(node)
        static_tf.sendTransform(transform(node.get_clock().now().to_msg(), BASE, LASER))
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw).resolve()
            staging = root / "staging"
            staging.mkdir(mode=0o700)
            with candidate(owned_arguments(executable, staging)) as process:
                assert capture.wait_for_service(timeout_sec=8)
                assert graph.wait_for_service(timeout_sec=4)
                empty = request(node, capture, CaptureGraph.Request())
                assert empty.status == CaptureGraph.Response.EMPTY
                assert not empty.keyframes and not empty.relative_prefix and not empty.pair_digest
                check_watermark(empty, 0)
                assert not list(staging.iterdir())
                services = dict(node.get_service_names_and_types_by_node(OWNED_NODE, "/"))
                assert (
                    not {
                        "/slam_toolbox/deserialize_map",
                        "/slam_toolbox/serialize_map",
                        "/slam_toolbox/pause_new_measurements",
                    }
                    & services.keys()
                ), services

                before = wait_for_scan(node, graph, publisher, odom_tf, 0.0, 0.37, count=1)
                check_watermark(before, 1, before.keyframes[0].id)
                # Repeated reads and stationary scans with duplicate headers do not commit.
                stamp = node.get_clock().now().to_msg()
                stationary = scan_in_room(stamp, 0.0, 0.37)
                stationary_odom = transform(stamp, ODOM, BASE, 0.0, 0.37)
                repeated = before
                for _ in range(6):
                    odom_tf.sendTransform(stationary_odom)
                    publisher.publish(stationary)
                    rclpy.spin_once(node, timeout_sec=0.05)
                    observed = read_snapshot(node, graph)
                    assert repeated.snapshot_sequence < observed.snapshot_sequence
                    assert observed.keyframes == before.keyframes
                    check_watermark(observed, 1, before.keyframes[0].id)
                    repeated = observed

                first = request(node, capture, CaptureGraph.Request())
                first_prefix = check_capture(first, staging, before.boot_id)
                assert first.keyframes == before.keyframes
                check_watermark(first, 1, before.keyframes[0].id)
                full = request(node, capture, CaptureGraph.Request())
                assert full.status == CaptureGraph.Response.UNAVAILABLE
                assert not full.keyframes and not full.relative_prefix and not full.pair_digest
                assert not full.graph_commit_watermark_valid
                # An occupied slot is rejected before sampling or serializing the mapper.
                assert (full.sample_stamp.sec, full.sample_stamp.nanosec) == (0, 0)
                assert pair_digest(first_prefix) == first.pair_digest
                saved_first = root / "first"
                first_prefix.parent.rename(saved_first)
                first_prefix = saved_first / "graph"

                # A fresh scan can be dispatched immediately before or after capture.
                stamp = node.get_clock().now().to_msg()
                odom_tf.sendTransform(transform(stamp, ODOM, BASE, 0.6, 0.55))
                publisher.publish(scan_in_room(stamp, 0.6, 0.55))
                raced = request(node, capture, CaptureGraph.Request())
                raced_prefix = check_capture(raced, staging, before.boot_id)
                after = wait_for_scan(node, graph, publisher, odom_tf, 0.6, 0.55, count=2)
                assert raced.keyframes in (before.keyframes, after.keyframes)
                (new_keyframe,) = set(frame.id for frame in after.keyframes) - {
                    frame.id for frame in before.keyframes
                }
                check_watermark(after, 2, new_keyframe)
                cut = before if raced.keyframes == before.keyframes else after
                check_watermark(raced, cut.graph_commit_sequence, cut.last_committed_keyframe_id)
                assert empty.snapshot_sequence < first.snapshot_sequence < raced.snapshot_sequence
                assert process.poll() is None
            assert process.returncode == 0
            wait_for_disappearance(node, OWNED_NODE, graph, capture)

            boots = {before.boot_id}
            for prefix, snapshot, attempt in (
                (first_prefix, first, "capture-first"),
                (raced_prefix, raced, "capture-raced"),
            ):
                args = candidate_arguments(executable, prefix, snapshot.pair_digest, attempt)
                with candidate(args) as process:
                    boots.add(observe(node, receipt, graph, prefix, attempt, boots, snapshot))
                    assert not read_snapshot(node, graph).graph_commit_watermark_valid
                assert process.returncode == 0
                wait_for_disappearance(node, "slam_graph_candidate", graph, receipt)

            # Owned capture must start a fresh graph. A loaded graph has no local commit history.
            preload_root = root / "preload"
            preload_root.mkdir(mode=0o700)
            rejected = owned_arguments(executable, preload_root)
            rejected.extend(
                ["-p", f"map_file_name:={first_prefix}", "-p", "map_start_at_dock:=true"]
            )
            with candidate(rejected) as process:
                assert process.wait(timeout=4) != 0
            assert not capture.service_is_ready()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    probe(sys.argv[1])
