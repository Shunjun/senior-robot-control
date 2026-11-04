"""Humble probe for exact native per-commit lookup, eviction, and owner revocation."""

import math
import os
import secrets
import sys
import time

import rclpy
from probe_bounded_scan_ingress import (
    BOUNDED_TOPIC,
    arguments,
    bounded_scan,
    endpoint_gid,
    spin_for,
    wait_for_graph,
)
from probe_candidate_load import candidate, request
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
)
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
from senior_motion_msgs.msg import BoundedRawLaserScan
from senior_slam_bridge.srv import ReadGraph, ReadGraphCommit
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

COMMIT_SERVICE = "/senior/slam/read_graph_commit"
MOUNT_X = 0.24
MOUNT_Y = -0.08
MOUNT_YAW = 0.21


def check_unavailable_data(response):
    assert not response.scan.capture.valid
    assert not response.scan.ranges and not response.keyframes
    assert response.raw_ray_angle_sign == 0


def check_sensor_pose(cut):
    (base,) = [frame for frame in cut.keyframes if frame.id == cut.committed_keyframe_id]
    cos_yaw, sin_yaw = math.cos(base.yaw), math.sin(base.yaw)
    assert math.isclose(
        cut.corrected_sensor_x, base.x + MOUNT_X * cos_yaw - MOUNT_Y * sin_yaw, abs_tol=1e-8
    )
    assert math.isclose(
        cut.corrected_sensor_y, base.y + MOUNT_X * sin_yaw + MOUNT_Y * cos_yaw, abs_tol=1e-8
    )
    difference = math.remainder(cut.corrected_sensor_yaw - base.yaw - MOUNT_YAW, math.tau)
    assert abs(difference) < 1e-8
    assert cut.raw_ray_angle_sign == 1


def probe(executable):
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["ROS_DOMAIN_ID"] = str(secrets.randbelow(12) + 140)
    rclpy.init(args=[])
    node = rclpy.create_node("slam_graph_commit_probe")
    bridge = rclpy.create_node("scan_receipt")
    try:
        graph = node.create_client(ReadGraph, SERVICE)
        commits = node.create_client(ReadGraphCommit, COMMIT_SERVICE)
        raw = node.create_publisher(LaserScan, SCAN, qos_profile_sensor_data)
        publisher = bridge.create_publisher(
            BoundedRawLaserScan, BOUNDED_TOPIC, qos_profile_sensor_data
        )
        tf = TransformBroadcaster(node)

        with candidate(arguments(executable, None)) as process:
            assert graph.wait_for_service(timeout_sec=8)
            assert commits.wait_for_service(timeout_sec=4)
            raw_gid, bridge_gid = endpoint_gid(node, SCAN), endpoint_gid(node, BOUNDED_TOPIC)
            end = time.monotonic() + 4
            while not publisher.get_subscription_count() and time.monotonic() < end:
                spin_for(node, 0.03)
            assert publisher.get_subscription_count() == 1 and not raw.get_subscription_count()

            boot = read_snapshot(node, graph).boot_id
            raw_sequence = 0
            offered = {}

            def lookup(sequence, owner=boot):
                return request(
                    node,
                    commits,
                    ReadGraphCommit.Request(boot_id=owner, graph_commit_sequence=sequence),
                )

            def publish_scan(x, yaw, mount_x=MOUNT_X):
                nonlocal raw_sequence
                raw_sequence += 1
                stamp = node.get_clock().now().to_msg()
                mount = transform(stamp, BASE, LASER, mount_x, MOUNT_YAW)
                mount.transform.translation.y = MOUNT_Y
                mount.transform.translation.z = 0.16
                tf.sendTransform([transform(stamp, ODOM, BASE, x, yaw), mount])
                scan = scan_in_room(stamp, x, yaw)
                original = bounded_scan(scan, stamp, bridge_gid, raw_gid, raw_sequence)
                # Python keeps scalar assignments as doubles until the ROS float32
                # serializer runs. Capture the exact typed wire representation.
                offered[raw_sequence] = serialize_message(original)
                publisher.publish(original)

            def commit_next(sequence, x, yaw):
                end = time.monotonic() + 4
                while time.monotonic() < end:
                    publish_scan(x, yaw)
                    spin_for(node, 0.12)
                    cut = lookup(sequence)
                    if cut.status == ReadGraphCommit.Response.OK:
                        assert cut.boot_id == boot and cut.graph_commit_sequence == sequence
                        assert (cut.map_frame, cut.odom_frame) == (MAP, ODOM)
                        assert cut.sample_stamp.sec > 0
                        assert serialize_message(cut.scan) == offered[cut.scan.capture.sequence]
                        assert len(cut.keyframes) == sequence
                        check_sensor_pose(cut)
                        return cut
                    # The source can be undiscovered only before the first admitted scan.
                    assert cut.status == ReadGraphCommit.Response.PENDING or (
                        sequence == 1 and cut.status == ReadGraphCommit.Response.UNAVAILABLE
                    )
                raise AssertionError(f"Graph commit {sequence} did not arrive")

            first = commit_next(1, 0.0, 0.37)
            response = lookup(2)
            assert response.status == ReadGraphCommit.Response.PENDING
            check_unavailable_data(response)
            for bad in (lookup(0), lookup(1, owner="foreign-boot")):
                assert bad.status == ReadGraphCommit.Response.UNAVAILABLE
                check_unavailable_data(bad)

            # Humble suppresses some early scans until sufficient observations arrive.
            # Stationary admitted scans may advance the raw counter but do not make graph cuts.
            for _ in range(5):
                publish_scan(0.0, 0.37)
                spin_for(node, 0.04)
            assert lookup(2).status == ReadGraphCommit.Response.PENDING

            for sequence in range(2, 9):
                latest = commit_next(sequence, 0.35 * (sequence - 1), 0.37 + sequence * 0.02)
            historical = lookup(1)
            assert historical.status == ReadGraphCommit.Response.OK
            assert historical.scan == first.scan and historical.keyframes == first.keyframes
            snapshot = read_snapshot(node, graph)
            assert snapshot.graph_commit_sequence == 8 and snapshot.keyframes == latest.keyframes

            ninth = commit_next(9, 2.8, 0.55)
            gap = lookup(1)
            assert gap.status == ReadGraphCommit.Response.GAP
            check_unavailable_data(gap)
            assert lookup(2).status == ReadGraphCommit.Response.OK

            # The DDS owner is unchanged, but the exact-time mount moved.
            publish_scan(3.15, 0.57, mount_x=MOUNT_X + 0.12)
            wait_for_graph(node, graph, lambda value: not value.last_committed_scan_valid)
            assert endpoint_gid(node, BOUNDED_TOPIC) == bridge_gid
            assert read_snapshot(node, graph).graph_commit_sequence == ninth.graph_commit_sequence
            publish_scan(3.2, 0.59)
            spin_for(node, 0.1)
            refused = lookup(ninth.graph_commit_sequence)
            assert refused.status == ReadGraphCommit.Response.UNAVAILABLE
            check_unavailable_data(refused)
        assert process.returncode == 0
    finally:
        bridge.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    probe(sys.argv[1])
