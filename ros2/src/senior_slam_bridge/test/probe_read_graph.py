"""Bounded DDS probe for the installed Humble slam_toolbox 2.6.10 bridge."""

import math
import os
import secrets
import subprocess
import sys
import time
from contextlib import contextmanager

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.qos import qos_profile_sensor_data
from senior_slam_bridge.srv import ReadGraph
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

MAP = "probe_map"
ODOM = "probe_odom"
BASE = "probe_base"
LASER = "probe_laser"
SCAN = "/probe/scan"
SERVICE = "/senior/slam/read_graph"


@contextmanager
def bridge_process(executable):
    args = [executable, "--ros-args"]
    for name, value in {
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
        "use_scan_matching": "true",
        "do_loop_closing": "false",
    }.items():
        args.extend(["-p", f"{name}:={value}"])
    process = subprocess.Popen(args)
    try:
        yield process
    finally:
        if process.poll() is None:
            process.terminate()
        try:
            process.wait(timeout=4)
        except subprocess.TimeoutExpired as error:
            process.kill()
            process.wait(timeout=2)
            raise AssertionError("SLAM bridge did not shut down within four seconds") from error


def transform(stamp, parent, child, x=0.0, yaw=0.0):
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x = x
    msg.transform.rotation.z = math.sin(yaw / 2)
    msg.transform.rotation.w = math.cos(yaw / 2)
    return msg


def scan_in_room(stamp, x, yaw):
    # Raycast four asymmetric walls to keep matching constrained in x/y/yaw.
    scan = LaserScan()
    scan.header.stamp = stamp
    scan.header.frame_id = LASER
    scan.angle_min = -math.pi / 2
    scan.angle_max = math.pi / 2
    scan.angle_increment = math.pi / 180
    scan.range_min = 0.05
    scan.range_max = 12.0
    for i in range(181):
        bearing = yaw + scan.angle_min + i * scan.angle_increment
        dx, dy = math.cos(bearing), math.sin(bearing)
        wall_x = 4.0 if dx > 0 else -2.0
        wall_y = 2.0 if dy > 0 else -3.0
        distance_x = (wall_x - x) / dx if dx else math.inf
        distance_y = wall_y / dy if dy else math.inf
        scan.ranges.append(min(distance_x, distance_y))
    return scan


def read_snapshot(node, client):
    pending = client.call_async(ReadGraph.Request())
    rclpy.spin_until_future_complete(node, pending, timeout_sec=1.5)
    if not pending.done():
        client.remove_pending_request(pending)
        raise AssertionError("ReadGraph response timed out")
    result = pending.result()
    assert result.boot_id
    assert result.snapshot_sequence > 0
    assert (result.map_frame, result.odom_frame) == (MAP, ODOM)
    assert result.sample_stamp.sec > 0
    return result


def wait_for_scan(node, client, publisher, broadcaster, x, yaw, count):
    deadline = time.monotonic() + 8.0
    stamp = node.get_clock().now().to_msg()
    odom = transform(stamp, ODOM, BASE, x, yaw)
    scan = scan_in_room(stamp, x, yaw)
    # Re-send the same observation to tolerate DDS discovery and TF delivery order.
    while time.monotonic() < deadline:
        broadcaster.sendTransform(odom)
        publisher.publish(scan)
        rclpy.spin_once(node, timeout_sec=0.05)
        graph = read_snapshot(node, client)
        assert graph.status != ReadGraph.Response.UNAVAILABLE
        if len(graph.keyframes) >= count:
            assert graph.status == ReadGraph.Response.OK
            assert len(graph.keyframes) == count
            return graph
    raise AssertionError(f"Did not get {count} Karto keyframes")


def check_no_velocity_publishers(node):
    for topic, types in node.get_topic_names_and_types():
        is_velocity = "cmd_vel" in topic or bool(
            {"geometry_msgs/msg/Twist", "geometry_msgs/msg/TwistStamped"} & set(types)
        )
        if is_velocity:
            assert node.count_publishers(topic) == 0, topic


def probe(executable):
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["ROS_DOMAIN_ID"] = str(secrets.randbelow(70) + 30)
    rclpy.init(args=[])
    node = rclpy.create_node("slam_graph_probe")
    try:
        publisher = node.create_publisher(LaserScan, SCAN, qos_profile_sensor_data)
        static_tf = StaticTransformBroadcaster(node)
        odom_tf = TransformBroadcaster(node)
        static_tf.sendTransform(transform(node.get_clock().now().to_msg(), BASE, LASER))
        client = node.create_client(ReadGraph, SERVICE)
        with bridge_process(executable):
            assert client.wait_for_service(timeout_sec=8.0), "ReadGraph did not start"
            empty = read_snapshot(node, client)
            assert empty.status == ReadGraph.Response.EMPTY
            assert not empty.keyframes
            first = wait_for_scan(node, client, publisher, odom_tf, 0.0, 0.37, count=1)
            full = wait_for_scan(node, client, publisher, odom_tf, 0.6, 0.55, count=2)
            assert empty.boot_id == first.boot_id == full.boot_id
            assert empty.snapshot_sequence < first.snapshot_sequence < full.snapshot_sequence
            assert [frame.id for frame in full.keyframes] == [0, 1]
            for frame in full.keyframes:
                assert all(math.isfinite(v) for v in (frame.x, frame.y, frame.yaw))
            assert math.isclose(first.keyframes[0].yaw, 0.37, abs_tol=0.02)
            assert math.isclose(full.keyframes[1].yaw, 0.55, abs_tol=0.12)
            check_no_velocity_publishers(node)

        deadline = time.monotonic() + 6.0
        while client.service_is_ready() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert not client.service_is_ready(), "Old ReadGraph service is still advertised"
        with bridge_process(executable):
            assert client.wait_for_service(timeout_sec=8.0), "ReadGraph did not restart"
            restarted = read_snapshot(node, client)
            assert restarted.status == ReadGraph.Response.EMPTY
            assert not restarted.keyframes
            assert restarted.boot_id != empty.boot_id
            check_no_velocity_publishers(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    probe(sys.argv[1])
