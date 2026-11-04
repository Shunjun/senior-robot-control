"""Exercise verified static lidar TF admission and terminal retirement on Humble."""

import os
import secrets
import sys
import time
from copy import deepcopy

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
from probe_graph_commit_delivery import COMMIT_SERVICE
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
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from senior_motion_msgs.msg import BoundedRawLaserScan, BoundedStaticTf, StaticTransformEdge
from senior_slam_bridge.srv import ReadGraph, ReadGraphCommit
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

TF_RECEIPT = "/senior/slam/static_lidar_tf_receipt"


def probe(executable, drift):
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["ROS_DOMAIN_ID"] = str(secrets.randbelow(8) + (166 if drift else 158))
    rclpy.init(args=[])
    node = rclpy.create_node("static_lidar_probe")
    bridge = rclpy.create_node("scan_receipt")
    tf_source = rclpy.create_node("robot_state_publisher")
    relay = rclpy.create_node("slam_static_lidar_tf_receipt")
    try:
        graph = node.create_client(ReadGraph, SERVICE)
        commits = node.create_client(ReadGraphCommit, COMMIT_SERVICE)
        raw = node.create_publisher(LaserScan, SCAN, qos_profile_sensor_data)
        scans = bridge.create_publisher(BoundedRawLaserScan, BOUNDED_TOPIC, qos_profile_sensor_data)
        static_tf = StaticTransformBroadcaster(tf_source)
        odom_tf = TransformBroadcaster(node)
        witness = relay.create_publisher(
            BoundedStaticTf,
            TF_RECEIPT,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        stamp = node.get_clock().now().to_msg()
        static_tf.sendTransform(transform(stamp, BASE, LASER, 0.24))

        args = arguments(executable, None)
        args.extend(["-p", f"verified_static_lidar_tf_topic:={TF_RECEIPT}"])
        args.extend(["-p", f"verified_static_lidar_frame:={LASER}"])
        with candidate(args) as process:
            assert graph.wait_for_service(timeout_sec=8)
            assert commits.wait_for_service(timeout_sec=4)
            raw_gid, bridge_gid = endpoint_gid(node, SCAN), endpoint_gid(node, BOUNDED_TOPIC)
            tf_gid, relay_gid = endpoint_gid(node, "/tf_static"), endpoint_gid(node, TF_RECEIPT)
            boot = read_snapshot(node, graph).boot_id
            sequence = 0

            def publish_scan(x=0.0, stamp=None):
                nonlocal sequence
                sequence += 1
                now = node.get_clock().now().to_msg()
                scan_stamp = now if stamp is None else stamp
                odom_tf.sendTransform(transform(scan_stamp, ODOM, BASE, x, 0.37))
                scan = scan_in_room(scan_stamp, x, 0.37)
                scans.publish(bounded_scan(scan, now, bridge_gid, raw_gid, sequence))

            def lookup():
                return request(
                    node, commits, ReadGraphCommit.Request(boot_id=boot, graph_commit_sequence=1)
                )

            for _ in range(3):
                publish_scan()
                spin_for(node, 0.06)
            assert lookup().status == ReadGraphCommit.Response.UNAVAILABLE

            admitted_ros = node.get_clock().now().to_msg()
            admitted_steady = time.monotonic()
            receipt = BoundedStaticTf(
                bridge_boot_id="static-lidar-probe",
                bridge_gid=relay_gid,
                raw_gid=raw_gid,
                generation=1,
                sequence=1,
                first_sequence=1,
                first_received_ros=admitted_ros,
                first_received_steady=admitted_steady,
                received_ros=admitted_ros,
                received_steady=admitted_steady,
                admitted_from_ros=admitted_ros,
                admitted_from_steady=admitted_steady,
                valid=True,
                admitted=True,
                edges=[
                    StaticTransformEdge(
                        parent_frame=BASE,
                        child_frame=LASER,
                        translation=[0.24, 0.0, 0.0],
                        rotation_xyzw=[0.0, 0.0, 0.0, 1.0],
                    )
                ],
            )
            # A real typed writer cannot substitute an unrelated raw endpoint.
            witness.publish(receipt)
            spin_for(node, 0.08)
            publish_scan()
            spin_for(node, 0.08)
            assert lookup().status == ReadGraphCommit.Response.UNAVAILABLE

            receipt.raw_gid = tf_gid
            receipt.valid = False
            receipt.admitted = False
            witness.publish(receipt)
            spin_for(node, 0.08)
            receipt.valid = True
            receipt.admitted = True
            witness.publish(receipt)
            spin_for(node, 0.08)
            publish_scan()
            spin_for(node, 0.08)
            assert lookup().status == ReadGraphCommit.Response.UNAVAILABLE

            # A new generation can promote the same valid startup cursor, even
            # when the relay's ROS clock is briefly ahead of the consumer.
            receipt.generation = 3
            receipt.admitted = False
            witness.publish(receipt)
            spin_for(node, 0.08)
            pre_admission_stamp = node.get_clock().now().to_msg()
            receipt.admitted = True
            receipt.admitted_from_ros = (
                node.get_clock().now() + Duration(nanoseconds=250_000_000)
            ).to_msg()
            receipt.admitted_from_steady = time.monotonic()
            witness.publish(receipt)
            spin_for(node, 0.06)
            stale = deepcopy(receipt)
            stale.generation = 2
            stale.admitted_from_ros = admitted_ros
            stale.admitted_from_steady = admitted_steady
            stale.edges[0].translation = [0.8, 0.0, 0.0]
            witness.publish(stale)
            spin_for(node, 0.24)
            publish_scan(stamp=pre_admission_stamp)
            spin_for(node, 0.08)
            assert lookup().status == ReadGraphCommit.Response.UNAVAILABLE

            end = time.monotonic() + 4
            while time.monotonic() < end:
                publish_scan()
                spin_for(node, 0.12)
                if lookup().status == ReadGraphCommit.Response.OK:
                    break
            else:
                raise AssertionError("Valid static lidar witness did not admit any graph commit")

            assert not raw.get_subscription_count()
            if drift:
                # Same raw DDS owner, different TF content, old typed receipt still current.
                static_tf.sendTransform(
                    transform(node.get_clock().now().to_msg(), BASE, LASER, 0.8)
                )
                spin_for(node, 0.15)
                for _ in range(3):
                    publish_scan(x=0.5)
                    spin_for(node, 0.1)
            else:
                # The relay revokes its current generation at the existing sequence.
                receipt.valid = False
                receipt.admitted = False
                witness.publish(receipt)
            state = wait_for_graph(node, graph, lambda value: not value.last_committed_scan_valid)
            assert state.graph_commit_sequence == 1
            assert lookup().status == ReadGraphCommit.Response.UNAVAILABLE
            receipt.valid = True
            receipt.admitted = True
            witness.publish(receipt)
            static_tf.sendTransform(transform(node.get_clock().now().to_msg(), BASE, LASER, 0.24))
            publish_scan(x=1.0)
            spin_for(node, 0.12)
            assert lookup().status == ReadGraphCommit.Response.UNAVAILABLE
        assert process.returncode == 0
    finally:
        relay.destroy_node()
        tf_source.destroy_node()
        bridge.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    probe(sys.argv[1], "--drift" in sys.argv)
