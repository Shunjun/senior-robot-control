"""Actual 50-thread regression: bound Ceres during the approved serial graph reload."""

import re
import subprocess
import sys
import tempfile
import uuid
from contextlib import contextmanager
from pathlib import Path

import rclpy
from probe_bounded_scan_ingress import (
    BOUNDED_TOPIC,
    arguments,
    bounded_scan,
    endpoint_gid,
    spin_for,
    wait_for_graph,
)
from probe_candidate_load import request, wait_for_disappearance
from probe_native_capture import CAPTURE_SERVICE, OWNED_NODE, check_capture
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
from senior_interfaces.msg import BoundedRawLaserScan
from senior_interfaces.srv import CaptureGraph, MappingGate, ReadGraph
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

reports = []


@contextmanager
def candidate(args):
    with tempfile.TemporaryFile(mode="w+b") as log:
        args += [
            "-p",
            "solver_plugin:=senior_mapping/BoundedCeresSolver",
            "-p",
            "debug_logging:=true",
        ]
        process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT)
        try:
            yield process
        finally:
            if process.poll() is None:
                process.terminate()
            try:
                process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)
                raise AssertionError("Production SLAM did not stop") from None
            log.seek(0)
            report = log.read().decode(errors="replace")
            reports.append(report)
            print(report)


rclpy.init()
node = rclpy.create_node("production_serial_probe")
relay = rclpy.create_node("scan_receipt")
try:
    raw = node.create_publisher(LaserScan, SCAN, qos_profile_sensor_data)
    scans = relay.create_publisher(BoundedRawLaserScan, BOUNDED_TOPIC, qos_profile_sensor_data)
    static = StaticTransformBroadcaster(node)
    odom = TransformBroadcaster(node)
    static.sendTransform(transform(node.get_clock().now().to_msg(), BASE, LASER))
    graph = node.create_client(ReadGraph, SERVICE)
    gate = node.create_client(MappingGate, "/senior/slam/mapping_gate")
    capture = node.create_client(CaptureGraph, CAPTURE_SERVICE)
    raw_gid, relay_gid = endpoint_gid(node, SCAN), endpoint_gid(node, BOUNDED_TOPIC)
    sequence = 0

    def publish(x):
        global sequence
        sequence += 1
        stamp = node.get_clock().now().to_msg()
        odom.sendTransform(transform(stamp, ODOM, BASE, x, 0.37))
        scan = scan_in_room(stamp, x, 0.37)
        scans.publish(bounded_scan(scan, stamp, relay_gid, raw_gid, sequence))
        spin_for(node, 0.08)

    def control(boot, accepting):
        query = MappingGate.Request()
        query.boot_id, query.accepting = boot, accepting
        result = request(node, gate, query)
        assert result.ok
        return result.graph_commit_sequence

    with tempfile.TemporaryDirectory(prefix="production-serial-") as directory:
        root = Path(directory)
        first = root / "first"
        second = root / "second"
        first.mkdir(mode=0o700)
        second.mkdir(mode=0o700)
        boot = str(uuid.uuid4())
        args = arguments(sys.argv[1], first) + [
            "-p",
            "production_session:=true",
            "-p",
            "production_boot_id:=" + boot,
        ]
        with candidate(args):
            assert graph.wait_for_service(timeout_sec=8) and gate.wait_for_service(timeout_sec=4)
            assert read_snapshot(node, graph).boot_id == boot
            spin_for(node, 0.4)
            publish(0.0)
            assert read_snapshot(node, graph).graph_commit_sequence == 0
            assert control(boot, True) == 0
            publish(0.0)
            wait_for_graph(node, graph, lambda result: result.graph_commit_sequence == 1)
            for _ in range(7):
                publish(0.2)
                if read_snapshot(node, graph).graph_commit_sequence == 2:
                    break
            wait_for_graph(node, graph, lambda result: result.graph_commit_sequence == 2)
            assert control(boot, False) == 2
            publish(0.35)
            assert read_snapshot(node, graph).graph_commit_sequence == 2
            saved = request(node, capture, CaptureGraph.Request())
            prefix = check_capture(saved, first, boot)
            assert saved.graph_commit_sequence == 2
        wait_for_disappearance(node, OWNED_NODE, graph, gate, capture)
        boot = str(uuid.uuid4())
        args = arguments(sys.argv[1], second) + [
            "-p",
            "production_session:=true",
            "-p",
            "production_boot_id:=" + boot,
            "-p",
            "production_graph_prefix:=" + str(prefix),
            "-p",
            "production_graph_digest:=" + saved.pair_digest,
            "-p",
            "production_seed:=[0.08, -0.03, 0.1]",
        ]
        with candidate(args):
            assert graph.wait_for_service(timeout_sec=8) and gate.wait_for_service(timeout_sec=4)
            loaded = read_snapshot(node, graph)
            assert loaded.boot_id == boot and loaded.graph_commit_sequence == 0
            assert list(loaded.keyframes) == list(saved.keyframes)
            assert control(boot, True) == 0
            publish(0.35)
            resumed = wait_for_graph(node, graph, lambda result: result.graph_commit_sequence == 1)
            assert len(resumed.keyframes) > len(saved.keyframes)
            control(boot, False)
    assert any(re.search(r"Residuals\s+[1-9]\d*", report) for report in reports), reports
    assert any(re.search(r"Threads\s+1\s+1", report) for report in reports), reports
    assert all("options.num_threads: 50" not in report for report in reports)
    print(
        "Native production gate, paused drop, exact capture, serial restore and resumed commit PASS"
    )
finally:
    relay.destroy_node()
    node.destroy_node()
    rclpy.shutdown()
