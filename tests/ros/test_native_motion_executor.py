"""Bounded DDS-only Humble smoke: actual entrypoint executor and ROS callbacks.

Publishes synthetic Odometry/String only, no Twist/actuator, no Gazebo. The main
AST retains its actual run/executor choice; factory is a small ROS node to avoid
starting the full output process. Both new callbacks execute through rclpy take.
"""

import ast
import json
import os
import time
from pathlib import Path

import pytest


@pytest.mark.parametrize(
    ("filename", "factory_name"), [("motion_fusion", "MotionFusion"), ("safety_node", "SafetyGate")]
)
def test_actual_entrypoint_dispatches_native_metadata(filename, factory_name):
    rclpy = pytest.importorskip("rclpy")
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from senior_robot.motion_core_output import MotionCoreOutput
    from senior_robot.motion_lifecycle import MotionExecutor, run, with_dds_info
    from senior_robot.motion_observation_adapter import (
        NativeObservationRosBridge,
        NativeObservations,
    )
    from senior_robot.simulation_motion_profile import SimulationMotionProfile
    from std_msgs.msg import String

    assert os.environ.get("ROS_DOMAIN_ID") == "173" and os.environ.get("ROS_LOCALHOST_ONLY") == "1"
    original = Path(__file__).resolve().parents[2] / "ros2/src/senior_robot/senior_robot"
    directory = Path(os.environ.get("SENIOR_SMOKE_ENTRYPOINT_DIR", str(original)))
    tree = ast.parse((directory / (filename + ".py")).read_text())
    main = next(x for x in tree.body if isinstance(x, ast.FunctionDef) and x.name == "main")
    metadata = []

    class Consumer(Node):
        def __init__(self):
            super().__init__("native_dds_smoke")
            self.declare_parameter("robot_id", "route-sim")
            p = SimulationMotionProfile()
            self.output = MotionCoreOutput(self, p)
            self.bridge = object.__new__(NativeObservationRosBridge)
            self.bridge.node = self
            self.bridge.allow_topology_only = True
            self.bridge._warn_at = 0.0
            self.bridge.core = NativeObservations(p.admission(), p.temporal(), p.fusion())
            self.wheel_sub = self.create_subscription(
                Odometry,
                "/wheel/odom",
                self.bridge._callback(self.bridge.core.wheel, "/wheel/odom"),
                10,
            )

            @with_dds_info
            def inspect(msg, info):
                metadata.append(info)

            self.inspect_sub = self.create_subscription(
                String, "/motion/core_reference", inspect, 10
            )
            self.drive = Node("drive")
            self.fusion = Node("motion_fusion")
            self.wheel = self.drive.create_publisher(Odometry, "/wheel/odom", 10)
            self.reference = self.fusion.create_publisher(String, "/motion/core_reference", 10)

        def destroy_node(self):
            self.output.close()
            self.drive.destroy_node()
            self.fusion.destroy_node()
            return super().destroy_node()

    def smoke(node, executor):
        assert isinstance(executor, MotionExecutor), "entrypoint chose metadata-dropping executor"
        executor.add_node(node)
        executor.add_node(node.drive)
        executor.add_node(node.fusion)
        deadline = time.monotonic() + 6
        seq = 0
        while time.monotonic() < deadline:
            seq += 1
            wheel = Odometry()
            wheel.header.frame_id = "odom"
            wheel.child_frame_id = "base_link"
            wheel.header.stamp = node.get_clock().now().to_msg()
            wheel.pose.pose.orientation.w = 1.0
            node.wheel.publish(wheel)
            value = String()
            value.data = json.dumps(
                dict(
                    schema=1,
                    boot_id="synthetic-smoke",
                    sequence=seq,
                    sampled_steady=time.monotonic(),
                )
            )
            node.reference.publish(value)
            executor.spin_once(timeout_sec=0.02)
            sources = node.bridge.core.snapshot(
                node.get_clock().now().nanoseconds / 1e9, time.monotonic()
            )["sources"]
            if sources and node.output._reference_sequence and metadata:
                sample = metadata[-1]
                keys = sorted(sample.keys()) if isinstance(sample, dict) else ["MessageInfo"]
                print(
                    "DDS_SMOKE "
                    + json.dumps(
                        dict(
                            entrypoint=filename,
                            rclpy_path=rclpy.__file__,
                            executor=type(executor).__name__,
                            metadata_keys=keys,
                            source_epoch=sources[0]["sample"]["epoch"],
                            reference_sequence=node.output._reference_sequence,
                        )
                    )
                )
                return
        pytest.fail(
            "native wheel/reference callback did not receive actual DDS samples within 6 seconds"
        )

    def owned_run(factory, **kwargs):
        run(factory, spin=smoke, **kwargs)

    scope = {"run": owned_run, "MotionExecutor": MotionExecutor, factory_name: Consumer}
    exec(
        compile(
            ast.Module(body=[main], type_ignores=[]), str(directory / (filename + ".py")), "exec"
        ),
        scope,
    )
    scope["main"]()
