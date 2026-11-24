"""Dedicated ROS reception/estimation process with one bounded snapshot slot."""

import multiprocessing as mp
import os

from vehicle_core.process_resources import LatestSnapshot, child_lifetime, stop_process


def _receive(parameters, remaps, snapshots, stop, parent_pid):
    child_lifetime(parent_pid)
    import rclpy
    from rclpy.parameter import Parameter
    from senior_sensor_ingress.ros_lifecycle import MotionExecutor

    from senior_motion_control.safety_node import SafetyGate

    args = ["--ros-args"]
    for source, target in remaps:
        args.extend(["-r", f"{source}:={target}"])
    rclpy.init(args=args)
    node = executor = None
    try:
        node = SafetyGate(
            parameter_overrides=[Parameter(name, value=value) for name, value in parameters],
            input_channel=snapshots,
        )
        executor = MotionExecutor()
        executor.add_node(node)
        while rclpy.ok() and not stop.is_set() and os.getppid() == parent_pid:
            executor.spin_once(timeout_sec=0.01)
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


class ControlInputs:
    def __init__(self, node):
        context = mp.get_context("spawn")
        self.slot = LatestSnapshot(context)
        self.stop = context.Event()
        self.version = 0
        # //Compatible: Humble exposes declared parameters through this prefix API.
        parameters = [
            (name, parameter.value)
            for name, parameter in node.get_parameters_by_prefix("").items()
            if parameter.type_.value != 0
        ]
        remaps = [
            (topic, node.resolve_topic_name(topic))
            for topic in (
                str(node._motion_core.profile.wheel_topic),
                str(node._motion_core.profile.imu_topic),
                "/motion/scan",
                "/tf",
                "/tf_static",
            )
        ]
        self.process = context.Process(
            target=_receive,
            args=(parameters, remaps, self.slot, self.stop, os.getpid()),
            name="motion-reception",
        )
        self.process.start()
        self._closed = False

    def update(self, node, sensors):
        row = self.slot.take(self.version)
        if row is None:
            return
        self.version, snapshot = row
        state, scans, scan_ready, scan_generation, reference = snapshot
        sensors.apply_decision_snapshot(state, reference)
        node._scans.clear()
        node._scans.extend(scans)
        node._input_scan_ready = scan_ready
        node._scan_generation = scan_generation

    def close(self):
        if self._closed:
            return
        self._closed = True
        stop_process(self.process, self.stop)
