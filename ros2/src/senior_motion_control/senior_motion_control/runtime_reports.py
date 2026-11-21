"""Independent report publisher; reports never acknowledge individual intentions."""

import multiprocessing as mp
import os

from vehicle_core.process_resources import LatestSnapshot, child_lifetime, stop_process


def _publish(parameters, slot, status_slot, stop, parent_pid):
    child_lifetime(parent_pid)
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from std_msgs.msg import String

    rclpy.init(args=[])
    node = Node(
        "motion_reporter", parameter_overrides=[Parameter(n, value=v) for n, v in parameters]
    )
    publisher = node.create_publisher(String, "/motion/runtime", 1)
    status_publisher = node.create_publisher(DiagnosticArray, "/safety/status", 1)
    version, status_version, latest = 0, 0, None
    try:
        while rclpy.ok() and not stop.is_set() and os.getppid() == parent_pid:
            row = slot.take(version)
            if row is not None:
                version, latest = row
            if latest is not None:
                message = String()
                message.data = latest
                publisher.publish(message)
            status_row = status_slot.take(status_version)
            if status_row is not None:
                status_version, data = status_row
                status = DiagnosticStatus()
                status.name, status.hardware_id = "safety_gate", data["body"]
                status.message = data["reason"]
                status.level = (
                    DiagnosticStatus.OK
                    if data["reason"] in {"hard_sweep_clear", "warning_overlap"}
                    else DiagnosticStatus.WARN
                )
                status.values = [
                    KeyValue(key=key, value=str(value)) for key, value in data["values"].items()
                ]
                message = DiagnosticArray()
                message.header.stamp = node.get_clock().now().to_msg()
                message.status = [status]
                status_publisher.publish(message)
            stop.wait(0.05)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


class RuntimeReports:
    def __init__(self, node):
        context = mp.get_context("spawn")
        self.slot = LatestSnapshot(context, 128 * 1024)
        self.status_slot = LatestSnapshot(context, 32 * 1024)
        self.stop = context.Event()
        parameters = [("use_sim_time", bool(node.get_parameter("use_sim_time").value))]
        self.process = context.Process(
            target=_publish,
            args=(parameters, self.slot, self.status_slot, self.stop, os.getpid()),
            name="motion-reporting",
        )
        self.process.start()
        self._closed = False

    def publish(self, message):
        self.slot.put(message.data)

    def status(self, value):
        self.status_slot.put(value)

    def close(self):
        if self._closed:
            return
        self._closed = True
        stop_process(self.process, self.stop)
