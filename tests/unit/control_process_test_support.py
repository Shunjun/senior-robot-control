"""Local resource adapters; all estimation/control/geometry stays production code.

Reports stop at a recording transport. Reception estimates in a separate sensor
owner, then crosses the real serialized mailbox and ControlInputs consumer.
These adapters do not emulate motion approval or restore core_reference ingress.
"""

import multiprocessing as mp
from collections import deque
from concurrent.futures import Future

from senior_robot.control_inputs import ControlInputs
from senior_robot.motion_sensor_runtime import MotionSensorRuntime

from senior_control.motion.process_resources import LatestSnapshot


class LocalWorker:
    executing = False
    """Deterministic execution resource for tests unrelated to process scheduling."""

    def submit(self, task):
        future = Future()
        LocalWorker.executing = True
        try:
            future.set_result(task())
        except Exception as exc:
            future.set_exception(exc)
        finally:
            LocalWorker.executing = False
        return future

    def close(self):
        pass


class RecordingReports:
    def __init__(self, node):
        self.publisher = node.create_publisher(object, "/motion/runtime", 1)
        self.statuses = []

    def publish(self, message):
        self.publisher.publish(message)

    def status(self, value):
        self.statuses.append(value)

    def close(self):
        pass


def local_resources(monkeypatch):
    import senior_robot.motion_core_output as output
    import senior_robot.runtime_reports as reports
    from source_time_test_support import DOMAIN

    import senior_control.motion.process_resources as processes

    monkeypatch.setattr("senior_control.sensors.time_proof.monotonic_domain", lambda: DOMAIN)

    monkeypatch.setattr(output, "ProcessWorker", lambda **_: LocalWorker())
    monkeypatch.setattr(processes, "ProcessWorker", lambda **_: LocalWorker())
    monkeypatch.setattr(reports, "RuntimeReports", RecordingReports)


class Reception:
    def __init__(self, core):
        self.core = core
        self.sensors = MotionSensorRuntime(core.profile, core.diagnostics)
        self.channel = ControlInputs.__new__(ControlInputs)
        self.channel.slot = LatestSnapshot(mp.get_context("spawn"))
        self.channel.version = 0
        self.core.node._scans = deque(getattr(core.node, "_scans", ()))

    def capture(self, packet):
        """Historic raw-source fixture enters only the reception-side offline adapter."""
        from dataclasses import replace

        from test_unified_motion_runtime import point

        from senior_control.motion.observations import Source
        from senior_control.sensors.time_proof import measurement_time

        for row in packet.get("sources", []):
            sample = row["sample"]
            source = row["source"]
            if source not in {"wheel", "imu"} or "values" not in sample:
                continue
            stamp = sample.get("stamp_ros", sample.get("end_ros"))
            values = sample["values"]
            values = (values[0], values[-1]) if source == "wheel" else (values[0],)
            raw = replace(
                point(
                    Source(source),
                    stamp,
                    sample["received_steady"],
                    values,
                    sample["sequence"],
                    measurement_steady=sample["measurement_steady"],
                ),
                epoch=sample["epoch"],
                measurement=measurement_time(sample["measurement"])
                if sample.get("measurement") is not None
                else None,
            )
            self.sensors.ingest(
                raw,
                packet["sampled_ros"],
                packet["sampled_steady"],
                sample.get("received_ros", packet["sampled_ros"]),
            )
        self.sensors.ingest_packet(
            {
                **packet,
                "sources": [
                    row
                    for row in packet.get("sources", [])
                    if row["source"] not in {"wheel", "imu"}
                ],
            }
        )

        # //Compatible: historic test captures retained pose/time but omitted the native capture
        # sequence and receipt ROS clock. Deterministic timestamp identities
        # and the later captured snapshot clock conservatively reconstruct the
        # offline test boundary; neither advances a receipt or invents a pose.
        self.sensors.poses = [
            {
                **row,
                "sequence": row.get("sequence", round(row["stamp_ros"] * 1e9)),
                "received_ros": row.get("received_ros", packet["sampled_ros"]),
            }
            for row in self.sensors.poses
        ]

    def publish(self, ros, steady):
        self.sensors.ingest_poses(self.sensors.poses)
        reference = self.sensors.estimate(ros, steady)
        state = {
            k: v for k, v in self.sensors.__dict__.items() if k not in {"profile", "diagnostics"}
        }
        assert self.channel.slot.put((state, tuple(self.core.node._scans), True, 1, reference))
        self.channel.update(self.core.node, self.core.sensors)
        return reference

    def close(self):
        # Release this test-only mailbox deterministically. No reception process
        # exists here; ProcessWorker lifecycle is exercised in its real-spawn tests.
        if self.channel is not None:
            self.channel.slot = None
        self.channel = self.sensors = self.core = None


def make_core(monkeypatch, node, profile):
    from senior_robot.motion_core_output import MotionCoreOutput

    local_resources(monkeypatch)
    core = MotionCoreOutput(node, profile)
    receiver = Reception(core)
    core._resources.callback(receiver.close)
    node._reception_test = receiver
    receiver.publish(node.get_clock().now().nanoseconds / 1e9, 10.0)
    return core


def publish_inputs(core, now=None, ros=None):
    receiver = core.node._reception_test
    ros = core.node.get_clock().now().nanoseconds / 1e9 if ros is None else ros
    if now is None:
        import senior_robot.motion_core_output as output

        now = output.time.monotonic()
    return receiver.publish(ros, now)


def capture_inputs(core, packet):
    import json

    packet = json.loads(packet.data) if hasattr(packet, "data") else packet
    core.node._reception_test.capture(packet)
    publish_inputs(core)


def read_inputs(core, now):
    publish_inputs(core, now)
    return core._inputs(now)


def read_observation(core, now):
    if core.node._scans:
        core._sync_learning_model(core.node._scans[-1].message)
    core.prefetch_scan()
    try:
        return core._observation(now)
    except ValueError as exc:
        if str(exc) != "scan_geometry_pending":
            raise
        # LocalWorker completed the real submitted geometry. Consume exactly
        # once; no wait/retry loop or synthetic approval is supplied.
        assert core.scan_geometry.flight.done()
        return core._observation(now)
