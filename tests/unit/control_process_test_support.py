"""Local resource adapters; all estimation/control/geometry stays production code.

Reports stop at a recording transport. Reception estimates in a separate sensor
owner, then crosses the real serialized mailbox and ControlInputs consumer.
These adapters do not emulate motion approval or restore core_reference ingress.
"""

import json
import multiprocessing as mp
from collections import deque
from concurrent.futures import Future
from pathlib import Path
from typing import TYPE_CHECKING, Any

from senior_robot.control_inputs import ControlInputs
from senior_robot.motion_sensor_runtime import MotionSensorRuntime

from senior_control.motion.process_resources import LatestSnapshot

if TYPE_CHECKING:
    from senior_control.motion.hardware_profile import HardwareMotionProfile


def generic_motion_profile(**overrides: Any) -> "HardwareMotionProfile":
    """Load explicit test numerics through the same profile model as deployment."""
    from senior_control.motion.hardware_profile import HardwareMotionProfile

    path = Path(__file__).resolve().parents[1] / "fixtures/generic_motion_profile.json"
    values = json.loads(path.read_text())
    values.update(overrides)
    return HardwareMotionProfile.model_validate(values)


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

    import senior_control.motion.process_resources as processes

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
        from senior_control.motion.observations import Frame, Source, SourceStatus
        from senior_control.motion.sample_types import PointObservation
        from senior_control.sensors.sample_time import SampleTime

        for row in packet.get("sources", []):
            sample = row["sample"]
            source = row["source"]
            if source not in {"wheel", "imu"} or "values" not in sample:
                continue
            values = sample["values"]
            values = (values[0], values[-1]) if source == "wheel" else (values[0],)
            raw = PointObservation(
                source=Source(source),
                frame=Frame.BODY,
                time=SampleTime(**sample["time"]),
                epoch=sample["epoch"],
                sequence=sample["sequence"],
                quality=1.0,
                status=SourceStatus.VALID,
                values=values,
                covariance=None,
                model_id=None,
                body_from_sensor=None,
            )
            self.sensors.ingest(
                raw,
                packet["sampled_ros"],
                packet["sampled_steady"],
                raw.time.received_ros_ns / 1e9,
            )
        # Condensed historical identity/geometry summaries are not raw samples.
        # Only explicit observations enter the strict current packet decoder.
        self.sensors.ingest_packet(
            {
                **packet,
                "sources": [
                    row
                    for row in packet.get("sources", [])
                    if row["source"] not in {"wheel", "imu"}
                    and ("values" in row["sample"] or "delta" in row["sample"])
                ],
            }
        )

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
