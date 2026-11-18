"""One stateful CPU child; no ROS, actuator, filesystem or supervisor-loop work."""

import multiprocessing as mp
import os
from dataclasses import dataclass

from senior_control.config import DeviceProfile
from senior_control.models import Pose2D
from senior_control.motion.learning_record import LearningGeometry
from senior_control.motion.process_resources import LatestSnapshot, child_lifetime, stop_process
from senior_control.runtime.learning_wire import RecordedFrame
from senior_control.safety.self_filter import ScanObservation, filter_self
from senior_control.safety.self_learning import SelfReturnModel


@dataclass(frozen=True)
class ComputeBatch:
    attempt_id: str
    segment_id: str
    geometry: LearningGeometry
    frames: tuple[RecordedFrame, ...]
    finish: bool = False


@dataclass(frozen=True)
class ComputeResult:
    attempt_id: str
    processed: int
    finished: bool
    points: tuple[tuple[float, float], ...] | None = None
    error: str | None = None


class LearningKernel:
    """Serial original model, fed only the immutable motion-window raw frames."""

    def __init__(self) -> None:
        self.identity: tuple[str, str] | None = None
        self.model: SelfReturnModel | None = None
        self.geometry: LearningGeometry | None = None
        self.processed = 0
        self.profile = DeviceProfile()

    def update(self, batch: ComputeBatch) -> ComputeResult:
        identity = batch.attempt_id, batch.segment_id
        if identity != self.identity:
            self.identity, self.geometry = identity, batch.geometry
            self.model = SelfReturnModel(batch.geometry.physical_polygon)
            self.model.begin_segment(batch.segment_id)
            self.processed = 0
        if batch.geometry != self.geometry:
            raise ValueError("learning_geometry_changed")
        assert self.model is not None
        for frame in batch.frames:
            geometry, capture = frame.geometry, frame.capture
            if geometry != self.geometry:
                raise ValueError("learning_geometry_changed")
            ranges = tuple(float(value) for value in frame.ranges)
            observation = filter_self(
                ScanObservation(
                    ranges,
                    geometry.angle_min,
                    geometry.angle_increment,
                    geometry.range_min,
                    geometry.range_max,
                    capture.stamp_ns / 1e9,
                    "ros",
                    Pose2D(*geometry.sensor_to_body),
                    frame.transform_stamp,
                    geometry.body_frame,
                    capture.stamp_ns / 1e9,
                ),
                self.profile,
            )
            if not self.model.observe(observation, capture=capture, segment_id=batch.segment_id):
                raise ValueError("learning_frame_not_in_segment")
            self.processed += 1
        points = None
        if batch.finish:
            count = self.model.validate()
            no_interior = any(self.model.frames) and not any(
                self.model.interior(point) for frame in self.model.frames for point in frame
            )
            if self.processed and (count or no_interior):
                points = self.model.learned
        return ComputeResult(batch.attempt_id, self.processed, batch.finish, points)


def _compute_loop(requests, responses, stop, parent_pid):
    child_lifetime(parent_pid)
    kernel, version = LearningKernel(), 0
    while not stop.is_set() and os.getppid() == parent_pid:
        row = requests.take(version)
        if row is None:
            stop.wait(0.005)
            continue
        version, batch = row
        try:
            result = kernel.update(batch)
        except Exception as exc:
            result = ComputeResult(batch.attempt_id, kernel.processed, True, error=str(exc)[:256])
        while not stop.is_set() and not responses.put(result):
            stop.wait(0.001)


class LearningCompute:
    """One acknowledged flight of <=4 frames; the service owns its bounded FIFO."""

    def __init__(self) -> None:
        context = mp.get_context("spawn")
        self.requests: LatestSnapshot[ComputeBatch] = LatestSnapshot(context, 262144)
        self.responses: LatestSnapshot[ComputeResult] = LatestSnapshot(context, 262144)
        self.stop = context.Event()
        self.version = 0
        self.flight: ComputeBatch | None = None
        self._sent = False
        self.process = context.Process(
            target=_compute_loop,
            args=(self.requests, self.responses, self.stop, os.getpid()),
            name="learning-compute",
        )
        self.process.start()
        self._closed = False

    def submit(self, batch: ComputeBatch) -> None:
        if self.flight is not None or len(batch.frames) > 4:
            raise ValueError("learning_compute_busy")
        self.flight = batch
        self._sent = self.requests.put(batch)

    def poll(self) -> ComputeResult | None:
        if self.flight is None:
            return None
        if not self.process.is_alive():
            batch, self.flight = self.flight, None
            return ComputeResult(batch.attempt_id, 0, True, error="learning_compute_exited")
        if not self._sent:
            self._sent = self.requests.put(self.flight)
            return None
        row = self.responses.take(self.version)
        if row is None:
            return None
        self.version, result = row
        if result.attempt_id != self.flight.attempt_id:
            raise ValueError("learning_compute_identity")
        self.flight = None
        return result

    def close(self) -> None:
        if not self._closed:
            self._closed = True
            stop_process(self.process, self.stop)
