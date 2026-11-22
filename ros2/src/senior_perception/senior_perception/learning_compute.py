"""One five-frame CPU task; no streaming model or per-batch acknowledgements."""

from concurrent.futures import Future
from dataclasses import dataclass
from functools import partial

from senior_perception.learning_record import LearningGeometry
from senior_perception.learning_wire import RecordedFrame
from senior_perception.self_filter import FilteredObservation, ScanObservation, filter_self
from senior_perception.self_learning import LEARNING_SAMPLE_COUNT, SelfReturnModel
from vehicle_core.config import DeviceProfile
from vehicle_core.process_resources import ProcessWorker
from vehicle_core.values import Pose2D


@dataclass(frozen=True)
class ComputeRequest:
    attempt_id: str
    geometry: LearningGeometry
    frames: tuple[RecordedFrame, ...]


@dataclass(frozen=True)
class ComputeResult:
    attempt_id: str
    points: tuple[tuple[float, float], ...] | None = None
    error: str | None = None


def decode_observation(frame: RecordedFrame) -> FilteredObservation:
    geometry, capture = frame.geometry, frame.capture
    return filter_self(
        ScanObservation(
            tuple(float(value) for value in frame.ranges),
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
        DeviceProfile(),
    )


def compute_samples(request: ComputeRequest) -> ComputeResult:
    if len(request.frames) != LEARNING_SAMPLE_COUNT:
        raise ValueError("learning_sample_count")
    captures = [frame.capture for frame in request.frames]
    origins = {
        (c.bridge_boot_id, c.source_epoch, c.raw_gid, c.capture_generation, c.time.clock_generation)
        for c in captures
    }
    if len(origins) != 1 or any(
        b.sequence <= a.sequence
        or b.stamp_ns <= a.stamp_ns
        or b.time.sample_steady_ns < a.time.sample_steady_ns
        for a, b in zip(captures, captures[1:], strict=False)
    ):
        raise ValueError("learning_sample_identity")
    if any(not request.geometry.same_structure(frame.geometry) for frame in request.frames):
        raise ValueError("learning_geometry_changed")
    observations = tuple(decode_observation(frame) for frame in request.frames)
    model = SelfReturnModel(request.geometry.physical_polygon)
    count = model.learn_samples(observations)
    hits = tuple(ray for observation in observations for ray in observation.external_hits)
    no_interior = bool(hits) and not any(
        model.interior(ray.endpoint) for ray in hits if ray.endpoint is not None
    )
    return ComputeResult(request.attempt_id, model.learned if count or no_interior else None)


class LearningCompute:
    """Lazy dedicated CPU resource, one whole request and one result per attempt."""

    def __init__(self) -> None:
        self.flight: ComputeRequest | None = None
        self._worker: ProcessWorker[ComputeResult] | None = None
        self._future: Future[ComputeResult] | None = None

    def submit(self, request: ComputeRequest) -> None:
        if self.flight is not None:
            raise ValueError("learning_compute_busy")
        if self._worker is None:
            self._worker = ProcessWorker(name="learning-compute")
        self._future = self._worker.submit(partial(compute_samples, request))
        self.flight = request

    def poll(self) -> ComputeResult | None:
        if self._future is None or not self._future.done():
            return None
        assert self.flight is not None
        request, future = self.flight, self._future
        self.flight = self._future = None
        try:
            result = future.result()
        except Exception as exc:
            return ComputeResult(request.attempt_id, error=str(exc)[:256])
        if result.attempt_id != request.attempt_id:
            raise ValueError("learning_compute_identity")
        return result

    def close(self) -> None:
        if self._worker is not None:
            self._worker.close()
            self._worker = None
        self.flight = self._future = None
