"""Optional spawned inference bound to original registered RGB-D captures.

Only the RGB image crosses into the child. Original identity and sampled depth
stay in a bounded parent cache; a late result can attach only to its own cache
entry, never the latest frame. No result qualifies static space or laser masks.
"""

import math
import multiprocessing
import queue
import threading
import time
from collections import OrderedDict, deque
from collections.abc import Callable
from dataclasses import dataclass
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path
from typing import Any, Protocol

from senior_control.vision.depth import GeometrySample
from senior_control.vision.detector import Detection, _ultralytics_runtime
from senior_control.vision.image_codec import MAX_IMAGE_BYTES, MAX_IMAGE_PIXELS
from senior_control.vision.synchronizer import ImageFrame

DetectionValue = tuple[str, float, tuple[float, float, float, float]]
Processor = Callable[[ImageFrame], list[DetectionValue]]
SUPPORTED_ULTRALYTICS_VERSION = "8.3.40"


@dataclass(frozen=True)
class CaptureIdentity:
    source_epoch: str
    rgb_source: str
    depth_source: str
    rgb_frame: str
    depth_frame: str
    registration_id: str
    rgb_stamp_ns: int
    depth_stamp_ns: int
    rgb_received_ros_ns: int
    depth_received_ros_ns: int
    rgb_received_steady: float
    depth_received_steady: float


class SampleGeometry(Protocol):
    @property
    def stamp(self) -> float: ...

    @property
    def samples(self) -> tuple[GeometrySample, ...]: ...


@dataclass(frozen=True)
class RetainedFrame:
    identity: CaptureIdentity
    rgb: ImageFrame
    geometry: SampleGeometry


@dataclass(frozen=True)
class BoundDetectionResult:
    frame: RetainedFrame
    detections: tuple[Detection, ...]
    generation: int
    serial: int


def _bounded(value: str, limit: int = 128) -> bool:
    try:
        return bool(value) and len(value.encode("utf-8")) <= limit
    except UnicodeError:
        return False


def _qualified(frame: RetainedFrame, max_skew_ns: int) -> bool:
    identity, rgb, geometry = frame.identity, frame.rgb, frame.geometry
    return (
        _bounded(identity.source_epoch, 64)
        and all(
            _bounded(value)
            for value in (
                identity.rgb_source,
                identity.depth_source,
                identity.rgb_frame,
                identity.depth_frame,
                identity.registration_id,
            )
        )
        and identity.rgb_frame == identity.depth_frame
        and min(
            identity.rgb_stamp_ns,
            identity.depth_stamp_ns,
            identity.rgb_received_ros_ns,
            identity.depth_received_ros_ns,
        )
        > 0
        and math.isfinite(identity.rgb_received_steady)
        and math.isfinite(identity.depth_received_steady)
        and abs(identity.rgb_stamp_ns - identity.depth_stamp_ns) <= max_skew_ns
        and rgb.stamp == identity.rgb_stamp_ns / 1e9
        and geometry.stamp == identity.depth_stamp_ns / 1e9
        and rgb.encoding in {"rgb8", "bgr8", "mono8"}
        and 0 < rgb.width <= 4096
        and 0 < rgb.height <= 4096
        and rgb.width * rgb.height <= MAX_IMAGE_PIXELS
        and len(rgb.data) <= MAX_IMAGE_BYTES
        and len(geometry.samples) <= 8192
        and all(
            0 <= sample.u < rgb.width and 0 <= sample.v < rgb.height for sample in geometry.samples
        )
    )


def _valid_detections(raw: Any, frame: ImageFrame) -> tuple[Detection, ...]:
    if not isinstance(raw, (list, tuple)) or len(raw) > 256:
        raise ValueError("detector_result_shape")
    detections = []
    for item in raw:
        label, confidence, box = item
        if (
            not isinstance(label, str)
            or not _bounded(label, 64)
            or not isinstance(box, (list, tuple))
            or len(box) != 4
        ):
            raise ValueError("detector_result_shape")
        confidence = float(confidence)
        coords = (float(box[0]), float(box[1]), float(box[2]), float(box[3]))
        if (
            not 0 <= confidence <= 1
            or not all(math.isfinite(value) for value in coords)
            or not 0 <= coords[0] <= coords[2] <= frame.width
            or not 0 <= coords[1] <= coords[3] <= frame.height
        ):
            raise ValueError("detector_result_values")
        detections.append(Detection(label, confidence, coords, frame.stamp))
    return tuple(detections)


def _child_main(
    jobs: Any, replies: Any, weights: str, device: str, processor: Processor | None
) -> None:
    try:
        if processor is None:
            if not Path(weights).is_file():
                replies.put(("disabled", "weights_missing"))
                return
            try:
                installed = version("ultralytics")
            except PackageNotFoundError:
                replies.put(("disabled", "runtime_missing"))
                return
            if installed != SUPPORTED_ULTRALYTICS_VERSION:
                replies.put(("disabled", "runtime_version_unsupported"))
                return
            try:
                infer = _ultralytics_runtime(Path(weights), device=device)
            except ImportError:
                replies.put(("disabled", "runtime_missing"))
                return
            except Exception:
                replies.put(("disabled", "weights_unloadable"))
                return

            def runtime(frame: ImageFrame) -> list[DetectionValue]:
                return [(item.label, item.confidence, item.box) for item in infer(frame)]

            run: Processor = runtime
        else:
            run = processor
        while True:
            item = jobs.get()
            if item is None:
                return
            generation, serial, frame = item
            try:
                result = _valid_detections(run(frame), frame)
                replies.put(("result", generation, serial, result))
            except Exception:
                replies.put(("error", generation, serial, "inference_failed"))
    except KeyboardInterrupt:
        return


class SpawnInferenceWorker:
    """One spawned detector, one latest pending frame, bounded retained history."""

    def __init__(
        self,
        *,
        weights_path: Path | None,
        device: str = "cpu",
        timeout_s: float = 5.0,
        retained_capacity: int = 4,
        max_skew_s: float = 0.03,
        max_result_age_s: float = 0.3,
        processor: Processor | None = None,
    ) -> None:
        if (
            not math.isfinite(timeout_s)
            or not 0.05 <= timeout_s <= 30
            or not 1 <= retained_capacity <= 8
            or not math.isfinite(max_skew_s)
            or not 0 <= max_skew_s <= 0.2
            or not math.isfinite(max_result_age_s)
            or not 0 < max_result_age_s <= 0.3
            or not _bounded(device, 32)
        ):
            raise ValueError("invalid inference limits")
        self._weights = weights_path
        self._device = device
        self._timeout = timeout_s
        self._capacity = retained_capacity
        self._max_skew_ns = int(max_skew_s * 1e9)
        self._max_result_age_s = max_result_age_s
        self._processor = processor
        self._context = multiprocessing.get_context("spawn")
        self._condition = threading.Condition()
        self._closed = False
        self._supervisor: threading.Thread | None = None
        self._process: Any = None
        self._pending: tuple[int, int, ImageFrame] | None = None
        self._busy = False
        self._generation = 0
        self._serial = 0
        self._active: str | None = None
        self._retired: set[str] = set()
        self._identity: tuple[str, ...] | None = None
        self._identity_failed = False
        self._last_rgb_ns = self._last_depth_ns = 0
        self._retained: OrderedDict[int, RetainedFrame] = OrderedDict()
        self._results: deque[BoundDetectionResult] = deque(maxlen=retained_capacity)
        self._issued: OrderedDict[int, BoundDetectionResult] = OrderedDict()
        self.dropped = 0
        self.failures = 0
        if weights_path is None:
            self._status = ("disabled", "weights_not_configured")
        elif not weights_path.is_file():
            self._status = ("disabled", "weights_missing")
        else:
            self._status = ("starting", "runtime_pending")

    @property
    def status(self) -> tuple[str, str]:
        with self._condition:
            return self._status

    @property
    def in_flight(self) -> bool:
        with self._condition:
            return self._busy

    @property
    def child_alive(self) -> bool:
        process = self._process
        return bool(process is not None and process.pid is not None and process.is_alive())

    def _retire_locked(self) -> None:
        if self._active is not None:
            self._retired.add(self._active)
        self._active = None
        self._identity = None
        self._identity_failed = False
        self._last_rgb_ns = self._last_depth_ns = 0
        self._pending = None
        self._retained.clear()
        self._results.clear()
        self._issued.clear()
        self._generation += 1

    def invalidate(self) -> None:
        with self._condition:
            self._retire_locked()
            self._condition.notify_all()

    def submit(self, frame: RetainedFrame) -> bool:
        if not _qualified(frame, self._max_skew_ns):
            with self._condition:
                if frame.identity.source_epoch == self._active:
                    self._retire_locked()
            return False
        identity = frame.identity
        lineage = (
            identity.rgb_source,
            identity.depth_source,
            identity.rgb_frame,
            identity.depth_frame,
            identity.registration_id,
        )
        with self._condition:
            if (
                self._closed
                or self._status[0] == "disabled"
                or self._status == ("failed", "worker_supervisor_failed")
                or identity.source_epoch in self._retired
            ):
                return False
            if self._active != identity.source_epoch:
                if len(self._retired) >= 256:
                    self._status = ("disabled", "epoch_capacity")
                    self._retire_locked()
                    return False
                self._retire_locked()
                self._active = identity.source_epoch
            if self._identity is not None and self._identity != lineage:
                self._identity_failed = True
                self._retained.clear()
                self._results.clear()
                self._issued.clear()
                self._pending = None
                self._generation += 1
                return False
            if (
                self._identity_failed
                or identity.rgb_stamp_ns <= self._last_rgb_ns
                or identity.depth_stamp_ns <= self._last_depth_ns
            ):
                self._retire_locked()
                return False
            self._identity = lineage
            self._last_rgb_ns, self._last_depth_ns = identity.rgb_stamp_ns, identity.depth_stamp_ns
            self._serial += 1
            serial = self._serial
            if self._pending is not None:
                self._retained.pop(self._pending[1], None)
                self.dropped += 1
            self._retained[serial] = frame
            while len(self._retained) > self._capacity:
                self._retained.popitem(last=False)
            self._pending = (self._generation, serial, frame.rgb)
            if self._supervisor is None:
                self._supervisor = threading.Thread(target=self._supervise, daemon=True)
                self._supervisor.start()
            self._condition.notify_all()
            return True

    def take_result(self) -> BoundDetectionResult | None:
        with self._condition:
            return self._results.popleft() if self._results else None

    def result_qualified(
        self,
        result: BoundDetectionResult,
        *,
        now_ros_ns: int,
        now_steady: float,
        max_age_s: float,
    ) -> bool:
        """Recheck original clocks and generation immediately before association."""
        if not math.isfinite(now_steady) or not math.isfinite(max_age_s) or max_age_s <= 0:
            return False
        max_age_s = min(max_age_s, self._max_result_age_s)
        capture = result.frame.identity
        with self._condition:
            return (
                not self._closed
                and self._status[0] == "ready"
                and result.generation == self._generation
                and self._issued.get(result.serial) is result
                and capture.source_epoch == self._active
                and not self._identity_failed
                and all(
                    0 <= now_ros_ns - stamp <= max_age_s * 1e9
                    for stamp in (
                        capture.rgb_stamp_ns,
                        capture.depth_stamp_ns,
                        capture.rgb_received_ros_ns,
                        capture.depth_received_ros_ns,
                    )
                )
                and all(
                    0 <= now_steady - stamp <= max_age_s
                    for stamp in (capture.rgb_received_steady, capture.depth_received_steady)
                )
            )

    def _supervise(self) -> None:
        jobs = replies = process = None
        try:
            while True:
                with self._condition:
                    self._condition.wait_for(lambda: self._closed or self._pending is not None)
                    if self._closed:
                        break
                    job = self._pending
                    self._pending = None
                    self._busy = True
                assert job is not None
                if process is None or not process.is_alive():
                    if jobs is not None:
                        jobs.cancel_join_thread()
                        jobs.close()
                    if replies is not None:
                        replies.cancel_join_thread()
                        replies.close()
                    jobs = self._context.Queue(maxsize=1)
                    replies = self._context.Queue(maxsize=1)
                    process = self._context.Process(
                        target=_child_main,
                        args=(jobs, replies, str(self._weights), self._device, self._processor),
                        daemon=True,
                    )
                    self._process = process
                    process.start()
                assert jobs is not None and replies is not None
                jobs.put(job, timeout=0.1)
                deadline = time.monotonic() + self._timeout
                response = None
                while time.monotonic() < deadline:
                    with self._condition:
                        if self._closed:
                            break
                    try:
                        remaining = max(0.001, deadline - time.monotonic())
                        response = replies.get(timeout=min(0.05, remaining))
                        break
                    except queue.Empty:
                        if not process.is_alive():
                            break
                with self._condition:
                    self._busy = False
                    if self._closed:
                        break
                    if response is None:
                        self.failures += 1
                        self._status = ("failed", "timeout_or_worker_exit")
                        self._results.clear()
                        self._retained.pop(job[1], None)
                        if job[0] == self._generation:
                            self._generation += 1
                            self._pending = None
                            self._retained.clear()
                            self._issued.clear()
                    elif response[0] == "disabled":
                        self._status = ("disabled", response[1])
                        self._pending = None
                        self._results.clear()
                        self._retained.clear()
                        self._issued.clear()
                    elif response[0] == "error":
                        self.failures += 1
                        self._status = ("failed", response[3])
                        self._results.clear()
                        self._retained.pop(response[2], None)
                        if response[1] == self._generation:
                            self._generation += 1
                            self._pending = None
                            self._retained.clear()
                            self._issued.clear()
                    elif response[0] == "result":
                        _, generation, serial, detections = response
                        original = self._retained.pop(serial, None)
                        if generation == self._generation and original is not None:
                            bound = BoundDetectionResult(original, detections, generation, serial)
                            self._results.append(bound)
                            self._issued[serial] = bound
                            while len(self._issued) > self._capacity:
                                self._issued.popitem(last=False)
                            self._status = ("ready", "inference_ok")
                    self._condition.notify_all()
                if response is None:
                    if process.is_alive():
                        process.terminate()
                        process.join(timeout=0.5)
                    if process.is_alive():
                        process.kill()
                        process.join(timeout=0.5)
                    process = None
        except Exception:
            with self._condition:
                self.failures += 1
                self._status = ("failed", "worker_supervisor_failed")
                self._results.clear()
                self._issued.clear()
                self._pending = None
                self._busy = False
                self._condition.notify_all()
        finally:
            if process is not None and process.pid is not None:
                if process.is_alive():
                    process.terminate()
                    process.join(timeout=0.5)
                if process.is_alive():
                    process.kill()
                    process.join(timeout=0.5)
            for channel in (jobs, replies):
                if channel is not None:
                    channel.cancel_join_thread()
                    channel.close()

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._pending = None
            self._retained.clear()
            self._results.clear()
            self._issued.clear()
            self._condition.notify_all()
            supervisor = self._supervisor
        process = self._process
        if process is not None and process.pid is not None and process.is_alive():
            process.terminate()
        if supervisor is not None:
            supervisor.join(timeout=1.5)
