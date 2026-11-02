"""Bind delayed inference to bounded, original registered RGB-D history."""

import math
import uuid
from collections import OrderedDict
from dataclasses import dataclass

from senior_control.vision.depth_runtime import CameraInfoFrame
from senior_control.vision.detector import Detection
from senior_control.vision.semantic_runtime import (
    ColorCapture,
    DepthSnapshot,
    SampledGeometry,
    SemanticAssociation,
    associate_samples,
    qualify_registration,
)
from senior_control.vision.spawn_detector import (
    BoundDetectionResult,
    CaptureIdentity,
    RetainedFrame,
    SpawnInferenceWorker,
)


@dataclass(frozen=True)
class BoundSemanticCandidate:
    epoch: str
    registration_id: str
    rgb: ColorCapture
    info: CameraInfoFrame
    depth: DepthSnapshot
    detections: tuple[Detection, ...]
    associations: tuple[SemanticAssociation, ...]


class SemanticCoordinator:
    """Keep inference metadata in-process; invalidate it on source/epoch changes."""

    def __init__(
        self, worker: SpawnInferenceWorker, *, registered_profile: bool, max_age_s: float
    ) -> None:
        if not math.isfinite(max_age_s) or not 0 < max_age_s <= 0.3:
            raise ValueError("invalid_semantic_age")
        self._worker = worker
        self._registered = registered_profile
        self._max_age = max_age_s
        self.epoch = uuid.uuid4().hex
        self._lineage: tuple[str, ...] | None = None
        self._issued: OrderedDict[int, tuple[BoundSemanticCandidate, BoundDetectionResult]] = (
            OrderedDict()
        )
        self._frames: OrderedDict[int, tuple[RetainedFrame, ColorCapture, CameraInfoFrame,
                                              DepthSnapshot, str]] = OrderedDict()

    @property
    def status(self) -> tuple[str, str]:
        return self._worker.status

    def invalidate(self) -> None:
        self._worker.invalidate()
        self._frames.clear()
        self._issued.clear()
        self._lineage = None
        self.epoch = uuid.uuid4().hex

    def _registration(
        self, rgb: ColorCapture, info: CameraInfoFrame, depth: DepthSnapshot,
        *, now_ros_ns: int, now_steady: float,
    ) -> str:
        if (
            not rgb.raw_source or len(rgb.raw_source.encode("utf-8")) > 128
            or rgb.image.stamp != rgb.stamp_ns / 1e9
            or (rgb.image.width, rgb.image.height) != (depth.width, depth.height)
            or not 0 <= now_ros_ns - rgb.received_ros_ns <= self._max_age * 1e9
            or not math.isfinite(rgb.received_steady)
            or not 0 <= now_steady - rgb.received_steady <= self._max_age
        ):
            raise ValueError("rgb_capture_unqualified")
        return qualify_registration(
            info, depth, rgb_stamp_ns=rgb.stamp_ns, rgb_frame=rgb.frame_id,
            registered_profile=self._registered, now_ros_ns=now_ros_ns,
            now_steady=now_steady, max_age_s=self._max_age,
        )

    def submit(
        self, rgb: ColorCapture, info: CameraInfoFrame, depth: DepthSnapshot,
        *, now_ros_ns: int, now_steady: float,
    ) -> bool:
        try:
            registration = self._registration(rgb, info, depth, now_ros_ns=now_ros_ns,
                                              now_steady=now_steady)
        except (ValueError, UnicodeError):
            self.invalidate()
            return False
        lineage = (rgb.raw_source, info.raw_source, depth.source_epoch, depth.raw_source,
                   depth.info_source, depth.calibration_id, registration)
        if self._lineage is not None and self._lineage != lineage:
            self.invalidate()
        self._lineage = lineage
        identity = CaptureIdentity(
            source_epoch=self.epoch, rgb_source=rgb.raw_source, depth_source=depth.raw_source,
            rgb_frame=rgb.frame_id, depth_frame=depth.frame_id, registration_id=registration,
            rgb_stamp_ns=rgb.stamp_ns, depth_stamp_ns=depth.stamp_ns,
            rgb_received_ros_ns=rgb.received_ros_ns, depth_received_ros_ns=depth.received_ros_ns,
            rgb_received_steady=rgb.received_steady, depth_received_steady=depth.received_steady,
        )
        frame = RetainedFrame(identity, rgb.image,
                              SampledGeometry(depth.stamp_ns / 1e9, depth.samples))
        if not self._worker.submit(frame):
            return False
        self._frames[rgb.stamp_ns] = (frame, rgb, info, depth, registration)
        while len(self._frames) > 4:
            self._frames.popitem(last=False)
        return True

    def take_result(self, *, now_ros_ns: int, now_steady: float) -> BoundSemanticCandidate | None:
        result = self._worker.take_result()
        if result is None:
            return None
        retained = self._frames.pop(result.frame.identity.rgb_stamp_ns, None)
        if (retained is None or retained[0] is not result.frame
            or result.frame.identity.source_epoch != self.epoch
            or not self._worker.result_qualified(result, now_ros_ns=now_ros_ns,
                                                  now_steady=now_steady, max_age_s=self._max_age)):
            return None
        _, rgb, info, depth, registration = retained
        try:
            current = self._registration(rgb, info, depth, now_ros_ns=now_ros_ns,
                                         now_steady=now_steady)
            if current != registration or len(result.detections) > 32:
                return None
            associations = associate_samples(result.detections, depth.samples)
        except (ValueError, UnicodeError):
            return None
        candidate = BoundSemanticCandidate(self.epoch, registration, rgb, info, depth,
                                           result.detections, associations)
        self._issued[result.serial] = (candidate, result)
        while len(self._issued) > 4:
            self._issued.popitem(last=False)
        return candidate

    def is_current(
        self, candidate: BoundSemanticCandidate, *, now_ros_ns: int, now_steady: float
    ) -> bool:
        """Recheck both source generation and original receipts before publication."""
        issued = next((item for item in self._issued.values() if item[0] is candidate), None)
        if issued is None or candidate.epoch != self.epoch:
            return False
        if not self._worker.result_qualified(issued[1], now_ros_ns=now_ros_ns,
                                              now_steady=now_steady, max_age_s=self._max_age):
            return False
        try:
            return self._registration(candidate.rgb, candidate.info, candidate.depth,
                                      now_ros_ns=now_ros_ns, now_steady=now_steady) == (
                                          candidate.registration_id
                                      )
        except (ValueError, UnicodeError):
            return False

    def close(self) -> None:
        self._frames.clear()
        self._issued.clear()
        self._worker.close()
