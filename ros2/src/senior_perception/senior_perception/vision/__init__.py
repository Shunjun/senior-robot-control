"""//Compatible: original vision exports load their one owner only on request."""

from importlib import import_module
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from senior_perception.vision.depth import (
        CalibrationError,
        CameraCalibration,
        CameraExtrinsic,
        ColumnStatus,
        GeometryConfig,
        GeometryObservation,
        GeometryPoint,
        project_depth,
    )
    from senior_perception.vision.detector import (
        Detection,
        DetectionBatch,
        Detector,
        DetectorDisabled,
        DisabledDetector,
        InferenceWorker,
        load_detector,
    )
    from senior_perception.vision.synchronizer import (
        AlignedRgbdFrame,
        DepthFrame,
        FrameSyncError,
        RgbdSynchronizer,
        align,
    )
    from senior_perception.vision.tracking import DynamicMask, Tracker, TrackingConfig, track
    from vehicle_core.image_frame import ImageFrame as ImageFrame

__all__ = [
    "AlignedRgbdFrame",
    "CalibrationError",
    "CameraCalibration",
    "CameraExtrinsic",
    "ColumnStatus",
    "DepthFrame",
    "Detection",
    "DetectionBatch",
    "Detector",
    "DetectorDisabled",
    "DisabledDetector",
    "DynamicMask",
    "FrameSyncError",
    "GeometryConfig",
    "GeometryObservation",
    "GeometryPoint",
    "ImageFrame",
    "InferenceWorker",
    "RgbdSynchronizer",
    "Tracker",
    "TrackingConfig",
    "align",
    "load_detector",
    "project_depth",
    "track",
]

_EXPORTS = {
    "CalibrationError": "senior_perception.vision.depth",
    "CameraCalibration": "senior_perception.vision.depth",
    "CameraExtrinsic": "senior_perception.vision.depth",
    "ColumnStatus": "senior_perception.vision.depth",
    "GeometryConfig": "senior_perception.vision.depth",
    "GeometryObservation": "senior_perception.vision.depth",
    "GeometryPoint": "senior_perception.vision.depth",
    "project_depth": "senior_perception.vision.depth",
    "Detection": "senior_perception.vision.detector",
    "DetectionBatch": "senior_perception.vision.detector",
    "Detector": "senior_perception.vision.detector",
    "DetectorDisabled": "senior_perception.vision.detector",
    "DisabledDetector": "senior_perception.vision.detector",
    "InferenceWorker": "senior_perception.vision.detector",
    "load_detector": "senior_perception.vision.detector",
    "AlignedRgbdFrame": "senior_perception.vision.synchronizer",
    "DepthFrame": "senior_perception.vision.synchronizer",
    "FrameSyncError": "senior_perception.vision.synchronizer",
    "ImageFrame": "vehicle_core.image_frame",
    "RgbdSynchronizer": "senior_perception.vision.synchronizer",
    "align": "senior_perception.vision.synchronizer",
    "DynamicMask": "senior_perception.vision.tracking",
    "Tracker": "senior_perception.vision.tracking",
    "TrackingConfig": "senior_perception.vision.tracking",
    "track": "senior_perception.vision.tracking",
}
_MODULES = ("depth", "detector", "synchronizer", "tracking")


def __getattr__(name: str) -> Any:
    if name in _EXPORTS:
        value = getattr(import_module(_EXPORTS[name]), name)
    elif name in _MODULES:
        value = import_module(f"{__name__}.{name}")
    else:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    globals()[name] = value
    return value


def __dir__() -> list[str]:
    return sorted(set(globals()) | set(__all__) | set(_MODULES))
