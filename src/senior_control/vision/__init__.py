"""//Compatible: original vision exports load their one owner only on request."""

from importlib import import_module
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from senior_control.vision.depth import (
        CalibrationError,
        CameraCalibration,
        CameraExtrinsic,
        ColumnStatus,
        GeometryConfig,
        GeometryObservation,
        GeometryPoint,
        project_depth,
    )
    from senior_control.vision.detector import (
        Detection,
        DetectionBatch,
        Detector,
        DetectorDisabled,
        DisabledDetector,
        InferenceWorker,
        load_detector,
    )
    from senior_control.vision.synchronizer import (
        AlignedRgbdFrame,
        DepthFrame,
        FrameSyncError,
        RgbdSynchronizer,
        align,
    )
    from senior_control.vision.tracking import DynamicMask, Tracker, TrackingConfig, track
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
    "CalibrationError": "senior_control.vision.depth",
    "CameraCalibration": "senior_control.vision.depth",
    "CameraExtrinsic": "senior_control.vision.depth",
    "ColumnStatus": "senior_control.vision.depth",
    "GeometryConfig": "senior_control.vision.depth",
    "GeometryObservation": "senior_control.vision.depth",
    "GeometryPoint": "senior_control.vision.depth",
    "project_depth": "senior_control.vision.depth",
    "Detection": "senior_control.vision.detector",
    "DetectionBatch": "senior_control.vision.detector",
    "Detector": "senior_control.vision.detector",
    "DetectorDisabled": "senior_control.vision.detector",
    "DisabledDetector": "senior_control.vision.detector",
    "InferenceWorker": "senior_control.vision.detector",
    "load_detector": "senior_control.vision.detector",
    "AlignedRgbdFrame": "senior_control.vision.synchronizer",
    "DepthFrame": "senior_control.vision.synchronizer",
    "FrameSyncError": "senior_control.vision.synchronizer",
    "ImageFrame": "vehicle_core.image_frame",
    "RgbdSynchronizer": "senior_control.vision.synchronizer",
    "align": "senior_control.vision.synchronizer",
    "DynamicMask": "senior_control.vision.tracking",
    "Tracker": "senior_control.vision.tracking",
    "TrackingConfig": "senior_control.vision.tracking",
    "track": "senior_control.vision.tracking",
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
