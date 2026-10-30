"""RGB-D geometry and optional detector semantics. Imports no detector runtime."""

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
    ImageFrame,
    RgbdSynchronizer,
    align,
)
from senior_control.vision.tracking import DynamicMask, TrackingConfig, track

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
    "TrackingConfig",
    "align",
    "load_detector",
    "project_depth",
    "track",
]
