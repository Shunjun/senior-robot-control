"""RGB-D geometry, synchronization, and optional detector semantics. No ROS, no camera."""

import importlib.util
import math
import subprocess
import sys
import threading
from collections.abc import Callable
from pathlib import Path

import pytest

from senior_control.vision import (
    AlignedRgbdFrame,
    CalibrationError,
    CameraCalibration,
    CameraExtrinsic,
    DepthFrame,
    Detection,
    DetectorDisabled,
    FrameSyncError,
    GeometryConfig,
    ImageFrame,
    InferenceWorker,
    RgbdSynchronizer,
    TrackingConfig,
    align,
    load_detector,
    project_depth,
    track,
)

ROOT = Path(__file__).resolve().parents[2]
WIDTH = 9
HEIGHT = 5
CONFIG = GeometryConfig(column_stride=1, row_stride=1)


def calibration(**overrides: object) -> CameraCalibration:
    values: dict[str, object] = {
        "width": WIDTH,
        "height": HEIGHT,
        "fx": 4.0,
        "fy": 4.0,
        "cx": 4.0,
        "cy": 2.0,
        "depth_unit_m": 0.001,
        "min_range_m": 0.2,
        "max_range_m": 8.0,
        "extrinsic": CameraExtrinsic(x=0.0, y=0.0, z=0.3, roll=0.0, pitch=0.0, yaw=0.0),
    }
    values.update(overrides)
    return CameraCalibration(**values)  # type: ignore[arg-type]


def rgb(stamp: float, width: int = WIDTH, height: int = HEIGHT) -> ImageFrame:
    return ImageFrame(stamp, width, height, "rgb8", bytes(width * height * 3))


def depth(
    stamp: float,
    pixel: Callable[[int, int], float] = lambda u, v: 3000.0,
    width: int = WIDTH,
    height: int = HEIGHT,
) -> DepthFrame:
    values = tuple(pixel(u, v) for v in range(height) for u in range(width))
    return DepthFrame(stamp, width, height, values)


def aligned(frame: DepthFrame, stamp: float | None = None) -> AlignedRgbdFrame:
    return align(rgb(frame.stamp if stamp is None else stamp), frame, max_skew_s=0.05)


def person_scene(u: int, v: int) -> float:
    return 1500.0 if 3 <= u <= 5 else 3000.0


def person_box(stamp: float) -> Detection:
    return Detection("person", 0.9, (2.0, 0.0, 6.0, 4.0), stamp)


def test_rgb_depth_time_mismatch_rejected() -> None:
    with pytest.raises(FrameSyncError, match="skew"):
        align(rgb(1.0), depth(1.2), max_skew_s=0.05)
    with pytest.raises(FrameSyncError, match="size"):
        align(rgb(1.0, width=4), depth(1.0), max_skew_s=0.05)
    with pytest.raises(FrameSyncError, match="stamp"):
        align(rgb(math.nan), depth(1.0), max_skew_s=0.05)
    pair = align(rgb(1.0), depth(1.02), max_skew_s=0.05)
    assert pair.skew_s == pytest.approx(0.02)

    sync = RgbdSynchronizer(max_skew_s=0.05, capacity=2)
    assert sync.offer_rgb(rgb(1.0)) is None
    assert sync.offer_depth(depth(1.2)) is None
    assert sync.rejected == 1
    matched = sync.offer_depth(depth(1.01))
    assert matched is not None and matched.rgb.stamp == 1.0


def test_invalid_depth_not_free() -> None:
    def pixel(u: int, v: int) -> float:
        return {
            1: 0.0,
            2: math.nan,
            3: math.inf,
            4: -5.0,
            5: 99000.0,
            6: 100.0,
        }.get(u, 3000.0)

    frame = aligned(depth(1.0, pixel))
    observation = project_depth(frame, calibration(), CONFIG)

    assert len(observation.rays) == WIDTH
    for column in range(1, 7):
        ray = observation.rays[column]
        assert not ray.valid
        assert not ray.no_return_free
        assert ray.clear_limit_m is None
        assert observation.status[column] == "invalid"
        assert observation.occlusion_mask[column]
    for column in (0, 7, 8):
        assert observation.status[column] == "hit"
        assert observation.rays[column].valid
        assert not observation.occlusion_mask[column]
    assert not any(ray.no_return_free for ray in observation.rays)
    assert observation.quality == pytest.approx(3 / 9)


def test_hit_geometry_uses_extrinsics_and_depth_unit() -> None:
    observation = project_depth(aligned(depth(1.0)), calibration(), CONFIG)
    center = observation.rays[4]
    assert center.valid and center.range_m == pytest.approx(3.0)
    assert center.angle == pytest.approx(0.0)
    assert observation.rays[0].angle == pytest.approx(math.atan2(1.0, 1.0))
    points = {point.ray_index: point for point in observation.points}
    # Rows 3 and 4 sit below the 0.05 m floor band and never become hits.
    assert points[4].x == pytest.approx(3.0)
    assert 0.05 <= points[4].z <= 1.5

    tilted = calibration(
        extrinsic=CameraExtrinsic(x=0.0, y=0.0, z=0.3, roll=0.0, pitch=0.0, yaw=math.pi / 2)
    )
    turned = project_depth(aligned(depth(1.0)), tilted, CONFIG)
    assert turned.rays[4].angle == pytest.approx(math.pi / 2)


def test_uncalibrated_camera_rejected() -> None:
    with pytest.raises(CalibrationError):
        calibration(fx=0.0)
    with pytest.raises(CalibrationError):
        calibration(depth_unit_m=0.0)
    with pytest.raises(CalibrationError):
        calibration(max_range_m=0.1)
    with pytest.raises(CalibrationError, match="missing"):
        CameraCalibration.from_mapping({"width": 9, "height": 5})
    with pytest.raises(CalibrationError, match="size"):
        project_depth(aligned(depth(1.0)), calibration(width=16), CONFIG)


def test_missing_weights_explicit_disabled(tmp_path: Path) -> None:
    missing = load_detector(tmp_path / "absent.pt")
    assert missing.enabled is False
    assert missing.reason == "weights_missing"
    with pytest.raises(DetectorDisabled):
        missing.infer(rgb(1.0))
    assert not (tmp_path / "absent.pt").exists()

    unset = load_detector(None)
    assert unset.enabled is False
    assert unset.reason == "weights_not_configured"

    weights = tmp_path / "present.pt"
    weights.write_bytes(b"not a real model")
    no_runtime = load_detector(weights, runtime_loader=_missing_runtime)
    assert no_runtime.enabled is False
    assert no_runtime.reason == "runtime_missing"

    geometry = project_depth(aligned(depth(1.0, person_scene)), calibration(), CONFIG)
    mask = track([], geometry)
    assert mask.semantics_available is False
    assert not any(mask.dynamic)
    assert set(mask.kinds) == {"unknown_static"}


def _missing_runtime(_path: Path) -> Callable[[ImageFrame], list[Detection]]:
    raise ImportError("no detector runtime")


def test_vision_import_loads_no_detector_runtime() -> None:
    code = (
        "import sys, senior_control.vision;"
        "bad = [n for n in ('torch', 'ultralytics', 'onnxruntime') if n in sys.modules];"
        "sys.exit(1 if bad else 0)"
    )
    done = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
    assert done.returncode == 0, done.stderr
    text = (ROOT / "pyproject.toml").read_text()
    dependencies = text.split("dependencies = [", 1)[1].split("]", 1)[0]
    assert "torch" not in dependencies and "ultralytics" not in dependencies


def test_person_mask_does_not_erase_occluded_wall() -> None:
    geometry = project_depth(aligned(depth(1.0, person_scene)), calibration(), CONFIG)
    mask = track([person_box(1.0)], geometry)

    assert mask.semantics_available is True
    assert [i for i, flag in enumerate(mask.dynamic) if flag] == [3, 4, 5]
    assert all(mask.kinds[i] == "person" for i in (3, 4, 5))
    # The wall remains geometry but a detector miss does not confirm it static.
    for column in (2, 6):
        assert not mask.dynamic[column]
        assert mask.kinds[column] == "unknown_static"
        assert geometry.status[column] == "hit"
        assert geometry.rays[column].valid
        assert geometry.rays[column].range_m > 3.0
    # Whatever the person hides is unknown, never free.
    assert all(mask.occluded[i] for i in (3, 4, 5))
    assert not any(mask.occluded[i] for i in (0, 1, 2, 6, 7, 8))
    # The mask does not rewrite geometry: the person still blocks safety.
    assert all(geometry.status[i] == "hit" for i in range(WIDTH))
    assert len(mask.dynamic) == len(geometry.rays) == len(mask.occluded) == len(mask.kinds)


def test_stale_low_confidence_and_other_labels_do_not_mask() -> None:
    geometry = project_depth(aligned(depth(5.0, person_scene)), calibration(), CONFIG)
    config = TrackingConfig(max_detection_age_s=0.3, min_confidence=0.5)

    stale = track([person_box(1.0)], geometry, config=config)
    assert stale.semantics_available is False
    assert not any(stale.dynamic)

    weak = track([Detection("person", 0.2, (2.0, 0.0, 6.0, 4.0), 5.0)], geometry, config=config)
    assert weak.semantics_available is True
    assert not any(weak.dynamic)
    assert set(weak.kinds) == {"unknown_static"}

    dog = track([Detection("dog", 0.9, (2.0, 0.0, 6.0, 4.0), 5.0)], geometry, config=config)
    assert [i for i, flag in enumerate(dog.dynamic) if flag] == [3, 4, 5]
    assert dog.kinds[4] == "moving"

    chair = track([Detection("chair", 0.9, (2.0, 0.0, 6.0, 4.0), 5.0)], geometry, config=config)
    assert not any(chair.dynamic)


def test_person_who_moved_since_the_detection_is_not_confirmed_static() -> None:
    original = project_depth(aligned(depth(1.0, person_scene)), calibration(), CONFIG)
    old_box = person_box(1.0)
    historical = track([old_box], original)
    assert historical.dynamic[4]

    def moved(u: int, v: int) -> float:
        return 1500.0 if 4 <= u <= 6 else 3000.0

    current = project_depth(aligned(depth(1.2, moved)), calibration(), CONFIG)
    mask = track([old_box], current)
    # A past box must stay bound to its original geometry; it cannot label this frame.
    assert not mask.semantics_available
    assert not any(mask.dynamic)
    assert set(mask.kinds) == {"unknown_static"}
    assert current.rays[6].valid


def test_person_shifted_past_the_old_box_center_is_not_confirmed_static() -> None:
    def shifted(u: int, v: int) -> float:
        return 1500.0 if 5 <= u <= 7 else 3000.0

    geometry = project_depth(aligned(depth(2.25, shifted)), calibration(), CONFIG)
    old_box = Detection("person", 0.9, (3.0, 0.0, 5.0, 4.0), 2.0)
    mask = track([old_box], geometry, config=TrackingConfig(max_object_speed_mps=3.2))
    assert not mask.semantics_available
    assert not any(mask.dynamic)
    assert set(mask.kinds) == {"unknown_static"}
    assert all(ray.valid for ray in geometry.rays)


def test_moving_column_is_unknown_without_independent_motion() -> None:
    from senior_control.vision.tracking import Tracker

    tracker = Tracker()
    tracker.update([], project_depth(aligned(depth(1.0)), calibration(), CONFIG))
    second = project_depth(
        aligned(depth(1.2, lambda u, v: 2000.0 if u == 6 else 3000.0)), calibration(), CONFIG
    )
    mask = tracker.update([], second)
    assert not mask.semantics_available
    assert not mask.motion_available
    assert not any(mask.dynamic)
    assert set(mask.kinds) == {"unknown_static"}


def test_whole_scene_change_is_not_read_as_object_motion() -> None:
    from senior_control.vision.tracking import Tracker

    tracker = Tracker()
    tracker.update([], project_depth(aligned(depth(1.0)), calibration(), CONFIG))
    closer = project_depth(aligned(depth(1.2, lambda u, v: 2000.0)), calibration(), CONFIG)
    mask = tracker.update([], closer)
    # The robot itself probably moved: nothing is marked and the mask says it cannot tell.
    assert mask.motion_available is False
    assert not any(mask.dynamic)
    assert set(mask.kinds) == {"unknown_static"}


def test_motion_needs_a_baseline_that_is_old_enough_and_not_too_old() -> None:
    from senior_control.vision.tracking import Tracker

    tracker = Tracker()
    tracker.update([], project_depth(aligned(depth(1.0)), calibration(), CONFIG))
    near = project_depth(
        aligned(depth(1.05, lambda u, v: 2000.0 if u == 6 else 3000.0)), calibration(), CONFIG
    )
    assert tracker.update([], near).motion_available is False  # 50 ms baseline is noise-level
    stale = project_depth(
        aligned(depth(9.0, lambda u, v: 2000.0 if u == 6 else 3000.0)), calibration(), CONFIG
    )
    assert tracker.update([], stale).motion_available is False  # the baseline expired


class BlockingDetector:
    enabled = True
    reason = "ready"

    def __init__(self) -> None:
        self.started = threading.Event()
        self.release = threading.Event()
        self.calls = 0

    def infer(self, frame: ImageFrame) -> list[Detection]:
        self.calls += 1
        self.started.set()
        assert self.release.wait(5.0)
        return [person_box(frame.stamp)]


def _load_node_module() -> object:
    path = ROOT / "ros2" / "src" / "senior_robot" / "senior_robot" / "perception_node.py"
    spec = importlib.util.spec_from_file_location("perception_node_under_test", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_slow_detector_does_not_block_geometry() -> None:
    module = _load_node_module()
    blocking = BlockingDetector()
    worker = InferenceWorker(blocking, max_pending=1)
    adapter = module.PerceptionAdapter(  # type: ignore[attr-defined]
        calibration=calibration(),
        synchronizer=RgbdSynchronizer(max_skew_s=0.05, capacity=4),
        geometry_config=CONFIG,
        tracking_config=TrackingConfig(),
        worker=worker,
    )
    try:
        assert adapter.published_topics == []
        outputs = []
        for index in range(4):
            stamp = 1.0 + index * 0.1
            assert adapter.ingest_rgb(rgb(stamp)) is None
            output = adapter.ingest_depth(depth(stamp, person_scene))
            assert output is not None
            outputs.append(output)
            if index == 0:
                assert blocking.started.wait(2.0)
        # Geometry came out for every pair while the detector stayed blocked.
        assert [o.geometry.stamp for o in outputs] == pytest.approx([1.0, 1.1, 1.2, 1.3])
        assert all(not any(o.mask.dynamic) for o in outputs)
        assert all(o.mask.semantics_available is False for o in outputs)
        assert all(o.geometry.status[4] == "hit" for o in outputs)
        assert blocking.calls == 1
        assert worker.dropped == 2

        blocking.release.set()
        assert worker.drain(2.0)
        assert adapter.ingest_rgb(rgb(1.4)) is None
        late = adapter.ingest_depth(depth(1.4, person_scene))
        assert late is not None
        # The asynchronous batch belongs to an earlier frame and cannot label 1.4.
        assert late.mask.semantics_available is False
        assert not any(late.mask.dynamic)
    finally:
        blocking.release.set()
        worker.close()


def test_absent_detector_keeps_geometry_rate(tmp_path: Path) -> None:
    module = _load_node_module()
    worker = InferenceWorker(load_detector(tmp_path / "none.pt"), max_pending=1)
    adapter = module.PerceptionAdapter(  # type: ignore[attr-defined]
        calibration=calibration(),
        synchronizer=RgbdSynchronizer(max_skew_s=0.05, capacity=4),
        geometry_config=CONFIG,
        tracking_config=TrackingConfig(),
        worker=worker,
    )
    try:
        for index in range(3):
            stamp = 2.0 + index * 0.05
            adapter.ingest_depth(depth(stamp))
            output = adapter.ingest_rgb(rgb(stamp))
            assert output is not None
            assert output.mask.semantics_available is False
            assert set(output.mask.kinds) == {"unknown_static"}
        assert worker.detector_status == ("disabled", "weights_missing")
    finally:
        worker.close()


def test_node_module_imports_without_ros_and_publishes_no_velocity() -> None:
    path = ROOT / "ros2" / "src" / "senior_robot" / "senior_robot" / "perception_node.py"
    text = path.read_text()
    assert "Twist" not in text and "create_publisher" not in text
    top_level = [line for line in text.splitlines() if line.startswith(("import ", "from "))]
    assert not any("rclpy" in line for line in top_level)
    assert "def main" in text
    code = (
        "import importlib.util,sys;"
        "sys.path.insert(0,sys.argv[2]);"
        "spec=importlib.util.spec_from_file_location('perception_isolated',sys.argv[1]);"
        "module=importlib.util.module_from_spec(spec);"
        "sys.modules[spec.name]=module;"
        "spec.loader.exec_module(module);"
        "bad=[name for name in ('rclpy','torch','ultralytics','onnxruntime') "
        "if name in sys.modules];"
        "sys.exit(1 if bad else 0)"
    )
    done = subprocess.run(
        [sys.executable, "-c", code, str(path), str(ROOT / "src")],
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert done.returncode == 0, done.stderr


def test_depth_message_decoding_honors_padding_and_rejects_unknown_encoding() -> None:
    module = _load_node_module()
    row = b"".join(value.to_bytes(2, "little") for value in (1000, 2000, 0))
    data = (row + b"\xff\xff") * 2
    frame = module.decode_depth(1.0, 3, 2, 8, "16UC1", data, False)  # type: ignore[attr-defined]
    assert tuple(frame.values) == (1000.0, 2000.0, 0.0, 1000.0, 2000.0, 0.0)
    with pytest.raises(ValueError, match="encoding"):
        module.decode_depth(1.0, 3, 2, 8, "rgb8", data, False)  # type: ignore[attr-defined]
    with pytest.raises(ValueError, match="shorter"):
        module.decode_depth(1.0, 3, 2, 8, "16UC1", data[:-3], False)  # type: ignore[attr-defined]


def test_ultralytics_runtime_expands_padded_mono_to_three_equal_channels(monkeypatch):
    from types import SimpleNamespace

    import numpy as np

    from senior_control.vision import detector
    from senior_control.vision.image_codec import decode_rgb

    received = []

    class Model:
        def predict(self, pixels, **kwargs):
            received.append(pixels.copy())
            empty = SimpleNamespace(tolist=lambda: [])
            return [
                SimpleNamespace(names={}, boxes=SimpleNamespace(xyxy=empty, conf=empty, cls=empty))
            ]

    original_import = detector.importlib.import_module
    monkeypatch.setattr(
        detector.importlib,
        "import_module",
        lambda name: (
            SimpleNamespace(YOLO=lambda _: Model())
            if name == "ultralytics"
            else original_import(name)
        ),
    )
    infer = detector._ultralytics_runtime(Path("fixture.pt"))
    frame = decode_rgb(1.0, 2, 2, 3, "mono8", bytes([1, 2, 99, 3, 4, 99]))
    assert infer(frame) == []
    assert received[0].shape == (2, 2, 3)
    assert np.array_equal(
        received[0], np.array([[[1, 1, 1], [2, 2, 2]], [[3, 3, 3], [4, 4, 4]]], dtype=np.uint8)
    )


@pytest.mark.parametrize(
    ("encoding", "expected"),
    [
        ("rgb8", [3, 2, 1]),
        ("bgr8", [1, 2, 3]),
    ],
)
def test_ultralytics_runtime_preserves_expected_bgr_channels(monkeypatch, encoding, expected):
    from types import SimpleNamespace

    from senior_control.vision import detector
    from senior_control.vision.image_codec import decode_rgb

    received = []

    class Model:
        def predict(self, pixels, **kwargs):
            received.append(pixels.copy())
            empty = SimpleNamespace(tolist=lambda: [])
            return [
                SimpleNamespace(names={}, boxes=SimpleNamespace(xyxy=empty, conf=empty, cls=empty))
            ]

    original_import = detector.importlib.import_module
    monkeypatch.setattr(
        detector.importlib,
        "import_module",
        lambda name: (
            SimpleNamespace(YOLO=lambda _: Model())
            if name == "ultralytics"
            else original_import(name)
        ),
    )
    infer = detector._ultralytics_runtime(Path("fixture.pt"))
    assert infer(decode_rgb(1.0, 1, 1, 3, encoding, bytes([1, 2, 3]))) == []
    assert received[0][0, 0].tolist() == expected
