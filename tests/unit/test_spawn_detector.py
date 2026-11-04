"""Spawned optional inference preserves capture identity and cannot block geometry."""

import time
from pathlib import Path

from detector_test_helpers import slow_detector, wait_until

from senior_control.vision import (
    CameraCalibration,
    CameraExtrinsic,
    DepthFrame,
    GeometryConfig,
    ImageFrame,
    align,
    project_depth,
)


def hung_detector(frame):
    while True:
        time.sleep(1.0)


def conditional_detector(frame):
    if frame.data[0]:
        raise RuntimeError("inference failed")
    return [("person", 0.9, (0.0, 0.0, 1.0, 1.0))]


def capture(epoch, stamp_ns, marker=0):
    from senior_control.vision.spawn_detector import CaptureIdentity, RetainedFrame

    identity = CaptureIdentity(
        source_epoch=epoch,
        rgb_source="rgb-gid",
        depth_source="depth-gid",
        rgb_frame="optical",
        depth_frame="optical",
        registration_id="registered-calibration",
        rgb_stamp_ns=stamp_ns,
        depth_stamp_ns=stamp_ns + 1,
        rgb_received_ros_ns=stamp_ns + 2,
        depth_received_ros_ns=stamp_ns + 3,
        rgb_received_steady=10.0,
        depth_received_steady=10.01,
    )
    frame = ImageFrame(stamp_ns / 1e9, 2, 2, "rgb8", bytes([marker]) * 12)
    depth = DepthFrame((stamp_ns + 1) / 1e9, 2, 2, (0.4,) * 4)
    calibration = CameraCalibration(
        2,
        2,
        2.0,
        2.0,
        1.0,
        1.0,
        1.0,
        0.1,
        8.0,
        CameraExtrinsic(0.2, 0.0, 0.1, 0.0, 0.0, 0.0),
    )
    geometry = project_depth(
        align(frame, depth, max_skew_s=0.03),
        calibration,
        GeometryConfig(row_stride=1, column_stride=1),
    )
    return RetainedFrame(identity=identity, rgb=frame, geometry=geometry)


def test_missing_weights_disabled_without_loading_or_downloading(tmp_path: Path):
    from senior_control.vision.spawn_detector import SpawnInferenceWorker

    worker = SpawnInferenceWorker(weights_path=tmp_path / "missing.pt")
    try:
        assert worker.status == ("disabled", "weights_missing")
        assert not worker.submit(capture("epoch-a", 1_000_000_000))
        assert not (tmp_path / "missing.pt").exists()
    finally:
        worker.close()


def test_late_result_keeps_original_rgb_depth_and_source_identity(tmp_path: Path):
    from senior_control.vision.spawn_detector import SpawnInferenceWorker

    weights = tmp_path / "provided.pt"
    weights.write_bytes(b"fixture")
    worker = SpawnInferenceWorker(weights_path=weights, processor=slow_detector, timeout_s=2.0)
    old = capture("epoch-a", 1_000_000_000)
    newer = capture("epoch-a", 1_100_000_000)
    try:
        start = time.monotonic()
        assert worker.submit(old)
        assert time.monotonic() - start < 0.1
        wait_until(lambda: worker.in_flight)
        assert worker.submit(newer)
        result = wait_until(worker.take_result)
        assert result.frame == old
        assert result.frame.identity.rgb_stamp_ns == 1_000_000_000
        assert result.frame.identity.depth_stamp_ns == 1_000_000_001
        assert result.detections[0].label == "person"
        assert result.frame != newer
    finally:
        worker.close()


def test_hung_child_times_out_and_shutdown_is_bounded(tmp_path: Path):
    from senior_control.vision.spawn_detector import SpawnInferenceWorker

    weights = tmp_path / "provided.pt"
    weights.write_bytes(b"fixture")
    worker = SpawnInferenceWorker(weights_path=weights, processor=hung_detector, timeout_s=0.3)
    assert worker.submit(capture("epoch-a", 1_000_000_000))
    wait_until(lambda: worker.status[0] == "failed")
    start = time.monotonic()
    worker.close()
    assert time.monotonic() - start < 2.0
    assert not worker.child_alive
    assert worker.take_result() is None


def test_old_epoch_late_result_and_failed_inference_cannot_reappear(tmp_path: Path):
    from senior_control.vision.spawn_detector import SpawnInferenceWorker

    weights = tmp_path / "provided.pt"
    weights.write_bytes(b"fixture")
    worker = SpawnInferenceWorker(weights_path=weights, processor=slow_detector, timeout_s=2.0)
    try:
        assert worker.submit(capture("old", 1_000_000_000))
        wait_until(lambda: worker.in_flight)
        worker.invalidate()
        new = capture("new", 1_100_000_000)
        assert worker.submit(new)
        result = wait_until(worker.take_result)
        assert result.frame == new
    finally:
        worker.close()

    failed = SpawnInferenceWorker(
        weights_path=weights, processor=conditional_detector, timeout_s=2.0
    )
    try:
        assert failed.submit(capture("epoch", 1_000_000_000, marker=1))
        wait_until(lambda: failed.status[0] == "failed")
        assert failed.take_result() is None
    finally:
        failed.close()


def busy_detector(frame):
    deadline = time.monotonic() + 0.8
    while time.monotonic() < deadline:
        pass
    return [("person", 0.9, (0.0, 0.0, 1.0, 1.0))]


def test_cpu_inference_isolated_from_geometry_and_latest_queue_bounded(tmp_path: Path):
    from senior_control.vision.spawn_detector import SpawnInferenceWorker

    weights = tmp_path / "provided.pt"
    weights.write_bytes(b"fixture")
    worker = SpawnInferenceWorker(weights_path=weights, processor=busy_detector, timeout_s=3.0)
    try:
        assert worker.submit(capture("epoch-a", 1_000_000_000))
        wait_until(lambda: worker.in_flight)
        before = time.monotonic()
        for index in range(1, 11):
            sample = capture("epoch-a", 1_000_000_000 + index * 10_000_000)
            assert len(sample.geometry.samples) == 4
            assert worker.submit(sample)
        assert time.monotonic() - before < 0.5
        assert worker.dropped >= 8
        assert wait_until(worker.take_result).frame.identity.rgb_stamp_ns == 1_000_000_000
    finally:
        worker.close()


def test_same_epoch_replay_or_invalid_frame_fences_inflight_result(tmp_path: Path):
    from dataclasses import replace

    from senior_control.vision.spawn_detector import SpawnInferenceWorker

    weights = tmp_path / "provided.pt"
    weights.write_bytes(b"fixture")
    worker = SpawnInferenceWorker(weights_path=weights, processor=slow_detector, timeout_s=2.0)
    frame = capture("epoch-a", 1_000_000_000)
    try:
        assert worker.submit(frame)
        wait_until(lambda: worker.in_flight)
        assert not worker.submit(frame)  # replay must revoke the in-flight generation
        time.sleep(0.3)
        assert worker.take_result() is None
        assert not worker.submit(capture("epoch-a", 1_100_000_000))
        good = capture("epoch-b", 1_200_000_000)
        assert worker.submit(good)
        assert wait_until(worker.take_result).frame == good
        wrong_frame = replace(good, identity=replace(good.identity, depth_frame="wrong"))
        assert not worker.submit(wrong_frame)
    finally:
        worker.close()


def test_unavailable_or_wrong_runtime_is_explicit_and_never_loads_model(tmp_path: Path):
    from importlib.metadata import PackageNotFoundError, version

    import pytest

    from senior_control.vision.spawn_detector import (
        SUPPORTED_ULTRALYTICS_VERSION,
        SpawnInferenceWorker,
    )

    try:
        installed = version("ultralytics")
    except PackageNotFoundError:
        reason = "runtime_missing"
    else:
        if installed == SUPPORTED_ULTRALYTICS_VERSION:
            pytest.skip("supported runtime installed; this is the missing-runtime test")
        reason = "runtime_version_unsupported"
    weights = tmp_path / "provided.pt"
    weights.write_bytes(b"fixture")
    worker = SpawnInferenceWorker(weights_path=weights, timeout_s=2.0)
    try:
        assert worker.submit(capture("epoch-a", 1_000_000_000))
        wait_until(lambda: worker.status[0] == "disabled")
        assert worker.status == ("disabled", reason)
        assert worker.take_result() is None
    finally:
        worker.close()


def test_bound_result_requires_original_fresh_receipts_and_current_epoch(tmp_path: Path):
    from senior_control.vision.spawn_detector import SpawnInferenceWorker

    weights = tmp_path / "provided.pt"
    weights.write_bytes(b"fixture")
    worker = SpawnInferenceWorker(
        weights_path=weights, processor=conditional_detector, timeout_s=2.0
    )
    frame = capture("epoch-a", 1_000_000_000)
    try:
        assert worker.submit(frame)
        result = wait_until(worker.take_result)
        assert worker.result_qualified(
            result, now_ros_ns=1_100_000_000, now_steady=10.2, max_age_s=0.3
        )
        assert not worker.result_qualified(
            result, now_ros_ns=1_000_000_000, now_steady=10.2, max_age_s=0.3
        )
        assert not worker.result_qualified(
            result, now_ros_ns=1_100_000_000, now_steady=10.4, max_age_s=1.0
        )
        worker.invalidate()
        assert not worker.result_qualified(
            result, now_ros_ns=1_100_000_000, now_steady=10.2, max_age_s=0.3
        )
    finally:
        worker.close()


def test_fabricated_result_cannot_borrow_an_active_epoch(tmp_path: Path):
    from dataclasses import replace

    from senior_control.vision.spawn_detector import SpawnInferenceWorker

    weights = tmp_path / "provided.pt"
    weights.write_bytes(b"fixture")
    worker = SpawnInferenceWorker(
        weights_path=weights, processor=conditional_detector, timeout_s=2.0
    )
    try:
        assert worker.submit(capture("epoch-a", 1_000_000_000))
        result = wait_until(worker.take_result)
        forged = replace(result, frame=capture("epoch-a", 1_050_000_000))
        assert not worker.result_qualified(
            forged, now_ros_ns=1_100_000_000, now_steady=10.2, max_age_s=0.3
        )
    finally:
        worker.close()


def test_detector_accepts_original_sampled_geometry_without_invented_laser_rays(tmp_path: Path):
    from dataclasses import replace

    from senior_control.vision.semantic_runtime import SampledGeometry
    from senior_control.vision.spawn_detector import SpawnInferenceWorker

    weights = tmp_path / "provided.pt"
    weights.write_bytes(b"fixture")
    old = capture("epoch", 1_000_000_000)
    original_samples = old.geometry.samples
    observed = replace(old, geometry=SampledGeometry(old.geometry.stamp, original_samples))
    worker = SpawnInferenceWorker(weights_path=weights, processor=conditional_detector)
    try:
        assert worker.submit(observed)
        result = wait_until(worker.take_result)
        assert result.frame.geometry.samples is original_samples
        assert not hasattr(result.frame.geometry, "rays")
    finally:
        worker.close()
