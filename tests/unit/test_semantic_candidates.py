"""3-D semantic candidates retain their exact registered source frames."""

from dataclasses import replace
from types import SimpleNamespace

import pytest

from senior_control.vision.depth_runtime import CameraInfoFrame


def color_info():
    return CameraInfoFrame(
        stamp_ns=1_000_000_000, received_ros_ns=1_000_000_001, received_steady=10.0,
        frame_id="optical", raw_source="color-info-gid", width=3, height=3,
        distortion_model="plumb_bob", d=(0.0,) * 5,
        k=(2.0, 0.0, 1.0, 0.0, 4.0, 1.0, 0.0, 0.0, 1.0),
        r=(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
        p=(2.0, 0.0, 1.0, 0.0, 0.0, 4.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0),
        binning_x=0, binning_y=0, roi=(0, 0, 0, 0, False),
    )


def depth_metadata():
    return SimpleNamespace(
        stamp_ns=1_000_000_000, info_stamp_ns=1_000_000_000,
        received_ros_ns=1_000_000_002, info_received_ros_ns=1_000_000_003,
        received_steady=10.01, info_received_steady=10.02,
        frame_id="optical", source_epoch="depth-epoch", raw_source="depth-gid",
        info_source="depth-info-gid", calibration_id="a" * 64,
        width=3, height=3, fx=2.0, fy=4.0, cx=1.0, cy=1.0,
    )


def test_registration_requires_explicit_profile_and_exact_calibrated_frames():
    from senior_control.vision.semantic_runtime import qualify_registration

    info, depth = color_info(), depth_metadata()
    registration = qualify_registration(
        info, depth, rgb_stamp_ns=1_000_000_000, rgb_frame="optical",
        registered_profile=True, now_ros_ns=1_100_000_000, now_steady=10.1, max_age_s=0.3,
    )
    assert len(registration) == 64
    for changed_info, enabled in (
        (info, False),
        (replace(info, frame_id="other"), True),
        (replace(info, stamp_ns=999_999_999), True),
        (replace(info, k=(3.0, *info.k[1:])), True),
        (replace(info, d=(0.0, 0.1, 0.0, 0.0, 0.0)), True),
    ):
        with pytest.raises(ValueError):
            qualify_registration(
                changed_info, depth, rgb_stamp_ns=1_000_000_000, rgb_frame="optical",
                registered_profile=enabled, now_ros_ns=1_100_000_000, now_steady=10.1,
                max_age_s=0.3,
            )


def test_candidates_preserve_y_range_and_depth_without_erasing_wall():
    from senior_control.vision import Detection
    from senior_control.vision.depth import GeometrySample
    from senior_control.vision.semantic_runtime import associate_samples

    samples = (
        GeometrySample(0, 1, 0, 1.0, 1.0, 0.0, 0.3),
        GeometrySample(0, 1, 1, 3.0, 3.0, 0.0, 0.1),
        GeometrySample(0, 1, 2, 0.2, 0.2, 0.0, -0.1),
    )
    detection = Detection("person", 0.9, (0.0, 0.0, 2.0, 1.0), 1.0)
    associations = associate_samples((detection,), samples)
    assert [(item.sample.v, item.object_mask) for item in associations] == [(0, 1), (1, 1)]
    assert [item.sample.depth_m for item in associations] == [1.0, 3.0]
    # Both are only candidates; same-column depth and occlusion stay distinguishable.
    assert samples[2] not in [item.sample for item in associations]


def wire_depth(stamp_ns=1_000_000_000):
    def stamp(ns):
        return SimpleNamespace(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)

    return SimpleNamespace(
        valid=True, reason="", stamp=stamp(stamp_ns), camera_info_stamp=stamp(stamp_ns),
        received_ros=stamp(1_000_000_000), camera_info_received_ros=stamp(1_000_000_000),
        transform_stamp=stamp(0), transform_static=True, received_steady=10.0,
        camera_info_received_steady=10.0, frame_id="optical", body_frame="base_link",
        raw_source="depth-gid", camera_info_source="depth-info-gid", source_epoch="depth-epoch",
        calibration_id="a" * 64, sequence=1, width=3, height=3, column_stride=1, row_stride=1,
        fx=2.0, fy=4.0, cx=1.0, cy=1.0, min_range_m=0.1, max_range_m=8.0,
        range_error_m=0.01, sensor_origin=[0.0, 0.0, 0.0],
        optical_to_body_quaternion=[0.0, 0.0, 0.0, 1.0],
        samples=[SimpleNamespace(u=1, v=1, depth_m=1.0, x=0.0, y=0.0, z=1.0)],
    )


def test_depth_snapshot_validates_real_pixel_and_transform_and_is_immutable():
    from senior_control.vision.semantic_runtime import depth_snapshot

    wire = wire_depth()
    depth = depth_snapshot(wire)
    assert depth.samples[0].z == 1.0
    wire.samples[0].z = 2.0
    assert depth.samples[0].z == 1.0
    with pytest.raises(ValueError):
        depth_snapshot(wire)
    wire = wire_depth()
    wire.samples *= 8193
    with pytest.raises(ValueError):
        depth_snapshot(wire)
    wire = wire_depth()
    wire.source_epoch = ""
    with pytest.raises(ValueError):
        depth_snapshot(wire)


def test_synchronizer_waits_for_future_clock_without_replacing_first_receipts():
    from senior_control.vision.semantic_runtime import (
        ColorCapture,
        SemanticSynchronizer,
        depth_snapshot,
    )
    from senior_control.vision.synchronizer import ImageFrame

    stamp = 1_002_000_000
    rgb = ColorCapture(stamp, 1_000_000_000, 10.0, "optical", "rgb-gid",
                       ImageFrame(stamp / 1e9, 3, 3, "rgb8", bytes(27)))
    info = replace(color_info(), stamp_ns=stamp, received_ros_ns=1_000_000_000,
                   received_steady=10.0)
    depth = depth_snapshot(wire_depth(stamp))
    sync = SemanticSynchronizer(max_age_s=0.3)
    sync.offer_rgb(rgb)
    sync.offer_info(info)
    sync.offer_depth(depth)
    assert sync.take_ready(now_ros_ns=1_000_000_000, now_steady=10.01) == (None, None)
    pair, reason = sync.take_ready(now_ros_ns=stamp, now_steady=10.015)
    assert reason is None and pair == (rgb, info, depth)
    assert pair[0].received_ros_ns == 1_000_000_000
    assert pair[2].received_steady == 10.0

    sync.offer_rgb(replace(rgb, stamp_ns=stamp + 1))
    sync.offer_info(replace(info, stamp_ns=stamp + 1))
    later = depth_snapshot(wire_depth(stamp + 1))
    sync.offer_depth(later)
    pair, reason = sync.take_ready(now_ros_ns=stamp, now_steady=10.021)
    assert pair is None and reason == "future_expired"


def test_late_semantic_result_binds_original_depth_not_newest_frame(tmp_path):
    from senior_control.vision.semantic_coordinator import SemanticCoordinator
    from senior_control.vision.semantic_runtime import ColorCapture, depth_snapshot
    from senior_control.vision.spawn_detector import SpawnInferenceWorker
    from senior_control.vision.synchronizer import ImageFrame
    from tests.unit.test_spawn_detector import slow_detector, wait_until

    weights = tmp_path / "fixture.pt"
    weights.write_bytes(b"fixture")
    worker = SpawnInferenceWorker(weights_path=weights, processor=slow_detector, timeout_s=2)
    coordinator = SemanticCoordinator(worker, registered_profile=True, max_age_s=0.3)

    def capture(stamp, distance):
        wire = wire_depth(stamp)
        wire.samples[0].depth_m = wire.samples[0].z = distance
        rgb = ColorCapture(stamp, 1_000_000_000, 10.0, "optical", "rgb-gid",
                           ImageFrame(stamp / 1e9, 3, 3, "rgb8", bytes(27)))
        info = replace(color_info(), stamp_ns=stamp, received_ros_ns=1_000_000_000,
                       received_steady=10.0)
        return rgb, info, depth_snapshot(wire)

    try:
        first = capture(1_000_000_000, 1.0)
        newest = capture(1_050_000_000, 2.0)
        assert coordinator.submit(*first, now_ros_ns=1_100_000_000, now_steady=10.1)
        wait_until(lambda: worker.in_flight)
        assert coordinator.submit(*newest, now_ros_ns=1_100_000_000, now_steady=10.1)
        result = wait_until(lambda: coordinator.take_result(
            now_ros_ns=1_100_000_000, now_steady=10.1
        ))
        assert result.rgb == first[0] and result.info == first[1] and result.depth == first[2]
        assert result.depth.samples[0].depth_m == 1.0
        assert result.associations[0].sample.z == 1.0
        assert coordinator.is_current(result, now_ros_ns=1_100_000_000, now_steady=10.1)
        assert not coordinator.is_current(result, now_ros_ns=1_100_000_000, now_steady=10.4)
        coordinator.invalidate()
        assert not coordinator.is_current(result, now_ros_ns=1_100_000_000, now_steady=10.1)
    finally:
        coordinator.close()


def test_semantic_epoch_invalidation_cancels_inflight_result(tmp_path):
    import time

    from senior_control.vision.semantic_coordinator import SemanticCoordinator
    from senior_control.vision.semantic_runtime import ColorCapture, depth_snapshot
    from senior_control.vision.spawn_detector import SpawnInferenceWorker
    from senior_control.vision.synchronizer import ImageFrame
    from tests.unit.test_spawn_detector import slow_detector, wait_until

    weights = tmp_path / "fixture.pt"
    weights.write_bytes(b"fixture")
    worker = SpawnInferenceWorker(weights_path=weights, processor=slow_detector, timeout_s=2)
    coordinator = SemanticCoordinator(worker, registered_profile=True, max_age_s=0.3)
    rgb = ColorCapture(1_000_000_000, 1_000_000_000, 10.0, "optical", "rgb-gid",
                       ImageFrame(1.0, 3, 3, "rgb8", bytes(27)))
    info = replace(color_info(), received_ros_ns=1_000_000_000, received_steady=10.0)
    try:
        assert coordinator.submit(rgb, info, depth_snapshot(wire_depth()),
                                  now_ros_ns=1_100_000_000, now_steady=10.1)
        wait_until(lambda: worker.in_flight)
        coordinator.invalidate()
        time.sleep(0.25)
        assert coordinator.take_result(now_ros_ns=1_100_000_000, now_steady=10.1) is None
    finally:
        coordinator.close()


def test_registration_rejects_stale_info_even_when_image_and_depth_are_fresh():
    from senior_control.vision.semantic_runtime import qualify_registration

    info, depth = color_info(), depth_metadata()
    stale_info = replace(info, received_steady=9.0)
    with pytest.raises(ValueError):
        qualify_registration(stale_info, depth, rgb_stamp_ns=1_000_000_000,
                             rgb_frame="optical", registered_profile=True,
                             now_ros_ns=1_100_000_000, now_steady=10.1, max_age_s=0.3)


def test_synchronizer_discards_missed_frames_and_rejects_clock_rollback():
    from senior_control.vision.semantic_runtime import ColorCapture, SemanticSynchronizer
    from senior_control.vision.synchronizer import ImageFrame

    sync = SemanticSynchronizer(max_age_s=0.3)
    rgb = ColorCapture(1_000_000_000, 1_000_000_000, 10.0, "optical", "rgb-gid",
                       ImageFrame(1.0, 3, 3, "rgb8", bytes(27)))
    sync.offer_rgb(rgb)
    assert sync.take_ready(now_ros_ns=1_000_000_000, now_steady=10.0) == (None, None)
    assert sync.take_ready(now_ros_ns=999_000_000, now_steady=10.001) == (
        None, "clock_unqualified"
    )
    sync.offer_rgb(rgb)
    with pytest.raises(ValueError):
        sync.offer_rgb(rgb)


def test_semantic_associations_bound_number_of_objects_and_never_assume_static():
    from senior_control.vision import Detection
    from senior_control.vision.depth import GeometrySample
    from senior_control.vision.semantic_runtime import associate_samples

    sample = GeometrySample(0, 1, 1, 1.0, 0.0, 0.0, 1.0)
    detection = Detection("person", 0.9, (0.0, 0.0, 2.0, 2.0), 1.0)
    assert associate_samples((), (sample,)) == ()  # No result is not static evidence.
    with pytest.raises(ValueError):
        associate_samples((detection,) * 33, (sample,))
