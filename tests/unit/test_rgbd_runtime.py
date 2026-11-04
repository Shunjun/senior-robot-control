"""Regressions for bounded RGB-D input and conservative geometry/semantics."""

import importlib.util
import math
import sys
from pathlib import Path

import pytest

from senior_control.vision import (
    CameraCalibration,
    CameraExtrinsic,
    DepthFrame,
    Detection,
    GeometryConfig,
    ImageFrame,
    Tracker,
    align,
    project_depth,
    track,
)

ROOT = Path(__file__).resolve().parents[2]


def node_module():
    path = ROOT / "ros2/src/senior_robot/senior_robot/perception_node.py"
    spec = importlib.util.spec_from_file_location("rgbd_runtime_node_test", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def geometry(values: tuple[float, ...], *, stamp: float = 1.0):
    rgb = ImageFrame(stamp, 3, 3, "rgb8", bytes(27))
    depth = DepthFrame(stamp, 3, 3, values)
    calibration = CameraCalibration(
        3, 3, 2.0, 4.0, 1.0, 1.0, 1.0, 0.1, 8.0,
        CameraExtrinsic(0.2316, 0.00224, 0.1282, 0.0, 0.0, 0.0),
    )
    return project_depth(align(rgb, depth, max_skew_s=0.0), calibration,
                         GeometryConfig(column_stride=1, row_stride=1))


def test_rgb_decoder_accepts_row_padding_and_rejects_oversized_images():
    module = node_module()
    frame = module.decode_rgb(1.0, 2, 2, 8, "rgb8", b"abcdef!!ghijkl??")
    assert frame.data == b"abcdefghijkl"
    with pytest.raises(ValueError, match="limit"):
        module.decode_rgb(1.0, 100_000, 100_000, 300_000, "rgb8", b"")


def test_low_collision_height_is_not_filtered_as_clear():
    # World 0.0882 m when nominal base is 0.06 m high; below the laser.
    observation = geometry((math.nan,) * 7 + (0.4,) + (math.nan,))
    assert any(point.z == pytest.approx(0.0282) for point in observation.points)
    assert not any(ray.no_return_free for ray in observation.rays)


def test_empty_or_future_semantics_do_not_confirm_static():
    observation = geometry((1.0,) * 9)
    empty = track([], observation, semantics_stamp=observation.stamp)
    assert set(empty.kinds) == {"unknown_static"}
    future = track([Detection("person", 0.9, (0.0, 0.0, 2.0, 2.0), 1.1)], observation)
    assert not future.semantics_available
    assert not any(future.dynamic)


def test_column_difference_without_independent_pose_is_not_object_motion():
    tracker = Tracker()
    tracker.update([], geometry((1.0,) * 9))
    changed = (1.0,) * 2 + (0.5,) + (1.0,) * 6
    mask = tracker.update([], geometry(changed, stamp=1.2))
    assert not mask.motion_available
    assert not any(mask.dynamic)


def test_projection_keeps_distinct_sample_heights_and_pixel_identity():
    values = (math.nan, 1.0, math.nan, math.nan, 0.6, math.nan,
              math.nan, 0.4, math.nan)
    observation = geometry(values)
    assert {(sample.u, sample.v) for sample in observation.samples} == {(1, 0), (1, 1), (1, 2)}
    assert {round(sample.z, 4) for sample in observation.samples} == {0.3782, 0.1282, 0.0282}


def test_depth_decode_limits_before_allocating():
    module = node_module()
    with pytest.raises(ValueError, match="limit"):
        module.decode_depth(1.0, 100_000, 100_000, 200_000, "16UC1", b"", False)


def test_optical_tf_is_converted_to_body_extrinsic_once():
    from senior_control.vision.depth import body_extrinsic_from_optical_tf

    extrinsic = body_extrinsic_from_optical_tf(
        (0.2316, 0.00224, 0.1282), (-0.5, 0.5, -0.5, 0.5)
    )
    assert (extrinsic.x, extrinsic.y, extrinsic.z) == pytest.approx((0.2316, 0.00224, 0.1282))
    assert (extrinsic.roll, extrinsic.pitch, extrinsic.yaw) == pytest.approx((0.0, 0.0, 0.0))


def test_depth_admission_preserves_receipt_and_invalidates_replaced_generation():
    from senior_control.vision.runtime_evidence import DepthAdmission, DepthHeader

    def header(*, sequence=1, epoch="a", valid=True, stamp_ns=1_000_000_000,
               received_steady=10.0):
        return DepthHeader(epoch, "camera-gid", sequence, stamp_ns, 1_050_000_000,
                           received_steady, "camera_optical_frame", "calibration-a", valid)

    admission = DepthAdmission(max_age_s=0.5)
    assert admission.accept(header(), now_ros_ns=1_100_000_000, now_steady=10.1)
    generation = admission.generation
    assert admission.latest is not None
    assert admission.latest.received_steady == 10.0
    assert not admission.accept(header(sequence=2, valid=False), now_ros_ns=1_100_000_000,
                                now_steady=10.1)
    assert admission.latest is None
    assert admission.generation > generation
    assert admission.accept(header(epoch="b"), now_ros_ns=1_100_000_000, now_steady=10.1)
    current = admission.generation
    assert not admission.accept(header(epoch="a", sequence=99, valid=False),
                                now_ros_ns=1_100_000_000, now_steady=10.1)
    assert admission.generation == current
    assert admission.latest is not None
    assert not admission.usable(now_ros_ns=1_600_000_001, now_steady=10.6)


def test_depth_admission_defers_future_and_rejects_expired_receipt():
    from senior_control.vision.runtime_evidence import DepthAdmission, DepthHeader

    admission = DepthAdmission(max_age_s=0.5)
    future = DepthHeader("a", "camera", 1, 1_200_000_000, 1_100_000_000, 10.1,
                         "frame", "cal", True)
    assert not admission.accept(future, now_ros_ns=1_100_000_000, now_steady=10.1)
    assert admission.latest is None
    assert admission.release(now_ros_ns=1_200_000_000, now_steady=10.11) == 1
    assert admission.latest is not None
    assert admission.latest.received_steady == 10.1
    assert admission.latest.received_ros_ns == 1_100_000_000
    past = DepthHeader("a", "camera", 2, 1_000_000_000, 1_050_000_000, 9.0,
                       "frame", "cal", True)
    assert not admission.accept(past, now_ros_ns=1_100_000_000, now_steady=10.1)
    assert admission.latest is None


def test_future_frame_expires_on_paused_clock_and_never_refreshes():
    from senior_control.vision.runtime_evidence import DepthAdmission, DepthHeader

    admission = DepthAdmission(max_age_s=0.5, future_wait_s=0.02)
    future = DepthHeader("a", "camera", 1, 1_002_000_000, 1_000_000_000, 10.0,
                         "frame", "cal", True)
    assert not admission.accept(future, now_ros_ns=1_000_000_000, now_steady=10.0)
    assert admission.release(now_ros_ns=1_000_000_000, now_steady=10.021) == 0
    assert admission.release(now_ros_ns=1_002_000_000, now_steady=10.022) == 0
    assert admission.latest is None


def test_depth_failure_drift_and_out_of_order_revoke_older_results():
    from dataclasses import replace

    from senior_control.vision.runtime_evidence import DepthAdmission, DepthHeader

    admission = DepthAdmission(max_age_s=0.5)
    h = DepthHeader("a", "camera", 1, 1_000_000_000, 1_000_000_000, 10.0,
                    "frame", "cal-a", True)
    assert admission.accept(h, now_ros_ns=1_000_000_000, now_steady=10.0)
    assert not admission.accept(replace(h, sequence=2, valid=False),
                                now_ros_ns=1_010_000_000, now_steady=10.01)
    assert admission.latest is None
    assert not admission.accept(replace(h, sequence=3), now_ros_ns=1_010_000_000,
                                now_steady=10.01)
    later = replace(h, sequence=4, stamp_ns=1_020_000_000,
                    received_ros_ns=1_020_000_000, received_steady=10.02)
    assert admission.accept(later, now_ros_ns=1_020_000_000, now_steady=10.02)
    changed = replace(later, sequence=5, stamp_ns=1_030_000_000,
                      received_ros_ns=1_030_000_000, received_steady=10.03,
                      calibration_id="cal-b")
    assert not admission.accept(changed, now_ros_ns=1_030_000_000, now_steady=10.03)
    assert admission.latest is None


def test_depth_admission_rejects_stale_or_fabricated_receipts():
    from senior_control.vision.runtime_evidence import DepthAdmission, DepthHeader

    h = DepthHeader("a", "camera", 1, 1_000_000_000, 1_000_000_000, 9.0,
                    "frame", "cal", True)
    admission = DepthAdmission(max_age_s=0.5)
    assert not admission.accept(h, now_ros_ns=1_000_000_000, now_steady=10.0)
    assert admission.latest is None


def test_depth_future_fifo_is_bounded_and_never_refreshes_original_receipts():
    from dataclasses import replace

    from senior_control.vision.runtime_evidence import DepthAdmission, DepthHeader

    admission = DepthAdmission(max_age_s=0.5)
    h = DepthHeader("a", "camera", 1, 1_000_001_000, 1_000_000_000, 10.0,
                    "frame", "cal", True)
    for index in range(10):
        assert not admission.accept(replace(h, sequence=index + 1, stamp_ns=h.stamp_ns + index),
                                    now_ros_ns=1_000_000_000, now_steady=10.0)
    assert admission.release(now_ros_ns=1_000_100_000, now_steady=10.01) == 8
    assert admission.latest is None
    assert admission.release(now_ros_ns=1_000_100_000, now_steady=10.01) == 2
    assert admission.latest is not None and admission.latest.sequence == 10
    assert admission.latest.received_ros_ns == 1_000_000_000
    assert admission.latest.received_steady == 10.0


def test_depth_failure_and_clock_rollback_fence_pending_future():
    from dataclasses import replace

    from senior_control.vision.runtime_evidence import DepthAdmission, DepthHeader

    admission = DepthAdmission(max_age_s=0.5)
    h = DepthHeader("a", "camera", 1, 1_001_000_000, 1_000_000_000, 10.0,
                    "frame", "cal", True)
    assert not admission.accept(h, now_ros_ns=1_000_000_000, now_steady=10.0)
    assert not admission.accept(replace(h, sequence=2, valid=False),
                                now_ros_ns=1_000_000_000, now_steady=10.001)
    assert admission.release(now_ros_ns=1_001_000_000, now_steady=10.002) == 0
    assert admission.latest is None
    assert not admission.accept(replace(h, sequence=3, stamp_ns=1_002_000_000),
                                now_ros_ns=900_000_000, now_steady=10.003)
    assert not admission.accept(replace(h, sequence=4, stamp_ns=1_003_000_000),
                                now_ros_ns=1_003_000_000, now_steady=10.004)
    assert admission.latest is None


def test_depth_calibration_drift_requires_new_source_epoch():
    from dataclasses import replace

    from senior_control.vision.runtime_evidence import DepthAdmission, DepthHeader

    admission = DepthAdmission(max_age_s=0.5)
    h = DepthHeader("a", "camera", 1, 1_000_000_000, 1_000_000_000, 10.0,
                    "frame", "cal-a", True)
    assert admission.accept(h, now_ros_ns=1_000_000_000, now_steady=10.0)
    h = replace(h, sequence=2, stamp_ns=1_001_000_000, received_ros_ns=1_001_000_000,
                received_steady=10.001, calibration_id="cal-b")
    assert not admission.accept(h, now_ros_ns=h.stamp_ns, now_steady=10.001)
    h = replace(h, sequence=3, stamp_ns=1_002_000_000, received_ros_ns=1_002_000_000,
                received_steady=10.002, calibration_id="cal-a")
    assert not admission.accept(h, now_ros_ns=h.stamp_ns, now_steady=10.002)
    assert admission.latest is None
    h = replace(h, source_epoch="b", sequence=1)
    assert admission.accept(h, now_ros_ns=h.stamp_ns, now_steady=10.002)
