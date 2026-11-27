from types import SimpleNamespace as N

import numpy as np
import pytest
from control_process_test_support import generic_motion_profile
from senior_motion_estimation.motion_sensor_runtime import MotionSensorRuntime
from senior_motion_estimation.optional_motion_inputs import rgbd_observation, static_matrix
from sensor_time_test_support import capture_fields, sample_time

from vehicle_core.diagnostics import RuntimeDiagnostics


def stamp(v):
    return N(sec=int(v), nanosec=round((v - int(v)) * 1e9))


def header(t, sequence, family):
    return N(
        valid=True,
        raw_gid=family,
        source_epoch="epoch",
        bridge_boot_id="raw",
        capture_generation=1,
        sequence=sequence,
        frame_id="camera",
        bridge_gid="02",
        reason="",
        **capture_fields(
            time=sample_time(
                stamp_ns=round((t) * 1e9),
                received_ros_ns=round((t) * 1e9),
                received_steady_ns=round((t + 10) * 1e9),
            )
        ),
    )


def candidate():
    tf = N(
        transforms=[
            N(
                header=N(frame_id="base_link", stamp=stamp(0)),
                child_frame_id="camera",
                transform=N(rotation=N(x=0, y=0, z=0, w=1), translation=N(x=0, y=0, z=0)),
            )
        ]
    )
    matrix = np.eye(4)
    matrix[0, 3] = -0.04
    return N(
        owner=N(valid=True, owner_lost=False),
        body=N(
            valid=True,
            body_frame="base_link",
            first_tf=tf,
            second_tf=tf,
            first_rgb=header(9.8, 1, "rgb"),
            first_depth=header(9.8, 1, "depth"),
            first_rgb_info=header(9.8, 1, "rgb_info"),
            first_depth_info=header(9.8, 1, "depth_info"),
            second_rgb=header(9.9, 2, "rgb"),
            second_depth=header(9.9, 2, "depth"),
            second_rgb_info=header(9.9, 2, "rgb_info"),
            second_depth_info=header(9.9, 2, "depth_info"),
            interval_start=stamp(9.8),
            interval_end=stamp(9.9),
            camera_transform=matrix.flatten(),
            matched=100,
            inliers=80,
        ),
    )


def test_rgbd_uses_actual_extrinsics_raw_capture_age_and_optional_lateral():
    o = rgbd_observation(candidate())
    assert o.values == pytest.approx((0.4, 0.0, 0.0))
    s = MotionSensorRuntime(generic_motion_profile(), RuntimeDiagnostics(lambda *_: None))
    s.ingest_observation(o, 10, 20)
    assert s.estimate(10, 20)[1].velocity.vx == pytest.approx(0.4)
    assert s.latest.vy.observed
    assert len(s.latest.vx.roots) == 8
    assert s.estimate(10.8, 20.8)[1] is None  # first required capture owns hard expiry


def test_bad_optional_calibration_never_changes_other_source():
    from test_unified_estimator import observation

    s = MotionSensorRuntime(generic_motion_profile(), RuntimeDiagnostics(lambda *_: None))
    s.ingest_observation(observation("wheel", (0.2, None, 0.1)), 10, 20)
    bad = candidate()
    bad.body.first_tf.transforms = []
    with pytest.raises(ValueError, match="calibration"):
        rgbd_observation(bad)
    assert s.estimate(10, 20)[1].velocity.vx == 0.2


def test_static_chain_composes_and_rejects_cycle_instead_of_assumed_identity():
    tf = candidate().body.first_tf
    tf.transforms[0].transform.translation = N(x=0.1, y=0.2, z=0)
    assert static_matrix(tf, "base_link", "camera")[:3, 3] == pytest.approx((0.1, 0.2, 0))
    tf.transforms[0].header.frame_id = "camera"
    with pytest.raises(ValueError, match="cyclic"):
        static_matrix(tf, "base_link", "camera")


def test_gps_body_twist_does_not_need_absolute_heading_or_other_motion_axes(monkeypatch):
    import sys
    from types import ModuleType

    from senior_motion_estimation.optional_motion_inputs import OptionalMotionInputs

    time_module = ModuleType("rclpy.time")
    time_module.Time = N(from_msg=lambda x: x)
    tf_module = ModuleType("tf2_ros")
    tf_module.TransformException = type("TransformException", (Exception,), {})
    monkeypatch.setitem(sys.modules, "rclpy.time", time_module)
    monkeypatch.setitem(sys.modules, "tf2_ros", tf_module)
    monkeypatch.setattr(
        "senior_motion_estimation.optional_motion_inputs.time", N(monotonic=lambda: 20.0)
    )
    s = MotionSensorRuntime(generic_motion_profile(), RuntimeDiagnostics(lambda *_: None))
    adapter = object.__new__(OptionalMotionInputs)
    adapter.node = N(get_clock=lambda: N(now=lambda: N(nanoseconds=10_000_000_000)))
    adapter.runtime = s
    adapter._gps_family = "raw_gnss_doppler"
    adapter._allow_topology = False
    message = N(
        header=N(stamp=stamp(9.9), frame_id="odom"),
        child_frame_id="base_link",
        twist=N(twist=N(linear=N(x=0.4, y=0.02), angular=N(z=0.0))),
    )
    info = dict(
        received_timestamp=9_900_000_000,
        native_received_steady=19.9,
        receipt_generation=1,
        receipt_sequence=1,
    )
    adapter.gps(
        message,
        dict(
            info,
            sample_time=sample_time(
                stamp_ns=round((9.9) * 1e9),
                received_ros_ns=round((9.9) * 1e9),
                received_steady_ns=round((19.9) * 1e9),
            ),
        ),
        N(sample_gid=b"gps"),
    )
    s.estimate(10.0, 20.0)
    assert s.latest.vx.observed and s.latest.vy.observed
    assert s.latest.wz.value is None and s.latest.healthy
    assert s.latest.vx.roots[0].family == "raw_gnss_doppler"
    assert s.independent is None
