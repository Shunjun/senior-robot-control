from types import SimpleNamespace as NS

import numpy as np

from senior_control.vision.raw_capture import RawCaptureHeader
from senior_control.vision.rgbd_frame_builder import build_frame


def inputs():
    header = RawCaptureHeader(
        100, 101, 1.1, "optical", "01", "02", "source", "boot", 1, 1, True, ""
    )
    w, h = 80, 80
    rgb = NS(
        width=w,
        height=h,
        step=w * 3,
        encoding="rgb8",
        is_bigendian=False,
        data=bytes([255, 0, 0]) * (w * h),
    )
    depth = NS(
        width=w,
        height=h,
        step=w * 4,
        encoding="32FC1",
        is_bigendian=False,
        data=np.ones((h, w), dtype="<f4").tobytes(),
    )
    k = (80.0, 0.0, 40.0, 0.0, 80.0, 40.0, 0.0, 0.0, 1.0)
    r = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    p = (80.0, 0.0, 40.0, 0.0, 0.0, 80.0, 40.0, 0.0, 0.0, 0.0, 1.0, 0.0)
    info = NS(
        width=w,
        height=h,
        distortion_model="plumb_bob",
        d=(0.0,) * 5,
        k=k,
        r=r,
        p=p,
        binning_x=0,
        binning_y=0,
        roi=(0, 0, 0, 0),
        roi_do_rectify=False,
    )
    rgb.capture = depth.capture = info.capture = header
    return (header,) * 4, rgb, depth, info, info


def test_rgb8_conversion_preserves_encoding_and_immutable_source_identity():
    headers, rgb, depth, rgb_info, depth_info = inputs()
    built = build_frame(
        headers,
        rgb,
        depth,
        rgb_info,
        depth_info,
        calibration_epoch="cal",
        registration="registered",
        tf_epoch="tf",
        dependencies=("tf",),
        exclusion=bytes(80 * 80),
        static_sim_declared=True,
    )
    assert built.raw_rgb_encoding == "rgb8"
    assert built.preprocessing_version == "rgb8-mono-v1"
    assert built.frame.image[0, 0] == 77
    assert built.frame.rgb_header == headers[0]
    assert not built.frame.image.flags.writeable


def test_unknown_scene_does_not_default_to_empty_exclusion():
    import pytest

    headers, rgb, depth, rgb_info, depth_info = inputs()
    with pytest.raises(ValueError, match="exclusion"):
        build_frame(
            headers,
            rgb,
            depth,
            rgb_info,
            depth_info,
            calibration_epoch="cal",
            registration="registered",
            tf_epoch="tf",
            dependencies=("tf",),
            exclusion=None,
            static_sim_declared=True,
        )


def test_rejects_capture_body_from_other_header():
    from dataclasses import replace

    import pytest

    headers, rgb, depth, rgb_info, depth_info = inputs()
    depth.capture = replace(headers[1], sequence=2)
    with pytest.raises(ValueError, match="capture"):
        build_frame(
            headers,
            rgb,
            depth,
            rgb_info,
            depth_info,
            calibration_epoch="cal",
            registration="registered",
            tf_epoch="tf",
            dependencies=("tf",),
            exclusion=bytes(80 * 80),
            static_sim_declared=True,
        )


def test_rejects_nonrectified_or_unregistered_camera_info():
    import pytest

    headers, rgb, depth, rgb_info, depth_info = inputs()
    depth_info = NS(**vars(depth_info))
    depth_info.p = tuple(90.0 if i == 0 else x for i, x in enumerate(depth_info.p))
    with pytest.raises(ValueError):
        build_frame(
            headers,
            rgb,
            depth,
            rgb_info,
            depth_info,
            calibration_epoch="cal",
            registration="registered",
            tf_epoch="tf",
            dependencies=("tf",),
            exclusion=bytes(80 * 80),
            static_sim_declared=True,
        )


def test_padded_big_endian_depth_is_decoded_from_real_rows():
    headers, rgb, depth, rgb_info, depth_info = inputs()
    depth.is_bigendian = True
    depth.step += 4
    depth.data = (np.ones(80, dtype=">f4").tobytes() + b"pad!") * 80
    built = build_frame(
        headers,
        rgb,
        depth,
        rgb_info,
        depth_info,
        calibration_epoch="cal",
        registration="registered",
        tf_epoch="tf",
        dependencies=("tf",),
        exclusion=bytes(80 * 80),
        static_sim_declared=True,
    )
    assert np.all(built.frame.depth == 1.0)


def test_bad_image_consumes_headers_and_prevents_same_capture_replay():
    from senior_control.vision.independent_motion import IndependentMotion, Profile
    from senior_control.vision.rgbd_frame_builder import offer_capture

    headers, rgb, depth, rgb_info, depth_info = inputs()
    estimator = IndependentMotion(
        Profile("sim", "fixture", True, "texture", "cal", 0.1, 0.1, 1.0, 0.1, 0.1)
    )
    rgb.step = 1
    result, _ = offer_capture(
        estimator,
        headers,
        rgb,
        depth,
        rgb_info,
        depth_info,
        calibration_epoch="cal",
        registration="registered",
        tf_epoch="tf",
        dependencies=("tf",),
        exclusion=bytes(80 * 80),
        static_sim_declared=True,
        now_ros_ns=101,
        now_steady=1.1,
    )
    assert result.reason == "invalid_frame"
    rgb.step = 240
    result, _ = offer_capture(
        estimator,
        headers,
        rgb,
        depth,
        rgb_info,
        depth_info,
        calibration_epoch="cal",
        registration="registered",
        tf_epoch="tf",
        dependencies=("tf",),
        exclusion=bytes(80 * 80),
        static_sim_declared=True,
        now_ros_ns=101,
        now_steady=1.1,
    )
    assert result.reason == "source_frontier_invalid"


def test_empty_exclusion_requires_explicit_static_sim_domain():
    import pytest

    headers, rgb, depth, rgb_info, depth_info = inputs()
    with pytest.raises(ValueError, match="static"):
        build_frame(
            headers,
            rgb,
            depth,
            rgb_info,
            depth_info,
            calibration_epoch="cal",
            registration="registered",
            tf_epoch="tf",
            dependencies=("tf",),
            exclusion=bytes(80 * 80),
            static_sim_declared=False,
        )


def test_missing_registration_declaration_cannot_build_a_frame():
    import pytest

    headers, rgb, depth, rgb_info, depth_info = inputs()
    with pytest.raises(ValueError, match="registration"):
        build_frame(
            headers,
            rgb,
            depth,
            rgb_info,
            depth_info,
            calibration_epoch="cal",
            registration="",
            tf_epoch="tf",
            dependencies=("tf",),
            exclusion=bytes(80 * 80),
            static_sim_declared=True,
        )


def test_built_frame_and_calibration_are_snapshots_of_mutable_input():
    headers, rgb, depth, rgb_info, depth_info = inputs()
    rgb.data = bytearray(rgb.data)
    depth_info.d = [0.0] * 5
    built = build_frame(
        headers,
        rgb,
        depth,
        rgb_info,
        depth_info,
        calibration_epoch="cal",
        registration="registered",
        tf_epoch="tf",
        dependencies=("tf",),
        exclusion=bytes(80 * 80),
        static_sim_declared=True,
    )
    rgb.data[0] = 0
    depth_info.d[0] = 0.5
    assert built.frame.image[0, 0] == 77
    assert built.depth_info.d == (0.0,) * 5


def test_original_image_hash_binds_exact_padded_native_body():
    import hashlib

    headers, rgb, depth, rgb_info, depth_info = inputs()
    rgb.step += 2
    rgb.data = (bytes([255, 0, 0]) * 80 + b"xy") * 80
    built = build_frame(
        headers,
        rgb,
        depth,
        rgb_info,
        depth_info,
        calibration_epoch="cal",
        registration="registered",
        tf_epoch="tf",
        dependencies=("tf",),
        exclusion=bytes(80 * 80),
        static_sim_declared=True,
    )
    assert built.rgb_layout.sha256 == hashlib.sha256(rgb.data).digest()
    assert built.depth_layout.sha256 == hashlib.sha256(depth.data).digest()


def test_numpy_camera_matrices_produce_native_float_calibration_snapshots():
    headers, rgb, depth, rgb_info, depth_info = inputs()
    for name in ("d", "k", "r", "p"):
        setattr(rgb_info, name, np.asarray(getattr(rgb_info, name), dtype=np.float64))
    built = build_frame(
        headers,
        rgb,
        depth,
        rgb_info,
        depth_info,
        calibration_epoch="cal",
        registration="registered",
        tf_epoch="tf",
        dependencies=("tf",),
        exclusion=bytes(80 * 80),
        static_sim_declared=True,
    )
    assert all(
        type(value) is float
        for name in ("d", "k", "r", "p")
        for value in getattr(built.rgb_info, name)
    )
    assert type(built.frame.calibration.fx) is float


def test_camera_matrices_reject_non_numeric_boolean_nonfinite_and_wrong_shape():
    import pytest

    for invalid in (True, np.bool_(True), "80.0", float("nan"), float("inf")):
        headers, rgb, depth, rgb_info, depth_info = inputs()
        rgb_info.k = (invalid, *rgb_info.k[1:])
        with pytest.raises(ValueError):
            build_frame(
                headers,
                rgb,
                depth,
                rgb_info,
                depth_info,
                calibration_epoch="cal",
                registration="registered",
                tf_epoch="tf",
                dependencies=("tf",),
                exclusion=bytes(80 * 80),
                static_sim_declared=True,
            )
    headers, rgb, depth, rgb_info, depth_info = inputs()
    rgb_info.k = rgb_info.k[:-1]
    with pytest.raises(ValueError):
        build_frame(
            headers,
            rgb,
            depth,
            rgb_info,
            depth_info,
            calibration_epoch="cal",
            registration="registered",
            tf_epoch="tf",
            dependencies=("tf",),
            exclusion=bytes(80 * 80),
            static_sim_declared=True,
        )


def test_numpy_roi_is_preserved_as_native_unsigned_integers():
    headers, rgb, depth, rgb_info, depth_info = inputs()
    rgb_info.roi = np.array([0, 0, 0, 0], dtype=np.uint32)
    built = build_frame(
        headers,
        rgb,
        depth,
        rgb_info,
        depth_info,
        calibration_epoch="cal",
        registration="registered",
        tf_epoch="tf",
        dependencies=("tf",),
        exclusion=bytes(80 * 80),
        static_sim_declared=True,
    )
    assert built.rgb_info.roi == (0, 0, 0, 0)
    assert all(type(value) is int for value in built.rgb_info.roi)


def test_camera_roi_rejects_boolean_float_string_and_out_of_range():
    import pytest

    for invalid in (False, np.bool_(False), 0.0, "0", -1, 2**32):
        headers, rgb, depth, rgb_info, depth_info = inputs()
        rgb_info.roi = (invalid, 0, 0, 0)
        with pytest.raises(ValueError):
            build_frame(
                headers,
                rgb,
                depth,
                rgb_info,
                depth_info,
                calibration_epoch="cal",
                registration="registered",
                tf_epoch="tf",
                dependencies=("tf",),
                exclusion=bytes(80 * 80),
                static_sim_declared=True,
            )
