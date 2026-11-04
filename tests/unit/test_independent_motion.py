"""Real RGB-D estimator fixtures; these do not qualify a live operating profile."""

from dataclasses import replace

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from senior_control.vision.independent_motion import (  # noqa: E402
    Calibration,
    Frame,
    IndependentMotion,
    Profile,
)
from senior_control.vision.raw_capture import RawCaptureHeader  # noqa: E402


def header(seq: int) -> RawCaptureHeader:
    stamp = 1_000_000_000 + seq * 100_000_000
    return RawCaptureHeader(
        stamp, stamp, 1.0 + seq * 0.1, "optical", "01", "02", "source", "boot", 1, seq, True, ""
    )


def scene(shift: int = 0):
    """A front-facing collection of separately textured patches at three depths."""
    rng = np.random.default_rng(7)
    image = np.zeros((240, 320), np.uint8)
    depth = np.zeros((240, 320), np.float32)
    for row in range(3):
        for col in range(4):
            x, y = 42 + col * 70 + shift, 42 + row * 70
            patch = rng.integers(0, 256, (45, 45), dtype=np.uint8)
            image[y - 22 : y + 23, x - 22 : x + 23] = patch
            depth[y - 22 : y + 23, x - 22 : x + 23] = 2 + ((row + col) % 3)
    return image, depth


def frame(seq: int, image, depth, **changes):
    h = header(seq)
    result = Frame(
        h,
        h,
        h,
        h,
        image,
        depth,
        Calibration(
            250.0,
            250.0,
            160.0,
            120.0,
            "cal",
            "reg",
            True,
            320,
            240,
            "mono8",
            "32FC1",
            True,
            "rectified",
            (0, 0, 320, 240),
            (1, 1),
        ),
        "tf",
        ("camera",),
        np.zeros(image.shape, np.uint8),
    )
    return replace(result, **changes)


def estimator():
    return IndependentMotion(
        Profile(
            "sim",
            "synthetic-test-only",
            True,
            "distributed non-repeating patches",
            "cal",
            0.4,
            0.4,
            2.0,
            0.1,
            0.1,
        )
    )


def test_first_frame_initializes_and_real_orb_identity_is_candidate():
    image, depth = scene()
    vo = estimator()
    assert (
        vo.offer(frame(1, image, depth), now_ros_ns=1_200_000_000, now_steady=1.2).reason
        == "initializing"
    )
    result = vo.offer(frame(2, image, depth), now_ros_ns=1_200_000_000, now_steady=1.2)
    assert result.reason == "candidate"
    assert np.allclose(result.camera_transform, np.eye(4), atol=1e-2)
    assert not result.hard_error_qualified


def test_repeated_frame_and_changed_calibration_revoke_pair():
    image, depth = scene()
    vo = estimator()
    first = frame(1, image, depth)
    vo.offer(first, now_ros_ns=1_200_000_000, now_steady=1.2)
    assert vo.offer(first, now_ros_ns=1_200_000_000, now_steady=1.2).reason != "candidate"
    changed = frame(
        2, image, depth, calibration=Calibration(250.0, 250.0, 160.0, 120.0, "changed", "reg", True)
    )
    assert vo.offer(changed, now_ros_ns=1_200_000_000, now_steady=1.2).reason != "candidate"


def test_missing_depth_and_unqualified_profile_never_produce_motion():
    image, depth = scene()
    vo = estimator()
    vo.offer(frame(1, image, depth), now_ros_ns=1_200_000_000, now_steady=1.2)
    assert (
        vo.offer(
            frame(2, image, np.zeros_like(depth)), now_ros_ns=1_200_000_000, now_steady=1.2
        ).camera_transform
        is None
    )
    vo = IndependentMotion(Profile("sim", "", False))
    assert (
        vo.offer(frame(1, image, depth), now_ros_ns=1_200_000_000, now_steady=1.2).camera_transform
        is None
    )


def test_camera_translation_and_body_transform_follow_rigid_oracle():
    from senior_control.vision.independent_motion import camera_to_body_transform

    first_image, first_depth = scene()
    # f=250, translation=0.048m gives exact pixel shifts at z=2,3,4.
    second_image = np.zeros_like(first_image)
    second_depth = np.zeros_like(first_depth)
    for row in range(3):
        for col in range(4):
            x, y = 42 + col * 70, 42 + row * 70
            z = 2 + ((row + col) % 3)
            dx = {2: 6, 3: 4, 4: 3}[z]
            second_image[y - 22 : y + 23, x - 22 + dx : x + 23 + dx] = first_image[
                y - 22 : y + 23, x - 22 : x + 23
            ]
            second_depth[y - 22 : y + 23, x - 22 + dx : x + 23 + dx] = z
    vo = estimator()
    vo.offer(frame(1, first_image, first_depth), now_ros_ns=1_200_000_000, now_steady=1.2)
    result = vo.offer(
        frame(2, second_image, second_depth), now_ros_ns=1_200_000_000, now_steady=1.2
    )
    assert result.reason == "candidate"
    camera = np.array(result.camera_transform)
    assert np.allclose(camera[:3, 3], [0.048, 0, 0], atol=0.018)
    extrinsic = np.array(
        [[0.0, -1.0, 0.0, 0.3], [1.0, 0.0, 0.0, 0.1], [0.0, 0.0, 1.0, 0.2], [0.0, 0.0, 0.0, 1.0]]
    )
    expected = extrinsic @ camera @ np.linalg.inv(extrinsic)
    assert np.allclose(camera_to_body_transform(camera, extrinsic, extrinsic), expected, atol=1e-9)
    assert np.allclose(expected[:3, 3], [0, 0.048, 0], atol=0.018)


def test_snapshot_and_invalid_times_or_tf_revoke():
    image, depth = scene()
    first = frame(1, image, depth)
    image[:] = 0
    depth[:] = 0
    assert first.image.any() and first.depth.any()
    with pytest.raises(ValueError):
        first.image.setflags(write=True)
    vo = estimator()
    assert vo.offer(first, now_ros_ns=1_000_000_000, now_steady=1.0).reason == "invalid_frame"
    image, depth = scene()
    vo.offer(frame(1, image, depth), now_ros_ns=1_200_000_000, now_steady=1.2)
    assert (
        vo.offer(
            frame(2, image, depth, tf_epoch="new"), now_ros_ns=1_200_000_000, now_steady=1.2
        ).reason
        == "pair_revoked"
    )


def test_flat_texture_planar_depth_and_exclusion_fail_closed():
    image, depth = scene()
    for second_image, second_depth, mask in [
        (np.zeros_like(image), depth, np.zeros_like(image)),
        (image, np.where(depth > 0, 2.0, 0.0).astype(np.float32), np.zeros_like(image)),
        (image, depth, np.full_like(image, 255)),
    ]:
        vo = estimator()
        # Both images must share the planar fixture for the degeneracy case.
        vo.offer(
            frame(1, image, second_depth, exclusion=mask), now_ros_ns=1_200_000_000, now_steady=1.2
        )
        assert (
            vo.offer(
                frame(2, second_image, second_depth, exclusion=mask),
                now_ros_ns=1_200_000_000,
                now_steady=1.2,
            ).camera_transform
            is None
        )


def test_rotation_about_optical_axis_uses_real_pnp():
    image, depth = scene()
    angle = 0.025
    matrix = cv2.getRotationMatrix2D((160.0, 120.0), -np.degrees(angle), 1.0)
    rotated = cv2.warpAffine(image, matrix, (320, 240), flags=cv2.INTER_NEAREST)
    rotated_depth = cv2.warpAffine(depth, matrix, (320, 240), flags=cv2.INTER_NEAREST)
    vo = estimator()
    vo.offer(frame(1, image, depth), now_ros_ns=1_200_000_000, now_steady=1.2)
    result = vo.offer(frame(2, rotated, rotated_depth), now_ros_ns=1_200_000_000, now_steady=1.2)
    assert result.reason == "candidate"
    transform = np.asarray(result.camera_transform)
    expected = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    assert np.allclose(transform[:2, :2], expected, atol=0.015)
    assert np.linalg.norm(transform[:3, 3]) < 0.04


def test_incorrect_second_depth_rejects_geometrically_inconsistent_pair():
    image, depth = scene()
    vo = estimator()
    vo.offer(frame(1, image, depth), now_ros_ns=1_200_000_000, now_steady=1.2)
    result = vo.offer(frame(2, image, depth * 1.3), now_ros_ns=1_200_000_000, now_steady=1.2)
    assert result.reason == "holdout_inconsistent"
    assert result.camera_transform is None


def test_invalid_registration_and_unrectified_calibration_rejected():
    image, depth = scene()
    c = Calibration(250.0, 250.0, 160.0, 120.0, "cal", "reg", rectified=False)
    vo = estimator()
    assert (
        vo.offer(
            frame(1, image, depth, calibration=c), now_ros_ns=1_200_000_000, now_steady=1.2
        ).reason
        == "invalid_frame"
    )


def test_frame_rejects_oversized_data_before_copying():
    image = np.zeros((481, 641), np.uint8)
    depth = np.zeros_like(image, dtype=np.float32)
    with pytest.raises(ValueError, match="frame_budget"):
        frame(1, image, depth)


def test_independent_foreground_masks_exclude_moving_patch_in_both_frames():
    image, depth = scene()
    moved = image.copy()
    moved[20:65, 20:65] = np.roll(moved[20:65, 20:65], 9, axis=1)
    mask = np.zeros_like(image)
    mask[10:75, 10:75] = 255
    vo = estimator()
    vo.offer(frame(1, image, depth, exclusion=mask), now_ros_ns=1_200_000_000, now_steady=1.2)
    result = vo.offer(
        frame(2, moved, depth, exclusion=mask), now_ros_ns=1_200_000_000, now_steady=1.2
    )
    assert result.reason == "candidate"
    assert np.allclose(result.camera_transform, np.eye(4), atol=0.02)


def test_missing_opencv_returns_unavailable(monkeypatch):
    import importlib

    original_import = importlib.import_module

    def unavailable(name, *args, **kwargs):
        if name == "cv2":
            raise ModuleNotFoundError("cv2")
        return original_import(name, *args, **kwargs)

    image, depth = scene()
    vo = estimator()
    vo.offer(frame(1, image, depth), now_ros_ns=1_200_000_000, now_steady=1.2)
    monkeypatch.setattr(importlib, "import_module", unavailable)
    result = vo.offer(frame(2, image, depth), now_ros_ns=1_200_000_000, now_steady=1.2)
    assert result.reason == "dependency_unavailable"
    assert result.camera_transform is None


def test_exclusion_mask_protects_descriptor_neighborhood():
    from senior_control.vision.independent_motion import _feature_mask

    excluded = np.zeros((100, 100), np.uint8)
    excluded[50, 50] = 255
    permitted = _feature_mask(excluded)
    assert permitted[50, 50] == 0
    assert permitted[50, 35] == 0
    assert permitted[50, 10] == 255


def test_malformed_capture_identity_is_rejected():
    image, depth = scene()
    valid = frame(1, image, depth)
    invalid_header = replace(valid.rgb_header, raw_gid="not-a-gid")
    invalid = replace(valid, rgb_header=invalid_header)
    assert (
        estimator().offer(invalid, now_ros_ns=1_200_000_000, now_steady=1.2).reason
        == "invalid_frame"
    )


def test_frame_rejects_nonpixel_dtype_before_snapshot():
    image, depth = scene()
    with pytest.raises(ValueError, match="frame_budget"):
        frame(1, image.astype(np.float64), depth)


def test_profile_requires_explicit_domain_and_matching_sensor_calibration():
    image, depth = scene()
    incomplete = Profile("sim", "study", True)
    assert (
        IndependentMotion(incomplete)
        .offer(frame(1, image, depth), now_ros_ns=1_200_000_000, now_steady=1.2)
        .reason
        == "profile_unavailable"
    )
    other = replace(estimator().profile, calibration_epoch="other")
    assert (
        IndependentMotion(other)
        .offer(frame(1, image, depth), now_ros_ns=1_200_000_000, now_steady=1.2)
        .reason
        == "profile_unavailable"
    )


def test_profile_rejects_excluded_fraction_above_declared_domain():
    image, depth = scene()
    mask = np.full_like(image, 255)
    result = estimator().offer(
        frame(1, image, depth, exclusion=mask), now_ros_ns=1_200_000_000, now_steady=1.2
    )
    assert result.reason == "profile_domain_exceeded"


def test_estimated_translation_speed_above_profile_is_unavailable():
    first_image, first_depth = scene()
    second_image = np.zeros_like(first_image)
    second_depth = np.zeros_like(first_depth)
    for row in range(3):
        for col in range(4):
            x, y = 42 + col * 70, 42 + row * 70
            z = 2 + ((row + col) % 3)
            dx = {2: 6, 3: 4, 4: 3}[z]
            second_image[y - 22 : y + 23, x - 22 + dx : x + 23 + dx] = first_image[
                y - 22 : y + 23, x - 22 : x + 23
            ]
            second_depth[y - 22 : y + 23, x - 22 + dx : x + 23 + dx] = z
    vo = IndependentMotion(replace(estimator().profile, max_speed_m_s=0.1))
    vo.offer(frame(1, first_image, first_depth), now_ros_ns=1_200_000_000, now_steady=1.2)
    result = vo.offer(
        frame(2, second_image, second_depth), now_ros_ns=1_200_000_000, now_steady=1.2
    )
    assert result.reason == "profile_domain_exceeded"
    assert result.camera_transform is None


@pytest.mark.parametrize(
    "change",
    [
        {"depth_encoding": "16UC1"},
        {"rgb_encoding": "rgb8"},
        {"roi": (1, 0, 319, 240)},
        {"binning": (2, 2)},
        {"distortion_model": "plumb_bob"},
        {"depth_is_optical_z": False},
        {"width": 640},
    ],
)
def test_unsupported_calibration_encoding_or_roi_rejected(change):
    image, depth = scene()
    valid = frame(1, image, depth)
    invalid = replace(valid, calibration=replace(valid.calibration, **change))
    assert (
        estimator().offer(invalid, now_ros_ns=1_200_000_000, now_steady=1.2).reason
        == "invalid_frame"
    )


def test_real_pnp_recovers_small_6dof_transform():
    image, depth = scene()
    k = np.array([[250.0, 0.0, 160.0], [0.0, 250.0, 120.0], [0.0, 0.0, 1.0]])
    rotation, _ = cv2.Rodrigues(np.array([0.012, -0.014, 0.018]))
    translation = np.array([0.025, -0.018, 0.02])
    output = np.zeros_like(image)
    output_depth = np.zeros_like(depth)
    yy, xx = np.indices(depth.shape)
    for row in range(3):
        for col in range(4):
            x, y = 42 + col * 70, 42 + row * 70
            z = 2 + (row + col) % 3
            patch_mask = np.zeros_like(image)
            patch_mask[y - 22 : y + 23, x - 22 : x + 23] = 255
            homography = (
                k @ (rotation + np.outer(translation, [0.0, 0.0, 1.0]) / z) @ np.linalg.inv(k)
            )
            warped = cv2.warpPerspective(image, homography, (320, 240), flags=cv2.INTER_NEAREST)
            mask = (
                cv2.warpPerspective(patch_mask, homography, (320, 240), flags=cv2.INTER_NEAREST) > 0
            )
            output[mask] = warped[mask]
            coordinates = np.linalg.inv(homography) @ np.array(
                [xx[mask], yy[mask], np.ones(mask.sum())]
            )
            coordinates /= coordinates[2]
            points = np.linalg.inv(k) @ coordinates * z
            output_depth[mask] = (rotation @ points + translation[:, None])[2]
    vo = estimator()
    vo.offer(frame(1, image, depth), now_ros_ns=1_200_000_000, now_steady=1.2)
    result = vo.offer(frame(2, output, output_depth), now_ros_ns=1_200_000_000, now_steady=1.2)
    assert result.reason == "candidate"
    actual = np.array(result.camera_transform)
    assert np.allclose(actual[:3, :3], rotation, atol=0.015)
    assert np.allclose(actual[:3, 3], translation, atol=0.025)


def test_dominant_unexcluded_foreground_with_static_background_rejects_mixed_fit():
    image, depth = scene()
    second = np.zeros_like(image)
    second_depth = np.zeros_like(depth)
    for row in range(3):
        for col in range(4):
            x, y = 42 + col * 70, 42 + row * 70
            z = 2 + (row + col) % 3
            dx = {2: 6, 3: 4, 4: 3}[z] if (row + col) % 3 else 0
            second[y - 22 : y + 23, x - 22 + dx : x + 23 + dx] = image[
                y - 22 : y + 23, x - 22 : x + 23
            ]
            second_depth[y - 22 : y + 23, x - 22 + dx : x + 23 + dx] = z
    vo = estimator()
    vo.offer(frame(1, image, depth), now_ros_ns=1_200_000_000, now_steady=1.2)
    result = vo.offer(frame(2, second, second_depth), now_ros_ns=1_200_000_000, now_steady=1.2)
    assert result.camera_transform is None


@pytest.mark.parametrize(
    "kind",
    [
        "future",
        "expired",
        "out_of_order",
        "source",
        "boot",
        "generation",
        "sync",
        "registration",
        "tf",
    ],
)
def test_source_time_and_epoch_changes_revoke_pair(kind):
    image, depth = scene()
    first = frame(1, image, depth)
    second = frame(2, image, depth)
    if kind == "future":
        now_ros, now_steady = 1_150_000_000, 1.2
    elif kind == "expired":
        now_ros, now_steady = 1_600_000_000, 1.6
    else:
        now_ros, now_steady = 1_200_000_000, 1.2
    if kind == "out_of_order":
        second = first
    if kind in {"source", "boot", "generation"}:
        change = {
            "source": {"source_epoch": "replacement"},
            "boot": {"bridge_boot_id": "replacement"},
            "generation": {"capture_generation": 2},
        }[kind]
        for field in ("rgb_header", "depth_header", "rgb_info_header", "depth_info_header"):
            second = replace(second, **{field: replace(getattr(second, field), **change)})
    if kind == "sync":
        second = replace(second, depth_header=replace(second.depth_header, stamp_ns=1_199_999_999))
    if kind == "registration":
        second = replace(second, calibration=replace(second.calibration, registration="new"))
    if kind == "tf":
        second = replace(second, tf_epoch="new")
    vo = estimator()
    vo.offer(first, now_ros_ns=1_200_000_000, now_steady=1.2)
    assert vo.offer(second, now_ros_ns=now_ros, now_steady=now_steady).camera_transform is None
    assert (
        vo.offer(frame(3, image, depth), now_ros_ns=1_300_000_000, now_steady=1.3).reason
        != "candidate"
    )


def test_external_source_revocation_clears_retained_first_frame():
    image, depth = scene()
    vo = estimator()
    vo.offer(frame(1, image, depth), now_ros_ns=1_200_000_000, now_steady=1.2)
    vo.invalidate()
    assert (
        vo.offer(frame(2, image, depth), now_ros_ns=1_200_000_000, now_steady=1.2).reason
        == "initializing"
    )


def test_repeated_texture_and_single_patch_distribution_rejected():
    image, depth = scene()
    repeated = np.zeros_like(image)
    for row in range(3):
        for col in range(4):
            x, y = 42 + col * 70, 42 + row * 70
            repeated[y - 22 : y + 23, x - 22 : x + 23] = image[20:65, 20:65]
    one = np.zeros_like(image)
    one[20:65, 20:65] = image[20:65, 20:65]
    for sample in (repeated, one):
        vo = estimator()
        vo.offer(frame(1, sample, depth), now_ros_ns=1_200_000_000, now_steady=1.2)
        assert (
            vo.offer(
                frame(2, sample, depth), now_ros_ns=1_200_000_000, now_steady=1.2
            ).camera_transform
            is None
        )


def test_body_transform_full_extrinsics_matches_point_by_point_oracle():
    from senior_control.vision.independent_motion import camera_to_body_transform

    r1, _ = cv2.Rodrigues(np.array([0.2, -0.1, 0.3]))
    r2, _ = cv2.Rodrigues(np.array([-0.15, 0.25, 0.05]))
    rc, _ = cv2.Rodrigues(np.array([0.04, 0.03, -0.02]))
    t1, t2, tc = (
        np.array([0.3, -0.2, 0.1]),
        np.array([-0.1, 0.4, 0.2]),
        np.array([0.05, -0.03, 0.08]),
    )
    matrices = []
    for r, t in ((rc, tc), (r1, t1), (r2, t2)):
        value = np.eye(4)
        value[:3, :3], value[:3, 3] = r, t
        matrices.append(value)
    point_b1 = np.array([1.2, -0.4, 2.1])
    point_c1 = r1.T @ (point_b1 - t1)
    point_c2 = rc @ point_c1 + tc
    expected_b2 = r2 @ point_c2 + t2
    output = np.array(camera_to_body_transform(*matrices)) @ np.r_[point_b1, 1.0]
    assert np.allclose(output[:3], expected_b2, atol=1e-10)


def test_replay_after_out_of_order_and_external_invalidation_never_revives_pair():
    image, depth = scene()
    one, two = frame(1, image, depth), frame(2, image, depth)
    vo = estimator()

    def offer(item):
        return vo.offer(item, now_ros_ns=1_200_000_000, now_steady=1.2)

    assert offer(one).reason == "initializing"
    assert offer(two).reason == "candidate"
    for item in (one, one, two):
        assert offer(item).camera_transform is None
    vo.invalidate()
    for item in (one, two):
        assert offer(item).camera_transform is None
    three = frame(3, image, depth)
    four = frame(4, image, depth)
    assert vo.offer(three, now_ros_ns=1_400_000_000, now_steady=1.4).reason == "initializing"
    assert vo.offer(four, now_ros_ns=1_400_000_000, now_steady=1.4).reason == "candidate"


def test_first_receipts_regressing_in_either_clock_do_not_form_pair():
    image, depth = scene()
    for change in ({"received_ros_ns": 1_050_000_000}, {"received_steady": 1.05}):
        vo = estimator()
        vo.offer(frame(1, image, depth), now_ros_ns=1_200_000_000, now_steady=1.2)
        second = frame(2, image, depth)
        for field in ("rgb_header", "depth_header", "rgb_info_header", "depth_info_header"):
            second = replace(second, **{field: replace(getattr(second, field), **change)})
        assert vo.offer(second, now_ros_ns=1_200_000_000, now_steady=1.2).camera_transform is None


def generation_frame(index, generation, image, depth):
    result = frame(index, image, depth)
    for field in ("rgb_header", "depth_header", "rgb_info_header", "depth_info_header"):
        result = replace(
            result, **{field: replace(getattr(result, field), capture_generation=generation)}
        )
    return result


def test_new_generation_recovers_and_retired_lineage_cannot_return():
    image, depth = scene()
    vo = estimator()

    def now(n):
        return {"now_ros_ns": 1_000_000_000 + n * 100_000_000, "now_steady": 1.0 + n * 0.1}

    assert vo.offer(generation_frame(1, 1, image, depth), **now(1)).reason == "initializing"
    assert vo.offer(generation_frame(2, 1, image, depth), **now(2)).reason == "candidate"
    assert vo.offer(generation_frame(3, 2, image, depth), **now(3)).reason == "initializing"
    assert vo.offer(generation_frame(4, 2, image, depth), **now(4)).reason == "candidate"
    for index in (5, 6):
        assert (
            vo.offer(generation_frame(index, 1, image, depth), **now(index)).camera_transform
            is None
        )
    assert vo.offer(generation_frame(7, 2, image, depth), **now(7)).reason == "initializing"
    assert vo.offer(generation_frame(8, 2, image, depth), **now(8)).reason == "candidate"


def test_retired_source_capacity_exhaustion_fails_closed():
    image, depth = scene()
    vo = estimator()
    for index in range(1, 67):
        current = generation_frame(index, index, image, depth)
        result = vo.offer(
            current,
            now_ros_ns=current.rgb_header.stamp_ns,
            now_steady=current.rgb_header.received_steady,
        )
        assert result.reason == ("initializing" if index <= 65 else "source_frontier_invalid")
    current = generation_frame(67, 67, image, depth)
    assert (
        vo.offer(
            current,
            now_ros_ns=current.rgb_header.stamp_ns,
            now_steady=current.rgb_header.received_steady,
        ).camera_transform
        is None
    )


def test_pose_quality_rejects_near_degenerate_nonplanar_rays():
    from senior_control.vision.independent_motion import _pose_quality

    rng = np.random.default_rng(12)
    points = np.column_stack(
        (rng.uniform(-0.05, 0.05, 80), rng.uniform(-0.05, 0.05, 80), rng.uniform(90.0, 110.0, 80))
    )
    matrix = np.array([[250.0, 0.0, 160.0], [0.0, 250.0, 120.0], [0.0, 0.0, 1.0]])
    rvec, tvec = np.zeros((3, 1)), np.zeros((3, 1))
    pixels = cv2.projectPoints(points, rvec, tvec, matrix, None)[0].reshape(-1, 2)
    assert (
        _pose_quality(points, pixels, np.arange(80) % 2, matrix, rvec, tvec)
        == "ill_conditioned_pose"
    )


def test_pose_quality_rejects_inconsistent_spatial_group_solutions():
    from senior_control.vision.independent_motion import _pose_quality

    rng = np.random.default_rng(13)
    points = np.column_stack(
        (rng.uniform(-1.5, 1.5, 120), rng.uniform(-1.0, 1.0, 120), rng.uniform(2.0, 5.0, 120))
    )
    matrix = np.array([[250.0, 0.0, 160.0], [0.0, 250.0, 120.0], [0.0, 0.0, 1.0]])
    rvec, tvec = np.zeros((3, 1)), np.zeros((3, 1))
    groups = np.arange(120) % 2
    pixels = cv2.projectPoints(points, rvec, tvec, matrix, None)[0].reshape(-1, 2)
    pixels[groups == 1] += [5.0, 0.0]
    assert _pose_quality(points, pixels, groups, matrix, rvec, tvec) == "unstable_pose"


def test_retired_boot_cannot_return_with_forged_new_source_epoch():
    image, depth = scene()
    vo = estimator()

    def source(index, boot, bridge, epoch, raw):
        value = frame(index, image, depth)
        for field in ("rgb_header", "depth_header", "rgb_info_header", "depth_info_header"):
            value = replace(
                value,
                **{
                    field: replace(
                        getattr(value, field),
                        bridge_boot_id=boot,
                        bridge_gid=bridge,
                        source_epoch=epoch,
                        raw_gid=raw,
                    )
                },
            )
        return value

    def offer(index, boot, bridge, epoch, raw):
        value = source(index, boot, bridge, epoch, raw)
        return vo.offer(
            value, now_ros_ns=value.rgb_header.stamp_ns, now_steady=value.rgb_header.received_steady
        )

    assert offer(1, "boot", "02", "source", "01").reason == "initializing"
    assert offer(2, "boot2", "03", "new", "04").reason == "initializing"
    assert offer(3, "boot2", "03", "new", "04").reason == "candidate"
    assert offer(4, "boot", "02", "forged", "05").camera_transform is None
    assert offer(5, "boot", "02", "forged", "05").camera_transform is None
    assert offer(6, "boot2", "03", "new", "04").reason == "initializing"
    assert offer(7, "boot2", "03", "new", "04").reason == "candidate"


def test_retired_raw_gid_cannot_return_with_changed_epoch():
    image, depth = scene()
    vo = estimator()

    def offer(index, gid, epoch):
        value = frame(index, image, depth)
        for field in ("rgb_header", "depth_header", "rgb_info_header", "depth_info_header"):
            value = replace(
                value, **{field: replace(getattr(value, field), raw_gid=gid, source_epoch=epoch)}
            )
        return vo.offer(
            value, now_ros_ns=value.rgb_header.stamp_ns, now_steady=value.rgb_header.received_steady
        )

    assert offer(1, "01", "source").reason == "initializing"
    assert offer(2, "04", "replacement").reason == "initializing"
    assert offer(3, "04", "replacement").reason == "candidate"
    assert offer(4, "01", "forged").camera_transform is None
    assert offer(5, "01", "forged").camera_transform is None


def test_replacement_generation_accepts_sequence_reset_and_new_pair():
    image, depth = scene()
    vo = estimator()
    for index in (1, 2, 3, 4):
        generation = 1 if index < 3 else 2
        sequence = index if index < 3 else index - 2
        value = generation_frame(index, generation, image, depth)
        for field in ("rgb_header", "depth_header", "rgb_info_header", "depth_info_header"):
            value = replace(value, **{field: replace(getattr(value, field), sequence=sequence)})
        result = vo.offer(
            value, now_ros_ns=value.rgb_header.stamp_ns, now_steady=value.rgb_header.received_steady
        )
        assert result.reason == ("initializing" if sequence == 1 else "candidate")


def test_mutable_dependencies_are_rejected_instead_of_aliasing_caller():
    image, depth = scene()
    mutable = ["camera"]
    with pytest.raises(ValueError, match="dependencies"):
        replace(frame(1, image, depth), dependencies=mutable)
    mutable[0] = "mutated_after_capture"


@pytest.mark.parametrize("dependencies", [("x",) * 65, ("",), ("x" * 257,), (["x"],), iter(["x"])])
def test_dependency_identities_have_immutable_bounded_shape(dependencies):
    image, depth = scene()
    with pytest.raises(ValueError, match="dependencies"):
        replace(frame(1, image, depth), dependencies=dependencies)


def test_calibration_container_metadata_cannot_alias_mutable_values():
    image, depth = scene()
    calibration = frame(1, image, depth).calibration
    with pytest.raises(ValueError, match="calibration_metadata"):
        replace(calibration, roi=[0, 0, 320, 240])
    with pytest.raises(ValueError, match="calibration_metadata"):
        replace(calibration, binning=(1, [1]))


def test_output_constructor_rejects_mutable_transform_or_pair():
    from senior_control.vision.independent_motion import Result

    with pytest.raises(ValueError, match="result_metadata"):
        Result("candidate", camera_transform=[[1.0, 0.0, 0.0, 0.0]] * 4)
    with pytest.raises(ValueError, match="result_metadata"):
        Result("candidate", pair=[])


def test_metadata_scalars_cannot_hide_mutable_objects():
    image, depth = scene()
    original = frame(1, image, depth)
    with pytest.raises(ValueError, match="calibration_metadata"):
        replace(original.calibration, epoch=["cal"])
    with pytest.raises(ValueError, match="frame_metadata"):
        replace(original, tf_epoch=["tf"])
    with pytest.raises(ValueError, match="frame_metadata"):
        replace(original, rgb_header=replace(original.rgb_header, reason=[]))
    with pytest.raises(ValueError, match="profile_metadata"):
        replace(estimator().profile, texture_domain=["texture"])


def test_unusable_new_source_still_retires_old_source_and_new_source_recovers():
    image, depth = scene()
    vo = estimator()
    first = frame(1, image, depth)
    assert vo.offer(first, now_ros_ns=1_100_000_000, now_steady=1.1).reason == "initializing"

    def new_source(index, mask=None):
        value = frame(index, image, depth, exclusion=np.zeros_like(image) if mask is None else mask)
        for name in ("rgb_header", "depth_header", "rgb_info_header", "depth_info_header"):
            value = replace(
                value, **{name: replace(getattr(value, name), raw_gid="03", source_epoch="new")}
            )
        return value

    blocked = new_source(2, np.full_like(image, 255))
    assert vo.offer(blocked, now_ros_ns=1_200_000_000, now_steady=1.2).camera_transform is None
    for index in (2, 3):
        old = frame(index, image, depth)
        assert (
            vo.offer(
                old, now_ros_ns=old.rgb_header.stamp_ns, now_steady=old.rgb_header.received_steady
            ).camera_transform
            is None
        )
    for index, expected in ((4, "initializing"), (5, "candidate")):
        current = new_source(index)
        assert (
            vo.offer(
                current,
                now_ros_ns=current.rgb_header.stamp_ns,
                now_steady=current.rgb_header.received_steady,
            ).reason
            == expected
        )


def test_untrusted_new_header_cannot_retire_current_source():
    image, depth = scene()
    vo = estimator()
    vo.offer(frame(1, image, depth), now_ros_ns=1_100_000_000, now_steady=1.1)
    bad = frame(2, image, depth)
    for name in ("rgb_header", "depth_header", "rgb_info_header", "depth_info_header"):
        bad = replace(
            bad, **{name: replace(getattr(bad, name), raw_gid="not-gid", source_epoch="new")}
        )
    assert vo.offer(bad, now_ros_ns=1_200_000_000, now_steady=1.2).camera_transform is None
    assert (
        vo.offer(frame(2, image, depth), now_ros_ns=1_200_000_000, now_steady=1.2).reason
        == "initializing"
    )
    assert (
        vo.offer(frame(3, image, depth), now_ros_ns=1_300_000_000, now_steady=1.3).reason
        == "candidate"
    )


def test_extreme_finite_principal_point_returns_unavailable_without_numerical_exception():
    image, depth = scene()
    original = frame(1, image, depth)
    calibration = replace(original.calibration, cx=1e308)
    vo = estimator()
    vo.offer(
        frame(1, image, depth, calibration=calibration), now_ros_ns=1_200_000_000, now_steady=1.2
    )
    result = vo.offer(
        frame(2, image, depth, calibration=calibration), now_ros_ns=1_200_000_000, now_steady=1.2
    )
    assert result.camera_transform is None
    assert result.reason == "invalid_frame"


@pytest.mark.parametrize("unusable", ["calibration", "depth_holes", "weak_texture", "profile"])
def test_valid_new_source_retires_old_independent_of_fit_qualification(unusable):
    image, depth = scene()
    vo = estimator()
    vo.offer(frame(1, image, depth), now_ros_ns=1_100_000_000, now_steady=1.1)
    new = frame(2, image, depth)
    for name in ("rgb_header", "depth_header", "rgb_info_header", "depth_info_header"):
        new = replace(new, **{name: replace(getattr(new, name), raw_gid="03", source_epoch="new")})
    if unusable == "calibration":
        new = replace(new, calibration=replace(new.calibration, cx=1e308))
    elif unusable == "depth_holes":
        new = replace(new, depth=np.zeros_like(depth))
    elif unusable == "weak_texture":
        new = replace(new, image=np.zeros_like(image))
    else:
        new = replace(new, calibration=replace(new.calibration, epoch="other"))
    vo.offer(new, now_ros_ns=1_200_000_000, now_steady=1.2)
    for index in (2, 3):
        old = frame(index, image, depth)
        assert (
            vo.offer(
                old, now_ros_ns=old.rgb_header.stamp_ns, now_steady=old.rgb_header.received_steady
            ).camera_transform
            is None
        )


@pytest.mark.parametrize("failure", ["svd", "opencv", "nonfinite_solver"])
def test_numerical_solver_failure_is_explicit_unavailable(monkeypatch, failure):
    image, depth = scene()
    vo = estimator()
    vo.offer(frame(1, image, depth), now_ros_ns=1_200_000_000, now_steady=1.2)
    if failure == "svd":

        def fail(*args, **kwargs):
            raise np.linalg.LinAlgError("no convergence")

        monkeypatch.setattr(np.linalg, "svd", fail)
    elif failure == "opencv":

        def fail(*args, **kwargs):
            raise cv2.error("solver failure")

        monkeypatch.setattr(cv2, "solvePnPRansac", fail)
    else:

        def fail(*args, **kwargs):
            return True, np.full((3, 1), np.nan), np.zeros((3, 1)), np.arange(40).reshape(-1, 1)

        monkeypatch.setattr(cv2, "solvePnPRansac", fail)
    result = vo.offer(frame(2, image, depth), now_ros_ns=1_200_000_000, now_steady=1.2)
    assert result.reason == "numerical_failure"
    assert result.camera_transform is None


def test_extreme_integer_intrinsics_are_rejected_without_overflow():
    image, depth = scene()
    original = frame(1, image, depth)
    bad = replace(original.calibration, fx=10**1000)
    result = estimator().offer(
        frame(1, image, depth, calibration=bad), now_ros_ns=1_200_000_000, now_steady=1.2
    )
    assert result.reason == "invalid_frame"


def test_body_transform_overflow_is_rejected():
    from senior_control.vision.independent_motion import camera_to_body_transform

    first = np.eye(4)
    second = np.eye(4)
    first[0, 3] = -1e308
    second[0, 3] = 1e308
    with pytest.raises(ValueError, match="invalid_rigid_transform"):
        camera_to_body_transform(np.eye(4), first, second)


def test_extreme_receipt_and_profile_numbers_do_not_escape_as_overflow():
    image, depth = scene()
    original = frame(1, image, depth)
    huge = 10**1000
    malformed = replace(original, rgb_header=replace(original.rgb_header, received_steady=huge))
    assert (
        estimator().offer(malformed, now_ros_ns=1_200_000_000, now_steady=1.2).reason
        == "invalid_frame"
    )
    profile = replace(estimator().profile, max_speed_m_s=huge)
    assert (
        IndependentMotion(profile).offer(original, now_ros_ns=1_200_000_000, now_steady=1.2).reason
        == "profile_unavailable"
    )


def test_source_replacement_with_shape_mismatch_retires_previous():
    image, depth = scene()
    vo = estimator()
    vo.offer(frame(1, image, depth), now_ros_ns=1_100_000_000, now_steady=1.1)
    value = frame(2, image, depth)
    for name in ("rgb_header", "depth_header", "rgb_info_header", "depth_info_header"):
        value = replace(
            value, **{name: replace(getattr(value, name), raw_gid="03", source_epoch="new")}
        )
    value = replace(value, depth=np.zeros((100, 100), np.float32))
    assert vo.offer(value, now_ros_ns=1_200_000_000, now_steady=1.2).reason == "invalid_frame"
    for index in (2, 3):
        old = frame(index, image, depth)
        assert (
            vo.offer(
                old, now_ros_ns=old.rgb_header.stamp_ns, now_steady=old.rgb_header.received_steady
            ).camera_transform
            is None
        )


def test_header_only_observation_retires_source_and_consumes_header():
    image, depth = scene()
    vo = estimator()
    vo.offer(frame(1, image, depth), now_ros_ns=1_100_000_000, now_steady=1.1)
    new = frame(2, image, depth)
    names = ("rgb_header", "depth_header", "rgb_info_header", "depth_info_header")
    for name in names:
        new = replace(new, **{name: replace(getattr(new, name), raw_gid="03", source_epoch="new")})
    headers = tuple(getattr(new, name) for name in names)
    assert vo.observe_unusable_headers(headers, now_ros_ns=1_200_000_000, now_steady=1.2)
    assert vo.offer(new, now_ros_ns=1_200_000_000, now_steady=1.2).camera_transform is None
    for index in (2, 3):
        old = frame(index, image, depth)
        assert (
            vo.offer(
                old, now_ros_ns=old.rgb_header.stamp_ns, now_steady=old.rgb_header.received_steady
            ).camera_transform
            is None
        )
    for index, expected in ((4, "initializing"), (5, "candidate")):
        value = frame(index, image, depth)
        for name in names:
            value = replace(
                value, **{name: replace(getattr(value, name), raw_gid="03", source_epoch="new")}
            )
        assert (
            vo.offer(
                value,
                now_ros_ns=value.rgb_header.stamp_ns,
                now_steady=value.rgb_header.received_steady,
            ).reason
            == expected
        )


def test_header_only_invalid_capture_cannot_retire_current():
    image, depth = scene()
    vo = estimator()
    first = frame(1, image, depth)
    vo.offer(first, now_ros_ns=1_100_000_000, now_steady=1.1)
    other = frame(2, image, depth)
    invalid = replace(other.rgb_header, raw_gid="invalid", source_epoch="new")
    assert not vo.observe_unusable_headers((invalid,) * 4, now_ros_ns=1_200_000_000, now_steady=1.2)
    assert vo.offer(other, now_ros_ns=1_200_000_000, now_steady=1.2).reason == "initializing"
    assert (
        vo.offer(frame(3, image, depth), now_ros_ns=1_300_000_000, now_steady=1.3).reason
        == "candidate"
    )
