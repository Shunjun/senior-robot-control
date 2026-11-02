import numpy as np

from senior_control.vision.rgbd_body_motion import body_pose_increment


def test_body_increment_inverts_rotation_and_translation_together():
    static_shift = np.array(
        [[0.0, -1.0, 0.0, 1.0], [1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
    )
    actual = np.asarray(body_pose_increment(static_shift, np.eye(4), np.eye(4)))
    expected = np.array(
        [[0.0, 1.0, 0.0, 0.0], [-1.0, 0.0, 0.0, 1.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
    )
    assert np.allclose(actual, expected)


def test_planar_twist_uses_se2_log_and_rejects_unvalidated_tilt():
    import pytest

    from senior_control.vision.rgbd_body_motion import planar_body_twist

    delta = np.array(
        [[0.0, -1.0, 0.0, 1.0], [1.0, 0.0, 0.0, 1.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
    )
    twist = planar_body_twist(delta, dt=1.0, max_tilt_rad=0.01, max_vertical_m=0.01)
    assert np.allclose(twist, (np.pi / 2, 0.0, np.pi / 2))
    delta[2, 3] = 0.1
    with pytest.raises(ValueError):
        planar_body_twist(delta, dt=1.0, max_tilt_rad=0.01, max_vertical_m=0.01)


def test_lever_arm_and_two_extrinsics_match_full_matrix_oracle():
    theta = 0.3
    c, s = np.cos(theta), np.sin(theta)
    camera_motion = np.array(
        [[c, -s, 0.0, 0.2], [s, c, 0.0, -0.1], [0.0, 0.0, 1.0, 0.03], [0.0, 0.0, 0.0, 1.0]]
    )
    first = np.eye(4)
    first[:3, 3] = (0.5, -0.2, 0.8)
    second = np.eye(4)
    second[:3, 3] = (0.45, -0.1, 0.75)
    oracle = np.linalg.inv(second @ camera_motion @ np.linalg.inv(first))
    actual = body_pose_increment(camera_motion, first, second)
    assert np.allclose(actual, oracle, atol=1e-12)


def test_nonfinite_inverse_result_is_rejected():
    import pytest

    extreme = np.eye(4)
    angle = np.pi / 4
    extreme[:2, :2] = [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
    extreme[:2, 3] = [1.7e308, 1.7e308]
    with pytest.raises(ValueError):
        body_pose_increment(extreme, np.eye(4), np.eye(4))
