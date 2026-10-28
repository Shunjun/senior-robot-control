from senior_control.models import Pose2D, Velocity
from senior_control.safety.motion_estimator import (
    ImuYaw,
    MotionEstimator,
    RelativeMotion,
    WheelOdom,
    body_displacement,
    degraded_reverse_gate,
    supports_degraded_reverse,
)


def estimator() -> MotionEstimator:
    return MotionEstimator(max_skew=0.05, min_quality=0.5, max_yaw_error=0.05, max_covariance=1.0)


def sample(
    *,
    source: str = "scan_match",
    degenerate: bool = False,
    loop_closure: bool = False,
    dx: float = 0.25,
    wheel_dx: float = 1.0,
    now: float = 10.0,
    covariance: tuple[float, float, float] = (0.25, 0.25, 0.25),
    yaw: float = 0.0,
):
    scan = RelativeMotion(
        Pose2D(dx, 0.0, yaw),
        covariance,
        1.0,
        degenerate,
        now,
        source,
        loop_closure,
        0.5,
    )
    wheel = WheelOdom(Pose2D(wheel_dx, 0.0, 0.0), now, 0.5)
    imu = ImuYaw(0.0, now, 0.5)
    return wheel, scan, imu, None, now


def test_scan_degeneracy_disables_degraded_reverse():
    gate = estimator()
    degenerate = gate.update(*sample(degenerate=True))
    decision = degraded_reverse_gate(degenerate)
    assert not supports_degraded_reverse(degenerate)
    assert not decision.allowed
    assert decision.velocity == Velocity(0.0, 0.0)
    for source in ("wheel", "ground_truth"):
        disguised = gate.update(*sample(source=source))
        assert not supports_degraded_reverse(disguised)
        assert degraded_reverse_gate(disguised).velocity == Velocity(0.0, 0.0)
    wheel, _scan, imu, visual, now = sample()
    missing = gate.update(wheel, None, imu, visual, now)
    assert not supports_degraded_reverse(missing)
    qualified = gate.update(*sample())
    assert supports_degraded_reverse(qualified)
    assert qualified.pose_delta == Pose2D(0.25, 0.0, 0.0)
    assert qualified.velocity == Velocity(0.5, 0.0)
    assert body_displacement(qualified) == 0.25
    assert qualified.covariance == (0.25, 0.25, 0.25)


def test_nan_imu_yaw_is_not_independent():
    wheel, scan, _imu, visual, now = sample(yaw=0.3)
    estimate = estimator().update(wheel, scan, ImuYaw(float("nan"), now, 0.5), visual, now)
    assert estimate.quality == 0.0
    assert not supports_degraded_reverse(estimate)


def test_source_must_be_scan_match_and_covariance_within_cap():
    gate = MotionEstimator(max_skew=0.05, min_quality=0.5, max_yaw_error=0.05, max_covariance=1.0)
    for source in ("wheel_odom", "odom", "ekf"):
        disguised = gate.update(*sample(source=source))
        assert not supports_degraded_reverse(disguised)
    loose = gate.update(*sample(covariance=(1.0e6, 1.0e6, 1.0e6)))
    assert not supports_degraded_reverse(loose)


def test_unset_covariance_cap_fails_closed():
    gate = MotionEstimator(max_skew=0.05, min_quality=0.5, max_yaw_error=0.05)
    estimate = gate.update(*sample())
    assert estimate.quality == 0.0
    assert not supports_degraded_reverse(estimate)


def test_loop_jump_not_motion():
    estimate = estimator().update(*sample(dx=4.0, wheel_dx=1.0, loop_closure=True))
    assert estimate.quality == 0.0
    assert estimate.pose_delta == Pose2D(0.0, 0.0, 0.0)
    assert estimate.velocity == Velocity(0.0, 0.0)
    assert body_displacement(estimate) == 0.0
