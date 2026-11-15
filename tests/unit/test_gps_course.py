from senior_control.localization.gps_course import CourseFix, GPSCourseObserver
from senior_control.motion.estimator import Origin


def motion(o, start, end, vx=0.2, wz=0):
    for i in range(round(start * 10), round(end * 10) + 1):
        o.record_motion(stamp_ros=i / 10, wheel_vx=vx, imu_wz=wz, generation=1)


def fix(i, x, *, sigma=0.01, vx=0.2):
    return CourseFix(x, 0, sigma, vx, 0, 0.01, Origin("gnss", "boot", i, i, i, i))


def test_precision_and_baseline_are_required_without_blocking_other_motion():
    o = GPSCourseObserver()
    assert o.observe(fix(1, 0, sigma=None), wheel_vx=0.2, imu_wz=0, now_ros=1, now_steady=1) is None
    motion(o, 1, 2)
    assert o.observe(fix(2, 0), wheel_vx=0.2, imu_wz=0, now_ros=2, now_steady=2) is None
    motion(o, 2, 3)
    assert o.observe(fix(3, 0.15), wheel_vx=0.2, imu_wz=0, now_ros=3, now_steady=3) is None
    motion(o, 3, 4)
    result = o.observe(fix(4, 0.4), wheel_vx=0.2, imu_wz=0, now_ros=4, now_steady=4)
    assert result.yaw_rad == 0
    assert result.deadline_steady == 5


def test_reverse_course_is_vehicle_heading_and_turn_resets():
    o = GPSCourseObserver()
    motion(o, 1, 1, vx=-0.2)
    assert o.observe(fix(1, 0, vx=-0.2), wheel_vx=-0.2, imu_wz=0, now_ros=1, now_steady=1) is None
    motion(o, 1, 3, vx=-0.2)
    result = o.observe(fix(3, -0.4, vx=-0.2), wheel_vx=-0.2, imu_wz=0, now_ros=3, now_steady=3)
    assert abs(result.yaw_rad) < 1e-9
    assert o.observe(fix(4, -0.6), wheel_vx=-0.2, imu_wz=0.2, now_ros=4, now_steady=4) is None


def test_bad_precision_slip_replay_and_expired_sample_are_unready():
    o = GPSCourseObserver()
    for i in range(1, 5):
        assert (
            o.observe(fix(i, i * 0.05, sigma=1), wheel_vx=0.05, imu_wz=0, now_ros=i, now_steady=i)
            is None
        )
    assert o.observe(fix(4, 20), wheel_vx=0.05, imu_wz=0, now_ros=4, now_steady=4) is None
    assert o.observe(fix(5, 20), wheel_vx=0.05, imu_wz=0, now_ros=6, now_steady=6) is None


def test_explicit_enu_adapter_initializes_without_heading_or_tf(monkeypatch):
    from types import SimpleNamespace as N

    from senior_robot.optional_motion_inputs import OptionalMotionInputs

    from senior_control.runtime.diagnostics import RuntimeDiagnostics

    adapter = object.__new__(OptionalMotionInputs)
    adapter._course_frame, adapter._gps_family = "enu", "gnss"
    adapter._allow_topology = False
    observer = GPSCourseObserver()
    runtime = N(
        gps_course=observer,
        gps_heading=None,
        wheel_linear=lambda _, **kw: (0.2, 10.0),
        latest=N(
            independent_axes=(None, N(observed=True, sources=("imu",), deadline_steady=10, value=0))
        ),
        diagnostics=RuntimeDiagnostics(lambda *_: None),
    )
    adapter.runtime = runtime
    covariance = [0.0] * 36
    covariance[0] = covariance[7] = 0.0001
    for i, x in ((1, 0), (3, 0.4)):
        motion(observer, 1, i)
        adapter.node = N(get_clock=lambda i=i: N(now=lambda: N(nanoseconds=i * 1e9)))
        monkeypatch.setattr("senior_robot.optional_motion_inputs.time.monotonic", lambda i=i: i)
        message = N(
            header=N(stamp=N(sec=i, nanosec=0), frame_id="enu"),
            child_frame_id="enu",
            pose=N(pose=N(position=N(x=x, y=0)), covariance=covariance),
            twist=N(twist=N(linear=N(x=0.2, y=0)), covariance=covariance),
        )
        adapter.gps(
            message,
            dict(
                received_timestamp=i * 1e9,
                native_received_steady=i,
                receipt_sequence=i,
                receipt_generation=1,
            ),
            N(sample_gid=b"g"),
        )
    assert runtime.gps_heading.yaw_rad == 0


def test_continuous_turn_cannot_be_hidden_by_zero_yaw_rate_at_gps_capture():
    o = GPSCourseObserver()
    motion(o, 1, 1)
    assert o.observe(fix(1, 0), wheel_vx=0.2, imu_wz=0, now_ros=1, now_steady=1) is None
    motion(o, 1.1, 2, wz=0.2)
    motion(o, 2.1, 3)
    assert o.observe(fix(3, 0.4), wheel_vx=0.2, imu_wz=0, now_ros=3, now_steady=3) is None


def test_position_jumps_inconsistent_with_wheel_are_not_heading():
    o = GPSCourseObserver()
    motion(o, 1, 1)
    o.observe(fix(1, 0), wheel_vx=0.2, imu_wz=0, now_ros=1, now_steady=1)
    motion(o, 1.1, 3)
    assert o.observe(fix(3, 4), wheel_vx=0.2, imu_wz=0, now_ros=3, now_steady=3) is None
