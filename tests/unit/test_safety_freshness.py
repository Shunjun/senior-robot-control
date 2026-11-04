from senior_control.safety.freshness import SafetyFreshness


def test_scan_and_measurement_expire_on_steady_clock_or_ros_rollback():
    state = SafetyFreshness(0.5)
    assert state.scan(10.0, 100.0, 10.0)
    assert state.measured(10.0, 100.0, 10.0)
    assert state.usable(10.1, 100.1)
    assert not state.usable(10.1, 100.6)
    assert state.scan(10.2, 100.7, 10.2)
    assert state.measured(10.2, 100.7, 10.2)
    assert not state.usable(9.0, 100.8)
    assert not state.usable(10.2, 100.9)


def test_old_or_out_of_order_stamps_are_rejected():
    state = SafetyFreshness(0.5)
    assert not state.scan(1.0, 5.0, 2.0)
    assert state.scan(2.0, 5.1, 2.0)
    assert not state.scan(1.9, 5.2, 2.0)
    assert not state.measured(1.0, 5.2, 2.0)
