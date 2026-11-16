from senior_control.safety.freshness import SafetyFreshness


def test_original_measurement_expiry_and_true_monotonic_regression():
    state = SafetyFreshness(0.5)
    assert state.scan(100, 100.1, 100.1)
    assert state.usable(100.49)
    assert not state.usable(100.5)
    assert state.scan(100.6, 100.7, 100.7)
    assert not state.usable(99)
    assert not state.usable(100.8)


def test_queued_old_or_out_of_order_measurements_cannot_refresh():
    state = SafetyFreshness(0.5)
    assert not state.scan(1, 5, 5)
    assert state.scan(5, 5.1, 5.1)
    assert not state.scan(4.9, 5.2, 5.2)
