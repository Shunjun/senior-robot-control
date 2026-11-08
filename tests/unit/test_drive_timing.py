import pytest

from senior_control.drive_timing import ClockWindow, DelayBudget


def test_asymmetric_exchange_preserves_bound_and_expires():
    clock = ClockWindow()
    # Receiver clock leads by five seconds. Forward delay 80 ms, reverse 20 ms.
    assert clock.observe(10.0, 15.08, 15.09, 10.11)
    bounds = clock.bounds(10.11)
    assert bounds is not None
    lower, upper = bounds
    assert lower <= 4.98 and upper >= 5.08
    assert clock.bounds(20.12) is None


def test_clock_jump_clears_estimate():
    clock = ClockWindow()
    assert clock.observe(10.0, 15.02, 15.025, 10.045)
    assert not clock.observe(11.0, 21.02, 21.025, 11.045)
    assert clock.bounds(11.045) is None


def test_browser_age_upper_bound_and_window():
    clock = ClockWindow()
    assert clock.observe(10.0, 15.02, 15.025, 10.045)
    assert clock.age_upper(10.2, 15.45) == pytest.approx(0.271, abs=0.001)
    assert clock.age_upper(10.2, 15.5) > 0.3


def test_delay_budget_requires_recent_samples_and_uses_ceiling_p90():
    budget = DelayBudget()
    budget.note(1.0, 0.019)
    budget.note(1.1, 0.0541)
    assert budget.compensation(1.1) is None
    budget.note(1.2, 0.024)
    assert budget.compensation(1.2) == 0.055
    assert budget.compensation(11.01) is None
