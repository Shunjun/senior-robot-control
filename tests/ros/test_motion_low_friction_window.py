"""Regression checks for the live low-friction observation window."""

import pytest
from motion_low_friction_window import stopped_with_fresh_request


def sample():
    return dict(
        started=10.0,
        now=12.2,
        outputs=[(10.01 + i * 0.1, 0.0, 0.0) for i in range(22)],
        requests=[(10.01, 0.0)] + [(10.1 + i * 0.1, 0.05) for i in range(22)],
        reason_times=[10.2],
        motion_times=[10.2],
    )


def test_queued_zero_before_first_nonzero_request_does_not_poison_window():
    assert stopped_with_fresh_request(**sample())


@pytest.mark.parametrize(
    "fault", ["early_output", "later_zero", "stale", "duration", "count", "reason", "motion"]
)
def test_window_rejects_unsafe_or_incomplete_evidence(fault):
    data = sample()
    if fault == "early_output":
        data["outputs"][0] = (10.01, 0.01, 0.0)
    elif fault == "later_zero":
        data["requests"][5] = (10.5, 0.0)
    elif fault == "stale":
        data["now"] = 12.6
    elif fault == "duration":
        data["now"] = 12.0
    elif fault == "count":
        data["requests"] = data["requests"][:5]
    elif fault == "reason":
        data["reason_times"] = [10.01]
    elif fault == "motion":
        data["motion_times"] = [10.01]
    assert not stopped_with_fresh_request(**data)
