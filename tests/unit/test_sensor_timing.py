import math

import pytest

from senior_control.sensors.policy import default_policy


@pytest.mark.parametrize(
    "age,band", [(0.299, "normal"), (0.3, "low"), (0.999, "low"), (1, "invalid")]
)
def test_common_boundary(age, band):
    state = default_policy().age(measured_steady=20 - age, received_steady=20, now_steady=20)
    assert state.band == band
    assert state.deadline_steady == pytest.approx(21 - age)


def test_original_production_time_includes_transport_and_processing_delay():
    p = default_policy()
    assert p.age(measured_steady=19.4, received_steady=20, now_steady=20.4).band == "invalid"
    assert p.age(measured_steady=19.4, received_steady=20, now_steady=20.2).band == "low"


@pytest.mark.parametrize(
    "field,value",
    [
        ("measured_steady", 21),
        ("measured_steady", -1),
        ("received_steady", 19),
        ("now_steady", 19),
        ("now_steady", math.nan),
        ("measured_steady", math.inf),
    ],
)
def test_invalid_monotonic_clock_has_no_numeric_age(field, value):
    args = dict(measured_steady=20, received_steady=20, now_steady=20)
    args[field] = value
    with pytest.raises(ValueError):
        default_policy().age(**args)


def test_policy_milliseconds_keep_one_second_hard_age_projection():
    policy = default_policy()
    assert (policy.soft_age_ms, policy.hard_age_ms, policy.hard_age_s) == (300, 1000, 1)
    with pytest.raises(TypeError):
        type(policy)(soft_age_s=0.3, hard_age_ms=1000, diagnostics_period_s=5)


def test_later_consumption_preserves_original_deadline():
    p = default_policy()
    a = p.age(measured_steady=20, received_steady=20.1, now_steady=20.2)
    b = p.age(measured_steady=20, received_steady=20.1, now_steady=20.9)
    assert a.deadline_steady == b.deadline_steady == 21
    assert b.seconds == 0.9
