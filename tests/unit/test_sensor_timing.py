import pytest

from senior_control.sensors.policy import default_policy


@pytest.mark.parametrize(
    "age,band", [(0.299, "normal"), (0.3, "low"), (0.999, "low"), (1, "invalid")]
)
def test_common_boundary(age, band):
    state = default_policy().age(
        stamp_ros=10 - age, received_ros=10, received_steady=20, now_ros=10, now_steady=20
    )
    assert state.band == band


def test_transport_delay_and_paused_ros_never_refresh_age():
    policy = default_policy()
    assert (
        policy.age(
            stamp_ros=10, received_ros=10.6, received_steady=20, now_ros=10.6, now_steady=20.4
        ).band
        == "invalid"
    )
    assert (
        policy.age(
            stamp_ros=10, received_ros=10.6, received_steady=20, now_ros=10.6, now_steady=20.2
        ).band
        == "low"
    )


@pytest.mark.parametrize("field,value", [("stamp_ros", 11), ("now_steady", 19), ("now_ros", 9)])
def test_illegal_clock_is_distinct(field, value):
    args = dict(stamp_ros=10, received_ros=10, received_steady=20, now_ros=10, now_steady=20)
    args[field] = value
    with pytest.raises(ValueError):
        default_policy().age(**args)


def test_future_receipt_clock_is_not_a_soft_age_or_valid_prediction():
    with pytest.raises(ValueError):
        default_policy().age(
            stamp_ros=10.0, received_ros=10.1, received_steady=20.0, now_ros=10.0, now_steady=20.0
        )


def test_fast_ros_progress_tightens_original_deadline_without_extending_it():
    result = default_policy().age(
        stamp_ros=10.0, received_ros=10.0, received_steady=20.0, now_ros=10.9, now_steady=20.1
    )
    assert result.deadline_steady == pytest.approx(20.2)
