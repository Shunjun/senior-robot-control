"""Manual authority fencing without transport, browser clocks, or vehicle output."""

from dataclasses import replace

import pytest

from senior_control.motion.contracts import ManualIntent
from senior_control.motion.manual_session import ManualSession


def intent(*, gesture: int = 1, sequence: int = 1) -> ManualIntent:
    return ManualIntent("authority-a", gesture, sequence, 0.5, -0.25, "low")


def session() -> ManualSession:
    return ManualSession("authority-a", ttl=1.0, now_steady=10.0)


def take_intent(control: ManualSession, *, now_steady: float) -> ManualIntent:
    latest = control.take_latest(now_steady=now_steady)
    assert latest is not None
    return latest.intent


def test_continuous_updates_refresh_local_liveness() -> None:
    control = session()
    for sequence, now in enumerate((10.5, 11.0, 11.5, 12.0), start=1):
        assert control.accept(intent(sequence=sequence), now_steady=now)
    assert control.deadline_steady == 13.0
    assert take_intent(control, now_steady=12.5) == intent(sequence=4)
    control.expire(now_steady=12.75)
    assert not control.closed
    control.expire(now_steady=13.0)
    assert control.closed


@pytest.mark.parametrize(
    "bad",
    [
        replace(intent(sequence=2), authority_id="authority-b"),
        intent(sequence=1),
        intent(sequence=0),
        intent(gesture=0, sequence=2),
        intent(gesture=-1, sequence=2),
        intent(sequence=-1),
        replace(intent(sequence=2), gear="turbo"),  # type: ignore[arg-type]
    ],
)
def test_rejected_packet_cannot_extend_lease_or_replace_latest(bad: ManualIntent) -> None:
    control = session()
    assert control.accept(intent(), now_steady=10.0)
    assert not control.accept(bad, now_steady=10.75)
    assert control.deadline_steady == 11.0
    assert take_intent(control, now_steady=10.8) == intent()
    control.expire(now_steady=11.0)
    assert control.closed


@pytest.mark.parametrize("axis", ["linear", "turn"])
@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf"), -1.01, 1.01])
def test_nonfinite_and_out_of_range_axes_are_rejected(axis: str, value: float) -> None:
    control = session()
    bad = replace(intent(), linear=value) if axis == "linear" else replace(intent(), turn=value)
    assert not control.accept(bad, now_steady=10.75)
    assert control.take_latest(now_steady=10.8) is None
    assert control.deadline_steady == 11.0
    assert control.accept(intent(), now_steady=10.9)


def test_normalized_endpoints_and_zero_remain_intents() -> None:
    control = session()
    for sequence, value in enumerate((-1.0, 0.0, 1.0), start=1):
        sample = replace(intent(sequence=sequence), linear=value, turn=-value, gear="high")
        assert control.accept(sample, now_steady=10.0)
        assert take_intent(control, now_steady=10.0) == sample
    assert not control.closed


def test_gesture_is_monotonic_separately_from_packet_sequence() -> None:
    control = session()
    assert control.accept(intent(gesture=3, sequence=10), now_steady=10.0)
    assert not control.accept(intent(gesture=2, sequence=11), now_steady=10.1)
    assert not control.accept(intent(gesture=4, sequence=9), now_steady=10.2)
    assert control.accept(intent(gesture=4, sequence=11), now_steady=10.3)
    assert control.accept(intent(gesture=4, sequence=12), now_steady=10.4)


@pytest.mark.parametrize("ending", ["expiry", "release", "connection", "fault"])
def test_closed_authority_rejects_one_hundred_larger_sequences(ending: str) -> None:
    control = session()
    assert control.accept(intent(), now_steady=10.0)
    if ending == "expiry":
        control.expire(now_steady=11.0)
    else:
        # The owner maps each lifecycle cause to close(); this module owns no connection.
        control.close()
    control.close()
    for sequence in range(2, 102):
        assert not control.accept(intent(gesture=sequence, sequence=sequence), now_steady=11.0)
    assert control.closed
    assert control.take_latest(now_steady=11.0) is None


@pytest.mark.parametrize("operation", ["accept", "take_latest", "expire"])
def test_deadline_equality_closes_even_without_an_expire_tick(operation: str) -> None:
    control = session()
    assert control.accept(intent(), now_steady=10.0)
    if operation == "accept":
        assert not control.accept(intent(sequence=2), now_steady=11.0)
    elif operation == "take_latest":
        assert control.take_latest(now_steady=11.0) is None
    else:
        control.expire(now_steady=11.0)
    assert control.closed


def test_queue_keeps_only_latest_sample_and_take_consumes_once() -> None:
    control = session()
    for sequence in range(1, 1001):
        assert control.accept(intent(sequence=sequence), now_steady=10.0)
    assert take_intent(control, now_steady=10.5) == intent(sequence=1000)
    assert control.take_latest(now_steady=10.5) is None
    assert control.deadline_steady == 11.0
    assert not control.accept(intent(sequence=999), now_steady=10.6)
    assert control.take_latest(now_steady=10.6) is None


@pytest.mark.parametrize("operation", ["accept", "take_latest", "expire"])
@pytest.mark.parametrize("now", [9.9, float("nan"), float("inf"), -float("inf")])
def test_bad_clock_permanently_closes_and_discards_pending(operation: str, now: float) -> None:
    control = session()
    assert control.accept(intent(), now_steady=10.0)
    if operation == "accept":
        assert not control.accept(intent(sequence=2), now_steady=now)
    elif operation == "take_latest":
        assert control.take_latest(now_steady=now) is None
    else:
        control.expire(now_steady=now)
    assert control.closed
    assert control.take_latest(now_steady=10.1) is None
    assert not control.accept(intent(sequence=100), now_steady=10.1)


def test_rejected_packet_still_observes_steady_clock_progress() -> None:
    control = session()
    assert not control.accept(replace(intent(), authority_id="other"), now_steady=10.8)
    assert not control.accept(intent(), now_steady=10.7)
    assert control.closed


@pytest.mark.parametrize("ttl", [0.0, -1.0, float("nan"), float("inf"), -float("inf")])
def test_invalid_ttl_is_a_configuration_error(ttl: float) -> None:
    with pytest.raises(ValueError):
        ManualSession("authority-a", ttl=ttl, now_steady=10.0)


@pytest.mark.parametrize("authority", ["", " "])
def test_empty_authority_is_a_configuration_error(authority: str) -> None:
    with pytest.raises(ValueError):
        ManualSession(authority, ttl=1.0, now_steady=10.0)


@pytest.mark.parametrize("now", [float("nan"), float("inf"), -float("inf")])
def test_invalid_creation_clock_is_a_configuration_error(now: float) -> None:
    with pytest.raises(ValueError):
        ManualSession("authority-a", ttl=1.0, now_steady=now)


def test_ttl_and_creation_time_are_required() -> None:
    with pytest.raises(TypeError):
        ManualSession("authority-a")  # type: ignore[call-arg]


@pytest.mark.parametrize("now,ttl", [(1e308, 1e308), (1e308, 1.0)])
def test_deadline_must_be_finite_and_representably_after_now(now: float, ttl: float) -> None:
    with pytest.raises(ValueError):
        ManualSession("authority-a", ttl=ttl, now_steady=now)


def test_new_authority_requires_an_explicit_new_session() -> None:
    previous = session()
    previous.close()
    current = ManualSession("authority-b", ttl=2.0, now_steady=11.0)
    assert not current.accept(intent(sequence=100), now_steady=11.0)
    fresh = replace(intent(), authority_id="authority-b")
    assert current.accept(fresh, now_steady=11.0)
    assert take_intent(current, now_steady=11.0) == fresh
    assert not previous.accept(fresh, now_steady=11.0)


@pytest.mark.parametrize("axis", ["linear", "turn"])
def test_unrepresentably_large_axis_is_rejected_without_raising(axis: str) -> None:
    control = session()
    huge = 10**1000
    bad = replace(intent(), linear=huge) if axis == "linear" else replace(intent(), turn=huge)
    assert not control.accept(bad, now_steady=10.5)
    assert control.accept(intent(), now_steady=10.75)


@pytest.mark.parametrize("field", ["ttl", "now_steady"])
def test_unrepresentably_large_configuration_is_a_value_error(field: str) -> None:
    with pytest.raises(ValueError):
        if field == "ttl":
            ManualSession("authority-a", ttl=10**1000, now_steady=10.0)
        else:
            ManualSession("authority-a", ttl=1.0, now_steady=10**1000)


def test_delayed_receipt_sets_deadline_from_receipt_not_processing() -> None:
    control = session()
    assert control.accept(intent(), now_steady=10.5, received_steady=10.1)
    assert control.deadline_steady == 11.1
    latest = control.take_latest(now_steady=10.6)
    assert latest is not None
    assert latest.intent == intent()
    assert latest.received_steady == 10.1
    assert latest.deadline_steady == 11.1
    control.expire(now_steady=11.1)
    assert control.closed


@pytest.mark.parametrize("received", [float("nan"), float("inf"), 10.6, 9.5, 9.4])
def test_bad_receipt_does_not_refresh_or_replace(received: float) -> None:
    control = session()
    assert control.accept(intent(), now_steady=10.0)
    assert not control.accept(intent(sequence=2), now_steady=10.5, received_steady=received)
    assert control.deadline_steady == 11.0
    assert take_intent(control, now_steady=10.6) == intent()


def test_receipt_monotonicity_is_independent_of_processing_clock() -> None:
    control = session()
    assert control.accept(intent(), now_steady=10.4, received_steady=10.2)
    assert not control.accept(intent(sequence=2), now_steady=10.5, received_steady=10.1)
    assert not control.closed
    assert control.accept(intent(sequence=2), now_steady=10.6, received_steady=10.2)
    assert control.deadline_steady == 11.2


def test_expiration_precedes_processing_a_later_received_packet() -> None:
    control = session()
    assert control.accept(intent(), now_steady=10.0)
    assert not control.accept(intent(sequence=2), now_steady=11.0, received_steady=10.9)
    assert control.closed


def test_shorter_queue_cutoff_is_the_single_session_deadline() -> None:
    control = session()
    assert control.accept(
        intent(), now_steady=10.2, received_steady=10.0, queue_deadline_steady=10.5
    )
    assert control.deadline_steady == 10.5
    latest = control.take_latest(now_steady=10.3)
    assert latest is not None and latest.deadline_steady == 10.5
    assert not control.accept(intent(sequence=2), now_steady=10.5, received_steady=10.4)
    assert control.closed


@pytest.mark.parametrize("cutoff", [10.5, 10.4, float("nan"), float("inf")])
def test_invalid_or_expired_queue_cutoff_cannot_refresh(cutoff: float) -> None:
    control = session()
    assert not control.accept(
        intent(), now_steady=10.5, received_steady=10.0, queue_deadline_steady=cutoff
    )
    assert control.deadline_steady == 11.0
    assert control.accept(intent(), now_steady=10.6)


def test_taken_snapshot_deadline_never_changes_after_new_admission() -> None:
    control = session()
    assert control.accept(intent(), now_steady=10.0)
    taken = control.take_latest(now_steady=10.0)
    assert taken is not None
    assert control.accept(intent(sequence=2), now_steady=10.8)
    assert taken.intent.sequence == 1
    assert taken.deadline_steady == 11.0
    assert control.deadline_steady == 11.8


def test_large_integer_deadline_sum_is_rejected_as_configuration_error() -> None:
    with pytest.raises(ValueError):
        ManualSession("authority-a", ttl=10**308, now_steady=10**308)


def test_large_integer_receipt_sum_rejects_without_raising_or_renewing() -> None:
    control = ManualSession("authority-a", ttl=10**308, now_steady=10**307)
    assert not control.accept(intent(), now_steady=10**308, received_steady=10**308)
    assert not control.closed
    assert control.deadline_steady == 11 * 10**307
