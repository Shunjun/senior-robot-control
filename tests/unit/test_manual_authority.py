"""Prepared permission is not an active motion lease or proof of a UI edge."""

import asyncio
from dataclasses import FrozenInstanceError, replace

import pytest

from senior_control.motion.contracts import ManualIntent
from senior_control.motion.manual_authority import (
    ManualAuthorityContext,
    ManualAuthorityError,
    ManualAuthorityManager,
    ManualAuthorityState,
)


def context() -> ManualAuthorityContext:
    return ManualAuthorityContext("boot-a", "robot-a", 1, 1, "browser-a")


def manager(*, capacity: int = 4) -> ManualAuthorityManager:
    identities = iter(("authority-a", "authority-b", "authority-c", "authority-d"))
    return ManualAuthorityManager(
        context(), ttl=1.0, max_receipts=capacity, authority_factory=lambda: next(identities)
    )


def sample(authority: str, *, sequence: int = 1) -> ManualIntent:
    return ManualIntent(authority, 1, sequence, 0.5, -0.25, "low")


def test_prepared_long_wait_does_not_start_or_expire_a_motion_lease() -> None:
    owner = manager()
    receipt = owner.acquire(context(), "request-a", "payload-a", now_steady=10.0)
    assert receipt.state is ManualAuthorityState.PREPARED
    owner.expire(now_steady=10000.0)
    assert owner.take_latest(context(), receipt.authority_id, now_steady=10000.0) is None
    assert owner.acquire(context(), "request-a", "payload-a", now_steady=10000.0) == receipt
    assert owner.accept(context(), sample(receipt.authority_id), now_steady=10000.0)
    active = owner.acquire(context(), "request-a", "payload-a", now_steady=10000.2)
    assert active.state is ManualAuthorityState.ACTIVE
    latest = owner.take_latest(context(), receipt.authority_id, now_steady=10000.2)
    assert latest is not None and latest.deadline_steady == 10001.0
    assert receipt.state is ManualAuthorityState.PREPARED  # old immutable receipt stays old
    owner.expire(now_steady=10001.0)
    closed = owner.acquire(context(), "request-a", "payload-a", now_steady=10001.0)
    assert closed.state is ManualAuthorityState.CLOSED
    for seq in range(2, 102):
        assert not owner.accept(
            context(), sample(receipt.authority_id, sequence=seq), now_steady=10001.0
        )


def test_invalid_first_intent_keeps_prepared_without_starting_ttl() -> None:
    owner = manager()
    receipt = owner.acquire(context(), "r", "p", now_steady=10.0)
    assert not owner.accept(
        context(), replace(sample(receipt.authority_id), linear=float("nan")), now_steady=100.0
    )
    assert not owner.accept(
        context(), sample(receipt.authority_id), now_steady=100.0, received_steady=99.0
    )
    assert not owner.accept(
        context(), sample(receipt.authority_id), now_steady=100.0, received_steady=9.9
    )
    assert (
        owner.acquire(context(), "r", "p", now_steady=100.0).state is ManualAuthorityState.PREPARED
    )
    assert owner.accept(context(), sample(receipt.authority_id), now_steady=100.1)


def test_first_queued_intent_uses_original_receipt_and_fixed_cutoff() -> None:
    owner = manager()
    receipt = owner.acquire(context(), "r", "p", now_steady=10.0)
    assert owner.accept(
        context(),
        sample(receipt.authority_id),
        now_steady=100.4,
        received_steady=100.0,
        queue_deadline_steady=100.6,
    )
    first = owner.take_latest(context(), receipt.authority_id, now_steady=100.4)
    assert first is not None and first.deadline_steady == 100.6
    assert owner.accept(
        context(), sample(receipt.authority_id, sequence=2), now_steady=100.5, received_steady=100.3
    )
    assert first.deadline_steady == 100.6
    second = owner.take_latest(context(), receipt.authority_id, now_steady=100.5)
    assert second is not None and second.deadline_steady == 101.3
    assert owner.take_latest(context(), receipt.authority_id, now_steady=100.5) is None


def test_same_request_conflicting_payload_does_not_replace_or_renew() -> None:
    owner = manager()
    receipt = owner.acquire(context(), "r", "p", now_steady=10.0)
    assert owner.accept(context(), sample(receipt.authority_id), now_steady=10.0)
    with pytest.raises(ManualAuthorityError, match="conflict"):
        owner.acquire(context(), "r", "different", now_steady=10.9)
    assert owner.acquire(context(), "r", "p", now_steady=11.0).state is ManualAuthorityState.CLOSED


@pytest.mark.asyncio
async def test_simultaneous_same_id_serial_calls_generate_only_once() -> None:
    calls: list[int] = []

    def factory() -> str:
        calls.append(1)
        return "one-authority"

    owner = ManualAuthorityManager(context(), ttl=1.0, max_receipts=1, authority_factory=factory)

    async def acquire() -> str:
        await asyncio.sleep(0)
        return owner.acquire(context(), "r", "p", now_steady=10.0).authority_id

    assert await asyncio.gather(*(acquire() for _ in range(20))) == ["one-authority"] * 20
    assert len(calls) == 1


def test_capacity_keeps_closed_tombstones_and_does_not_evict_current() -> None:
    owner = manager(capacity=2)
    first = owner.acquire(context(), "r1", "p", now_steady=10.0)
    assert owner.release(context(), first.authority_id, now_steady=10.0)
    second = owner.acquire(context(), "r2", "p", now_steady=10.0)
    with pytest.raises(ManualAuthorityError, match="capacity"):
        owner.acquire(context(), "r3", "p", now_steady=100.0)
    assert (
        owner.acquire(context(), "r1", "p", now_steady=100.0).state is ManualAuthorityState.CLOSED
    )
    assert owner.accept(context(), sample(second.authority_id), now_steady=100.0)


@pytest.mark.parametrize("method", ["release", "stop", "revoke"])
def test_close_is_permanent_and_old_acquire_returns_closed(method: str) -> None:
    owner = manager()
    receipt = owner.acquire(context(), "r", "p", now_steady=10.0)
    assert owner.accept(context(), sample(receipt.authority_id), now_steady=10.0)
    if method == "revoke":
        owner.revoke(now_steady=10.1)
    else:
        close = owner.release if method == "release" else owner.stop
        assert close(context(), receipt.authority_id, now_steady=10.1)
        assert close(context(), receipt.authority_id, now_steady=10.1)
    for seq in range(2, 102):
        assert not owner.accept(
            context(), sample(receipt.authority_id, sequence=seq), now_steady=10.2
        )
    assert owner.acquire(context(), "r", "p", now_steady=10.2).state is ManualAuthorityState.CLOSED
    assert owner.take_latest(context(), receipt.authority_id, now_steady=10.2) is None


def test_old_release_does_not_revoke_replacement_authority() -> None:
    owner = manager()
    first = owner.acquire(context(), "r1", "p", now_steady=10.0)
    second = owner.acquire(context(), "r2", "p", now_steady=10.0)
    assert owner.release(context(), first.authority_id, now_steady=10.1)
    assert owner.acquire(context(), "r1", "p", now_steady=10.1).state is ManualAuthorityState.CLOSED
    assert owner.accept(context(), sample(second.authority_id), now_steady=10.2)
    assert not owner.release(context(), "unknown", now_steady=10.2)
    assert owner.take_latest(context(), second.authority_id, now_steady=10.2) is not None


def test_context_replacement_revokes_and_old_context_cannot_register_again() -> None:
    owner = manager(capacity=1)
    first = owner.acquire(context(), "r", "p", now_steady=10.0)
    newer = replace(context(), controller_generation=2, session_id="browser-b")
    owner.replace_context(newer, now_steady=10.1)
    assert not owner.accept(context(), sample(first.authority_id), now_steady=10.2)
    with pytest.raises(ManualAuthorityError, match="context"):
        owner.acquire(context(), "late", "p", now_steady=10.2)
    with pytest.raises(ManualAuthorityError, match="context"):
        owner.replace_context(context(), now_steady=10.2)
    second = owner.acquire(newer, "r", "p", now_steady=100.0)
    assert not owner.release(context(), first.authority_id, now_steady=100.0)
    assert owner.accept(newer, sample(second.authority_id), now_steady=100.0)


@pytest.mark.parametrize(
    "changed",
    [
        replace(context(), boot_id="other"),
        replace(context(), robot_id="other"),
        replace(context(), session_id="other"),
    ],
)
def test_untrusted_different_context_never_rebinds_implicitly(
    changed: ManualAuthorityContext,
) -> None:
    owner = manager()
    receipt = owner.acquire(context(), "r", "p", now_steady=10.0)
    assert not owner.accept(changed, sample(receipt.authority_id), now_steady=10.1)
    with pytest.raises(ManualAuthorityError, match="context"):
        owner.replace_context(changed, now_steady=10.1)
    assert owner.accept(context(), sample(receipt.authority_id), now_steady=10.2)


def test_context_new_agent_epoch_can_restart_controller_generation() -> None:
    owner = manager()
    newer = replace(context(), agent_epoch=2, controller_generation=1)
    owner.replace_context(newer, now_steady=10.0)
    receipt = owner.acquire(newer, "r", "p", now_steady=10.0)
    assert owner.accept(newer, sample(receipt.authority_id), now_steady=10.0)
    with pytest.raises(ManualAuthorityError, match="context"):
        owner.replace_context(context(), now_steady=10.1)


@pytest.mark.parametrize("state", ["prepared", "active"])
def test_bad_manager_clock_closes_authority_without_reviving(state: str) -> None:
    owner = manager()
    receipt = owner.acquire(context(), "r", "p", now_steady=10.0)
    if state == "active":
        assert owner.accept(context(), sample(receipt.authority_id), now_steady=10.0)
    owner.expire(now_steady=9.9)
    assert owner.acquire(context(), "r", "p", now_steady=10.1).state is ManualAuthorityState.CLOSED


@pytest.mark.parametrize("capacity", [0, -1])
def test_capacity_requires_positive_integer(capacity: int) -> None:
    with pytest.raises(ValueError):
        manager(capacity=capacity)


@pytest.mark.parametrize("ttl", [0.0, -1.0, float("nan"), float("inf")])
def test_ttl_requires_finite_positive_configuration(ttl: float) -> None:
    with pytest.raises(ValueError):
        ManualAuthorityManager(context(), ttl=ttl, max_receipts=1)


def test_identity_collision_cannot_close_or_replace_current() -> None:
    owner = ManualAuthorityManager(
        context(), ttl=1.0, max_receipts=2, authority_factory=lambda: "same"
    )
    receipt = owner.acquire(context(), "r1", "p", now_steady=10.0)
    with pytest.raises(ManualAuthorityError, match="identity"):
        owner.acquire(context(), "r2", "p", now_steady=10.0)
    assert owner.accept(context(), sample(receipt.authority_id), now_steady=10.0)


def test_context_and_receipts_are_immutable() -> None:
    owner = manager()
    receipt = owner.acquire(context(), "r", "p", now_steady=10.0)
    with pytest.raises(FrozenInstanceError):
        receipt.state = ManualAuthorityState.ACTIVE  # type: ignore[misc]
    with pytest.raises(FrozenInstanceError):
        receipt.context.session_id = "changed"  # type: ignore[misc]


def test_disconnect_retires_context_without_a_successor_and_cannot_be_undone() -> None:
    owner = manager()
    receipt = owner.acquire(context(), "r", "p", now_steady=10.0)
    assert owner.retire_context(context(), now_steady=10.1)
    assert owner.retire_context(context(), now_steady=10.1)
    owner.replace_context(context(), now_steady=10.2)
    assert owner.acquire(context(), "r", "p", now_steady=10.2).state is ManualAuthorityState.CLOSED
    with pytest.raises(ManualAuthorityError, match="context"):
        owner.acquire(context(), "late-new-id", "p", now_steady=10.2)
    assert not owner.accept(context(), sample(receipt.authority_id), now_steady=10.2)
    newer = replace(context(), controller_generation=2)
    owner.replace_context(newer, now_steady=10.3)
    fresh = owner.acquire(newer, "r", "p", now_steady=10.3)
    assert not owner.retire_context(context(), now_steady=10.3)
    assert owner.accept(newer, sample(fresh.authority_id), now_steady=10.3)


@pytest.mark.parametrize("now", [float("nan"), float("inf"), -float("inf")])
def test_nonfinite_clock_revokes_prepared_permission(now: float) -> None:
    owner = manager()
    receipt = owner.acquire(context(), "r", "p", now_steady=10.0)
    owner.expire(now_steady=now)
    assert owner.acquire(context(), "r", "p", now_steady=10.1).state is ManualAuthorityState.CLOSED
    assert not owner.accept(context(), sample(receipt.authority_id), now_steady=10.1)


@pytest.mark.parametrize("invalid", ["", " "])
def test_invalid_factory_identity_does_not_consume_capacity(invalid: str) -> None:
    identities = iter((invalid, "valid"))
    owner = ManualAuthorityManager(
        context(), ttl=1.0, max_receipts=1, authority_factory=lambda: next(identities)
    )
    with pytest.raises(ManualAuthorityError, match="identity"):
        owner.acquire(context(), "r", "p", now_steady=10.0)
    assert owner.acquire(context(), "r", "p", now_steady=10.0).authority_id == "valid"


@pytest.mark.parametrize("field", ["request_id", "fingerprint"])
def test_missing_idempotency_fields_do_not_consume_capacity(field: str) -> None:
    owner = manager(capacity=1)
    with pytest.raises(ManualAuthorityError, match="request"):
        owner.acquire(
            context(),
            "" if field == "request_id" else "r",
            "" if field == "fingerprint" else "p",
            now_steady=10.0,
        )
    assert (
        owner.acquire(context(), "r", "p", now_steady=10.0).state is ManualAuthorityState.PREPARED
    )


@pytest.mark.parametrize("generation", [0, -1, True])
def test_context_generation_is_positive_and_not_boolean(generation: int) -> None:
    with pytest.raises(ValueError):
        replace(context(), controller_generation=generation)
    with pytest.raises(ValueError):
        replace(context(), agent_epoch=generation)


def test_prepared_replacement_old_receipt_does_not_activate_either_identity() -> None:
    owner = manager()
    old = owner.acquire(context(), "r1", "p", now_steady=10.0)
    fresh = owner.acquire(context(), "r2", "p", now_steady=10.1)
    assert old.state is ManualAuthorityState.PREPARED
    assert not owner.accept(context(), sample(old.authority_id), now_steady=100.0)
    assert owner.take_latest(context(), fresh.authority_id, now_steady=100.0) is None
    assert (
        owner.acquire(context(), "r2", "p", now_steady=100.0).state is ManualAuthorityState.PREPARED
    )
