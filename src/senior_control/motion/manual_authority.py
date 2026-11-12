"""Serial prepared/active authority ownership, independent of transport and ROS.

Only the authenticated composition root may bind/replace contexts and invoke
acquire. This module cannot prove browser receipt of an acquire acknowledgement
or a real release/press edge, and never turns prepared permission into motion.
"""

import secrets
from collections.abc import Callable
from dataclasses import dataclass, replace
from enum import Enum
from typing import Literal

from senior_control.motion.contracts import AcceptedManualIntent, ManualIntent
from senior_control.motion.manual_session import ManualSession, _finite_number


def _nonempty(value: str) -> bool:
    return isinstance(value, str) and bool(value.strip())


@dataclass(frozen=True)
class ManualAuthorityContext:
    """Trusted identity; controller_generation increases within an agent epoch."""

    boot_id: str
    robot_id: str
    agent_epoch: int
    controller_generation: int
    session_id: str

    def __post_init__(self) -> None:
        if not all(_nonempty(value) for value in (self.boot_id, self.robot_id, self.session_id)):
            raise ValueError("context identity is required")
        for generation in (self.agent_epoch, self.controller_generation):
            if type(generation) is not int or generation < 1:
                raise ValueError("context generations must be positive integers")


class ManualAuthorityState(str, Enum):
    PREPARED = "prepared"
    ACTIVE = "active"
    CLOSED = "closed"


@dataclass(frozen=True)
class ManualAuthorityReceipt:
    """Point-in-time status, never a capability that bypasses current admission."""

    request_id: str
    authority_id: str
    context: ManualAuthorityContext
    state: ManualAuthorityState
    reason: str


class ManualAuthorityError(ValueError):
    """A request failed without creating an authority or evicting a tombstone."""

    def __init__(
        self,
        code: Literal["context", "conflict", "capacity", "identity", "clock", "request"],
        message: str,
    ) -> None:
        super().__init__(f"{code}: {message}")
        self.code = code


@dataclass
class _Authority:
    receipt: ManualAuthorityReceipt
    fingerprint: str
    prepared_at: float
    session: ManualSession | None = None


def _new_identity() -> str:
    return secrets.token_urlsafe(32)


class ManualAuthorityManager:
    """One serial owner with one open authority and bounded per-context receipts.

    Methods are synchronous and contain no awaits: callers must serialize them
    on one owner (not share this instance across threads). An idempotent acquire
    returns a fresh immutable view of the same identity's current state. Closed
    receipts are retained until an explicit strictly newer context retires the
    entire namespace. Capacity exhaustion never evicts a receipt.

    Context replacement uses a lexicographic (agent_epoch, controller_generation)
    frontier, not an unbounded set of retired connections. Boot and robot stay
    fixed for this manager's lifetime; a new boot requires a new composition.
    """

    def __init__(
        self,
        context: ManualAuthorityContext,
        *,
        ttl: float,
        max_receipts: int,
        authority_factory: Callable[[], str] = _new_identity,
    ) -> None:
        if not _finite_number(ttl) or ttl <= 0:
            raise ValueError("manual TTL must be finite and positive")
        if type(max_receipts) is not int or max_receipts < 1:
            raise ValueError("receipt capacity must be a positive integer")
        self._context = context
        self._context_retired = False
        self._ttl = ttl
        self._max_receipts = max_receipts
        self._factory = authority_factory
        self._requests: dict[str, _Authority] = {}
        self._authorities: dict[str, _Authority] = {}
        self._current: _Authority | None = None
        self._last_now: float | None = None

    def acquire(
        self,
        context: ManualAuthorityContext,
        request_id: str,
        fingerprint: str,
        *,
        now_steady: float,
    ) -> ManualAuthorityReceipt:
        """Prepare once for a trusted request/fingerprint; never renew on retry.

        The caller computes the canonical payload fingerprint, rather than
        accepting a browser-supplied assertion that two payloads are identical.
        A new request replaces the open authority only after validation/capacity
        and identity allocation succeed. No active session is constructed here.
        """
        self._require_clock(now_steady)
        self._require_context(context)
        if not _nonempty(request_id) or not _nonempty(fingerprint):
            raise ManualAuthorityError("request", "request id and fingerprint are required")
        previous = self._requests.get(request_id)
        if previous is not None:
            if fingerprint != previous.fingerprint:
                raise ManualAuthorityError("conflict", "request payload changed")
            return previous.receipt
        if self._context_retired:
            raise ManualAuthorityError("context", "controller context has been retired")
        if len(self._requests) >= self._max_receipts:
            raise ManualAuthorityError("capacity", "receipt namespace is full")
        identity = self._factory()
        if not _nonempty(identity) or identity in self._authorities:
            raise ManualAuthorityError("identity", "authority factory returned an invalid identity")
        receipt = ManualAuthorityReceipt(
            request_id, identity, context, ManualAuthorityState.PREPARED, ""
        )
        record = _Authority(receipt, fingerprint, now_steady)
        if self._current is not None:
            self._close(self._current, "replaced")
        self._requests[request_id] = record
        self._authorities[identity] = record
        self._current = record
        return receipt

    def accept(
        self,
        context: ManualAuthorityContext,
        intent: ManualIntent,
        *,
        now_steady: float,
        received_steady: float | None = None,
        queue_deadline_steady: float | None = None,
    ) -> bool:
        """Activate on the first legal sample, or update the one active session.

        Prepared permission has no movement deadline. A rejected first packet
        creates no active lease. Actual user-edge and acknowledgement ordering
        remain the UI/transport owner's responsibility, not a numeric gesture
        assertion that this module could authenticate.
        """
        if not self._observe(now_steady) or context != self._context or self._context_retired:
            return False
        record = self._authorities.get(intent.authority_id)
        if record is None or record.receipt.state is ManualAuthorityState.CLOSED:
            return False
        # //Compatible: same-stack callers may omit the original receipt timestamp.
        received = now_steady if received_steady is None else received_steady
        if not _finite_number(received) or received < record.prepared_at:
            return False
        session = record.session
        if session is None:
            try:
                session = ManualSession(intent.authority_id, ttl=self._ttl, now_steady=now_steady)
            except ValueError:
                return False
        if not session.accept(
            intent,
            now_steady=now_steady,
            received_steady=received,
            queue_deadline_steady=queue_deadline_steady,
        ):
            return False
        record.session = session
        record.receipt = replace(record.receipt, state=ManualAuthorityState.ACTIVE)
        return True

    def take_latest(
        self, context: ManualAuthorityContext, authority_id: str, *, now_steady: float
    ) -> AcceptedManualIntent | None:
        """Hand off one frozen receipt/deadline snapshot; never hold a target here."""
        if not self._observe(now_steady) or context != self._context or self._context_retired:
            return None
        record = self._authorities.get(authority_id)
        if record is None or record.session is None:
            return None
        return record.session.take_latest(now_steady=now_steady)

    def release(
        self, context: ManualAuthorityContext, authority_id: str, *, now_steady: float
    ) -> bool:
        """Idempotently close exactly the named authority, including prepared ones."""
        if not self._observe(now_steady) or context != self._context:
            return False
        record = self._authorities.get(authority_id)
        if record is None:
            return False
        self._close(record, "released")
        return True

    def stop(
        self, context: ManualAuthorityContext, authority_id: str, *, now_steady: float
    ) -> bool:
        """Ordinary release/stop also closes the entire authority, not just gesture."""
        return self.release(context, authority_id, now_steady=now_steady)

    def revoke(self, *, now_steady: float) -> None:
        """Close current permission on a fault or other trusted owner revocation."""
        self._observe(now_steady)
        if self._current is not None:
            self._close(self._current, "revoked")

    def expire(self, *, now_steady: float) -> None:
        """Expire only active sessions; prepared permission has no short TTL."""
        self._observe(now_steady)

    def retire_context(self, context: ManualAuthorityContext, *, now_steady: float) -> bool:
        """Revoke a disconnected context even before its successor is known.

        Known acquire retries still report closed; new requests cannot register
        against this context. Replacing with the same context never un-retires it.
        """
        if not self._observe(now_steady) or context != self._context:
            return False
        if self._current is not None:
            self._close(self._current, "context_retired")
        self._context_retired = True
        return True

    def replace_context(self, context: ManualAuthorityContext, *, now_steady: float) -> None:
        """Explicitly retire the entire namespace; never re-register an older one."""
        self._require_clock(now_steady)
        if context == self._context:
            return
        current = self._context
        if (context.boot_id, context.robot_id) != (current.boot_id, current.robot_id) or (
            context.agent_epoch,
            context.controller_generation,
        ) <= (current.agent_epoch, current.controller_generation):
            raise ManualAuthorityError("context", "replacement must be a newer trusted context")
        if self._current is not None:
            self._close(self._current, "context_replaced")
        self._requests.clear()
        self._authorities.clear()
        self._current = None
        self._context = context
        self._context_retired = False

    def _observe(self, now: float) -> bool:
        if not _finite_number(now) or (self._last_now is not None and now < self._last_now):
            if self._current is not None:
                self._close(self._current, "invalid_clock")
            return False
        self._last_now = now
        record = self._current
        if record is not None and record.session is not None:
            record.session.expire(now_steady=now)
            if record.session.closed:
                self._close(record, "expired")
        return True

    def _require_clock(self, now: float) -> None:
        if not self._observe(now):
            raise ManualAuthorityError("clock", "processing clock must be finite and monotonic")

    def _require_context(self, context: ManualAuthorityContext) -> None:
        if context != self._context:
            raise ManualAuthorityError("context", "request belongs to a retired or foreign context")

    @staticmethod
    def _close(record: _Authority, reason: str) -> None:
        if record.receipt.state is ManualAuthorityState.CLOSED:
            return
        if record.session is not None:
            record.session.close()
        record.receipt = replace(record.receipt, state=ManualAuthorityState.CLOSED, reason=reason)
