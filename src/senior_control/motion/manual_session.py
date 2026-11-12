"""One locally timed, permanently fenced manual authority and one latest slot."""

from math import isfinite

from senior_control.motion.contracts import AcceptedManualIntent, ManualIntent


def _finite_number(value: float) -> bool:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return False
    try:
        return isfinite(value)
    except OverflowError:
        return False


def _normalized(value: float) -> bool:
    return _finite_number(value) and -1.0 <= value <= 1.0


class ManualSession:
    """Admit intent for an authority supplied by its authenticated owner.

    The owner creates a new instance for each new authority and calls close on
    release, connection replacement, or fault. Authority generation and acquire
    idempotency belong to that owner. Increasing gesture numbers cannot prove a
    real user release/press edge; the UI must enforce that separately.

    All times belong to one local steady clock; receipt and processing are distinct. Equal
    times are allowed; a backward or nonfinite clock permanently closes the
    session. The initial lease starts at construction, even before any intent.
    Instances are owned serially by the ingress; they are not thread-safe.
    """

    def __init__(self, authority_id: str, *, ttl: float, now_steady: float) -> None:
        if not isinstance(authority_id, str) or not authority_id.strip():
            raise ValueError("manual authority is required")
        if not _finite_number(ttl) or ttl <= 0:
            raise ValueError("manual TTL must be finite and positive")
        if not _finite_number(now_steady):
            raise ValueError("creation time must be finite")
        deadline = now_steady + ttl
        if not _finite_number(deadline) or deadline <= now_steady:
            raise ValueError("manual TTL must produce a finite future deadline")
        self._authority_id = authority_id
        self._ttl = ttl
        self._deadline_steady = deadline
        self._last_now_steady = now_steady
        self._last_gesture = -1
        self._last_sequence = -1
        self._closed = False
        self._latest: AcceptedManualIntent | None = None
        self._last_received_steady: float | None = None

    @property
    def authority_id(self) -> str:
        """Identity supplied by the owner; this session never rotates it."""
        return self._authority_id

    @property
    def deadline_steady(self) -> float:
        """Last admitted lease deadline; inspect closed before using it."""
        return self._deadline_steady

    @property
    def closed(self) -> bool:
        """Whether this authority has been permanently revoked."""
        return self._closed

    def accept(
        self,
        intent: ManualIntent,
        *,
        now_steady: float,
        received_steady: float | None = None,
        queue_deadline_steady: float | None = None,
    ) -> bool:
        """Admit with one deadline derived from original local receipt time.

        Processing time must not regress. Accepted receipt times are separately
        nondecreasing (equal times are legal); future/invalid/expired receipts
        are rejected without renewing the lease or advancing admitted counters.
        A supplied queue cutoff can only shorten the receipt-derived deadline.
        Expiration of the existing session always precedes new admission.
        """
        if not self._live(now_steady):
            return False
        if not self._admissible(intent):
            return False
        # //Compatible: omitted receipt is only for immediate same-stack admission.
        received = now_steady if received_steady is None else received_steady
        deadline = self._receipt_deadline(received, now_steady, queue_deadline_steady)
        if deadline is None:
            return False
        self._last_gesture = intent.gesture
        self._last_sequence = intent.sequence
        self._last_received_steady = received
        self._deadline_steady = deadline
        self._latest = AcceptedManualIntent(intent, received, deadline)
        return True

    def _receipt_deadline(
        self, received: float, now: float, queue_deadline: float | None
    ) -> float | None:
        if not _finite_number(received) or received > now:
            return None
        if self._last_received_steady is not None and received < self._last_received_steady:
            return None
        deadline = received + self._ttl
        if not _finite_number(deadline) or deadline <= now:
            return None
        if queue_deadline is not None:
            if not _finite_number(queue_deadline) or queue_deadline <= now:
                return None
            deadline = min(deadline, queue_deadline)
        return deadline

    def expire(self, *, now_steady: float) -> None:
        """Close at or beyond the deadline; never renew an idle lease."""
        self._live(now_steady)

    def close(self) -> None:
        """Idempotently revoke this authority and discard its pending intent."""
        self._closed = True
        self._latest = None

    def take_latest(self, *, now_steady: float) -> AcceptedManualIntent | None:
        """Consume one immutable sample with its original receipt and fixed deadline.

        Empty reads do not replay the last sample or renew the lease. This is
        an ingress handoff, not a held target: downstream retains this snapshot
        unchanged and revokes held targets when its authority closes. A later
        admission cannot change the deadline of a previously consumed sample.
        """
        if not self._live(now_steady):
            return None
        latest = self._latest
        self._latest = None
        return latest

    def _live(self, now_steady: float) -> bool:
        if self._closed:
            return False
        if not _finite_number(now_steady) or now_steady < self._last_now_steady:
            self.close()
            return False
        self._last_now_steady = now_steady
        if now_steady >= self._deadline_steady:
            self.close()
            return False
        return True

    def _admissible(self, intent: ManualIntent) -> bool:
        if intent.authority_id != self._authority_id:
            return False
        if type(intent.gesture) is not int or type(intent.sequence) is not int:
            return False
        if intent.gesture < 0 or intent.gesture < self._last_gesture:
            return False
        if intent.sequence < 0 or intent.sequence <= self._last_sequence:
            return False
        return (
            intent.gear in ("low", "high")
            and _normalized(intent.linear)
            and _normalized(intent.turn)
        )
