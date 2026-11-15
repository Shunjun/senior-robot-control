"""Shared source identity and ordered deferred receipt primitives."""

import math
from collections import deque
from dataclasses import dataclass
from typing import Generic, Literal, TypeVar

from senior_control.runtime.diagnostics import DiagnosticDeque

_DeferredValue = TypeVar("_DeferredValue")


@dataclass(frozen=True)
class DeferredSample(Generic[_DeferredValue]):
    stamp_ns: int
    received_ros_ns: int
    received_steady: float
    value: _DeferredValue


class DeferredSourceQueue(Generic[_DeferredValue]):
    """Bounded, source-ordered future observations retaining first receipt time."""

    def __init__(self, *, max_pending: int, diagnostic_name: str = "motion.deferred") -> None:
        if max_pending < 1:
            raise ValueError("deferred source capacity must be positive")
        self.max_pending = max_pending
        self._pending: deque[DeferredSample[_DeferredValue]] = DiagnosticDeque(
            diagnostic_name, max_pending
        )

    def offer(self, item: DeferredSample[_DeferredValue]) -> bool:
        if (
            item.stamp_ns <= 0
            or item.received_ros_ns <= 0
            or not math.isfinite(item.received_steady)
            or item.received_steady <= 0
            or len(self._pending) >= self.max_pending
            or (self._pending and item.stamp_ns <= self._pending[-1].stamp_ns)
        ):
            return False
        self._pending.append(item)
        return True

    def peek(self) -> DeferredSample[_DeferredValue] | None:
        return self._pending[0] if self._pending else None

    def latest(self) -> DeferredSample[_DeferredValue] | None:
        return self._pending[-1] if self._pending else None

    def consume(self) -> DeferredSample[_DeferredValue] | None:
        return self._pending.popleft() if self._pending else None

    def clear(self) -> None:
        self._pending.clear()

    def status(
        self, *, now_ros_ns: int, now_steady: float, max_age: float, pending_wait: float
    ) -> Literal["empty", "waiting", "ready", "expired", "clock_rollback"]:
        item = self.peek()
        if item is None:
            return "empty"
        if now_ros_ns < item.received_ros_ns:
            return "clock_rollback"
        if (
            not all(math.isfinite(value) for value in (now_steady, max_age, pending_wait))
            or max_age <= 0
            or pending_wait <= 0
            or not 0 <= now_steady - item.received_steady <= min(max_age, pending_wait)
            or (now_ros_ns - item.received_ros_ns) / 1e9 > max_age
        ):
            return "expired"
        if item.stamp_ns > now_ros_ns:
            return "waiting"
        if (now_ros_ns - item.stamp_ns) / 1e9 > max_age:
            return "expired"
        return "ready"


class SourceEpochFence:
    """Remember retired producer identities; exhaustion blocks rather than forgetting."""

    def __init__(self, *, max_retired: int) -> None:
        if max_retired < 1:
            raise ValueError("epoch capacity must be positive")
        self.max_retired = max_retired
        self.active: str | None = None
        self._retired: set[str] = set()
        self._exhausted = False

    def retire_active(self) -> None:
        if self.active is None:
            return
        if len(self._retired) >= self.max_retired:
            self._exhausted = True
        else:
            self._retired.add(self.active)
        self.active = None

    def is_retired(self, epoch: str) -> bool:
        return epoch in self._retired

    def retire_unadmitted(self, epoch: str) -> None:
        """Fence a replaced deferred source without making it the active source."""
        if not epoch or epoch in self._retired:
            return
        if self.active == epoch:
            self.retire_active()
        elif len(self._retired) >= self.max_retired:
            self._exhausted = True
        else:
            self._retired.add(epoch)

    def accept(self, epoch: str) -> bool:
        if self._exhausted or not epoch or epoch in self._retired:
            return False
        if self.active is not None and self.active != epoch:
            self.retire_active()
            if self._exhausted:
                return False
        self.active = epoch
        return True


class ProducerTopology:
    """Require one expected ROS graph endpoint; this is not DDS authentication."""

    def __init__(self, name: str, namespace: str) -> None:
        if not name or not namespace:
            raise ValueError("expected producer identity is required")
        self._name = name
        self._namespace = namespace
        self._gid: bytes | None = None

    def observe(self, endpoints: list[tuple[str, str, bytes]]) -> tuple[bool, bool]:
        gid = None
        if len(endpoints) == 1:
            name, namespace, candidate = endpoints[0]
            if name == self._name and namespace == self._namespace and candidate and any(candidate):
                gid = candidate
        changed = gid != self._gid
        self._gid = gid
        return gid is not None, changed
