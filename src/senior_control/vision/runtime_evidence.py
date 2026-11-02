"""Bounded source admission for independently timestamped depth observations.

Admission qualifies identity and freshness only. It does not certify geometry,
registration, collision clearance, sensor coverage or any velocity command.
"""

import math
from collections import deque
from dataclasses import dataclass


@dataclass(frozen=True)
class DepthHeader:
    source_epoch: str
    raw_source: str
    sequence: int
    stamp_ns: int
    received_ros_ns: int
    received_steady: float
    frame_id: str
    calibration_id: str
    valid: bool


def _bounded_identity(value: str, limit: int) -> bool:
    try:
        return bool(value) and len(value.encode("utf-8")) <= limit
    except UnicodeError:
        return False


class DepthAdmission:
    """Revoke on newer input, bound future samples, and never refresh receipt times.

    The caller checks the actual ROS producer topology. A change there must call
    :meth:`retire`. The monotonic clock must be shared by producer and consumer.
    """

    def __init__(self, *, max_age_s: float, future_wait_s: float = 0.02) -> None:
        if not math.isfinite(max_age_s) or max_age_s <= 0:
            raise ValueError("max_age_s must be finite and positive")
        if not math.isfinite(future_wait_s) or not 0 < future_wait_s <= max_age_s:
            raise ValueError("invalid future wait")
        self.max_age_s = max_age_s
        self.future_wait_s = future_wait_s
        self.latest: DepthHeader | None = None
        self.generation = 0
        self._active: str | None = None
        self._retired: set[str] = set()
        self._exhausted = False
        self._sequence = 0
        self._frontier_ns = 0
        self._identity: tuple[str, str, str] | None = None
        self._identity_failed = False
        self._pending: deque[DepthHeader] = deque()
        self._last_ros_ns: int | None = None

    def retire(self) -> None:
        if self._active is not None:
            if len(self._retired) >= 256:
                self._exhausted = True
            else:
                self._retired.add(self._active)
        self._active = None
        self._sequence = 0
        self._frontier_ns = 0
        self._identity = None
        self._identity_failed = False
        self._pending.clear()
        self.latest = None
        self.generation += 1

    def _clock_ok(self, now_ros_ns: int, now_steady: float) -> bool:
        if now_ros_ns <= 0 or not math.isfinite(now_steady):
            self.latest = None
            self._pending.clear()
            self.generation += 1
            return False
        if self._last_ros_ns is not None and now_ros_ns < self._last_ros_ns:
            self.retire()
            self._last_ros_ns = now_ros_ns
            return False
        self._last_ros_ns = now_ros_ns
        return True

    def accept(self, header: DepthHeader, *, now_ros_ns: int, now_steady: float) -> bool:
        if not self._clock_ok(now_ros_ns, now_steady):
            return False
        if not _bounded_identity(header.source_epoch, 64):
            self.retire()
            return False
        if self._exhausted or header.source_epoch in self._retired:
            return False
        if header.source_epoch != self._active:
            self.retire()
            if self._exhausted:
                return False
            self._active = header.source_epoch
        if header.sequence <= self._sequence:
            return False
        self._sequence = header.sequence
        self.generation += 1
        self.latest = None
        identity = (header.raw_source, header.frame_id, header.calibration_id)
        if self._identity is not None and self._identity != identity:
            self._identity_failed = True
        shape_ok = (
            _bounded_identity(header.raw_source, 128)
            and _bounded_identity(header.frame_id, 128)
            and _bounded_identity(header.calibration_id, 128)
            and header.stamp_ns > 0
            and header.received_ros_ns > 0
            and math.isfinite(header.received_steady)
            and 0 <= now_steady - header.received_steady <= self.max_age_s
            and 0 <= now_ros_ns - header.received_ros_ns <= self.max_age_s * 1e9
            and header.stamp_ns > self._frontier_ns
            and not self._identity_failed
        )
        if not header.valid or not shape_ok:
            self._pending.clear()
            self._frontier_ns = max(self._frontier_ns, header.stamp_ns, header.received_ros_ns)
            return False
        self._identity = identity
        self._frontier_ns = header.stamp_ns
        if header.stamp_ns > now_ros_ns or self._pending:
            if len(self._pending) >= 16:
                self._pending.clear()
                return False
            self._pending.append(header)
            return False
        if now_ros_ns - header.stamp_ns > self.max_age_s * 1e9:
            return False
        self.latest = header
        return True

    def release(self, *, now_ros_ns: int, now_steady: float) -> int:
        if not self._clock_ok(now_ros_ns, now_steady):
            return 0
        released = 0
        for _ in range(8):
            if not self._pending:
                break
            header = self._pending[0]
            wait = now_steady - header.received_steady
            if not 0 <= wait <= min(self.future_wait_s, self.max_age_s):
                self._pending.clear()
                self.latest = None
                self.generation += 1
                break
            if header.stamp_ns > now_ros_ns:
                break
            self._pending.popleft()
            if (
                now_ros_ns - header.stamp_ns > self.max_age_s * 1e9
                or now_ros_ns - header.received_ros_ns > self.max_age_s * 1e9
            ):
                self._pending.clear()
                self.latest = None
                self.generation += 1
                break
            self.latest = header
            self.generation += 1
            released += 1
        if self._pending:
            self.latest = None
        return released

    @property
    def pending_headers(self) -> tuple[DepthHeader, ...]:
        return tuple(self._pending)

    def usable(self, *, now_ros_ns: int, now_steady: float) -> bool:
        header = self.latest
        return (
            header is not None
            and not self._pending
            and (self._last_ros_ns is None or now_ros_ns >= self._last_ros_ns)
            and math.isfinite(now_steady)
            and 0 <= now_steady - header.received_steady <= self.max_age_s
            and 0 <= now_ros_ns - header.received_ros_ns <= self.max_age_s * 1e9
            and 0 <= now_ros_ns - header.stamp_ns <= self.max_age_s * 1e9
        )
