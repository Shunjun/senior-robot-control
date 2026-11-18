"""Bounded source admission for independently timestamped depth observations.

Admission qualifies identity and freshness only. It does not certify geometry,
registration, collision clearance, sensor coverage or any velocity command.
"""

import math
from dataclasses import dataclass, field

from senior_control.sensors.policy import sensor_current


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
    measurement_steady: float = field(kw_only=True)


def _bounded_identity(value: str, limit: int) -> bool:
    try:
        return bool(value) and len(value.encode("utf-8")) <= limit
    except UnicodeError:
        return False


class DepthAdmission:
    """Authenticated source ordering and immutable monotonic measurement age."""

    def __init__(self, *, max_age_ms: float) -> None:
        if not math.isfinite(max_age_ms) or max_age_ms <= 0:
            raise ValueError("max_age_s must be finite and positive")
        self.max_age_ms = max_age_ms
        self.latest: DepthHeader | None = None
        self.generation = 0
        self._active: str | None = None
        self._retired: set[str] = set()
        self._exhausted = False
        self._sequence = 0
        self._frontier_steady = -math.inf
        self._identity: tuple[str, str, str] | None = None
        self._identity_failed = False
        self._last_steady: float | None = None

    def retire(self) -> None:
        if self._active is not None:
            if len(self._retired) >= 256:
                self._exhausted = True
            else:
                self._retired.add(self._active)
        self._active = None
        self._sequence = 0
        self._frontier_steady = -math.inf
        self._identity = None
        self._identity_failed = False
        self.latest = None
        self.generation += 1

    def _clock_ok(self, now_steady: float) -> bool:
        if not math.isfinite(now_steady) or (
            self._last_steady is not None and now_steady < self._last_steady
        ):
            self.retire()
            return False
        self._last_steady = now_steady
        return True

    def accept(self, header: DepthHeader, *, now_ros_ns: int, now_steady: float) -> bool:
        if not self._clock_ok(now_steady):
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
        current = sensor_current(
            measured_steady=header.measurement_steady,
            received_steady=header.received_steady,
            now_steady=now_steady,
            max_age_ms=self.max_age_ms,
        )
        shape_ok = (
            _bounded_identity(header.raw_source, 128)
            and _bounded_identity(header.frame_id, 128)
            and _bounded_identity(header.calibration_id, 128)
            and header.stamp_ns >= 0
            and math.isfinite(header.measurement_steady)
            and header.measurement_steady > self._frontier_steady
            and not self._identity_failed
        )
        # Valid original timing may advance the source frontier; a future or
        # malformed message cannot poison it using a receiver-generated time.
        if current:
            self._frontier_steady = max(self._frontier_steady, header.measurement_steady)
        if not header.valid or not shape_ok or not current:
            return False
        self._identity = identity
        self.latest = header
        return True

    def usable(self, *, now_ros_ns: int, now_steady: float) -> bool:
        if not self._clock_ok(now_steady):
            return False
        header = self.latest
        return header is not None and sensor_current(
            measured_steady=header.measurement_steady,
            received_steady=header.received_steady,
            now_steady=now_steady,
            max_age_ms=self.max_age_ms,
        )
