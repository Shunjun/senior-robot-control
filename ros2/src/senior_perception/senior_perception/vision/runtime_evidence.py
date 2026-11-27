"""Bounded source admission for independently timestamped depth observations.

Admission qualifies identity and freshness only. It does not certify geometry,
registration, collision clearance, sensor coverage or any velocity command.
"""

import math
from dataclasses import dataclass, field

from vehicle_core.sensor_policy import sensor_current
from vehicle_core.source_order import SourceOrder


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
    boot_id: str = field(default="", kw_only=True)
    capture_generation: int = field(default=1, kw_only=True)


class DepthAdmission:
    """Consume each source stamp once; safety adoption keeps its existing age policy."""

    def __init__(self, *, max_age_ms: float) -> None:
        if not math.isfinite(max_age_ms) or max_age_ms <= 0:
            raise ValueError("max_age_s must be finite and positive")
        self.max_age_ms = max_age_ms
        self.latest: DepthHeader | None = None
        self.generation = 0
        self._order = SourceOrder()
        self._last_steady: float | None = None

    def retire(self) -> None:
        # //Compatible: old callers invalidate current evidence; no source is blacklisted.
        if self.latest is not None:
            self.latest = None
            self.generation += 1

    def confirm_boot(self, boot: str) -> None:
        if self._order.boot_id != boot:
            self._order.confirm_boot(boot)
            self.retire()

    def _clock_ok(self, now_steady: float) -> bool:
        if not math.isfinite(now_steady):
            self.retire()
            return False
        rollback = self._last_steady is not None and now_steady < self._last_steady
        self._last_steady = now_steady
        if rollback:
            self.retire()
        return not rollback

    def accept(self, header: DepthHeader, *, now_ros_ns: int, now_steady: float) -> bool:
        if not self._clock_ok(now_steady):
            return False
        if not header.valid:
            if header.boot_id and self._order.boot_id not in (None, header.boot_id):
                return False
            if header.capture_generation < self._order.generation:
                return False
            self.retire()
            return False
        if (
            not header.frame_id
            or not header.calibration_id
            or header.stamp_ns < 0
            or header.stamp_ns > header.received_ros_ns
            or not math.isfinite(header.measurement_steady)
            or not math.isfinite(header.received_steady)
            or header.measurement_steady > header.received_steady
            or header.received_steady > now_steady
        ):
            self.retire()
            return False
        # //Compatible: pure legacy callers have one consumer-local stream.
        boot = header.boot_id or "legacy-depth-consumer"
        if not self._order.accept_capture(
            boot, header.capture_generation, header.stamp_ns, round(header.received_steady * 1e9)
        ):
            return False
        self.latest = header
        self.generation += 1
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
