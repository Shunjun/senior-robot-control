"""One sensor-age definition; command lifetimes and compute budgets are separate."""

import json
import math
from dataclasses import dataclass
from functools import lru_cache
from importlib.resources import files
from typing import TYPE_CHECKING, Literal

from senior_control.sensors.sample_time import SampleTime

if TYPE_CHECKING:
    from senior_control.runtime.diagnostics import DiagnosticsSettings


class SensorClockError(ValueError):
    """A numeric age cannot be assigned to these observation clocks."""


@dataclass(frozen=True)
class Age:
    seconds: float
    band: Literal["normal", "low", "invalid"]
    freshness: float
    deadline_steady: float


@dataclass(frozen=True)
class SensorTiming:
    soft_age_ms: float
    hard_age_ms: float
    diagnostics_period_s: float

    def __post_init__(self) -> None:
        if (
            not all(
                math.isfinite(v)
                for v in (self.soft_age_ms, self.hard_age_ms, self.diagnostics_period_s)
            )
            or not 0 <= self.soft_age_ms < self.hard_age_ms
            or self.diagnostics_period_s <= 0
        ):
            raise ValueError("invalid common sensor timing")

    @property
    def hard_age_s(self) -> float:
        """Seconds projection for receipt transport and monotonic-clock arithmetic.

        This is derived, read-only, and is never an accepted configuration key.
        """
        return self.hard_age_ms / 1000

    def sample_age(self, time: SampleTime, *, now_steady_ns: int) -> Age:
        """Apply the existing age bands to the original integer mapping."""
        if type(now_steady_ns) is not int or not 0 <= now_steady_ns < 1 << 64:
            raise SensorClockError("now_steady_ns outside integer nanosecond range")
        if now_steady_ns < time.received_steady_ns:
            raise SensorClockError("reversed monotonic sensor clock")
        return self._age_ns(time.sample_steady_ns, now_steady_ns)

    def _age_ns(self, measured: int, now: int) -> Age:
        elapsed_ns = now - measured
        soft_ns = round(self.soft_age_ms * 1_000_000)
        hard_ns = round(self.hard_age_ms * 1_000_000)
        if elapsed_ns >= hard_ns:
            band: Literal["normal", "low", "invalid"] = "invalid"
            freshness = 0.0
        elif elapsed_ns < soft_ns:
            band = "normal"
            freshness = 1.0
        else:
            band = "low"
            freshness = 0.1 + 0.9 * (hard_ns - elapsed_ns) / (hard_ns - soft_ns)
        return Age(elapsed_ns / 1_000_000_000, band, freshness, (measured + hard_ns) / 1e9)

    def age(self, *, measured_steady: float, received_steady: float, now_steady: float) -> Age:
        values = (measured_steady, received_steady, now_steady)
        if not all(math.isfinite(v) for v in values):
            raise SensorClockError("nonfinite sensor clock")
        try:
            measured, received, now = (round(v * 1_000_000_000) for v in values)
        except OverflowError as exc:
            raise SensorClockError("sensor clock outside numeric range") from exc
        if measured < 0 or received < measured or now < received:
            raise SensorClockError("future or reversed monotonic sensor clock")
        return self._age_ns(measured, now)


@lru_cache(maxsize=1)
def default_policy() -> SensorTiming:
    """Explicitly load the installed common defaults, shared by real and simulation."""
    values = json.loads(files("senior_control.sensors").joinpath("defaults.json").read_text())
    return SensorTiming(**values)


def ros_sensor_parameters(
    diagnostics: "DiagnosticsSettings | None" = None,
) -> dict[str, int | float]:
    from senior_control.runtime.diagnostics import QueueWarningPolicy

    q = QueueWarningPolicy() if diagnostics is None else diagnostics.queue
    p = default_policy()
    return dict(
        queue_warn_count=q.count,
        queue_warn_ratio=float(q.ratio),
        queue_warn_repeat_s=float(q.repeat_s),
        queue_warn_recovery_ratio=float(q.recovery_ratio),
        queue_warn_oldest_wait_ms=float(q.oldest_wait_ms),
        sensor_soft_age_ms=float(p.soft_age_ms),
        sensor_hard_age_ms=float(p.hard_age_ms),
        sensor_diagnostics_period_s=float(
            p.diagnostics_period_s if diagnostics is None else diagnostics.period_s
        ),
    )


def sensor_current(
    *,
    measured_steady: float,
    received_steady: float,
    now_steady: float,
    max_age_ms: float | None = None,
) -> bool:
    """Use the immutable source front, including time spent in every downstream queue."""
    max_age_s = None if max_age_ms is None else max_age_ms / 1000
    try:
        age = default_policy().age(
            measured_steady=measured_steady,
            received_steady=received_steady,
            now_steady=now_steady,
        )
    except (ValueError, OverflowError):
        return False
    limit = (
        default_policy().hard_age_s
        if max_age_s is None
        else min(max_age_s, default_policy().hard_age_s)
    )
    return age.seconds < limit
