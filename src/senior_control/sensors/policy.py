"""One sensor-age definition; command lifetimes and compute budgets are separate."""

import json
import math
from dataclasses import dataclass
from functools import lru_cache
from importlib.resources import files
from typing import TYPE_CHECKING, Literal

if TYPE_CHECKING:
    from senior_control.runtime.diagnostics import DiagnosticsSettings


@dataclass(frozen=True)
class Age:
    seconds: float
    band: Literal["normal", "low", "invalid"]
    freshness: float
    deadline_steady: float


@dataclass(frozen=True)
class SensorTiming:
    soft_age_s: float
    hard_age_s: float
    diagnostics_period_s: float

    def __post_init__(self) -> None:
        if (
            not all(
                math.isfinite(v)
                for v in (self.soft_age_s, self.hard_age_s, self.diagnostics_period_s)
            )
            or not 0 <= self.soft_age_s < self.hard_age_s
            or self.diagnostics_period_s <= 0
        ):
            raise ValueError("invalid common sensor timing")

    def age(
        self,
        *,
        stamp_ros: float,
        received_ros: float,
        received_steady: float,
        now_ros: float,
        now_steady: float,
    ) -> Age:
        values = (stamp_ros, received_ros, received_steady, now_ros, now_steady)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("nonfinite sensor clock")
        try:
            stamp, receipt, steady, ros, now = (round(v * 1_000_000_000) for v in values)
        except OverflowError as exc:
            raise ValueError("sensor clock outside numeric range") from exc
        if stamp < 0 or receipt < stamp or ros < receipt or now < steady:
            raise ValueError("future or reversed sensor clock")
        initial = receipt - stamp
        age = max(ros - stamp, initial + now - steady) / 1_000_000_000
        band: Literal["normal", "low", "invalid"] = (
            "invalid" if age >= self.hard_age_s else "low" if age >= self.soft_age_s else "normal"
        )
        freshness = (
            0.0
            if band == "invalid"
            else 1.0
            if band == "normal"
            else 0.1 + 0.9 * (self.hard_age_s - age) / (self.hard_age_s - self.soft_age_s)
        )
        return Age(
            age,
            band,
            freshness,
            min(
                received_steady + self.hard_age_s - initial / 1_000_000_000,
                now_steady + self.hard_age_s - age,
            ),
        )


@lru_cache(maxsize=1)
def default_policy() -> SensorTiming:
    """Explicitly load the installed common defaults, shared by real and simulation."""
    values = json.loads(files("senior_control.sensors").joinpath("defaults.json").read_text())
    return SensorTiming(**values)


def ros_sensor_parameters(diagnostics: "DiagnosticsSettings | None" = None) -> dict[str, float]:
    from senior_control.runtime.diagnostics import QueueWarningPolicy

    q = QueueWarningPolicy() if diagnostics is None else diagnostics.queue
    p = default_policy()
    return dict(
        queue_warn_count=q.count,
        queue_warn_ratio=q.ratio,
        queue_warn_repeat_s=q.repeat_s,
        queue_warn_recovery_ratio=q.recovery_ratio,
        queue_warn_oldest_wait_ms=q.oldest_wait_ms,
        sensor_soft_age_s=p.soft_age_s,
        sensor_hard_age_s=p.hard_age_s,
        sensor_diagnostics_period_s=p.diagnostics_period_s
        if diagnostics is None
        else diagnostics.period_s,
    )


def sensor_current(
    *,
    stamp_ros: float,
    received_ros: float,
    received_steady: float,
    now_ros: float,
    now_steady: float,
    max_age_s: float | None = None,
) -> bool:
    """Shared boolean facade for non-estimating ingress owners; malformed clocks fail."""
    try:
        age = default_policy().age(
            stamp_ros=stamp_ros,
            received_ros=received_ros,
            received_steady=received_steady,
            now_ros=now_ros,
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
