"""Absolute steady-clock slots: late work skips slots instead of catching up."""

import math


class ControlCadence:
    def __init__(self, period_ms: float = 50) -> None:
        if not math.isfinite(period_ms) or period_ms <= 0:
            raise ValueError("invalid_control_period")
        self.period = period_ms / 1000
        self.origin: float | None = None
        self.next_slot = 0
        self.last: float | None = None

    def observe(self, now: float) -> None:
        if not math.isfinite(now) or self.last is not None and now < self.last:
            raise ValueError("invalid_control_clock")
        self.last = now
        if self.origin is None:
            self.origin = now

    def due(self, now: float) -> bool:
        self.observe(now)
        assert self.origin is not None  # observe establishes the first clock sample.
        return now >= self.origin + self.next_slot * self.period

    def finished(self, now: float) -> None:
        if self.origin is None or self.last is None or not math.isfinite(now) or now < self.last:
            raise ValueError("invalid_control_clock")
        self.last = now
        self.next_slot = max(self.next_slot + 1, math.floor((now - self.origin) / self.period) + 1)
