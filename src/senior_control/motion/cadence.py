"""Absolute steady-clock slots: late work skips slots instead of catching up."""

import math


class ControlCadence:
    def __init__(self, period_s=0.05):
        if not math.isfinite(period_s) or period_s <= 0:
            raise ValueError("invalid_control_period")
        self.period = period_s
        self.origin = None
        self.next_slot = 0
        self.last = None

    def observe(self, now):
        if not math.isfinite(now) or self.last is not None and now < self.last:
            raise ValueError("invalid_control_clock")
        self.last = now
        if self.origin is None:
            self.origin = now

    def due(self, now):
        self.observe(now)
        return now >= self.origin + self.next_slot * self.period

    def finished(self, now):
        if self.origin is None or not math.isfinite(now) or now < self.last:
            raise ValueError("invalid_control_clock")
        self.last = now
        self.next_slot = max(self.next_slot + 1, math.floor((now - self.origin) / self.period) + 1)
