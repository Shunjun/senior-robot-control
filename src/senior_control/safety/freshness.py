"""Original monotonic measurement age for safety inputs."""

import math


class SafetyFreshness:
    def __init__(self, max_age: float) -> None:
        if not math.isfinite(max_age) or max_age <= 0:
            raise ValueError("max_age must be positive")
        self.max_age = max_age
        self.scan_stamp: float | None = None
        self.scan_received: float | None = None
        self._last_steady: float | None = None

    def _clock(self, steady: float) -> bool:
        valid = math.isfinite(steady) and (self._last_steady is None or steady >= self._last_steady)
        if not valid:
            self.scan_received = self.scan_stamp = None
        self._last_steady = steady if math.isfinite(steady) else None
        return valid

    def scan(self, measured: float, received: float, now: float) -> bool:
        valid = (
            self._clock(now)
            and math.isfinite(measured)
            and math.isfinite(received)
            and 0 <= measured <= received <= now
            and now - measured < self.max_age
            and (self.scan_stamp is None or measured > self.scan_stamp)
        )
        if not valid:
            self.scan_received = None
            return False
        self.scan_stamp, self.scan_received = measured, received
        return True

    def usable(self, steady: float) -> bool:
        return (
            self._clock(steady)
            and self.scan_received is not None
            and self.scan_stamp is not None
            and self.scan_stamp <= self.scan_received <= steady
            and steady - self.scan_stamp < self.max_age
        )
