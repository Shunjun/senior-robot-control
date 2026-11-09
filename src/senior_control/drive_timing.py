"""Conservative clock evidence for short-lived motion authority."""

import math
from collections import deque
from dataclasses import dataclass

from senior_control.protocol import DRIVE_TRANSPORT_SECONDS

WINDOW_SECONDS = 10.0
_MAX_SAMPLES = 20
_MAX_OFFSET_WIDTH = 0.250
# Relative oscillator drift and combined timestamp resolution/quantization margin.
_DRIFT = 0.0002
_TIMESTAMP_ERROR = 0.001


@dataclass(frozen=True)
class _Exchange:
    sender_received: float
    receiver_sent: float
    lower: float
    upper: float


class ClockWindow:
    """Bound receiver minus sender time using completed four-timestamp exchanges."""

    def __init__(self) -> None:
        self._samples: deque[_Exchange] = deque(maxlen=_MAX_SAMPLES)

    def clear(self) -> None:
        self._samples.clear()

    def observe(self, t1: float, t2: float, t3: float, t4: float) -> bool:
        if (
            not all(map(math.isfinite, (t1, t2, t3, t4)))
            or not 0 <= t4 - t1 <= 1.0
            or not 0 <= t3 - t2 <= t4 - t1 + _TIMESTAMP_ERROR
            or (
                self._samples
                and (
                    t4 <= self._samples[-1].sender_received or t3 <= self._samples[-1].receiver_sent
                )
            )
        ):
            self.clear()
            return False
        padding = _TIMESTAMP_ERROR + (t4 - t1) * _DRIFT
        lower, upper = t3 - t4 - padding, t2 - t1 + padding
        self._samples.append(_Exchange(t4, t3, lower, upper))
        if self.bounds(t4) is None:
            self.clear()
            return False
        return True

    def bounds(self, sender_now: float) -> tuple[float, float] | None:
        return self._bounds(sender_now, receiver=False)

    def receiver_bounds(self, receiver_now: float) -> tuple[float, float] | None:
        return self._bounds(receiver_now, receiver=True)

    def _bounds(self, now: float, *, receiver: bool) -> tuple[float, float] | None:
        lower, upper = -math.inf, math.inf
        for sample in self._samples:
            anchor = sample.receiver_sent if receiver else sample.sender_received
            elapsed = now - anchor
            if not 0 <= elapsed <= WINDOW_SECONDS:
                continue
            margin = elapsed * _DRIFT
            lower = max(lower, sample.lower - margin)
            upper = min(upper, sample.upper + margin)
        return (lower, upper) if 0 <= upper - lower <= _MAX_OFFSET_WIDTH else None

    def age_upper(self, sampled_at: float, receiver_now: float) -> float:
        bounds = self.receiver_bounds(receiver_now)
        if bounds is None:
            return math.inf
        lower, upper = bounds
        age = receiver_now - sampled_at
        # A wholly future interval is an invalid source time.
        return age - lower if age >= lower and math.isfinite(age - upper) else math.inf


class DelayBudget:
    def __init__(self) -> None:
        self._samples: deque[tuple[float, float]] = deque(maxlen=_MAX_SAMPLES)

    def clear(self) -> None:
        self._samples.clear()

    def note(self, now: float, delay: float) -> None:
        if math.isfinite(now) and math.isfinite(delay) and delay >= 0:
            self._samples.append((now, delay))

    def compensation(self, now: float) -> float | None:
        samples = sorted(delay for at, delay in self._samples if 0 <= now - at <= WINDOW_SECONDS)
        if len(samples) < 3:
            return None
        p90 = samples[math.ceil(len(samples) * 0.9) - 1]
        return min(DRIVE_TRANSPORT_SECONDS, math.ceil(p90 * 1000) / 1000)


class ClockReceiver:
    """Bind a confirmation to one challenge on the authenticated socket."""

    def __init__(self) -> None:
        self.window = ClockWindow()
        self._pending: tuple[str, float, float, float] | None = None

    def sent(self, nonce: str, t1: float, t2: float, t3: float) -> None:
        self._pending = nonce, t1, t2, t3

    def confirm(self, nonce: str, t4: float, now: float) -> float | None:
        pending, self._pending = self._pending, None
        if pending is None or pending[0] != nonce or not 0 <= now - pending[3] <= WINDOW_SECONDS:
            self.window.clear()
            return None
        _, t1, t2, t3 = pending
        if not self.window.observe(t1, t2, t3, t4):
            return None
        bounds = self.window.receiver_bounds(now)
        if bounds is None:
            return None
        return max(0.0, t4 - t3 + bounds[1])
