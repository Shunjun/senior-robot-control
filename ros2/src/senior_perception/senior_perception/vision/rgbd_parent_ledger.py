"""Parent capture frontier and bounded retained tickets for diagnostic workers.

The receiving entry owns publisher and capture ordering checks. This ledger
associates worker tickets with the already accepted capture lifetime.
"""

import math
from dataclasses import dataclass

from senior_perception.vision.independent_motion import _headers_valid, _immutable_header
from senior_perception.vision.raw_capture import RawCaptureHeader


@dataclass(frozen=True)
class CaptureTicket:
    number: int
    generation: int
    binding_generation: int
    headers: tuple[RawCaptureHeader, ...]


def _lineage(headers: tuple[RawCaptureHeader, ...]) -> tuple[tuple[object, ...], ...]:
    return tuple(
        (
            h.bridge_boot_id,
            h.capture_generation,
            h.time.clock_generation,
            h.frame_id,
        )
        for h in headers
    )


class CaptureLedger:
    def __init__(self, *, capacity: int, fresh_after_steady: float) -> None:
        if type(capacity) is not int or not 1 <= capacity <= 64:
            raise ValueError("invalid_capacity")
        if (
            type(fresh_after_steady) not in (int, float)
            or not math.isfinite(fresh_after_steady)
            or fresh_after_steady <= 0
        ):
            raise ValueError("invalid_capture_floor")
        self._fresh_after_steady = fresh_after_steady
        self.capacity = capacity
        self.generation = 1
        self._number = 0
        self._lineage: tuple[tuple[object, ...], ...] | None = None
        self._frontier: tuple[RawCaptureHeader, ...] | None = None
        self._tickets: dict[int, CaptureTicket] = {}
        self._closed = False
        self._last_clock: float | None = None
        self._binding_generation = 0

    def invalidate(self) -> None:
        self.generation += 1
        self._tickets.clear()

    def advance_fresh_floor(self, steady: float) -> None:
        """Retire live tickets without discarding source history after local loss."""
        if (
            self._closed
            or type(steady) not in (int, float)
            or not math.isfinite(steady)
            or steady <= 0
            or steady < self._fresh_after_steady
            or (self._last_clock is not None and steady < self._last_clock)
        ):
            self.invalidate()
            raise ValueError("invalid_fresh_floor")
        self._fresh_after_steady = steady
        self.invalidate()

    def observe_binding(self, generation: int) -> bool:
        if type(generation) is not int or generation <= 0:
            self.invalidate()
            return False
        if generation < self._binding_generation:
            return False
        if generation > self._binding_generation:
            if self._binding_generation:
                self.invalidate()
            self._binding_generation = generation
        return not self._closed

    def observe_clock(self, now_ros_ns: int, now_steady: float) -> bool:
        if type(now_steady) not in (float, int) or not math.isfinite(now_steady) or now_steady <= 0:
            self.invalidate()
            return False
        if self._last_clock is not None:
            if now_steady < self._last_clock:
                self.invalidate()
                self._last_clock = now_steady
                return False
        self._last_clock = now_steady
        return not self._closed

    def offer(
        self,
        headers: tuple[RawCaptureHeader, ...],
        *,
        bridges: tuple[str, ...],
        binding_generation: int,
        now_ros_ns: int,
        now_steady: float,
    ) -> CaptureTicket | None:
        if not self.observe_clock(now_ros_ns, now_steady):
            return None
        if not self.observe_binding(binding_generation):
            return None
        if (
            self._closed
            or type(headers) is not tuple
            or len(headers) != 4
            or not all(_immutable_header(h) for h in headers)
        ):
            self.invalidate()
            return None
        if any(h.start_steady <= self._fresh_after_steady for h in headers):
            return None
        source = _lineage(headers)
        if len(bridges) != 4:
            self.invalidate()
            return None
        if self._frontier is not None:
            # Worker tickets cannot cross an already adopted source batch. No
            # origin blacklist, GID comparison or second sample-order frontier.
            for incoming, prior in zip(headers, self._frontier, strict=True):
                if incoming.bridge_boot_id == prior.bridge_boot_id:
                    if incoming.capture_generation < prior.capture_generation:
                        return None
                elif incoming.time.received_steady_ns <= prior.time.received_steady_ns:
                    return None
            if source == self._lineage and all(
                a.stamp_ns == b.stamp_ns for a, b in zip(headers, self._frontier, strict=True)
            ):
                return None  # Same capture already has a ticket, regardless of sequence.
        if (
            not _headers_valid(headers, now_ros_ns, now_steady)
            or type(binding_generation) is not int
            or binding_generation <= 0
        ):
            return None
        if self._lineage is not None and source != self._lineage:
            self.invalidate()
        self._lineage, self._frontier = source, headers
        expired = [
            n
            for n, t in self._tickets.items()
            if not self.current(
                t,
                binding_generation=t.binding_generation,
                now_ros_ns=now_ros_ns,
                now_steady=now_steady,
            )
        ]
        for number in expired:
            del self._tickets[number]
        if len(self._tickets) >= self.capacity:
            del self._tickets[min(self._tickets)]
        self._number += 1
        ticket = CaptureTicket(self._number, self.generation, binding_generation, headers)
        self._tickets[ticket.number] = ticket
        return ticket

    def current(
        self, ticket: CaptureTicket, *, binding_generation: int, now_ros_ns: int, now_steady: float
    ) -> bool:
        if not self.observe_clock(now_ros_ns, now_steady):
            return False
        if not self.observe_binding(binding_generation):
            return False
        return (
            type(ticket) is CaptureTicket
            and self._tickets.get(ticket.number) == ticket
            and ticket.generation == self.generation
            and ticket.binding_generation == binding_generation
            and math.isfinite(now_steady)
            and _headers_valid(ticket.headers, now_ros_ns, now_steady)
        )

    def pair_current(
        self,
        first: CaptureTicket,
        second: CaptureTicket,
        *,
        binding_generation: int,
        now_ros_ns: int,
        now_steady: float,
    ) -> bool:
        """Recheck both retained capture tickets under the same current TF lifetime."""
        if not self.current(
            first,
            binding_generation=binding_generation,
            now_ros_ns=now_ros_ns,
            now_steady=now_steady,
        ):
            return False
        if not self.current(
            second,
            binding_generation=binding_generation,
            now_ros_ns=now_ros_ns,
            now_steady=now_steady,
        ):
            return False
        return (
            first.number < second.number
            and first.generation == second.generation
            and first.binding_generation == second.binding_generation
            and all(
                a.measurement_steady < b.measurement_steady
                for a, b in zip(first.headers, second.headers, strict=True)
            )
        )
