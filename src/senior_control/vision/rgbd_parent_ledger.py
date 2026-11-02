"""Parent capture frontier and bounded retained tickets for diagnostic workers.

The caller must authenticate each bridge GID with MessageInfo and current graph
before calling. This contract cannot authenticate a DDS publisher itself.
"""

import math
from dataclasses import dataclass

from senior_control.vision.independent_motion import _headers_valid, _immutable_header
from senior_control.vision.raw_capture import RawCaptureHeader


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
            h.bridge_gid,
            h.raw_gid,
            h.source_epoch,
            h.frame_id,
        )
        for h in headers
    )


class CaptureLedger:
    def __init__(
        self, *, capacity: int, fresh_after_ros_ns: int, fresh_after_steady: float
    ) -> None:
        if type(capacity) is not int or not 1 <= capacity <= 64:
            raise ValueError("invalid_capacity")
        if (
            type(fresh_after_ros_ns) is not int
            or fresh_after_ros_ns <= 0
            or type(fresh_after_steady) not in (int, float)
            or not math.isfinite(fresh_after_steady)
            or fresh_after_steady <= 0
        ):
            raise ValueError("invalid_capture_floor")
        self._fresh_after_ros_ns = fresh_after_ros_ns
        self._fresh_after_steady = fresh_after_steady
        self.capacity = capacity
        self.generation = 1
        self._number = 0
        self._lineage: tuple[tuple[object, ...], ...] | None = None
        self._frontier: tuple[RawCaptureHeader, ...] | None = None
        self._retired: set[tuple[tuple[object, ...], ...]] = set()
        self._retired_streams: tuple[set[tuple[object, ...]], ...] = tuple(set() for _ in range(4))
        self._retired_raw_gids: tuple[set[str], ...] = tuple(set() for _ in range(4))
        self._retired_bridge_gids: tuple[set[str], ...] = tuple(set() for _ in range(4))
        self._retired_boots: set[str] = set()
        self._tickets: dict[int, CaptureTicket] = {}
        self._closed = False
        self._last_clock: tuple[int, float] | None = None
        self._binding_generation = 0

    def invalidate(self) -> None:
        self.generation += 1
        self._tickets.clear()

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
        if (
            type(now_ros_ns) is not int
            or now_ros_ns <= 0
            or type(now_steady) not in (float, int)
            or not math.isfinite(now_steady)
            or now_steady <= 0
        ):
            self.invalidate()
            self._closed = True
            return False
        if self._last_clock is not None:
            ros, steady = self._last_clock
            if now_ros_ns < ros or now_steady < steady:
                self.invalidate()
                self._closed = True
                return False
        self._last_clock = (now_ros_ns, now_steady)
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
        if any(
            h.stamp_ns <= self._fresh_after_ros_ns
            or h.received_ros_ns <= self._fresh_after_ros_ns
            or h.received_steady <= self._fresh_after_steady
            for h in headers
        ):
            return None
        source = _lineage(headers)
        if (
            source in self._retired
            or any(value in self._retired_streams[i] for i, value in enumerate(source))
            or any(
                h.raw_gid in self._retired_raw_gids[i]
                or h.bridge_gid in self._retired_bridge_gids[i]
                or h.bridge_boot_id in self._retired_boots
                for i, h in enumerate(headers)
            )
        ):
            return None
        if len(bridges) != 4 or any(
            h.bridge_gid != gid for h, gid in zip(headers, bridges, strict=True)
        ):
            self.invalidate()
            return None
        if self._lineage is not None and source != self._lineage:
            assert self._frontier is not None
            old = self._frontier
            same_boot = all(
                a.bridge_boot_id == b.bridge_boot_id for a, b in zip(old, headers, strict=True)
            )
            if same_boot:
                replacement = all(
                    a.bridge_gid == b.bridge_gid for a, b in zip(old, headers, strict=True)
                ) and all(
                    b.capture_generation > a.capture_generation
                    for a, b in zip(old, headers, strict=True)
                )
            else:
                replacement = all(
                    a.bridge_boot_id != b.bridge_boot_id and a.bridge_gid != b.bridge_gid
                    for a, b in zip(old, headers, strict=True)
                )
            if not replacement or len(self._retired) >= 64:
                self.invalidate()
                self._closed = True
                return None
            self._retired.add(self._lineage)
            for i, (a, b) in enumerate(zip(old, headers, strict=True)):
                if a.raw_gid != b.raw_gid:
                    self._retired_raw_gids[i].add(a.raw_gid)
                if a.bridge_gid != b.bridge_gid:
                    self._retired_bridge_gids[i].add(a.bridge_gid)
                if a.bridge_boot_id != b.bridge_boot_id:
                    self._retired_boots.add(a.bridge_boot_id)
            for i, (previous, current) in enumerate(zip(self._lineage, source, strict=True)):
                if previous != current:
                    self._retired_streams[i].add(previous)
            self.invalidate()
            self._lineage, self._frontier = source, None
        if self._frontier is not None and any(
            b.sequence <= a.sequence
            or b.stamp_ns <= a.stamp_ns
            or b.received_ros_ns <= a.received_ros_ns
            or b.received_steady <= a.received_steady
            for a, b in zip(self._frontier, headers, strict=True)
        ):
            return None
        self._lineage, self._frontier = source, headers
        if (
            not _headers_valid(headers, now_ros_ns, now_steady)
            or type(binding_generation) is not int
            or binding_generation <= 0
        ):
            self.invalidate()
            return None
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
                a.stamp_ns < b.stamp_ns for a, b in zip(first.headers, second.headers, strict=True)
            )
        )
