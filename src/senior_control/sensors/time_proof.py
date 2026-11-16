"""Immutable same-host source timing, established before DDS/worker queues.

A driver_read timestamp describes the earliest observable driver receipt, not a
claim about physical acquisition or unknown device/USB/kernel buffering.
"""

import math
from dataclasses import dataclass
from typing import Any

from senior_control.motion.navigation_clock import monotonic_domain


@dataclass(frozen=True)
class MeasurementTime:
    """Physical coordinate, never a host expiry or control-computation timestamp.

    Host driver receipt uses boot CLOCK_MONOTONIC. Simulation uses an explicit
    world/reset epoch and physical tick; producer incarnation remains separate.
    """

    clock_domain: str
    epoch: str | None
    seconds: float

    def __post_init__(self) -> None:
        if (
            not isinstance(self.clock_domain, str)
            or not self.clock_domain
            or type(self.seconds) not in (int, float)
            or not math.isfinite(self.seconds)
            or self.seconds < 0
            or self.epoch is not None
            and (not isinstance(self.epoch, str) or not self.epoch)
        ):
            raise ValueError("invalid measurement coordinate")

    def same_axis(self, other: "MeasurementTime") -> bool:
        return (self.clock_domain, self.epoch) == (other.clock_domain, other.epoch)

    def elapsed(self, earlier: "MeasurementTime") -> float:
        if not self.same_axis(earlier):
            raise ValueError("incompatible physical measurement axes")
        return self.seconds - earlier.seconds

    @classmethod
    def host(cls, seconds: float, domain: str | None = None) -> "MeasurementTime":
        return cls("host:" + (monotonic_domain() if domain is None else domain), None, seconds)


@dataclass(frozen=True)
class MeasurementInterval:
    start: MeasurementTime
    end: MeasurementTime

    def __post_init__(self) -> None:
        if self.end.elapsed(self.start) < 0:
            raise ValueError("reversed physical measurement interval")

    @property
    def duration(self) -> float:
        return self.end.elapsed(self.start)


@dataclass(frozen=True)
class SourceTime:
    clock_domain: str
    source_epoch: str
    sequence: int
    capture_start_steady_ns: int
    capture_end_steady_ns: int
    time_basis: str
    measurement_tick_ns: int | None = None
    measurement_epoch: str | None = None

    def __post_init__(self) -> None:
        if (
            not isinstance(self.clock_domain, str)
            or not 1 <= len(self.clock_domain) <= 36
            or not isinstance(self.source_epoch, str)
            or not self.source_epoch
            or len(self.source_epoch) > 64
            or type(self.sequence) is not int
            or not 1 <= self.sequence < 2**64
            or not all(
                type(t) is int and 0 <= t < 2**64
                for t in (self.capture_start_steady_ns, self.capture_end_steady_ns)
            )
            or self.capture_start_steady_ns > self.capture_end_steady_ns
            or not isinstance(self.time_basis, str)
            or self.time_basis not in {"driver_read", "sim_step", "control_compute"}
        ):
            raise ValueError("invalid source time proof")
        if self.time_basis == "sim_step":
            if (
                type(self.measurement_tick_ns) is not int
                or not 0 <= self.measurement_tick_ns < 2**64
                or not isinstance(self.measurement_epoch, str)
                or not self.measurement_epoch
                or len(self.measurement_epoch) > 64
            ):
                raise ValueError("missing simulation physical measurement proof")
        elif self.measurement_tick_ns is not None or self.measurement_epoch is not None:
            raise ValueError("physical simulation tick on a non-simulation source")

    @property
    def start_steady(self) -> float:
        return self.capture_start_steady_ns / 1e9

    @property
    def end_steady(self) -> float:
        return self.capture_end_steady_ns / 1e9

    @property
    def measurement(self) -> MeasurementTime:
        if self.time_basis == "sim_step":
            assert self.measurement_tick_ns is not None
            return MeasurementTime(
                "sim_step:" + self.clock_domain,
                self.measurement_epoch,
                self.measurement_tick_ns / 1e9,
            )
        return MeasurementTime.host(self.end_steady, self.clock_domain)


def source_time(wire: Any, *, domain: str | None = None) -> SourceTime:
    def uint64(name: str) -> int:
        value = getattr(wire, name)
        if type(value) is not int or not 0 <= value < 2**64:
            raise ValueError("invalid source uint64: " + name)
        return value

    try:
        proof = SourceTime(
            wire.clock_domain,
            wire.source_epoch,
            uint64("sequence"),
            uint64("capture_start_steady_ns"),
            uint64("capture_end_steady_ns"),
            wire.time_basis,
            uint64("measurement_tick_ns") if wire.time_basis == "sim_step" else None,
            wire.measurement_epoch if wire.time_basis == "sim_step" else None,
        )
    except (AttributeError, TypeError, OverflowError) as exc:
        raise ValueError("invalid source time wire proof") from exc
    if proof.clock_domain != (monotonic_domain() if domain is None else domain):
        raise ValueError("foreign monotonic source domain")
    return proof


class SourceFrontier:
    """Original producer order; a receive callback never supplies a new timestamp."""

    def __init__(self) -> None:
        self.latest: SourceTime | None = None
        self.retired: set[str] = set()

    def accept(self, incoming: SourceTime) -> bool:
        old = self.latest
        if incoming.source_epoch in self.retired:
            return False
        if old is not None:
            if incoming.end_steady < old.end_steady:
                return False
            if incoming.source_epoch == old.source_epoch:
                if incoming.sequence <= old.sequence:
                    return False
                if (
                    incoming.measurement.same_axis(old.measurement)
                    and incoming.measurement.seconds < old.measurement.seconds
                ):
                    return False
            else:
                if len(self.retired) >= 256:
                    return False
                self.retired.add(old.source_epoch)
        self.latest = incoming
        return True


def measurement_time(value: Any) -> MeasurementTime:
    """Decode a typed local record; never infer a physical axis from a ROS stamp."""
    if isinstance(value, MeasurementTime):
        return value
    if not isinstance(value, dict) or set(value) != {"clock_domain", "epoch", "seconds"}:
        raise ValueError("invalid serialized measurement coordinate")
    if (
        not isinstance(value["clock_domain"], str)
        or value["epoch"] is not None
        and not isinstance(value["epoch"], str)
        or type(value["seconds"]) not in (int, float)
    ):
        raise ValueError("invalid serialized measurement coordinate")
    return MeasurementTime(value["clock_domain"], value["epoch"], value["seconds"])


def measurement_interval(value: Any) -> MeasurementInterval:
    if isinstance(value, MeasurementInterval):
        return value
    if not isinstance(value, dict) or set(value) != {"start", "end"}:
        raise ValueError("invalid serialized measurement interval")
    return MeasurementInterval(measurement_time(value["start"]), measurement_time(value["end"]))
