"""Integer ROS measurement coordinates and their immutable first-receipt mapping."""

from dataclasses import dataclass

_INT64_MAX = (1 << 63) - 1
_UINT64_MAX = (1 << 64) - 1


def _coordinate(name: str, value: int, maximum: int) -> None:
    if type(value) is not int or not 0 <= value <= maximum:
        raise ValueError(f"{name} outside integer nanosecond range")


@dataclass(frozen=True)
class SampleTime:
    """A measurement mapped once at ingress, retained through downstream queues."""

    stamp_ns: int
    received_ros_ns: int
    received_steady_ns: int
    sample_steady_ns: int
    clock_generation: int

    def __post_init__(self) -> None:
        _coordinate("stamp_ns", self.stamp_ns, _INT64_MAX)
        _coordinate("received_ros_ns", self.received_ros_ns, _INT64_MAX)
        _coordinate("received_steady_ns", self.received_steady_ns, _UINT64_MAX)
        _coordinate("sample_steady_ns", self.sample_steady_ns, _UINT64_MAX)
        _coordinate("clock_generation", self.clock_generation, _UINT64_MAX)
        if self.stamp_ns > self.received_ros_ns:
            raise ValueError("measurement is in the future")
        expected = self.received_steady_ns - (self.received_ros_ns - self.stamp_ns)
        if expected < 0 or self.sample_steady_ns != expected:
            raise ValueError("invalid first-receipt mapping")


def map_sample(
    *, stamp_ns: int, received_ros_ns: int, received_steady_ns: int, clock_generation: int
) -> SampleTime:
    """Map a standard header stamp using integer subtraction, including stamp zero."""
    for name, value, maximum in (
        ("stamp_ns", stamp_ns, _INT64_MAX),
        ("received_ros_ns", received_ros_ns, _INT64_MAX),
        ("received_steady_ns", received_steady_ns, _UINT64_MAX),
        ("clock_generation", clock_generation, _UINT64_MAX),
    ):
        _coordinate(name, value, maximum)
    return SampleTime(
        stamp_ns,
        received_ros_ns,
        received_steady_ns,
        received_steady_ns - (received_ros_ns - stamp_ns),
        clock_generation,
    )


@dataclass(frozen=True)
class StampInterval:
    """Original ROS endpoints; callers convert their difference to seconds if needed."""

    start_ns: int
    end_ns: int

    def __post_init__(self) -> None:
        _coordinate("start_ns", self.start_ns, _INT64_MAX)
        _coordinate("end_ns", self.end_ns, _INT64_MAX)
        if self.end_ns < self.start_ns:
            raise ValueError("reversed ROS stamp interval")


def interval_ns(first: SampleTime, last: SampleTime) -> StampInterval:
    """Join accepted endpoints of one source; generations are local to that owner."""
    if first.clock_generation != last.clock_generation:
        raise ValueError("clock generation changed within source interval")
    return StampInterval(first.stamp_ns, last.stamp_ns)


@dataclass(frozen=True)
class ClockObservation:
    """Notification for the owner to revoke affected history and queued evidence."""

    generation: int
    discontinuity: bool


class ClockTracker:
    """Detect ordinary ROS/steady rollback without rate inference or reset proofs."""

    def __init__(self) -> None:
        self._previous: tuple[int, int] | None = None
        self._generation = 1

    def observe(self, *, ros_ns: int, steady_ns: int) -> ClockObservation:
        """Advance the local generation on either clock decreasing."""
        _coordinate("ros_ns", ros_ns, _INT64_MAX)
        _coordinate("steady_ns", steady_ns, _UINT64_MAX)
        rollback = self._previous is not None and (
            ros_ns < self._previous[0] or steady_ns < self._previous[1]
        )
        if rollback:
            if self._generation == _UINT64_MAX:
                raise ValueError("clock generation exhausted")
            self._generation += 1
        self._previous = ros_ns, steady_ns
        return ClockObservation(self._generation, rollback)
